"""Framework-owned control-loop orchestration independent of ROS timers."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import sys
from threading import current_thread, Event, Lock, Thread
import time
from typing import Callable, Protocol

from bxi_example_py_elf3.mod_api import MotorFrame, TransitionSpec

from .control_scheduler import ControlCycleMetrics, ControlScheduler, LoggerLike
from .controller import RobotControlFramework, RobotObservation
from .mod_nodes import ExecutorLike


class ControlPlatformAdapter(Protocol):
    """Minimal platform boundary used by the reusable control runtime."""

    def startup_step(self, now: float) -> bool:
        """Advance platform startup and return whether control may run."""
        ...

    def snapshot_control_inputs(
        self,
    ) -> tuple[RobotObservation, Sequence[str]]:
        """Return one coherent observation and pending edge events."""
        ...

    def publish_motor_frame(self, frame: MotorFrame) -> None:
        """Send one framework output frame to the platform."""
        ...


@dataclass(frozen=True)
class ControlRuntimeConfig:
    """Validated scheduler and supervision settings."""

    period_sec: float = 0.02
    compute_budget_sec: float = 0.002
    deadline_tolerance_sec: float = 0.0
    maintenance_hz: float = 5.0
    statistics_interval_sec: float = 30.0
    maintenance_guard_sec: float = 0.005
    python_switch_interval_sec: float = 0.001
    cpu_affinity: int = -1
    realtime_priority: int = 0

    @classmethod
    def from_mapping(cls, raw: object) -> "ControlRuntimeConfig":
        if raw is None:
            raw = {}
        if not isinstance(raw, Mapping):
            raise ValueError("control_runtime must be a YAML map")

        allowed = {
            "period_sec",
            "compute_budget_sec",
            "deadline_tolerance_sec",
            "maintenance_hz",
            "statistics_interval_sec",
            "maintenance_guard_sec",
            "python_switch_interval_sec",
            "cpu_affinity",
            "realtime_priority",
        }
        unknown = set(raw) - allowed
        if unknown:
            raise ValueError(
                f"control_runtime contains unknown fields: {sorted(unknown)}"
            )

        defaults = cls()
        config = cls(
            period_sec=_number(raw, "period_sec", defaults.period_sec),
            compute_budget_sec=_number(
                raw, "compute_budget_sec", defaults.compute_budget_sec
            ),
            deadline_tolerance_sec=_number(
                raw,
                "deadline_tolerance_sec",
                defaults.deadline_tolerance_sec,
            ),
            maintenance_hz=_number(raw, "maintenance_hz", defaults.maintenance_hz),
            statistics_interval_sec=_number(
                raw,
                "statistics_interval_sec",
                defaults.statistics_interval_sec,
            ),
            maintenance_guard_sec=_number(
                raw,
                "maintenance_guard_sec",
                defaults.maintenance_guard_sec,
            ),
            python_switch_interval_sec=_number(
                raw,
                "python_switch_interval_sec",
                defaults.python_switch_interval_sec,
            ),
            cpu_affinity=_integer(raw, "cpu_affinity", defaults.cpu_affinity),
            realtime_priority=_integer(
                raw, "realtime_priority", defaults.realtime_priority
            ),
        )
        config._validate()
        return config

    def _validate(self) -> None:
        if self.period_sec <= 0.0:
            raise ValueError("control_runtime.period_sec must be greater than zero")
        if not 0.0 <= self.compute_budget_sec < self.period_sec:
            raise ValueError(
                "control_runtime.compute_budget_sec must be non-negative and "
                "less than period_sec"
            )
        if self.deadline_tolerance_sec < 0.0:
            raise ValueError(
                "control_runtime.deadline_tolerance_sec must be non-negative"
            )
        if self.maintenance_hz <= 0.0:
            raise ValueError("control_runtime.maintenance_hz must be greater than zero")
        if self.statistics_interval_sec <= 0.0:
            raise ValueError(
                "control_runtime.statistics_interval_sec must be greater than zero"
            )
        if not 0.0 <= self.maintenance_guard_sec < self.period_sec:
            raise ValueError(
                "control_runtime.maintenance_guard_sec must be non-negative and "
                "less than period_sec"
            )
        if self.python_switch_interval_sec <= 0.0:
            raise ValueError(
                "control_runtime.python_switch_interval_sec must be greater than zero"
            )
        if self.cpu_affinity < -1:
            raise ValueError("control_runtime.cpu_affinity must be -1 or a CPU index")
        if not 0 <= self.realtime_priority <= 99:
            raise ValueError("control_runtime.realtime_priority must be in [0, 99]")


class RobotControlRuntime:
    """Own the framework, deterministic scheduler and background supervision."""

    def __init__(
        self,
        system_config: Mapping[str, object],
        *,
        built_in_mod_root: Path,
        dof_num: int,
        ros_node: object,
        platform: ControlPlatformAdapter,
        logger: LoggerLike | None = None,
        fatal_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.config = ControlRuntimeConfig.from_mapping(
            system_config.get("control_runtime")
        )
        framework_config = {
            key: value
            for key, value in system_config.items()
            if key != "control_runtime"
        }
        self._platform = platform
        self._logger = logger
        self._external_fatal_callback = fatal_callback
        self._framework_lock = Lock()
        self._stop_event = Event()
        self._maintenance_thread: Thread | None = None
        self._deadline_miss_queue: SimpleQueue[dict[str, object]] = SimpleQueue()
        self._started = False
        self._closed = False
        self._original_python_switch_interval: float | None = None
        self._last_control_events: tuple[str, ...] = ()
        self._last_reported_total_cycles = 0
        self._last_reported_total_deadline_misses = 0
        self._last_reported_total_skipped_periods = 0

        self.framework = RobotControlFramework(
            framework_config,
            built_in_mod_root=built_in_mod_root,
            dof_num=dof_num,
            ros_node=ros_node,
            control_period=self.config.period_sec,
        )
        self.scheduler = ControlScheduler(
            self._run_control_cycle,
            period_sec=self.config.period_sec,
            compute_budget_sec=self.config.compute_budget_sec,
            deadline_tolerance_sec=self.config.deadline_tolerance_sec,
            cpu_affinity=self.config.cpu_affinity,
            realtime_priority=self.config.realtime_priority,
            logger=logger,
            fatal_callback=self._on_control_fatal,
            deadline_miss_callback=self._enqueue_deadline_miss,
        )
        for message in self.framework.startup_messages():
            self._log("info", message)

    @property
    def is_running(self) -> bool:
        return self._started and self.scheduler.is_running

    @property
    def period_sec(self) -> float:
        return self.config.period_sec

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("RobotControlRuntime is closed")
        if self._started:
            return

        self._stop_event.clear()
        original_interval = sys.getswitchinterval()
        self._original_python_switch_interval = original_interval
        sys.setswitchinterval(self.config.python_switch_interval_sec)
        maintenance_thread = Thread(
            target=self._maintenance_loop,
            name="bxi-maintenance",
            daemon=False,
        )
        self._maintenance_thread = maintenance_thread
        try:
            maintenance_thread.start()
            self.scheduler.start()
            self._started = True
        except BaseException:
            self._stop_event.set()
            self.scheduler.stop()
            self._join_maintenance_thread()
            self._restore_python_switch_interval()
            raise

    def stop(self) -> None:
        self._stop_event.set()
        first_error: Exception | None = None
        try:
            self.scheduler.stop()
        except Exception as exc:
            first_error = exc

        background_error = self._join_maintenance_thread()
        if first_error is None:
            first_error = background_error
        self._started = False
        self._restore_python_switch_interval()
        if first_error is not None:
            raise first_error

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        first_error: Exception | None = None
        try:
            self.stop()
        except Exception as exc:
            first_error = exc
        try:
            with self._framework_lock:
                self.framework.close()
        except Exception as exc:
            if first_error is None:
                first_error = exc
            else:
                self._log("warning", f"control framework cleanup also failed: {exc}")
        if first_error is not None:
            raise first_error

    def attach_executor(self, executor: ExecutorLike) -> None:
        with self._framework_lock:
            self.framework.attach_executor(executor)

    def detach_executor(self) -> None:
        with self._framework_lock:
            self.framework.detach_executor()

    def extract_remote_events(
        self,
        values: object,
        *,
        sync_only: bool = False,
    ) -> list[str]:
        with self._framework_lock:
            return self.framework.extract_remote_events(values, sync_only=sync_only)

    def request_state(
        self,
        state_name: str,
        *,
        trigger: str,
        transition: TransitionSpec = None,
        delay: float = 0.0,
    ) -> None:
        with self._framework_lock:
            self.framework.request_state(
                state_name,
                trigger=trigger,
                transition=transition,
                delay=delay,
            )

    def snapshot(self, *, include_graph: bool = False) -> dict[str, object] | None:
        """Return a coherent status snapshot, or None near a control deadline."""
        if not self._wait_for_maintenance_window(self.config.period_sec):
            return None
        if not self._framework_lock.acquire(blocking=False):
            return None
        try:
            info = self.framework.snapshot(include_graph=include_graph)
        finally:
            self._framework_lock.release()
        info["control_timing"] = self.scheduler.status_snapshot()
        info["events"] = list(self._last_control_events)
        return info

    def _run_control_cycle(self) -> ControlCycleMetrics:
        if not self._platform.startup_step(time.monotonic()):
            return ControlCycleMetrics(state="startup", active=False)

        snapshot_started_ns = time.monotonic_ns()
        observation, events = self._platform.snapshot_control_inputs()
        events = tuple(events)
        self._last_control_events = events
        snapshot_finished_ns = time.monotonic_ns()

        framework_started_ns = snapshot_finished_ns
        with self._framework_lock:
            frame = self.framework.update(
                observation,
                events,
                self.config.period_sec,
            )
            state_name = self.framework.current_state_name
        framework_finished_ns = time.monotonic_ns()

        publish_started_ns = framework_finished_ns
        if frame is not None:
            self._platform.publish_motor_frame(frame)
        publish_finished_ns = time.monotonic_ns()
        return ControlCycleMetrics(
            state=state_name,
            snapshot_ns=snapshot_finished_ns - snapshot_started_ns,
            framework_ns=framework_finished_ns - framework_started_ns,
            publish_ns=publish_finished_ns - publish_started_ns,
        )

    def _maintenance_loop(self) -> None:
        period_sec = 1.0 / self.config.maintenance_hz
        next_maintenance_at = time.monotonic() + period_sec
        next_statistics_at = time.monotonic() + self.config.statistics_interval_sec
        while not self._stop_event.is_set():
            delay = max(0.0, next_maintenance_at - time.monotonic())
            if self._stop_event.wait(delay):
                break
            now = time.monotonic()
            next_maintenance_at += period_sec
            if now >= next_maintenance_at:
                skipped = int((now - next_maintenance_at) // period_sec) + 1
                next_maintenance_at += skipped * period_sec

            self._drain_deadline_misses()

            if self._wait_for_maintenance_window(self.config.period_sec):
                if self._framework_lock.acquire(blocking=False):
                    try:
                        self.framework.maintenance_update()
                    except Exception as exc:
                        self._log("warning", f"control maintenance failed: {exc}")
                    finally:
                        self._framework_lock.release()

            now = time.monotonic()
            if now >= next_statistics_at:
                self._report_control_timing()
                next_statistics_at = now + self.config.statistics_interval_sec

    def _enqueue_deadline_miss(self, miss: dict[str, object]) -> None:
        self._deadline_miss_queue.put(dict(miss))

    def _drain_deadline_misses(self) -> None:
        while True:
            try:
                miss = self._deadline_miss_queue.get_nowait()
            except Empty:
                return
            logged = self._log(
                "warning",
                "control deadline miss: "
                f"state={miss['state']}, "
                f"finish_late={miss['finish_late_ms']:.2f}ms, "
                f"wake_late={miss['wake_late_ms']:.2f}ms, "
                f"cycle={miss['cycle_ms']:.2f}ms, "
                f"count={miss['count']}",
            )
            if not logged:
                self._deadline_miss_queue.put(miss)
                return

    def _join_maintenance_thread(self) -> Exception | None:
        thread = self._maintenance_thread
        if thread is not None and thread is not current_thread() and thread.ident:
            thread.join(timeout=5.0)
            if thread.is_alive():
                return RuntimeError(
                    "control maintenance thread did not stop within timeout"
                )
        self._maintenance_thread = None
        return None

    def _wait_for_maintenance_window(self, timeout_sec: float) -> bool:
        wait_deadline = time.monotonic() + max(0.0, timeout_sec)
        while not self.scheduler.maintenance_window_available(
            self.config.maintenance_guard_sec
        ):
            if self._stop_event.is_set() or time.monotonic() >= wait_deadline:
                return False
            self._stop_event.wait(0.0005)
        return not self._stop_event.is_set()

    def _report_control_timing(self) -> None:
        timing = self.scheduler.timing_snapshot(reset_window=True)
        total_cycles = int(timing["total_cycles"])
        total_misses = int(timing["total_deadline_misses"])
        total_skipped = int(timing["total_skipped_periods"])
        cycles_since_report = max(
            0,
            total_cycles - self._last_reported_total_cycles,
        )
        misses_since_report = max(
            0,
            total_misses - self._last_reported_total_deadline_misses,
        )
        skipped_since_report = max(
            0,
            total_skipped - self._last_reported_total_skipped_periods,
        )
        wake = timing["wake_late_ms"]
        cycle = timing["cycle_ms"]
        interval = timing["frame_interval_ms"]
        message = (
            "control timing: "
            f"cycles={cycles_since_report}, state={timing['state']}, "
            f"wake_p99={wake['p99']:.2f}ms, "
            f"cycle_p99={cycle['p99']:.2f}ms, "
            f"cycle_max={cycle['max']:.2f}ms, "
            f"interval_p99={interval['p99']:.2f}ms, "
            f"interval_max={interval['max']:.2f}ms, "
            f"deadline_misses={misses_since_report}, "
            f"skipped_periods={skipped_since_report}, "
            f"total_misses={total_misses}"
        )
        logged = self._log("info", message)
        if logged:
            self._last_reported_total_cycles = total_cycles
            self._last_reported_total_deadline_misses = total_misses
            self._last_reported_total_skipped_periods = total_skipped

    def _on_control_fatal(self, message: str) -> None:
        self._stop_event.set()
        callback = self._external_fatal_callback
        if callback is not None:
            callback(message)

    def _restore_python_switch_interval(self) -> None:
        original = self._original_python_switch_interval
        self._original_python_switch_interval = None
        if original is not None:
            sys.setswitchinterval(original)

    def _log(self, level: str, message: str) -> bool:
        logger = self._logger
        if logger is None:
            return False
        try:
            # rclpy binds severity to the Python source location of a log call.
            # Keep each severity on a distinct call line instead of using one
            # dynamic getattr call for INFO, WARNING and ERROR.
            if level == "info":
                logger.info(message)
                return True
            if level == "warning":
                logger.warning(message)
                return True
            if level == "error":
                logger.error(message)
                return True
            return False
        except Exception:
            return False


def _number(raw: Mapping[object, object], name: str, default: float) -> float:
    value = raw.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"control_runtime.{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"control_runtime.{name} must be finite")
    return result


def _integer(raw: Mapping[object, object], name: str, default: int) -> int:
    value = raw.get(name, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"control_runtime.{name} must be an integer")
    return int(value)


__all__ = [
    "ControlPlatformAdapter",
    "ControlRuntimeConfig",
    "RobotControlRuntime",
]
