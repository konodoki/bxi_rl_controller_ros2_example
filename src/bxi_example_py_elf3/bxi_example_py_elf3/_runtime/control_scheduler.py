"""Absolute-time scheduler for the framework control data path."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import os
from threading import current_thread, Event, Lock, Thread
import time
from typing import Protocol

import numpy as np


class LoggerLike(Protocol):
    def info(self, message: str) -> None:
        ...

    def warning(self, message: str) -> None:
        ...

    def error(self, message: str) -> None:
        ...


@dataclass(frozen=True)
class ControlCycleMetrics:
    """Measurements produced by one platform-specific control cycle."""

    state: str
    active: bool = True
    snapshot_ns: int = 0
    framework_ns: int = 0
    publish_ns: int = 0


def _percentile_summary(values_ns: list[int]) -> dict[str, float]:
    if not values_ns:
        return {
            "p50": 0.0,
            "p95": 0.0,
            "p99": 0.0,
            "max": 0.0,
        }
    values_ms = np.asarray(values_ns, dtype=np.float64) / 1_000_000.0
    return {
        "p50": float(np.percentile(values_ms, 50)),
        "p95": float(np.percentile(values_ms, 95)),
        "p99": float(np.percentile(values_ms, 99)),
        "max": float(np.max(values_ms)),
    }


class ControlScheduler:
    """Run all framework states on one drift-free, absolute time line."""

    def __init__(
        self,
        cycle: Callable[[], ControlCycleMetrics],
        *,
        period_sec: float,
        compute_budget_sec: float,
        deadline_tolerance_sec: float = 0.0,
        cpu_affinity: int = -1,
        realtime_priority: int = 0,
        logger: LoggerLike | None = None,
        fatal_callback: Callable[[str], None] | None = None,
        deadline_miss_callback: Callable[[dict[str, object]], None] | None = None,
        thread_name: str = "bxi-control",
    ) -> None:
        if period_sec <= 0.0:
            raise ValueError("control period must be greater than zero")
        if compute_budget_sec < 0.0 or compute_budget_sec >= period_sec:
            raise ValueError(
                "control compute budget must be non-negative and less than period"
            )
        if deadline_tolerance_sec < 0.0:
            raise ValueError("control deadline tolerance must be non-negative")
        if cpu_affinity < -1:
            raise ValueError("control CPU affinity must be -1 or a CPU index")
        if realtime_priority < 0 or realtime_priority > 99:
            raise ValueError("control realtime priority must be in [0, 99]")

        self._cycle = cycle
        self.period_ns = int(round(period_sec * 1_000_000_000.0))
        self.compute_budget_ns = int(round(compute_budget_sec * 1_000_000_000.0))
        self.deadline_tolerance_ns = int(
            round(deadline_tolerance_sec * 1_000_000_000.0)
        )
        self.cpu_affinity = int(cpu_affinity)
        self.realtime_priority = int(realtime_priority)
        self._logger = logger
        self._fatal_callback = fatal_callback
        self._deadline_miss_callback = deadline_miss_callback
        self._thread_name = thread_name

        self._stop_event = Event()
        self._thread: Thread | None = None
        self._stats_lock = Lock()
        self._reset_window_locked()
        self._total_cycles = 0
        self._total_deadline_misses = 0
        self._total_skipped_periods = 0
        self._last_state = "startup"
        self._last_miss: dict[str, object] | None = None
        self._fatal_error: str | None = None
        self._next_wake_ns: int | None = None

    @property
    def is_running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def fatal_error(self) -> str | None:
        with self._stats_lock:
            return self._fatal_error

    def start(self) -> None:
        if self.is_running:
            return
        self._stop_event.clear()
        thread = Thread(
            target=self._run,
            name=self._thread_name,
            daemon=False,
        )
        self._thread = thread
        thread.start()
        self._log(
            "info",
            "control scheduler started: "
            f"period={self.period_ns / 1_000_000.0:.2f}ms, "
            f"budget={self.compute_budget_ns / 1_000_000.0:.2f}ms, "
            f"cpu={self.cpu_affinity}, rt_priority={self.realtime_priority}",
        )

    def stop(self, timeout_sec: float = 5.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is None:
            return
        if thread is not current_thread():
            thread.join(timeout=max(0.0, float(timeout_sec)))
        if thread.is_alive():
            raise RuntimeError("control scheduler did not stop within timeout")
        self._thread = None

    def timing_snapshot(self, *, reset_window: bool = False) -> dict[str, object]:
        with self._stats_lock:
            wake = list(self._wake_ns)
            cycle = list(self._cycle_ns)
            frame_interval = list(self._frame_interval_ns)
            snapshot = list(self._snapshot_ns)
            framework = list(self._framework_ns)
            publish = list(self._publish_ns)
            finish_late = list(self._finish_late_ns)
            window_cycles = self._window_cycles
            window_deadline_misses = self._window_deadline_misses
            window_skipped_periods = self._window_skipped_periods
            total_cycles = self._total_cycles
            total_deadline_misses = self._total_deadline_misses
            total_skipped_periods = self._total_skipped_periods
            state = self._last_state
            last_miss = dict(self._last_miss) if self._last_miss else None
            fatal_error = self._fatal_error
            if reset_window:
                self._reset_window_locked()
        return {
            "period_ms": self.period_ns / 1_000_000.0,
            "compute_budget_ms": self.compute_budget_ns / 1_000_000.0,
            "cycles": window_cycles,
            "deadline_misses": window_deadline_misses,
            "skipped_periods": window_skipped_periods,
            "total_cycles": total_cycles,
            "total_deadline_misses": total_deadline_misses,
            "total_skipped_periods": total_skipped_periods,
            "state": state,
            "wake_late_ms": _percentile_summary(wake),
            "cycle_ms": _percentile_summary(cycle),
            "frame_interval_ms": _percentile_summary(frame_interval),
            "snapshot_ms": _percentile_summary(snapshot),
            "framework_ms": _percentile_summary(framework),
            "publish_ms": _percentile_summary(publish),
            "finish_late_ms": _percentile_summary(finish_late),
            "last_miss": last_miss,
            "fatal_error": fatal_error,
        }

    def status_snapshot(self) -> dict[str, object]:
        """Return cumulative counters without percentile work on the hot path."""
        with self._stats_lock:
            return {
                "period_ms": self.period_ns / 1_000_000.0,
                "compute_budget_ms": self.compute_budget_ns / 1_000_000.0,
                "total_cycles": self._total_cycles,
                "total_deadline_misses": self._total_deadline_misses,
                "total_skipped_periods": self._total_skipped_periods,
                "state": self._last_state,
                "last_miss": dict(self._last_miss) if self._last_miss else None,
                "fatal_error": self._fatal_error,
            }

    def maintenance_window_available(self, minimum_sec: float) -> bool:
        """Return whether non-control work has time before the next wakeup."""
        minimum_ns = max(0, int(round(float(minimum_sec) * 1_000_000_000.0)))
        next_wake_ns = self._next_wake_ns
        return next_wake_ns is None or time.monotonic_ns() + minimum_ns < next_wake_ns

    def _run(self) -> None:
        self._configure_current_thread()
        deadline_ns = time.monotonic_ns() + self.period_ns
        was_active = False
        last_active_finished_ns: int | None = None
        while not self._stop_event.is_set():
            wake_target_ns = deadline_ns - self.compute_budget_ns
            self._next_wake_ns = wake_target_ns
            if not self._wait_until(wake_target_ns):
                break
            wake_ns = time.monotonic_ns()
            cycle_started_ns = wake_ns
            try:
                metrics = self._cycle()
                if not isinstance(metrics, ControlCycleMetrics):
                    raise TypeError("control cycle must return ControlCycleMetrics")
            except BaseException as exc:
                message = f"control scheduler stopped after cycle failure: {exc}"
                with self._stats_lock:
                    self._fatal_error = message
                self._log("error", message)
                if self._fatal_callback is not None:
                    try:
                        self._fatal_callback(message)
                    except Exception as callback_exc:
                        self._log(
                            "error",
                            f"control fatal callback also failed: {callback_exc}",
                        )
                self._stop_event.set()
                break
            finished_ns = time.monotonic_ns()

            if not metrics.active:
                was_active = False
                last_active_finished_ns = None
                deadline_ns = finished_ns + self.period_ns
                continue
            if not was_active:
                with self._stats_lock:
                    self._reset_window_locked()
                deadline_ns = finished_ns + self.period_ns
                was_active = True
                last_active_finished_ns = finished_ns
                continue

            wake_late_ns = max(0, wake_ns - wake_target_ns)
            cycle_ns = max(0, finished_ns - cycle_started_ns)
            frame_interval_ns = (
                max(0, finished_ns - last_active_finished_ns)
                if last_active_finished_ns is not None
                else 0
            )
            last_active_finished_ns = finished_ns
            finish_late_ns = max(0, finished_ns - deadline_ns)
            deadline_missed = finish_late_ns > self.deadline_tolerance_ns

            next_deadline_ns = deadline_ns + self.period_ns
            skipped_periods = 0
            if finished_ns >= next_deadline_ns:
                skipped_periods = (
                    (finished_ns - next_deadline_ns) // self.period_ns
                ) + 1
                next_deadline_ns += skipped_periods * self.period_ns

            miss_event = self._record_cycle(
                metrics,
                wake_late_ns=wake_late_ns,
                cycle_ns=cycle_ns,
                frame_interval_ns=frame_interval_ns,
                finish_late_ns=finish_late_ns,
                deadline_missed=deadline_missed,
                skipped_periods=int(skipped_periods),
            )
            if miss_event is not None and self._deadline_miss_callback is not None:
                try:
                    self._deadline_miss_callback(miss_event)
                except Exception as exc:
                    self._log("error", f"deadline miss callback failed: {exc}")
            deadline_ns = next_deadline_ns
        self._next_wake_ns = None

    def _wait_until(self, target_ns: int) -> bool:
        while not self._stop_event.is_set():
            remaining_ns = target_ns - time.monotonic_ns()
            if remaining_ns <= 0:
                return True
            if self._stop_event.wait(remaining_ns / 1_000_000_000.0):
                return False
        return False

    def _record_cycle(
        self,
        metrics: ControlCycleMetrics,
        *,
        wake_late_ns: int,
        cycle_ns: int,
        frame_interval_ns: int,
        finish_late_ns: int,
        deadline_missed: bool,
        skipped_periods: int,
    ) -> dict[str, object] | None:
        miss_event: dict[str, object] | None = None
        with self._stats_lock:
            self._window_cycles += 1
            self._total_cycles += 1
            self._last_state = metrics.state
            self._wake_ns.append(wake_late_ns)
            self._cycle_ns.append(cycle_ns)
            self._frame_interval_ns.append(frame_interval_ns)
            self._snapshot_ns.append(max(0, int(metrics.snapshot_ns)))
            self._framework_ns.append(max(0, int(metrics.framework_ns)))
            self._publish_ns.append(max(0, int(metrics.publish_ns)))
            self._finish_late_ns.append(finish_late_ns)
            self._window_skipped_periods += skipped_periods
            self._total_skipped_periods += skipped_periods
            if deadline_missed:
                self._window_deadline_misses += 1
                self._total_deadline_misses += 1
                miss_event = {
                    "state": metrics.state,
                    "wake_late_ms": wake_late_ns / 1_000_000.0,
                    "cycle_ms": cycle_ns / 1_000_000.0,
                    "finish_late_ms": finish_late_ns / 1_000_000.0,
                    "count": self._total_deadline_misses,
                }
                self._last_miss = miss_event
        return dict(miss_event) if miss_event is not None else None

    def _configure_current_thread(self) -> None:
        if self.cpu_affinity >= 0:
            try:
                os.sched_setaffinity(0, {self.cpu_affinity})
            except (AttributeError, OSError) as exc:
                self._log(
                    "warning",
                    f"cannot set control CPU affinity to {self.cpu_affinity}: {exc}",
                )
        if self.realtime_priority > 0:
            try:
                os.sched_setscheduler(
                    0,
                    os.SCHED_FIFO,
                    os.sched_param(self.realtime_priority),
                )
            except (AttributeError, OSError) as exc:
                self._log(
                    "warning",
                    "cannot enable SCHED_FIFO for control thread; "
                    f"using normal scheduling: {exc}",
                )

    def _reset_window_locked(self) -> None:
        self._wake_ns: list[int] = []
        self._cycle_ns: list[int] = []
        self._frame_interval_ns: list[int] = []
        self._snapshot_ns: list[int] = []
        self._framework_ns: list[int] = []
        self._publish_ns: list[int] = []
        self._finish_late_ns: list[int] = []
        self._window_cycles = 0
        self._window_deadline_misses = 0
        self._window_skipped_periods = 0

    def _log(self, level: str, message: str) -> None:
        logger = self._logger
        if logger is None:
            return
        try:
            # rclpy binds severity to a Python log call's source location.
            # A shared dynamic call line can emit INFO once and then reject a
            # later WARNING or ERROR from that same line.
            if level == "info":
                logger.info(message)
                return
            if level == "warning":
                logger.warning(message)
                return
            if level == "error":
                logger.error(message)
        except Exception:
            pass


__all__ = ["ControlCycleMetrics", "ControlScheduler", "LoggerLike"]
