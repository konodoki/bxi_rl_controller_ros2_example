"""Atomic named-joint assembly backed by selectable bounded synchronizers."""
from __future__ import annotations

from collections import OrderedDict
import math
import time

import numpy as np

from bxi_example_py_elf3.framework.joints import (
    CompiledJointMap, JointLayout, JointStateBuffer, JointStateView,
)

from .synchronization import (
    ApproximateTimeSynchronizer, ExactTimeSynchronizer, QueueSynchronizer,
    TimeSynchronizer,
)


class SynchronizedJointStateSource:
    """Demultiplex caller-configured named groups into synchronized states.

    Each sample is consumed at most once. The default policy pairs nearby
    timestamps; exact/time and explicit arrival-order queue policies are also
    available. No interpolation or reuse of the other group's latest value.
    The caller serializes update/read using its input lock.
    Groups are supplied by the application, without robot-specific inference.
    Layout, output buffer and per-group queue storage are fixed at construction.
    """

    def __init__(self, *groups: JointLayout,
                 slop_sec: float = 0.02, queue_size: int = 8,
                 timeout_sec: float = 1.0, use_header_stamp: bool = True,
                 policy: str = "approximate",
                 dtype: np.dtype | type = np.float64):
        policies = {
            "approximate": ApproximateTimeSynchronizer,
            "exact": ExactTimeSynchronizer,
            "time": TimeSynchronizer,
            "queue": QueueSynchronizer,
        }
        if policy not in policies:
            raise ValueError(f"unknown joint sync policy: {policy!r}; expected {tuple(policies)}")
        if policy in {"exact", "time"} and not use_header_stamp:
            raise ValueError("exact/time joint synchronization requires header stamps, not receipt time")
        options = {"timeout_sec": timeout_sec}
        if policy == "approximate":
            options["slop_sec"] = slop_sec
        self.groups = groups
        self._synchronizer = policies[policy](len(groups), queue_size, **options)
        self.policy = policy
        self.layout = JointLayout(tuple(n for g in groups for n in g.names), label="synchronized robot")
        self._sets = tuple(frozenset(g.names) for g in groups)
        self._all_names = frozenset(self.layout.names)
        self._buffer = JointStateBuffer(self.layout, dtype=dtype)
        self.timeout_sec = timeout_sec
        self.queue_size = queue_size
        self.use_header_stamp = use_header_stamp
        self._positions = tuple(np.empty((queue_size, g.dof_num), dtype=dtype) for g in self.groups)
        self._velocities = tuple(np.empty_like(p) for p in self._positions)
        self._next = [0] * len(groups)
        self._last_received: float | None = None
        self._mappings: OrderedDict[tuple[str, ...], tuple[int, CompiledJointMap]] = OrderedDict()
        self._full_frames = self._full_dropped = 0

    @property
    def ready(self) -> bool:
        return self._last_received is not None

    @property
    def view(self) -> JointStateView:
        if not self.ready:
            raise RuntimeError("joint synchronizer is waiting for a complete synchronized group set")
        return self._buffer.view

    def is_fresh(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return self.ready and now - self._last_received < self.timeout_sec

    def diagnostics(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        info = self._synchronizer.diagnostics()
        return {"policy": self.policy, "received": info["received"],
                "pending": info["pending"],
                "groups": tuple(g.label for g in self.groups),
                "pairs": info["matches"], "full_frames": self._full_frames,
                "dropped": info["dropped"] + self._full_dropped,
                "last_pair_age_sec": None if not self.ready else now - self._last_received,
                "stamp_source": "header" if self.use_header_stamp else "receipt"}

    def require_fresh(self, now: float | None = None) -> None:
        if not self.is_fresh(now):
            raise RuntimeError(
                "joint feedback synchronization timeout (not an IMU failure): "
                f"timeout={self.timeout_sec:.3f}s, {self.diagnostics(now)}"
            )

    def _mapping(self, names):
        key = tuple(names)
        cached = self._mappings.get(key)
        if cached is not None:
            self._mappings.move_to_end(key)
            return cached
        incoming = JointLayout.create(key, label="incoming joint group")
        joint_set = frozenset(key)
        if joint_set == self._all_names:
            group, target = -1, self.layout
        else:
            try:
                group = self._sets.index(joint_set)
            except ValueError:
                raise ValueError(
                    "joint feedback must contain one complete configured group or the full layout; "
                    f"received={key}"
                ) from None
            target = self.groups[group]
        result = group, CompiledJointMap.compile(incoming, target, require_exact=True)
        self._mappings[key] = result
        if len(self._mappings) > 16:
            self._mappings.popitem(last=False)
        return result

    def update(self, names, position, velocity, *, timestamp_ns: int = 0,
               received_at: float | None = None) -> JointStateView | None:
        now = time.monotonic() if received_at is None else float(received_at)
        if not math.isfinite(now):
            raise ValueError("joint receive time must be finite")
        stamp = int(timestamp_ns) if self.use_header_stamp else int(now * 1e9)
        if stamp < 0 or (self.use_header_stamp and stamp == 0):
            raise ValueError("joint sync requires nonzero header.stamp; configure receipt time explicitly for unstamped drivers")
        stamp = np.int64(stamp)  # Validate range before changing queue or output.
        group, mapping = self._mapping(names)
        position = np.asarray(position, dtype=self._buffer.position.dtype)
        velocity = np.asarray(velocity, dtype=self._buffer.velocity.dtype)
        expected = (len(names),)
        if position.shape != expected or velocity.shape != expected:
            raise ValueError(f"joint arrays must match names: expected={expected}, position={position.shape}, velocity={velocity.shape}")
        if not np.isfinite(position).all() or not np.isfinite(velocity).all():
            raise ValueError("joint feedback position/velocity must be finite")
        if group == -1:
            if stamp <= max(self._synchronizer.last_timestamps):
                self._full_dropped += 1
                return None
            mapping.map_into(position, self._buffer.position)
            mapping.map_into(velocity, self._buffer.velocity)
            self._buffer.view.timestamp_ns = int(stamp)
            self._last_received = now
            self._synchronizer.clear(watermark_ns=int(stamp))
            self._full_frames += 1
            return self._buffer.view

        slot = self._next[group]
        result = self._synchronizer.add(
            group, int(stamp), slot, received_at=now, key=slot,
        )
        if not result.accepted:
            return None  # Never overwrite a still-queued slot on rejected input.
        self._next[group] = (slot + 1) % self.queue_size
        mapping.map_into(position, self._positions[group][slot])
        mapping.map_into(velocity, self._velocities[group][slot])
        if result.samples is None:
            return None
        offset = 0
        for g, sample in enumerate(result.samples):
            end = offset + self.groups[g].dof_num
            self._buffer.position[offset:end] = self._positions[g][sample.value]
            self._buffer.velocity[offset:end] = self._velocities[g][sample.value]
            offset = end
        self._buffer.view.timestamp_ns = min(s.timestamp_ns for s in result.samples)
        self._last_received = min(s.received_at for s in result.samples)
        return self._buffer.view


__all__ = ["SynchronizedJointStateSource"]
