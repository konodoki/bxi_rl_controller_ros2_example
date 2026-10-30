"""ROS-independent bounded synchronizers for caller-serialized input streams.

This is not a drop-in ROS Subscriber/filter API: ``add`` takes an explicit
stream index, integer nanosecond stamp, and opaque payload, and returns a result.
TimeSynchronizer uses exact timestamps, as in message_filters. Arrival-order
pairing is deliberately named QueueSynchronizer, not TimeSynchronizer.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
from numbers import Integral
import time
from typing import Generic, Hashable, TypeVar


T = TypeVar("T")


@dataclass(frozen=True, slots=True)
class SynchronizedSample(Generic[T]):
    timestamp_ns: int
    received_at: float
    value: T


@dataclass(frozen=True, slots=True)
class SynchronizationResult(Generic[T]):
    accepted: bool
    samples: tuple[SynchronizedSample[T], ...] | None = None


_PENDING = SynchronizationResult(True)
_REJECTED = SynchronizationResult(False)


def _integer(value, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return int(value)


class TimeSynchronizer(Generic[T]):
    """Bounded exact-time queue synchronization for two or more streams.

    Every stream must supply the same integer timestamp. Samples are consumed
    once; duplicate or already-consumed timestamps are dropped, as are queued
    samples older than the selected sample in each stream. ``timeout_sec`` uses
    monotonic reception time, independently of the sample's clock.

    Payloads are retained by reference. An optional stream-local ``key`` lets a
    caller reuse a preallocated buffer slot: an accepted add replaces that key's
    queued sample. Check result.accepted before overwriting that buffer; process
    result.samples before the next add. No threads, callbacks or locks are owned.
    """
    policy = "time"
    _ordered_timestamps = True

    def __init__(self, stream_count: int, queue_size: int = 8, *,
                 timeout_sec: float = 1.0):
        self.stream_count = _integer(stream_count, "stream_count", minimum=2)
        self.queue_size = _integer(queue_size, "queue_size", minimum=1)
        if self.queue_size > 64:
            raise ValueError("queue_size must be in [1, 64]")
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("timeout_sec must be finite and positive")
        self.timeout_sec = float(timeout_sec)
        self._queues: tuple[OrderedDict[Hashable, SynchronizedSample[T]], ...] = tuple(
            OrderedDict() for _ in range(self.stream_count)
        )
        self._last_stamps = [-1] * self.stream_count
        self._received_counts = [0] * self.stream_count
        self._last_received: float | None = None
        self._matches = self._dropped = 0

    @property
    def last_timestamps(self) -> tuple[int, ...]:
        return tuple(self._last_stamps)

    def diagnostics(self) -> dict:
        return {"policy": self.policy, "received": tuple(self._received_counts),
                "pending": tuple(len(q) for q in self._queues),
                "matches": self._matches, "dropped": self._dropped}

    def is_fresh(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return self._last_received is not None and now - self._last_received < self.timeout_sec

    def clear(self, *, watermark_ns: int | None = None) -> None:
        """Discard pending samples; reset time or advance after a full frame.

        Counters are cumulative. A watermark prevents older half-frames being
        emitted after an externally supplied complete frame.
        """
        watermark = -1 if watermark_ns is None else _integer(watermark_ns, "watermark_ns")
        if watermark_ns is not None and watermark < max(self._last_stamps):
            raise ValueError("watermark cannot move backward; use clear() to reset clocks")
        for queue in self._queues:
            queue.clear()
        self._last_stamps[:] = [watermark] * self.stream_count
        self._last_received = None

    def _select(self, stream: int, sample: SynchronizedSample[T]):
        selected = []
        for i, queue in enumerate(self._queues):
            candidate = sample if i == stream else next(
                (s for s in queue.values() if s.timestamp_ns == sample.timestamp_ns), None
            )
            if candidate is None:
                return None
            selected.append(candidate)
        return tuple(selected)

    def add(self, stream: int, timestamp_ns: int, value: T, *,
            received_at: float | None = None,
            key: Hashable | None = None) -> SynchronizationResult[T]:
        stream = _integer(stream, "stream")
        if stream >= self.stream_count:
            raise ValueError("stream index is outside configured stream_count")
        stamp = _integer(timestamp_ns, "timestamp_ns")
        now = time.monotonic() if received_at is None else float(received_at)
        if not math.isfinite(now):
            raise ValueError("received_at must be finite")
        # Validate everything before touching liveness or queues.
        if key is None:
            key = object()
        hash(key)
        self._received_counts[stream] += 1
        for queue in self._queues:
            for old_key, old in tuple(queue.items()):
                if now - old.received_at >= self.timeout_sec:
                    del queue[old_key]
                    self._dropped += 1
        queue = self._queues[stream]
        if self._ordered_timestamps and (
            stamp <= self._last_stamps[stream]
            or any(s.timestamp_ns == stamp for s in queue.values())
        ):
            self._dropped += 1
            return _REJECTED
        if key in queue:
            del queue[key]
            self._dropped += 1
        sample = SynchronizedSample(stamp, now, value)
        queue[key] = sample
        if len(queue) > self.queue_size:
            queue.popitem(last=False)
            self._dropped += 1
        selected = self._select(stream, sample)
        if selected is None:
            return _PENDING
        for i, chosen in enumerate(selected):
            self._last_stamps[i] = chosen.timestamp_ns
            for old_key, old in tuple(self._queues[i].items()):
                if old is chosen:
                    del self._queues[i][old_key]
                elif self._ordered_timestamps and old.timestamp_ns <= chosen.timestamp_ns:
                    del self._queues[i][old_key]
                    self._dropped += 1
        self._last_received = min(s.received_at for s in selected)
        self._matches += 1
        return SynchronizationResult(True, selected)


class ExactTimeSynchronizer(TimeSynchronizer[T]):
    """Explicit exact-time spelling; same matching semantics as TimeSynchronizer."""
    policy = "exact"


class ApproximateTimeSynchronizer(TimeSynchronizer[T]):
    """Consume samples whose entire timestamp span is <= slop_sec.

    Two streams use nearest-neighbor matching. For N streams, scan bounded
    timestamp windows around the new sample, avoiding an exponential Cartesian
    product. The complete N-way span (not just each distance to the anchor) must
    satisfy slop. This implements an eager nearest-window policy, not the ROS C++
    adaptive ApproximateTime policy or a drop-in copy of every Python tie-break.
    """
    policy = "approximate"

    def __init__(self, stream_count: int, queue_size: int = 8, *,
                 slop_sec: float = 0.02, timeout_sec: float = 1.0):
        super().__init__(stream_count, queue_size, timeout_sec=timeout_sec)
        if not math.isfinite(slop_sec) or slop_sec <= 0 or timeout_sec <= slop_sec:
            raise ValueError("slop_sec must be finite, positive, and smaller than timeout_sec")
        self.slop_ns = int(slop_sec * 1e9)

    def _select(self, stream: int, sample: SynchronizedSample[T]):
        anchor = sample.timestamp_ns
        if self.stream_count == 2:
            other = self._queues[1 - stream]
            if not other:
                return None
            closest = min(other.values(), key=lambda s: abs(s.timestamp_ns - anchor))
            if abs(closest.timestamp_ns - anchor) > self.slop_ns:
                return None
            return (sample, closest) if stream == 0 else (closest, sample)

        candidates = [
            (sample,) if i == stream else tuple(
                s for s in queue.values() if abs(s.timestamp_ns - anchor) <= self.slop_ns
            ) for i, queue in enumerate(self._queues)
        ]
        if not all(candidates):
            return None
        left_edges = {s.timestamp_ns for group in candidates for s in group
                      if s.timestamp_ns <= anchor}
        best = best_score = None
        for left in sorted(left_edges):
            selected = []
            for group in candidates:
                options = [s for s in group if left <= s.timestamp_ns <= left + self.slop_ns]
                if not options:
                    break
                selected.append(min(options, key=lambda s: abs(s.timestamp_ns - anchor)))
            if len(selected) != self.stream_count:
                continue
            stamps = [s.timestamp_ns for s in selected]
            score = (max(stamps) - min(stamps), sum(abs(t - anchor) for t in stamps))
            if best_score is None or score < best_score:
                best, best_score = tuple(selected), score
        return best


class QueueSynchronizer(TimeSynchronizer[T]):
    """FIFO arrival-order grouping, NOT timestamp synchronization.

    One oldest unconsumed arrival per stream, even with different/repeated
    timestamps. Bounded queues and reception-age expiry still apply. Opt in only
    when this weaker guarantee is actually intended.
    """
    policy = "queue"
    _ordered_timestamps = False

    def _select(self, stream: int, sample: SynchronizedSample[T]):
        if not all(self._queues):
            return None
        return tuple(next(iter(q.values())) for q in self._queues)


__all__ = ["ApproximateTimeSynchronizer", "ExactTimeSynchronizer", "TimeSynchronizer",
           "QueueSynchronizer", "SynchronizedSample", "SynchronizationResult"]
