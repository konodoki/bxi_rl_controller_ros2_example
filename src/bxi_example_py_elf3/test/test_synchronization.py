"""Public N-stream synchronization contracts; pure Python, no ROS/hardware."""
import itertools

import pytest

from bxi_example_py_elf3.framework.platform import (
    ApproximateTimeSynchronizer, ExactTimeSynchronizer, QueueSynchronizer,
    TimeSynchronizer, SynchronizationResult, SynchronizedSample,
)


TIME_POLICIES = (ApproximateTimeSynchronizer, ExactTimeSynchronizer, TimeSynchronizer)
ALL_POLICIES = TIME_POLICIES + (QueueSynchronizer,)


@pytest.mark.parametrize("cls", ALL_POLICIES)
@pytest.mark.parametrize("order", list(itertools.permutations(range(3))))
def test_three_streams_any_arrival_order_consume_once(cls, order):
    sync = cls(3)
    for index, stream in enumerate(order):
        result = sync.add(stream, 1_000_000_000, f"stream-{stream}", received_at=1)
        assert isinstance(result, SynchronizationResult) and result.accepted
        if index < 2:
            assert result.samples is None
    assert tuple(s.value for s in result.samples) == ("stream-0", "stream-1", "stream-2")
    assert all(isinstance(s, SynchronizedSample) for s in result.samples)
    assert sync.diagnostics()["pending"] == (0, 0, 0)
    assert sync.add(0, 1_100_000_000, "next", received_at=1.1).samples is None
    assert sync.diagnostics()["matches"] == 1


@pytest.mark.parametrize("cls", (ExactTimeSynchronizer, TimeSynchronizer))
def test_exact_integer_ns_no_nearby_match_and_queued_common_timestamp(cls):
    sync = cls(2)
    assert sync.add(0, 10, "a", received_at=1).samples is None
    assert sync.add(1, 11, "b", received_at=1).samples is None
    result = sync.add(1, 10, "b-exact", received_at=1)
    assert tuple(s.value for s in result.samples) == ("a", "b-exact")
    assert sync.add(0, 11, "a-next", received_at=1).samples is not None
    assert sync.last_timestamps == (11, 11)


def test_two_stream_approximate_nearest_and_inclusive_boundary():
    sync = ApproximateTimeSynchronizer(2, slop_sec=.01)
    sync.add(0, 1_000_000_000, "old", received_at=1)
    sync.add(0, 1_020_000_000, "nearest", received_at=1)
    result = sync.add(1, 1_019_000_000, "upper", received_at=1)
    assert result.samples[0].value == "nearest"
    assert sync.add(0, 2_000_000_000, "a", received_at=2).samples is None
    assert sync.add(1, 2_010_000_001, "outside", received_at=2).samples is None
    result = sync.add(1, 2_010_000_000, "boundary", received_at=2)
    assert result.samples[1].value == "boundary"


def test_n_way_approximate_checks_total_span_not_only_anchor_distances():
    sync = ApproximateTimeSynchronizer(3, slop_sec=.01)
    sync.add(0, 1_000_000_000, "left", received_at=1)
    sync.add(1, 1_020_000_000, "right", received_at=1)
    # Each side is 10 ms from the anchor, but the whole set spans 20 ms.
    assert sync.add(2, 1_010_000_000, "anchor", received_at=1).samples is None
    result = sync.add(0, 1_010_000_000, "closer-left", received_at=1)
    assert tuple(s.value for s in result.samples) == ("closer-left", "right", "anchor")
    stamps = [s.timestamp_ns for s in result.samples]
    assert max(stamps) - min(stamps) == 10_000_000


def test_n_way_window_search_can_use_second_nearest_to_find_valid_set():
    sync = ApproximateTimeSynchronizer(3, slop_sec=.01)
    sync.add(0, 94_000_000, "individually-closest", received_at=1)
    sync.add(0, 107_000_000, "valid-alternative", received_at=1)
    sync.add(1, 108_000_000, "right", received_at=1)
    result = sync.add(2, 100_000_000, "anchor", received_at=1)
    assert tuple(s.value for s in result.samples) == ("valid-alternative", "right", "anchor")


def test_fifo_is_arrival_order_not_exact_or_nearest_timestamp():
    sync = QueueSynchronizer(2)
    sync.add(0, 100, "first-arrival", received_at=1)
    sync.add(0, 50, "second-arrival-older-stamp", received_at=1)
    result = sync.add(1, 50, "first-upper", received_at=1)
    assert tuple(s.value for s in result.samples) == ("first-arrival", "first-upper")
    result = sync.add(1, 50, "repeated-stamp-is-new-arrival", received_at=1)
    assert result.samples[0].value == "second-arrival-older-stamp"
    assert result.samples[1].value == "repeated-stamp-is-new-arrival"
    assert sync.diagnostics()["pending"] == (0, 0)


@pytest.mark.parametrize("cls", ALL_POLICIES)
def test_overflow_and_receive_expiry_are_bounded_even_if_header_is_equal(cls):
    sync = cls(2, queue_size=2, timeout_sec=.2)
    for i in range(10):
        assert sync.add(0, 100+i, i, received_at=1).accepted
    assert sync.diagnostics()["pending"] == (2, 0)
    assert sync.diagnostics()["dropped"] == 8
    assert sync.add(1, 109, "late", received_at=1.21).samples is None
    assert sync.diagnostics()["pending"] == (0, 1)
    assert not sync.is_fresh(1.21)


@pytest.mark.parametrize("cls", TIME_POLICIES)
def test_duplicates_out_of_order_watermarks_and_explicit_reset(cls):
    sync = cls(2)
    assert sync.add(0, 100, "a", received_at=1).accepted
    assert not sync.add(0, 100, "duplicate", received_at=1).accepted
    assert sync.add(1, 100, "b", received_at=1).samples is not None
    assert not sync.add(0, 99, "late", received_at=1).accepted
    assert sync.is_fresh(1.1)
    assert not sync.is_fresh(2)
    sync.clear(watermark_ns=200)
    assert sync.diagnostics()["pending"] == (0, 0)
    assert not sync.is_fresh(1)
    assert not sync.add(0, 199, "pre-full-frame", received_at=1).accepted
    with pytest.raises(ValueError, match="backward"):
        sync.clear(watermark_ns=100)
    sync.clear()
    assert sync.add(0, 1, "new-clock", received_at=1).accepted


@pytest.mark.parametrize("cls", ALL_POLICIES)
def test_preallocated_slot_keys_can_be_reused_without_keeping_old_samples(cls):
    sync = cls(2, queue_size=2)
    sync.add(0, 100, "buffer-slot-0-old", key=0, received_at=1)
    sync.add(0, 101, "buffer-slot-1", key=1, received_at=1)
    sync.add(0, 102, "buffer-slot-0-new", key=0, received_at=1)
    assert sync.diagnostics()["pending"] == (2, 0)
    # Force selection of the new slot for time strategies; FIFO selects slot 1.
    result = sync.add(1, 102, "upper", key=0, received_at=1)
    assert result.samples[0].value != "buffer-slot-0-old"
    assert sync.diagnostics()["dropped"] >= 1


@pytest.mark.parametrize("cls", ALL_POLICIES)
@pytest.mark.parametrize("stream,stamp,now,key", [
    (-1, 1, 1., None), (2, 1, 1., None), (True, 1, 1., None),
    (0, -1, 1., None), (0, 1.2, 1., None), (0, True, 1., None),
    (0, 1, float("nan"), None), (0, 1, float("inf"), None),
    (0, 1, 1., []),
])
def test_invalid_add_is_atomic(cls, stream, stamp, now, key):
    sync = cls(2)
    old = sync.diagnostics()
    with pytest.raises((ValueError, TypeError)):
        sync.add(stream, stamp, object(), received_at=now, key=key)
    assert sync.diagnostics() == old


@pytest.mark.parametrize("cls", ALL_POLICIES)
@pytest.mark.parametrize("count,size,timeout", [
    (1, 8, 1), (True, 8, 1), (2, 0, 1), (2, 65, 1),
    (2, 1.2, 1), (2, True, 1), (2, 8, 0), (2, 8, float("nan")),
])
def test_invalid_configuration(cls, count, size, timeout):
    with pytest.raises(ValueError):
        cls(count, queue_size=size, timeout_sec=timeout)


@pytest.mark.parametrize("slop", [0, -1, 1, float("nan"), float("inf")])
def test_invalid_approximate_slop(slop):
    with pytest.raises(ValueError):
        ApproximateTimeSynchronizer(2, slop_sec=slop, timeout_sec=1)


@pytest.mark.parametrize("streams", [3, 4])
def test_n_way_window_search_matches_exhaustive_feasibility(streams):
    import random
    rng = random.Random(1701)
    for _ in range(100):
        sync = ApproximateTimeSynchronizer(streams, slop_sec=10e-9)
        queues = [rng.sample(range(80, 121), 3) for _ in range(streams - 1)]
        anchor = rng.randrange(80, 121)
        for i, stamps in enumerate(queues):
            for stamp in stamps:
                sync.add(i, stamp, stamp, received_at=1)
        expected = any(max(values + (anchor,)) - min(values + (anchor,)) <= 10
                       for values in itertools.product(*queues))
        result = sync.add(streams - 1, anchor, anchor, received_at=1)
        assert (result.samples is not None) == expected
        if expected:
            stamps = [s.timestamp_ns for s in result.samples]
            assert max(stamps) - min(stamps) <= 10
