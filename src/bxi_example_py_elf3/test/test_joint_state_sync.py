"""Shared platform split-topic synchronization; no ROS nodes/hardware started."""
import importlib
from threading import Event, Lock
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from bxi_example_py_elf3.framework.joints import JointLayout
from bxi_example_py_elf3.framework.platform.joint_sync import SynchronizedJointStateSource


LOWER = JointLayout(("waist", "left_leg", "right_leg"))
UPPER = JointLayout(("left_arm", "right_arm"))


def source(**kwargs):
    return SynchronizedJointStateSource(LOWER, UPPER, **kwargs)


def update(src, group, stamp, now=1.0, value=1.0, reverse=False):
    names = src.groups[group].names
    positions = np.arange(len(names), dtype=float) + value
    velocities = positions * 2
    if reverse:
        names, positions, velocities = names[::-1], positions[::-1], velocities[::-1]
    return src.update(names, positions, velocities, timestamp_ns=stamp, received_at=now)


@pytest.mark.parametrize("first", [0, 1])
def test_complete_pair_only_reordered_groups_and_stable_output(first):
    src = source()
    assert update(src, first, 1_000_000_000, reverse=True) is None
    assert not src.ready
    with pytest.raises(RuntimeError, match="complete synchronized group set"):
        _ = src.view
    frame = update(src, 1 - first, 1_010_000_000, now=1.01, reverse=True)
    assert frame.layout.names == LOWER.names + UPPER.names
    np.testing.assert_array_equal(frame.position, [1, 2, 3, 1, 2])
    np.testing.assert_array_equal(frame.velocity, [2, 4, 6, 2, 4])
    saved = frame.position.copy()
    assert update(src, 0, 1_100_000_000, now=1.1, value=10) is None
    np.testing.assert_array_equal(frame.position, saved)  # no torn half-frame
    next_frame = update(src, 1, 1_100_000_000, now=1.1, value=20)
    assert next_frame is frame
    np.testing.assert_array_equal(frame.position, [10, 11, 12, 20, 21])


def test_no_stale_half_reuse_and_one_publisher_cannot_keep_source_fresh():
    src = source(timeout_sec=.2)
    update(src, 0, 1_000_000_000)
    update(src, 1, 1_000_000_000)
    assert update(src, 0, 1_010_000_000, now=1.01) is None
    assert src.diagnostics()["pairs"] == 1
    assert not src.is_fresh(1.21)
    with pytest.raises(RuntimeError, match="not an IMU failure"):
        src.require_fresh(1.21)
    # Even equal headers cannot match after excessive local receive delay.
    assert update(src, 1, 1_010_000_000, now=1.3) is None


def test_nearest_timestamp_pair_slop_and_nonmonotonic_arrival():
    src = source(slop_sec=.02)
    update(src, 0, 1_000_000_000)
    update(src, 0, 1_030_000_000)
    frame = update(src, 1, 1_025_000_000)
    assert frame.timestamp_ns == 1_025_000_000
    assert src.diagnostics()["pending"] == (0, 0)
    assert update(src, 0, 1_020_000_000) is None  # cannot roll output backward
    assert update(src, 0, 1_100_000_000) is None
    assert update(src, 1, 1_120_000_001) is None  # just beyond slop
    assert update(src, 1, 1_119_000_000) is not None


def test_queue_is_bounded_and_duplicate_stamps_do_not_refresh_liveness():
    src = source(queue_size=3)
    for i in range(20):
        update(src, 0, 1_000_000_000 + i)
    assert src.diagnostics(1)["pending"] == (3, 0)
    assert src.diagnostics(1)["dropped"] == 17
    update(src, 1, 1_000_000_019)
    assert update(src, 0, 1_000_000_019, now=10) is None
    assert update(src, 1, 1_000_000_019, now=10) is None
    assert not src.is_fresh(10)


@pytest.mark.parametrize("names,position,velocity", [
    (("waist", "waist", "right_leg"), [0]*3, [0]*3),
    (("waist",), [0], [0]),
    (("unknown",), [0], [0]),
    (LOWER.names, [0]*2, [0]*3),
    (LOWER.names, [0]*3, [0]*2),
    (LOWER.names, [[0]*3], [0]*3),
    (LOWER.names, [np.nan, 0, 0], [0]*3),
    (LOWER.names, [0]*3, [0, np.inf, 0]),
])
def test_malformed_frames_cannot_partially_modify_output_or_enqueue(names, position, velocity):
    src = source()
    update(src, 0, 1_000_000_000)
    update(src, 1, 1_000_000_000)
    old = src.view.position.copy()
    old_diag = src.diagnostics(1)
    with pytest.raises((ValueError, TypeError)):
        src.update(names, position, velocity, timestamp_ns=2_000_000_000, received_at=2)
    np.testing.assert_array_equal(src.view.position, old)
    assert src.diagnostics(1) == old_diag


def test_unstamped_driver_requires_explicit_receipt_mode():
    src = source()
    with pytest.raises(ValueError, match="nonzero header.stamp"):
        update(src, 0, 0)
    src = source(use_header_stamp=False)
    assert update(src, 0, 0, now=1) is None
    assert update(src, 1, 0, now=1.01) is not None
    assert src.diagnostics(1)["stamp_source"] == "receipt"


@pytest.mark.parametrize("slop,size,timeout", [(0, 8, 1), (float("nan"), 8, 1),
                                                 (.02, 0, 1), (.02, 65, 1),
                                                 (.02, True, 1), (.02, 8, .01)])
def test_invalid_configuration(slop, size, timeout):
    with pytest.raises(ValueError):
        source(slop_sec=slop, queue_size=size, timeout_sec=timeout)


@pytest.fixture
def demo():
    with patch("bxi_example_py_elf3.framework.platform.cpu_affinity.bootstrap_process_scheduling"):
        return importlib.import_module("bxi_example_py_elf3.bxi_example_demo")


def test_configured_groups_must_not_overlap():
    with pytest.raises(ValueError, match="duplicate names"):
        SynchronizedJointStateSource(LOWER, LOWER)


def platform(demo):
    node = SimpleNamespace(
        _joint_source=source(), _joint_received=False, joint_state_sync=True,
        _next_bad_joint_state_warning=0., _next_joint_startup_info=0.,
        lock_in=Lock(), get_logger=Mock(return_value=Mock()),
        _shutting_down=Event(), context=SimpleNamespace(ok=lambda: True),
        imu_required=False, _imu_received=False, step=0, robot_reset=Mock(return_value=True),
        topic_prefix="hardware/", _second_reset_at=0.,
    )
    node.update = MethodType(demo.BxiExample._update_joint_state, node)
    node.start = MethodType(demo.BxiExample.startup_step, node)
    return node


def test_startup_waits_through_discovery_and_for_a_fresh_complete_pair(demo):
    node = platform(demo)
    update(node._joint_source, 0, 1_000_000_000)
    assert not node.start(1)
    assert not node.start(18)  # no fixed discovery timeout / no unseen zeros
    node.robot_reset.assert_not_called()
    update(node._joint_source, 0, 18_000_000_000, now=18)
    update(node._joint_source, 1, 18_010_000_000, now=18.01)
    node._joint_received = True
    assert not node.start(18.01)
    node.robot_reset.assert_called_once_with(1, False)
    assert node.step == 1


def test_imu_startup_gate_is_not_bypassed_by_a_valid_pair(demo):
    node = platform(demo)
    update(node._joint_source, 0, 1_000_000_000)
    update(node._joint_source, 1, 1_000_000_000)
    node.imu_required = True
    node._imu_startup_started_at = None
    node._next_imu_startup_warning = 0.
    assert not node.start(1)
    node.robot_reset.assert_not_called()


def test_callback_accepts_half_frames_and_throttles_across_good_bad_alternation(demo):
    node = platform(demo)
    with patch("bxi_example_py_elf3.bxi_example_demo.time.monotonic", return_value=1):
        node.update(LOWER.names, [1]*3, [0]*3, timestamp_ns=1_000_000_000)
        assert not node._joint_received
        for i in range(20):
            node.update(("unknown",), [0], [0], timestamp_ns=1_000_000_000)
            node.update(LOWER.names, [1]*3, [0]*3, timestamp_ns=1_000_000_001+i)
        node.update(UPPER.names, [2]*2, [0]*2, timestamp_ns=1_000_000_020)
    assert node._joint_received
    node.get_logger().error.assert_called_once()
    assert node._joint_source.view.layout.dof_num == 5


def test_runtime_refuses_old_pair_before_using_or_publishing_it(demo):
    node = platform(demo)
    update(node._joint_source, 0, 1_000_000_000)
    update(node._joint_source, 1, 1_000_000_000)
    node._check_imu_runtime_safety = Mock()
    with patch("bxi_example_py_elf3.framework.platform.joint_sync.time.monotonic", return_value=3):
        with pytest.raises(RuntimeError, match="synchronization timeout"):
            demo.BxiExample.snapshot_control_inputs(node)


def test_queued_values_are_owned_and_buffers_reused():
    from bxi_example_py_elf3.framework.platform import SynchronizedJointStateSource as PublicSource
    assert PublicSource is SynchronizedJointStateSource
    src = source()
    buffers = tuple(id(a) for a in src._positions + src._velocities)
    positions = np.array([1., 2., 3.])
    src.update(LOWER.names, positions, positions, timestamp_ns=1_000_000_000, received_at=1)
    positions.fill(999)  # A driver may recycle its incoming message memory.
    update(src, 1, 1_000_000_000)
    np.testing.assert_array_equal(src.view.position[:3], [1, 2, 3])
    for i in range(1, 30):
        update(src, 0, 1_000_000_000 + i)
        update(src, 1, 1_000_000_000 + i)
    assert tuple(id(a) for a in src._positions + src._velocities) == buffers


@pytest.mark.parametrize("stopping,context_ok", [(True, True), (False, False)])
def test_shutdown_does_not_turn_old_pair_into_derived_sync_failure(demo, stopping, context_ok):
    from collections import deque
    node = platform(demo)
    update(node._joint_source, 0, 1_000_000_000)
    update(node._joint_source, 1, 1_000_000_000)
    node._joint_source.require_fresh = Mock(side_effect=AssertionError("unexpected watchdog"))
    node._check_imu_runtime_safety = Mock()
    if stopping:
        node._shutting_down.set()
    node.context.ok = lambda: context_ok
    node._joint_snapshot, node._observation = Mock(), object()
    node._imu_protection_latched = False
    node.pending_remote_events = deque()
    for dest, src, size in (("_quat_xyzw_snapshot", "quat_xyzw", 4),
                            ("_quat_wxyz_snapshot", "quat_wxyz", 4),
                            ("_omega_snapshot", "omega", 3),
                            ("_linear_acceleration_snapshot", "linear_acceleration", 3),
                            ("_cmd_snapshot", "raw_cmd_vel", 3)):
        setattr(node, dest, np.zeros(size))
        setattr(node, src, np.zeros(size))
    observation, events = demo.BxiExample.snapshot_control_inputs(node)
    assert observation is node._observation and events == ()
    node._joint_source.require_fresh.assert_not_called()


@pytest.mark.parametrize("policy", ["approximate", "exact", "time", "queue"])
def test_selectable_joint_sync_policy(policy):
    src = source(policy=policy)
    assert update(src, 0, 1_000_000_000) is None
    frame = update(src, 1, 1_001_000_000)
    if policy in {"exact", "time"}:
        assert frame is None
        frame = update(src, 1, 1_000_000_000, value=20)
    assert frame is not None
    assert src.diagnostics(1)["policy"] == policy
    np.testing.assert_array_equal(frame.position[:3], [1, 2, 3])


@pytest.mark.parametrize("policy", ["exact", "time"])
def test_exact_joint_policy_rejects_receipt_time_configuration(policy):
    with pytest.raises(ValueError, match="requires header stamps"):
        source(policy=policy, use_header_stamp=False)


def test_unknown_joint_sync_policy_fails_explicitly():
    with pytest.raises(ValueError, match="unknown joint sync policy"):
        source(policy="typo")


def test_rejected_add_cannot_overwrite_still_pending_numeric_slot():
    src = source(policy="exact", queue_size=2)
    update(src, 0, 100, value=1)
    update(src, 0, 101, value=2)
    # Ring cursor is at slot 0. A duplicate must not overwrite its pending data.
    assert update(src, 0, 101, value=999) is None
    frame = update(src, 1, 100)
    np.testing.assert_array_equal(frame.position[:3], [1, 2, 3])


@pytest.mark.parametrize("policy", ["approximate", "exact", "time", "queue"])
def test_full_frame_can_flush_pending_halves_for_every_policy(policy):
    src = source(policy=policy)
    update(src, 0, 100)
    names = src.layout.names
    frame = src.update(names, [3]*5, [4]*5, timestamp_ns=200, received_at=1)
    assert src.diagnostics(1)["pending"] == (0, 0)
    assert frame.timestamp_ns == 200
    assert update(src, 0, 300, value=10) is None
    frame = update(src, 1, 300, value=20)
    np.testing.assert_array_equal(frame.position, [10, 11, 12, 20, 21])


def test_fifo_joint_source_uses_queue_head_not_nearest_timestamp():
    src = source(policy="queue")
    update(src, 0, 100, value=1)
    update(src, 0, 200, value=2)
    frame = update(src, 1, 999, value=3)
    np.testing.assert_array_equal(frame.position, [1, 2, 3, 3, 4])
    frame = update(src, 1, 999, value=4)
    np.testing.assert_array_equal(frame.position, [2, 3, 4, 4, 5])


# Actual arm message names from the real-hardware startup log.
LEFT_ARM = JointLayout(tuple("l_" + n for n in (
    "shoulder_y_joint", "shoulder_x_joint", "shoulder_z_joint", "elbow_y_joint",
    "wrist_x_joint", "wrist_y_joint", "wrist_z_joint")), label="left arm")
RIGHT_ARM = JointLayout(tuple("r_" + n[2:] for n in LEFT_ARM.names), label="right arm")


def hardware_source(demo, **kwargs):
    lower = JointLayout(demo.ELF3_RESET_JOINTS.names[:15], label="lower")
    upper = JointLayout(LEFT_ARM.names + RIGHT_ARM.names, label="upper")
    groups = demo._elf3_feedback_groups(lower, upper)
    assert tuple(g.dof_num for g in groups) == (15, 7, 7)
    return SynchronizedJointStateSource(*groups, **kwargs)


@pytest.mark.parametrize("policy", ["approximate", "exact", "time", "queue"])
@pytest.mark.parametrize("order", [(0, 1, 2), (0, 2, 1), (1, 0, 2),
                                  (1, 2, 0), (2, 0, 1), (2, 1, 0)])
def test_real_15_7_7_names_any_arrival_order_and_policy(demo, policy, order):
    src = hardware_source(demo, policy=policy)
    buffers = tuple(id(a) for a in src._positions + src._velocities)
    for index, group in enumerate(order):
        frame = update(src, group, 1_000_000_000, value=10 * group, reverse=True)
        if index < 2:
            assert frame is None and not src.ready
    assert frame.layout.dof_num == 29
    np.testing.assert_array_equal(frame.position, np.r_[np.arange(15),
                                                       np.arange(7) + 10,
                                                       np.arange(7) + 20])
    assert src.diagnostics(1)["received"] == (1, 1, 1)
    assert src.diagnostics(1)["pairs"] == 1
    assert tuple(id(a) for a in src._positions + src._velocities) == buffers


@pytest.mark.parametrize("policy", ["approximate", "exact", "time", "queue"])
def test_combined_14_joint_frame_is_not_a_configured_group(demo, policy):
    src = hardware_source(demo, policy=policy)
    update(src, 0, 100)
    old_diag = src.diagnostics(1)
    with pytest.raises(ValueError, match="one complete configured group"):
        src.update(LEFT_ARM.names + RIGHT_ARM.names, [0] * 14, [0] * 14,
                   timestamp_ns=100, received_at=1)
    assert src.diagnostics(1) == old_diag
    assert not src.ready


@pytest.mark.parametrize("with_head", [False, True])
@pytest.mark.parametrize("policy", ["approximate", "exact", "time", "queue"])
def test_complete_simulation_29_or_31_frame_bypasses_multi_group_wait(demo, with_head, policy):
    names = demo.ELF3_RESET_JOINTS.names if with_head else demo.ELF3_RESET_JOINTS.names[:29]
    groups = demo._elf3_feedback_groups(JointLayout(names[:15]), JointLayout(names[15:]))
    src = SynchronizedJointStateSource(*groups, policy=policy)
    update(src, 1, 100)
    frame = src.update(names[::-1], np.arange(len(names)), np.zeros(len(names)),
                       timestamp_ns=200, received_at=1)
    assert frame.layout.names == names
    np.testing.assert_array_equal(frame.position, np.arange(len(names))[::-1])
    assert src.is_fresh(1) and src.diagnostics(1)["full_frames"] == 1
    assert src.diagnostics(1)["pending"] == (0,) * len(groups)
    assert update(src, 2, 100) is None  # Watermark blocks old split feedback.


def test_right_arm_cannot_stay_fresh_from_other_groups(demo):
    src = hardware_source(demo, timeout_sec=.2)
    for group in range(3):
        update(src, group, 1_000_000_000)
    saved = src.view.position.copy()
    for i in range(1, 10):
        now = 1 + i * .03
        for group in (0, 1):
            assert update(src, group, int(now * 1e9), now=now) is None
    assert not src.is_fresh(now)
    np.testing.assert_array_equal(src.view.position, saved)
    with pytest.raises(RuntimeError, match="synchronization timeout"):
        src.require_fresh(now)


def test_callback_startup_gate_waits_for_both_separate_arms(demo):
    node = platform(demo)
    node._joint_source = hardware_source(demo)
    with patch("bxi_example_py_elf3.bxi_example_demo.time.monotonic", return_value=1):
        for group in (0, 1):
            names = node._joint_source.groups[group].names
            node.update(names, [1] * len(names), [0] * len(names), timestamp_ns=1_000_000_000)
            assert not node._joint_received and not node.start(1)
            node.robot_reset.assert_not_called()
        node.update(RIGHT_ARM.names, [1] * 7, [0] * 7, timestamp_ns=1_000_000_000)
        assert node._joint_received
        assert not node.start(1)
    node.robot_reset.assert_called_once_with(1, False)
    node.get_logger().error.assert_not_called()


def test_configured_head_names_not_silently_dropped(demo):
    upper = JointLayout(LEFT_ARM.names + RIGHT_ARM.names + ("head_z_joint", "head_y_joint"))
    groups = demo._elf3_feedback_groups(LOWER, upper)
    assert tuple(g.dof_num for g in groups) == (3, 7, 7, 2)
    assert groups[-1].names == ("head_z_joint", "head_y_joint")


def test_generic_source_uses_only_caller_groups_not_robot_prefixes():
    groups = tuple(JointLayout(names) for names in (
        ("alpha", "beta"), ("gamma",), ("delta", "epsilon"), ("zeta",),
    ))
    src = SynchronizedJointStateSource(*groups)
    assert update(src, 0, 100, value=1, reverse=True) is None
    assert update(src, 1, 100, value=3) is None
    assert update(src, 2, 100, value=4, reverse=True) is None
    frame = update(src, 3, 100, value=6)
    np.testing.assert_array_equal(frame.position, [1, 2, 3, 4, 5, 6])
    assert src.diagnostics(1)["received"] == (1, 1, 1, 1)
    # A complete group plus a fragment of another must be rejected atomically.
    old_diag = src.diagnostics(1)
    with pytest.raises(ValueError, match="complete configured group"):
        src.update(("alpha", "beta", "delta"), [1] * 3, [0] * 3,
                   timestamp_ns=200, received_at=1)
    assert src.diagnostics(1) == old_diag


@pytest.mark.parametrize("groups", [(), (LOWER,)])
def test_generic_source_needs_at_least_two_caller_groups(groups):
    with pytest.raises(ValueError, match="stream_count"):
        SynchronizedJointStateSource(*groups)
