"""Mod-local offline tests; no models, hardware, or ROS nodes are started."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from bxi_example_py_elf3.framework.mod_api import JointLayout, JointTargetBuffer, MotorFrame
from bxi_example_py_elf3.framework.joints import (
    CompiledCommandBinding, JointCommandDefaults, JointDefault,
)
from bxi_example_py_elf3.policies.joints import ELF3_ISAAC_JOINTS, ELF3_POLICY_JOINTS


# This file's declared location is <this Mod>/tests/. No package/sibling guesses.
MOD_ROOT = Path(__file__).resolve().parent.parent
assert (MOD_ROOT / "mod.yaml").is_file()


def load_state(filename, class_name):
    spec = importlib.util.spec_from_file_location(
        f"basic_actions_test_{filename}", MOD_ROOT / f"{filename}.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, class_name)


NormalState = load_state("normal_state", "NormalState")
PdBrakeState = load_state("pd_brake_state", "PdBrakeState")


def make_state(*, reverse=False, extra_robot_names=()):
    names = ELF3_ISAAC_JOINTS.names
    layout = JointLayout(names[::-1] if reverse else names)
    # Deliberately different default-target order: fixed slices would be wrong.
    defaults = JointTargetBuffer(JointLayout(ELF3_POLICY_JOINTS.names[::-1]))
    defaults.position[:] = np.arange(29, dtype=np.float32) * 0.01
    defaults.kp[:] = np.arange(29, dtype=np.float32) + 10
    defaults.kd[:] = np.arange(29, dtype=np.float32) + 1
    output = JointTargetBuffer(layout)
    output.position[:] = np.arange(29, dtype=np.float32) + 100
    output.kp[:] = np.arange(29, dtype=np.float32) + 200
    output.kd[:] = np.arange(29, dtype=np.float32) + 300
    policy = SimpleNamespace(default_target=defaults.view,
                             output=SimpleNamespace(joints=output.view))

    def step(_frame, _dt, *, advance):
        if advance:
            output.position[:] += 1
        return policy.output

    policy.step = Mock(side_effect=step)
    handle = SimpleNamespace(get=lambda: policy)
    state = NormalState("com.bxi.basic_actions/normal", 1, handle)
    brake = PdBrakeState("com.bxi.basic_actions/pd_brake", 2, handle)
    ctx = SimpleNamespace(
        robot_layout=JointLayout(ELF3_POLICY_JOINTS.names[::-1] + extra_robot_names),
        inference_frame=object(), current_cmd_vel=np.zeros(3, dtype=np.float32),
        current_quat_xyzw=np.array([0, 0, 0, 1]),
        preheat_model=Mock(), is_orientation_unsafe=Mock(return_value=False),
        set_motor_target=Mock(), request_state=Mock(),
    )
    return state, brake, ctx, policy, output, defaults


def assert_commands(state, frame, policy):
    arms = set(state._ARM_JOINTS.names)
    assert len(arms) == 14
    for index, name in enumerate(frame.layout.names):
        source = policy.default_target if name in arms else policy.output.joints
        source_index = source.layout.index(name)
        for actual, expected in ((frame.qpos, source.position),
                                 (frame.kp, source.kp), (frame.kd, source.kd)):
            assert actual[index] == expected[source_index]
    np.testing.assert_array_equal(frame.vel, 0)
    np.testing.assert_array_equal(frame.torque, 0)


@pytest.mark.parametrize("reverse", [False, True])
def test_entry_and_running_frames_hold_exact_pd_brake_arm_commands(reverse):
    state, brake, ctx, policy, output, defaults = make_state(reverse=reverse)
    state.on_prepare(ctx, brake)
    ctx.preheat_model.assert_called_once()
    assert ctx.preheat_model.call_args.args[0] is policy
    snapshots = [value.copy() for value in (output.position, output.kp, output.kd,
                                           defaults.position, defaults.kp, defaults.kd)]
    frame = state.get_entry_frame(ctx)
    assert_commands(state, frame, policy)
    pd_frame = brake.get_entry_frame(ctx)
    for name in state._ARM_JOINTS.names:
        index, pd_index = frame.layout.index(name), pd_frame.layout.index(name)
        assert frame.qpos[index] == pd_frame.qpos[pd_index]
        assert frame.kp[index] == pd_frame.kp[pd_index]
        assert frame.kd[index] == pd_frame.kd[pd_index]
    for actual, expected in zip((output.position, output.kp, output.kd,
                                defaults.position, defaults.kp, defaults.kd), snapshots):
        np.testing.assert_array_equal(actual, expected)

    composer = state._composer
    # Even leftover velocity/torque in the reusable frame must not leak out.
    frame.vel.fill(42)
    frame.torque.fill(42)
    for advance in (False, False, True, True):
        running = state.sample_running_frame(ctx, 0.02, advance=advance)
        assert running is frame
        assert state._composer is composer
        assert policy.step.call_args.kwargs["advance"] is advance
        assert_commands(state, running, policy)
    state.on_update(ctx, 0.02)
    ctx.set_motor_target.assert_called_once_with(frame)
    assert_commands(state, frame, policy)


@pytest.mark.parametrize("extra_names", [
    (), ("head_z_joint", "head_y_joint"),
    ("head_z_joint", "head_y_joint", "aux_joint"),
])
def test_reordered_29_31_and_n_joint_robot_mapping_preserves_pd_arms(extra_names):
    state, _, ctx, policy, _, _ = make_state(extra_robot_names=extra_names)
    frame = state.get_entry_frame(ctx)
    defaults = JointCommandDefaults({
        name: JointDefault(position=-0.1, kp=5.0, kd=0.5) for name in extra_names
    })
    binding = CompiledCommandBinding(frame.layout, ctx.robot_layout, defaults)
    resolved = MotorFrame.empty(ctx.robot_layout)
    binding.resolve_into(frame, resolved)
    for i, name in enumerate(resolved.layout.names):
        if name in extra_names:
            assert resolved.qpos[i] == pytest.approx(-0.1)
            assert resolved.kp[i] == 5.0
            assert resolved.kd[i] == 0.5
        else:
            j = frame.layout.index(name)
            assert resolved.qpos[i] == frame.qpos[j]
            assert resolved.kp[i] == frame.kp[j]
            assert resolved.kd[i] == frame.kd[j]
    np.testing.assert_array_equal(resolved.vel, 0)
    np.testing.assert_array_equal(resolved.torque, 0)


def test_nan_arm_model_outputs_are_overridden_without_mutating_policy_buffers():
    state, _, ctx, policy, output, _ = make_state()
    indices = [output.layout.index(name) for name in state._ARM_JOINTS.names]
    output.position[indices] = np.nan
    output.kp[indices] = np.nan
    output.kd[indices] = np.nan
    frame = state.get_entry_frame(ctx)
    assert_commands(state, frame, policy)
    assert np.isfinite(frame.qpos).all()
    assert np.isfinite(frame.kp).all()
    assert np.isfinite(frame.kd).all()
    assert np.isnan(output.position[indices]).all()


def test_reenter_refreshes_pd_defaults_and_replaced_target_recompiles_mapping():
    state, brake, ctx, policy, _, defaults = make_state()
    state.on_prepare(ctx, brake)
    defaults.position[:] += 0.2
    defaults.kp[:] += 2
    state.on_prepare(ctx, brake)
    assert_commands(state, state.get_entry_frame(ctx), policy)
    replacement = JointTargetBuffer(JointLayout(ELF3_ISAAC_JOINTS.names[::-1]))
    replacement.position.fill(-2)
    replacement.kp.fill(8)
    replacement.kd.fill(1)
    policy.output.joints = replacement.view
    assert_commands(state, state.get_entry_frame(ctx), policy)


def test_orientation_safety_still_exits_without_inference_or_motor_command():
    state, _, ctx, policy, _, _ = make_state()
    ctx.is_orientation_unsafe.return_value = True
    state.on_update(ctx, 0.02)
    ctx.request_state.assert_called_once_with(
        "com.bxi.basic_actions/zero_torque", trigger="safety"
    )
    policy.step.assert_not_called()
    ctx.set_motor_target.assert_not_called()
