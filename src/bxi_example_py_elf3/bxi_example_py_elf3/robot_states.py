import math
import os
import pickle
import time
from typing import TYPE_CHECKING, Any, Optional

import numpy as np
from rclpy.qos import QoSProfile, qos_profile_sensor_data
import sensor_msgs.msg

from ament_index_python.packages import get_package_share_path
from bxi_example_py_elf3.utils.robot_state_base import MotorFrame, RobotControlState
from bxi_example_py_elf3.utils.state_machine import StateBehavior, TransitionProfile
from bxi_example_py_elf3.utils.tfs import quaternion_to_euler_array

if TYPE_CHECKING:
    from bxi_example_py_elf3.bxi_example_demo import BxiExample
else:
    BxiExample = Any
class NormalState(RobotControlState):
    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.normal,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.normal.target_dof_pos, ctx.normal.kps, ctx.normal.kds
        )

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.normal.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )
        return self._motor_frame(qpos, ctx.normal.kps, ctx.normal.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)
            
class NormalDepthState(RobotControlState):
    depth_image_topic = "/camera/depth/image_64x36"
    depth_uint16_scale = 0.001
    depth_timeout_sec = 1.0

    def on_bind(self, ctx: BxiExample) -> None:
        self.depth_rotated: Optional[np.ndarray] = None
        self._last_depth_time: Optional[float] = None
        self._depth_enter_time = time.monotonic()
        self._missing_depth_warned = False
        self._bad_depth_warned = False
        self._depth_timeout_warned = False
        qos = QoSProfile(
            depth=1,
            durability=qos_profile_sensor_data.durability,
            reliability=qos_profile_sensor_data.reliability,
        )
        self.depth_sub = ctx.create_subscription(
            sensor_msgs.msg.Image,
            self.depth_image_topic,
            self.depth_image_callback,
            qos,
        )

    def depth_image_callback(self, msg: sensor_msgs.msg.Image) -> None:
        depth_meters = self._depth_msg_to_meters(msg)
        if depth_meters is None:
            return

        depth_rotated = np.rot90(depth_meters, k=-1)
        self.depth_rotated = np.ascontiguousarray(
            depth_rotated.astype(np.float32)
        )
        self._last_depth_time = time.monotonic()
        self._missing_depth_warned = False
        self._depth_timeout_warned = False

    def _depth_msg_to_meters(
        self, msg: sensor_msgs.msg.Image
    ) -> Optional[np.ndarray]:
        encoding = msg.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            dtype = np.dtype(np.uint16).newbyteorder(">" if msg.is_bigendian else "<")
            scale = self.depth_uint16_scale
        elif encoding == "32fc1":
            dtype = np.dtype(np.float32).newbyteorder(">" if msg.is_bigendian else "<")
            scale = 1.0
        else:
            if not self._bad_depth_warned:
                print(
                    f"unsupported depth image encoding '{msg.encoding}' "
                    f"from {self.depth_image_topic}"
                )
                self._bad_depth_warned = True
            return None

        itemsize = dtype.itemsize
        row_values = int(msg.step) // itemsize
        if msg.width <= 0 or msg.height <= 0 or row_values < msg.width:
            if not self._bad_depth_warned:
                print(
                    f"invalid depth image layout from {self.depth_image_topic}: "
                    f"width={msg.width}, height={msg.height}, step={msg.step}"
                )
                self._bad_depth_warned = True
            return None

        expected_values = row_values * int(msg.height)
        expected_bytes = expected_values * itemsize
        if len(msg.data) < expected_bytes:
            if not self._bad_depth_warned:
                print(
                    f"incomplete depth image from {self.depth_image_topic}: "
                    f"got {len(msg.data)} bytes, expected {expected_bytes} bytes"
                )
                self._bad_depth_warned = True
            return None

        data = np.frombuffer(msg.data, dtype=dtype, count=expected_values)
        depth = data.reshape(int(msg.height), row_values)[:, : int(msg.width)]
        return (depth.astype(np.float32) * scale).copy()

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        self._depth_enter_time = time.monotonic()
        self._depth_timeout_warned = False

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.normal_depth.target_dof_pos, ctx.normal_depth.kps, ctx.normal_depth.kds
        )

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        cmd_vel = self.get_cmd_vel(ctx)
        depth_rotated = self.depth_rotated
        if depth_rotated is None:
            if not self._missing_depth_warned:
                print(f"waiting for depth image: {self.depth_image_topic}")
                self._missing_depth_warned = True
            return None

        qpos = ctx.normal_depth.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
            depth_rotated,
        )
        return self._motor_frame(qpos, ctx.normal_depth.kps, ctx.normal_depth.kds)

    def _is_depth_timed_out(self) -> bool:
        now = time.monotonic()
        last_depth_time = self._last_depth_time
        if last_depth_time is None:
            elapsed = now - self._depth_enter_time
        else:
            elapsed = now - last_depth_time
        return elapsed > self.depth_timeout_sec

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        if self._is_depth_timed_out():
            if not self._depth_timeout_warned:
                print(f"depth image timeout, switch to normal: {self.depth_image_topic}")
                self._depth_timeout_warned = True
            ctx.request_state("normal", trigger="no depth")
            return

        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)


class ZeroTorqueState(RobotControlState):
    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.joint_nominal_pos,
            np.zeros(ctx.dof_num, dtype=np.float32),
            np.zeros(ctx.dof_num, dtype=np.float32),
        )


class PdBrakeState(RobotControlState):
    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.pd_pos, ctx.normal.kps, ctx.normal.kds)

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.pd_pos, ctx.normal.kps, ctx.normal.kds)


class InitialPosState(RobotControlState):
    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.initial_pos, ctx.joint_kp, ctx.joint_kd)

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        return self._motor_frame(ctx.initial_pos, ctx.joint_kp, ctx.joint_kd)


class DanceState(RobotControlState):
    def __init__(self, name: str, state_id: int, start_frame: int = 100):
        super().__init__(name, state_id)
        self.start_frame = start_frame
        self.playing = True

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.dance.timestep = self.start_frame
        if hasattr(ctx.dance, "timeinit"):
            ctx.dance.timeinit = 0.0
        ctx.preheat_model(ctx.dance)

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True
        ctx.dance.timestep = self.start_frame

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.dance.target_dof_pos,
            ctx.dance.kps,
            ctx.dance.kds,
        )

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        if ctx.dance.timestep >= ctx.dance.motionpos.shape[0]:
            return None

        qpos = ctx.dance.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )

        if self.playing:
            ctx.dance.timestep += 50 * dt  # 模型动画是50hz播放的，dt是推理间隔

        return self._motor_frame(
            qpos,
            ctx.dance.kps,
            ctx.dance.kds,
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.dance.timestep >= ctx.dance.motionpos.shape[0]:
            print("Motion replay finished, resetting simulation.")
            ctx.dance.timestep = self.start_frame
            ctx.request_state(
                "normal",
                trigger="motion_finished",
                transition={
                    "base": "dual_running_blend",
                    "duration": 0.5,
                    "data": {"run_from": False},
                },
            )
            return

        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class MotionState(RobotControlState):
    policy_attr = ""
    finish_trigger = "flip_finished"
    end_frame_trim = 0
    end_transition = {}

    def __init__(self, name: str, state_id: int):
        super().__init__(name, state_id)
        self.playing = True

    def _policy(self, ctx: BxiExample) -> Any:
        return getattr(ctx, self.policy_attr)

    def on_enter_transition(self, ctx, from_state, progress, transition):
        policy = self._policy(ctx)
        policy.timestep = policy.start_frame
        return super().on_enter_transition(ctx, from_state, progress, transition)

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        policy = self._policy(ctx)
        if hasattr(policy, "timeinit"):
            policy.timeinit = 0.0
        ctx.preheat_model(policy)

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True
        policy = self._policy(ctx)
        policy.timestep = policy.start_frame
        if hasattr(policy, "timeinit"):
            policy.timeinit = 0.0

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        policy = self._policy(ctx)
        qpos = getattr(policy, "target_dof_pos", None)
        if qpos is None:
            qpos = getattr(policy, "default_dof_pos", None)
        if qpos is None:
            return None
        return self._motor_frame(qpos, policy.kps, policy.kds)

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        policy = self._policy(ctx)

        qpos = policy.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )

        if self.playing and not on_translation:
            policy.timestep += 50 * dt  # 模型动画是50hz播放的，dt是推理间隔

        return self._motor_frame(qpos, policy.kps, policy.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        policy = self._policy(ctx)

        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)

        if policy.timestep > policy.end_frame - self.end_frame_trim:
            print("Motion replay finished, resetting simulation.")
            ctx.request_state(
                "normal", trigger=self.finish_trigger, transition=self.end_transition
            )


class BackFlipState(MotionState):
    policy_attr = "back_flip"
    finish_trigger = "back_flip_finished"
    end_frame_trim = 30
    end_transition = {
        "base": "dual_running_blend",
        "duration": 0.45,
        "data": {
            "curve": "linear",
            "run_from": True,
        },  # 过渡的时候模型继续推理，同时推理下一个模型
    }


class ForwardFlipState(MotionState):
    policy_attr = "forward_flip"
    finish_trigger = "forward_flip_finished"
    end_frame_trim = 125
    end_transition = {
        "base": "dual_running_blend",
        "duration": 1.0,
        "data": {
            "curve": "smootherstep",
            "run_from": True,
        },  # 过渡的时候模型继续推理，同时推理下一个模型
    }


class BalletState(MotionState):
    policy_attr = "ballet"
    finish_trigger = "ballet_finished"
    end_frame_trim = 330
    end_transition = {
        "base": "dual_running_blend",
        "duration": 1.0,
        "data": {
            "curve": "smootherstep",
            "run_from": True,
        },  # 过渡的时候模型继续推理，同时推理下一个模型
    }


class HandPlayBackState(RobotControlState):
    start_frame = 0
    tail_trim_frames = 0
    return_time = 0.5
    file_name = "applause.pkl"

    def __init__(self, name, state_id):
        super().__init__(name, state_id)
        self.frame = 0.0
        self.applause_data, self.fps = self._load_applause_data()

    def _load_applause_data(self) -> tuple[np.ndarray, float]:
        data_path = os.path.join(
            get_package_share_path("bxi_example_py_elf3"),
            "data",
            self.file_name,
        )
        with open(data_path, "rb") as data_file:
            data = pickle.load(data_file)

        dof_pos = np.asarray(data["dof_pos"], dtype=np.float32)[:, -14:]
        start = min(self.start_frame, dof_pos.shape[0])
        end = max(start, dof_pos.shape[0] - self.tail_trim_frames)
        applause_data = dof_pos[start:end]
        if applause_data.shape[0] == 0:
            raise ValueError(
                f"HandPlayBack data is empty after frame trim: {data_path}"
            )

        return applause_data, float(data["fps"])

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.withoutarm,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def on_enter(self, ctx: BxiExample) -> None:
        self.frame = 0.0
        self.playing = True

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        qpos = ctx.withoutarm.target_dof_pos.copy()
        qpos[-14:] = self.applause_data[0]
        return self._motor_frame(qpos, ctx.withoutarm.kps, ctx.withoutarm.kds)

    def get_motor_frame(self, ctx, dt, on_translation):
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.withoutarm.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )
        if self.frame < self.applause_data.shape[0]:
            qpos[-14:] = self.applause_data[int(self.frame)]
        else:
            qpos[-14:] = self.applause_data[-1]
        if self.playing and not on_translation:
            self.frame += self.fps * dt
        return self._motor_frame(qpos, ctx.withoutarm.kps, ctx.withoutarm.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            ctx.request_state("zero_torque", trigger="safety")
            return
        if self.frame >= self.applause_data.shape[0]:
            ctx.request_state(
                "normal",
                trigger="applause_finished",
                transition={
                    "base": "dual_running_blend",
                    "duration": 1.0,
                },
            )
            return
        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class ApplauseState(HandPlayBackState):
    start_frame = 600
    tail_trim_frames = 600
    file_name = "isaaclab_model/applause.pkl"


class HelloState(RobotControlState):
    def __init__(self, name, state_id):
        super().__init__(name, state_id)

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.withoutarm,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True
        self.shaketime = 0

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        qpos = ctx.withoutarm.target_dof_pos.copy()
        qpos[22] = -0.9
        qpos[24] = 0.0
        qpos[25] = -0.3
        return self._motor_frame(qpos, ctx.withoutarm.kps, ctx.withoutarm.kds)

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        if self.shaketime < 50:
            self.kp = self.shaketime / 50 * ctx.withoutarm.kps
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.withoutarm.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )
        qpos[22] = -0.9
        qpos[24] = math.sin(self.shaketime / 10) * 0.5
        qpos[25] = -0.3
        if self.playing:
            self.shaketime += 1
        return self._motor_frame(qpos, self.kp, ctx.withoutarm.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            ctx.request_state("zero_torque", trigger="safety")
            return
        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)

    def on_action(self, ctx: BxiExample, action_name: str) -> bool:
        if action_name != "toggle_dance_pause":
            return False

        self.playing = not self.playing
        return True


class RecoverState(RobotControlState):
    end_frame_trim = 0

    def __init__(self, name: str, state_id: int):
        super().__init__(name, state_id)
        self.playing = True
        self.motion_selected = False

    def on_enter_transition(self, ctx, from_state, progress, transition):
        ctx.recover.timestep = ctx.recover.start_frame
        return super().on_enter_transition(ctx, from_state, progress, transition)

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        if self._configure_recover_motion(ctx):
            ctx.preheat_model(ctx.recover)

    def on_enter(self, ctx: BxiExample) -> None:
        self.playing = True
        if not self._configure_recover_motion(ctx):
            ctx.request_state("zero_torque", trigger="recover_pose_rejected")

    def _configure_recover_motion(self, ctx: BxiExample) -> bool:
        eu_ang = quaternion_to_euler_array(ctx.quat_xyzw)
        eu_ang[eu_ang > math.pi] -= 2 * math.pi

        if eu_ang[1] < -(math.pi / 4.0):
            # 躺地上
            ctx.recover.end_frame = 880
            ctx.recover.timestep = 600
            ctx.recover.start_frame = 600
            self.end_frame_trim = 20
            self.motion_selected = True
            return True
        elif eu_ang[1] > (math.pi / 4.0):
            # 趴地上
            ctx.recover.end_frame = 1690
            ctx.recover.timestep = 1350
            ctx.recover.start_frame = 1350
            self.end_frame_trim = 0
            self.motion_selected = True
            return True

        self.motion_selected = False
        return False

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        if not self.motion_selected:
            return None
        return self._motor_frame(
            ctx.recover.target_dof_pos, ctx.recover.kps, ctx.recover.kds
        )

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        if ctx.recover.timestep > ctx.recover.end_frame:
            return None

        qpos = ctx.recover.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
        )

        if self.playing:
            ctx.recover.timestep += 50 * dt  # 模型动画是50hz播放的，dt是推理间隔
        return self._motor_frame(qpos, ctx.recover.kps, ctx.recover.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.recover.timestep > ctx.recover.end_frame - self.end_frame_trim:
            ctx.request_state(
                "normal",
                trigger="recover_finished",
                transition={
                    "base": "dual_running_blend",
                    "duration": 0.5,
                    "data": {"run_from": True},  #
                },
            )
            return

        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)


class AmpRunState(RobotControlState):
    def __init__(self, name: str, state_id: int):
        super().__init__(name, state_id)
        self.max_vel = 0.0
        self.pre_cmd_vel_run = np.array([0.0, 0.0, 0.0])
        self.cmd_vel_run = np.array([0.0, 0.0, 0.0])

    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.amp_run,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def on_enter(self, ctx: BxiExample) -> None:
        self.max_vel = 0.0
        self.pre_cmd_vel_run = np.array([0.0, 0.0, 0.0])
        self.cmd_vel_run = np.array([0.0, 0.0, 0.0])

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        return self._motor_frame(
            ctx.amp_run.target_dof_pos, ctx.amp_run.kps, ctx.amp_run.kds
        )

    def process_cmd_vel(
        self,
        ctx: BxiExample,
        cmd_vel: np.ndarray,
    ) -> Optional[np.ndarray]:
        self.cmd_vel_run[:2] = 0.98 * self.pre_cmd_vel_run[:2] + 0.02 * cmd_vel[:2]
        self.cmd_vel_run[2] = cmd_vel[2]
        self.pre_cmd_vel_run = self.cmd_vel_run.copy()
        return self.cmd_vel_run

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        cmd_vel = self.get_cmd_vel(ctx)
        qpos, vel = ctx.amp_run.inference_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_wxyz,
            ctx.current_omega,
            cmd_vel,
        )

        if vel[0] > self.max_vel:
            self.max_vel = vel[0]
        if ctx.loop_count >= 100 + int(0.3 / ctx.dt):
            print(self.max_vel)
            ctx.loop_count = int(0.3 / ctx.dt)
            self.max_vel = 0.0

        return self._motor_frame(qpos, ctx.amp_run.kps, ctx.amp_run.kds)

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)


class NormalRunState(RobotControlState):
    def on_prepare_enter(
        self,
        ctx: BxiExample,
        from_state: StateBehavior[BxiExample],
        transition: TransitionProfile,
    ) -> None:
        super().on_prepare_enter(ctx, from_state, transition)
        ctx.preheat_model(
            ctx.normal_run,
            with_cmd_vel=True,
            cmd_vel=self.get_cmd_vel(ctx),
        )

    def on_enter(self, ctx: BxiExample) -> None:
        if hasattr(ctx.normal_run, "action"):
            ctx.normal_run.action = np.zeros_like(ctx.normal_run.action)

    def get_first_frame(self, ctx: BxiExample) -> Optional[MotorFrame]:
        qpos = ctx.normal_run.default_joint_pos.copy()
        if hasattr(ctx.normal_run, "target_q"):
            qpos += ctx.normal_run.target_q
        return self._motor_frame(
            qpos,
            ctx.normal_run.joint_stiffness,
            ctx.normal_run.joint_damping,
        )

    def get_motor_frame(
        self, ctx: BxiExample, dt: float, on_translation: bool
    ) -> Optional[MotorFrame]:
        cmd_vel = self.get_cmd_vel(ctx)
        qpos = ctx.normal_run.infer_step(
            ctx.current_q,
            ctx.current_dq,
            ctx.current_quat_xyzw,
            ctx.current_omega,
            cmd_vel,
        )
        return self._motor_frame(
            qpos,
            ctx.normal_run.joint_stiffness,
            ctx.normal_run.joint_damping,
        )

    def on_update(self, ctx: BxiExample, dt: float) -> None:
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            print("check safe error, zero_torque!")
            ctx.request_state("zero_torque", trigger="safety")
            return

        frame = self.get_motor_frame(ctx, dt, False)
        if frame is not None:
            ctx.set_motor_target(*frame)
