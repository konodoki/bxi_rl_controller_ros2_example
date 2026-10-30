import atexit
import fcntl
import os
import sys
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from bxi_example_py_elf3.framework.mod_api.hardware_launch import (
    declare_hardware_launch_arguments,
    hardware_node_from_context,
)

LOCK_FILE = "/tmp/bxi_example_hw.lock"
_lock_fd = None


def _release_lock():
    global _lock_fd
    if _lock_fd is None:
        return
    fd = _lock_fd
    _lock_fd = None
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        os.close(fd)
    except OSError:
        pass


def _acquire_lock():
    global _lock_fd
    if _lock_fd is not None:
        return

    # Create the lock file world-writable so a different user (or root)
    # can reuse the same system-wide lock. Force umask to 0 so the 0o666
    # mode is honored on creation regardless of the caller's umask.
    old_umask = os.umask(0)
    try:
        try:
            fd = os.open(LOCK_FILE, os.O_RDWR | os.O_CREAT, 0o666)
        except PermissionError as e:
            print(
                f"\n[ERROR] Cannot open lock file {LOCK_FILE}: {e}\n"
                f"        A stale lock file with restrictive permissions exists.\n"
                f"        Remove it and retry:  sudo rm {LOCK_FILE}\n",
                file=sys.stderr,
            )
            sys.exit(1)
    finally:
        os.umask(old_umask)

    # Repair perms on a file left behind by an older version of this
    # script (no-op unless we are the owner or root).
    try:
        os.chmod(LOCK_FILE, 0o666)
    except OSError:
        pass

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        try:
            os.lseek(fd, 0, os.SEEK_SET)
            holder = (
                os.read(fd, 64).decode("utf-8", errors="replace").strip() or "unknown"
            )
        except OSError:
            holder = "unknown"
        os.close(fd)
        print(
            f"\n[ERROR] bxi_example_hw is already running (pid={holder})! "
            f"Please stop the existing instance before starting a new one.\n",
            file=sys.stderr,
        )
        sys.exit(1)

    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode("utf-8"))
    _lock_fd = fd
    atexit.register(_release_lock)

def generate_launch_description():
    _acquire_lock()

    state_machine_config = os.path.join(
        get_package_share_path("bxi_example_py_elf3"),
        "config/elf3_state_machine.yaml",
    )
    cameras_launch = os.path.join(
        get_package_share_path("bxi_depth_camera"),
        "launch/cameras.launch.py",
    )
    dual_robot_launch = os.path.join(
        get_package_share_path("hardware_kaixuan_arms"),
        "launch/dual_robot.launch.py",
    )
    imu_launch = os.path.join(
        get_package_share_path("bxi_imu"),
        "launch/imu.launch.py",
    )

    return LaunchDescription(
        declare_hardware_launch_arguments()
        + [
            DeclareLaunchArgument(
                "imu_runtime_timeout_sec",
                default_value="1.0",
                description="Seconds without IMU data before locked protection",
            ),
            DeclareLaunchArgument(
                "imu_reader_priority",
                default_value="20",
                description="Scheduling priority for the IMU reader",
            ),
            DeclareLaunchArgument("joint_state_sync", default_value="true",
                                  description="Synchronize split lower/upper feedback"),
            DeclareLaunchArgument("joint_sync_policy", default_value="approximate",
                                  choices=["approximate", "exact", "time", "queue"],
                                  description="TimeSynchronizer is exact; queue is explicit FIFO, not time sync"),
            DeclareLaunchArgument("joint_sync_slop_sec", default_value="0.02",
                                  description="Maximum lower/upper timestamp difference"),
            DeclareLaunchArgument("joint_sync_queue_size", default_value="8",
                                  description="Bounded samples per joint group (1-64)"),
            DeclareLaunchArgument("joint_sync_timeout_sec", default_value="1.0",
                                  description="Maximum receive age of a complete pair"),
            DeclareLaunchArgument("joint_sync_use_header_stamp", default_value="true",
                                  description="Use shared-clock header stamps; false uses receipt time only"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cameras_launch),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(dual_robot_launch),
                launch_arguments={
                    "arms": "true",
                    "head": "false",
                    # 如需 hardware_elf3 内部也启用 IMU 参数，再加 "imu": "true"
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_launch),
                launch_arguments={
                    "imu_reader_priority": LaunchConfiguration("imu_reader_priority"),
                }.items(),
            ),
            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_demo",
                output="screen",
                parameters=[
                    {"/topic_prefix": "hardware/"},
                    # Both split drivers publish on hardware/actuator_states.
                    {
                        "/" + name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                        for name, value_type in (
                            ("joint_state_sync", bool),
                            ("joint_sync_policy", str),
                            ("joint_sync_slop_sec", float),
                            ("joint_sync_queue_size", int),
                            ("joint_sync_timeout_sec", float),
                            ("joint_sync_use_header_stamp", bool),
                        )
                    },
                    {"/state_machine_config": state_machine_config},
                    {
                        "/imu_runtime_timeout_sec": LaunchConfiguration(
                            "imu_runtime_timeout_sec"
                        )
                    },
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),
        ]
    )