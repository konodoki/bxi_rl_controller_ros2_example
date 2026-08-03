# livox_ros_driver2 (ROS 2 only)

This package publishes live Livox-SDK2 devices and contains no ROS 1 or LVX
playback path.

## Automatic device connection

`auto_connect` defaults to `true`. Livox devices discovered by the SDK are
accepted even when their IP address is not present in `lidar_configs`. The
driver records the device IP/handle, requests Normal work mode, enables IMU
output, and starts publishing as soon as the device reports success or sends
data.

Entries in `lidar_configs` are per-device overrides rather than a whitelist.
Their data type, scan pattern, blind spot, dual-return and extrinsic settings
are applied when the corresponding IP is discovered. An unlisted device keeps
its current settings.

The selected JSON file is still required as the Livox-SDK2 network template.
Its host IP, ports and product sections must match the local network interface
and the LiDAR families that should be discovered. Automatic connection cannot
discover devices across an unreachable subnet.

Set `auto_connect` to `false` to accept only IP addresses listed in
`lidar_configs`.

The driver keeps a stable slot for each IP, up to 32 devices. If a device stops
sending data for `device_timeout_sec`, it is treated as offline. Reconnection
with the same IP reuses its slot.

Per-device topics use the same logical-name fallback as `bxi_depth_camera`.
With one connected, unmapped LiDAR, `single_lidar_name` is used:

```text
/hardware/body_lidar/lidar
/hardware/body_lidar/imu
```

With two or more connected, unmapped LiDARs, the stable hardware serial is
used instead:

```text
/hardware/SN_3GGDJ5C0012345/lidar
/hardware/SN_3GGDJ5C0012345/imu
```

An explicit serial mapping always wins. Configure mappings in a copied ROS
parameter file:

```yaml
/livox_lidar_publisher:
  ros__parameters:
    lidar_serial_mappings:
      - "3GGDJ5C0012345=body_lidar"
      - "3GGDJ5C0098765=head_lidar"
```

Then load it at startup:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  config_file:=/path/to/robot_livox.yaml
```

`multi_topic = 1` enables this per-device layout and is now the default.
`multi_topic = 0` retains the shared legacy topics `livox/lidar` and
`livox/imu`.

## Parameters

- `xfer_format`: `0` for `sensor_msgs/msg/PointCloud2`, `1` for `CustomMsg`;
- `multi_topic`: `0` for shared topics, `1` for one topic per LiDAR;
- `publish_freq`: publishing frequency from 0.5 to 100 Hz;
- `auto_connect`: automatically accept unlisted discovered devices;
- `device_timeout_sec`: time without data before a device is considered offline;
- `topic_namespace`: per-device topic namespace, default `hardware`;
- `single_lidar_name`: logical name used for one unmapped LiDAR;
- `lidar_serial_mappings`: `SERIAL=logical_name` string array;
- `frame_id`: ROS frame ID;
- `user_config_path`: Livox-SDK2 JSON network configuration.

See [Livox-SDK2/README.md](Livox-SDK2/README.md) for the architecture-specific
prebuilt SDK library layout.
