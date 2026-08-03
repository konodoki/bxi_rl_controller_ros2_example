# Livox-SDK2 prebuilt libraries

The SDK headers in `include/` are shared by all targets. Put each platform's
prebuilt SDK libraries in its own architecture directory:

```text
lib/
├── x86_64/
│   ├── liblivox_lidar_sdk_static.a
│   └── liblivox_lidar_sdk_shared.so
├── aarch64/
│   ├── liblivox_lidar_sdk_static.a
│   └── liblivox_lidar_sdk_shared.so
└── armv7l/
    ├── liblivox_lidar_sdk_static.a
    └── liblivox_lidar_sdk_shared.so
```

Only one of the static or shared files is required. The build prefers the
static library when both exist. The bundled files are x86-64 and therefore live
under `lib/x86_64/`.

CMake selects the directory from `CMAKE_SYSTEM_PROCESSOR`:

- `x86_64` or `amd64` -> `x86_64`
- `aarch64` or `arm64` -> `aarch64`
- `armv7`, `armv7l`, or `armhf` -> `armv7l`

For a cross-compilation toolchain whose processor name is unusual, override
the directory explicitly:

```bash
colcon build --packages-select livox_ros_driver2 \
  --cmake-args -DLIVOX_SDK_ARCH=aarch64
```

The build deliberately fails if the selected directory has no SDK library. It
never falls back to another architecture.
