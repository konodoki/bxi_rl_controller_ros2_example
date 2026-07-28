# 框架控制调度

`RobotControlRuntime` 使用独立的绝对时间线程，以固定 50 Hz 时间轴调用
`RobotControlFramework.update()`。它同时拥有 Framework 生命周期、低频 Mod
维护、线程安全、调度统计和逐条超时告警。ROS 平台适配器只负责输入快照、启动步骤和
电机帧输出，不再通过 ROS Timer 驱动动作或维护 Framework。

配置位于 `config/elf3_state_machine.yaml` 的 `control_runtime` 分段：

```yaml
control_runtime:
  period_sec: 0.02
  compute_budget_sec: 0.002
  deadline_tolerance_sec: 0.0
  maintenance_hz: 5.0
  statistics_interval_sec: 60.0
  maintenance_guard_sec: 0.005
  python_switch_interval_sec: 0.001
  cpu_affinity: -1
  realtime_priority: 0
```

- `period_sec`：统一控制周期，所有状态和过渡共用。
- `compute_budget_sec`：在截止点前多久唤醒控制线程。默认提前 2 ms。
- `deadline_tolerance_sec`：完成时间超过截止点后的额外容差。
- `cpu_affinity`：`-1` 表示不绑核，非负值表示控制线程绑定的 CPU。
- `realtime_priority`：`0` 表示普通调度，`1..99` 尝试启用
  `SCHED_FIFO`；权限不足时记录警告并继续使用普通调度。
- `maintenance_hz`：Runtime 自有低频线程的 Mod 进程监管频率，不进入 50 Hz
  控制路径。
- `statistics_interval_sec`：周期性 INFO 调度统计的输出间隔。
- `maintenance_guard_sec`：距离下一次控制唤醒不足该时间时，跳过本轮
  状态快照或 Mod 维护，避免反向阻塞控制线程。
- `python_switch_interval_sec`：Python GIL 切换间隔。默认 1 ms，降低
  ROS Python 回调造成的控制线程唤醒长尾。

`statistics_interval_sec` 只控制 INFO 统计摘要的输出周期。摘要包含唤醒延迟、
完整控制周期耗时，以及自上次成功输出以来的控制周期、deadline miss 和跳过周期增量。

每次 deadline miss 都由控制线程轻量放入队列，再由现有 maintenance 线程逐条输出
WARNING；不等待统计周期、不合并、不限频，也不在 50 Hz 控制线程内执行日志 I/O。
默认 maintenance 频率为 5 Hz，因此 WARNING 最多延迟约 200 ms。一次输出失败会保留
该事件，并在后续 maintenance 周期重试。deadline miss 不自动切换状态。状态信息话题的
`control_timing` 字段提供累计计数和最后一次 miss。

Runtime 和 Scheduler 的 INFO、WARNING、ERROR 使用彼此独立的 Python 调用位置，
兼容 `rclpy` 对同一日志调用位置固定 severity 的要求；进入高负载状态后从 INFO
切换为 WARNING 不会再被 ROS 日志系统拒绝。

`control_runtime` 在构造 `RobotControlFramework` 前从系统配置中移除，因此不会
参与状态图或 Mod 配置合并。仿真、真机以及未来的非 ROS 平台适配器共用同一运行时。

`bxi_example_py_elf3_mjlab` 保持原有调度，不受这些参数影响。
