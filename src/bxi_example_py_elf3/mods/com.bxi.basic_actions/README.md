# 基础动作 Mod

本 Mod 提供 `normal`、`pd_brake`、`zero_torque` 等基础状态，状态、事件与切换配置见同目录 `mod.yaml`。

## normal：双臂固定 PD，腰腿保留策略控制

`normal` 不再把模型的 14 个双臂关节输出用于电机控制。左右各 7 个关节为：

- `shoulder_y_joint`、`shoulder_x_joint`、`shoulder_z_joint`；
- `elbow_y_joint`；
- `wrist_x_joint`、`wrist_y_joint`、`wrist_z_joint`。

以上名字分别加 `l_`、`r_` 前缀。位置目标、`kp` 和 `kd` 取自 normal 策略的 `default_target`，与 `pd_brake` 使用同一来源；这是固定默认姿态，不是进入 normal 时冻结当前角度，也不是零力矩。目标速度与前馈力矩明确置零。其余 15 个腰腿关节保持模型输出。

实现位于 `normal_state.py`，用按名称编译的 `JointCommandComposer` 和显式 `override=True` 的双臂 PD 层合成命令，不按模型数组或机器人消息中的固定下标截取。进入帧、运行帧和 `advance=False` 过渡采样均应用相同覆盖；不修改模型输出缓冲、推理历史或推进语义。索引和缓冲在 prepare 时准备，控制周期复用合成帧。

本改动仅影响 `normal`，不修改 `normal_run`、`amp_run`、其他动作或平台最终的外部命令 override。手动/导航速度输入、事件绑定、App 状态标识和现有姿态安全退出保持不变。29 自由度自然输出仍由平台具名映射到实际机器人布局；额外头部等关节继续由平台显式默认命令补齐。

注意：固定双臂改变了学习策略实际执行的动作，不能据此保证站立或步行稳定，尤其不能视为 ELF3 策略已适配 ELF4。先离线验证，再在仿真中分阶段验证；不要直接在真机切换试验。

## 离线回归测试

在已加载 ROS 与本控制包 Python 环境的终端执行：

```bash
python3 -m pytest -q /path/to/com.bxi.basic_actions/tests/test_normal_pd_arms.py
```

测试不启动硬件、ROS 节点或推理后端，覆盖：双臂与 `pd_brake` 的命令一致、腰腿输出保留、策略/默认目标乱序、29/31/N 机器人映射、五字段清零、`advance=False`、缓存复用及姿态安全退出。测试只定位自身 Mod 内的文件，安装或移动整个 Mod 后可原样执行。
