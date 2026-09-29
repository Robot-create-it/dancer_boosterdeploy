# Recovery 堵转后的调试流程

目标是找到首次起身中最早偏离正常轨迹的环节，而不是反复试到机器人站起来。
上次首次堵转在执行约 3.15 秒时发生，重点分析此前的手臂跟踪与机身抬升。

## 1. 恢复后的确认与输出核实

先确认故障关节的实际状态已恢复（最近一次为左肩2、右肩6、右肘9，早先还有左肘5），厂家诊断没有持续故障，
反馈随实际姿态变化；上次 ROS 的错误位、温度全零，不能作为正常的依据。
不要靠“模式能切 Prepare”判断电机已经健康。

可以先执行只读检查（不发布关节命令、不切模式）：

```bash
python scripts/check_k1_recovery_state.py --output logs/recovery_state_after_reset_01.json
```

它同时对比serial与parallel反馈，列出持续零q/dq/tau且parallel温度也为0的可疑臂关节。
空列表不等于已经证明健康，必须结合厂家故障诊断。
本机parallel温度能提供更多信息，但其reserve错误位同样不能依赖。

重新带负载前，需要核实 Custom 下 q/kp/kd 的底层力矩限制和可靠故障信号。
现有 Python 仿真手臂力矩会裁剪到配置的 14 Nm，硬件路径不下发这个上限。
不能假定二者等价，也不能把 50 Hz 调整目标角的近似限幅当作底层硬限流。
故障联锁需使用本机有效信号，不能只读上次始终为零的 reserve[0]。
当前代码保留连续保持和反馈联锁，已按用户要求移除手臂
`q_measured ± torque_limit/kp`目标角限幅；驱动保护阈值未改变，
移除后的实机验收尚未完成。验证步骤见[k1_recovery_hardware_validation.md](k1_recovery_hardware_validation.md)。

## 2. 先验证记录功能，再进行有支撑的单次测试

仿真可以先验证日志与分析链路：

```bash
python scripts/deploy.py --task k1_recovery --mujoco --recovery-posture facedown \
  --recovery-log logs/recovery_sim_trial01.jsonl
python scripts/analyze_k1_recovery.py logs/recovery_sim_trial01.jsonl
```

完成第 1 步并由现场人员布置防跌落/减载支撑后，才进入实机单次测试。
支撑应保留双手、双脚必要接触，不能把机器人完全悬空后把闭环策略表现当作正常起身。
测试从自然俯卧开始，同步拍摄侧面视频，观察撑地、屈髋屈膝、重心抬升的先后。
首轮不改模型、观测、kp/kd、phase、轨迹速度或动作幅度。

下面命令会控制实机，不是只读采样命令：

```bash
python scripts/deploy.py --task k1_recovery \
  --recovery-log logs/recovery_real_trial01.jsonl
```

X/x 进入当前姿态保持，A/r 开始。实机入口固定零次自动重试；
失败超时或 Ctrl+C 会进入 damping，现场支撑要能承受失去主动支撑的情况。
**零次重试不能在首次堵转瞬间自动停止，也不能把等待超时作为堵转处理。**
出现反馈停滞、撑地关节明显顶死或任何电机故障时，应按现场急停流程立即终止，
不为完成整段轨迹而继续运行。

另一个终端记录固件日志，用 wall_time 对齐控制日志；此命令只读：

```bash
journalctl -f -n 0 -t booster-daemon -o short-iso-precise \
  | tee logs/recovery_firmware_trial01.log
```

## 3. 每轮分析，确定下一项改动

```bash
python scripts/analyze_k1_recovery.py logs/recovery_real_trial01.jsonl
```

日志每个策略周期记录：实测 q/dq/tau、gravity/gyro、轨迹行与 phase、
100 维观测、轨迹 q、模型 residual、拟发送目标、PD 力矩估计和状态。
`target_proposed`保存策略输出（已施加绝对关节角边界，未施加相对实测角的手臂限幅）。
实机增加`type=command`记录，保存发布前最新q/dq、最终`target_published`和state_age。
分析器的`published_command_frames`应大于0；
`published_arm_p_over_nominal_joint_samples`统计已发布比例项超过原14Nm名义值的关节采样数。
没有command帧时，该统计为0不能说明目标角安全或已执行。
首行包含实际控制器名称、增益、限位、模型 SHA-256 与配置的 effort_limit。
文件逐行刷新，已有文件拒绝覆盖。

`target_proposed` 是策略产物，不是电机接收确认；
`pd_estimate=kp*(target-q)-kd*dq` 是计算值，不是实测力矩或底层硬限流。
分析输出的 estimated_over_limit_frames 只表示该估计超过仿真配置阈值，
不能单独证明实机超力矩或堵转。

| 观察结果 | 下一步核查 |
|---|---|
| target 在动，q 几乎不动，误差和 tau 上升 | 先查接触/机械阻挡、撑地受力、执行端限幅，避免直接增加 kp |
| target 在动，q 和 tau 都不响应或突然全零 | 查电机保护、实际反馈是否有效及指令是否被执行 |
| q 能跟踪，机身 gravity 却不向竖直变化 | 查手脚接触、重心转移、轨迹与真实初始姿态，结合视频 |
| residual 在失败前突变或持续很大 | 离线重放该帧观测，核对模型、相位、previous action 和传感器语义 |
| 某一关节先出现大误差，随后其他关节也失败 | 优先分析最早异常关节，避免把后续保护的零反馈当作最初原因 |

单次失败后停止并分析，不连续重新按 A/r。只在找到可检验的假设后改一个因素，
记录该轮配置，再按同样的初始姿态和支撑条件对比。
没有逐帧证据前，不默认把“增大 kp”“延长起身时间”或“关闭故障保护”作为解决方案。

## 4. 持续记录与实时查看（不自动执行运动）

底层反馈和实际发布命令可以在起身前开始记录，持续覆盖X之前、等待A、执行、退出后：

```bash
ros2 bag record -o logs/recovery_bus_trial03 /low_state /joint_ctrl
```

在已加载ROS及Booster消息环境的终端执行。停止记录按Ctrl+C，再用
`ros2 bag info logs/recovery_bus_trial03`确认话题和消息数量。
没有控制器发布时，joint_ctrl没有消息是正常的；low_state应有消息。
此bag只记录，不发布运动命令；不要在连接实机的ROS域直接播放带joint_ctrl的bag。
ROS消息到达不代表每个电机反馈有效，joint_ctrl发布也不等于电机执行确认。
录制可能丢包，消息数量和录制端警告仍需核对。

策略日志需要deploy命令带`--recovery-log`。现有JSONL从策略初始化后开始记录，
按策略周期（目标50Hz）逐行刷新，不能覆盖X到A之前，也不能回补过去的动作。
静置阶段尚未拼接obs100时observation为null。机器人处于故障时不为生成日志再次起身。

下面只读脚本可以提前等待日志文件出现；默认每50帧显示一次，并显示状态变化：

```bash
python scripts/watch_k1_recovery.py logs/recovery_sim_trial03.jsonl --joints 2 6 9
```

另一个终端可通过仿真验证记录链路：

```bash
python scripts/deploy.py --task k1_recovery --mujoco --recovery-posture facedown \
  --recovery-log logs/recovery_sim_trial03.jsonl
```

完整数据仍在JSONL中，显示抽样不影响记录。查看每帧及完整100维可加
`--every 1 --observation`；读取已完成文件加`--no-follow`。
查看脚本只显示，不执行自动故障保护；停止查看脚本不会停止控制器。
