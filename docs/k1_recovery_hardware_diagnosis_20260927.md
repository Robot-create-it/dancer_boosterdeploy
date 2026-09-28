# 2026-09-27 recovery 实机失败排查

结论：本次俯卧起身过程中，三个手臂电机明确上报 `locked-rotor`，且先于
重试和 Ctrl+C。退出 Damping 会撤掉主动支撑，但不能单独解释持续的手臂异常。
没有证据据此断言电机永久损坏，也不能把错误归因于 ONNX 的 GPU discovery warning。

## 时间线与直接证据

用户确认机器人实际俯卧，双臂有撑地动作。

| 时间（本机 UTC+8） | 事件 |
|---|---|
| 21:33:54.855 | Python recovery 开始 facedown |
| 21:33:58.003249 | Joint(9)，channel8 motorId=1：16384，locked-rotor |
| 21:33:58.033300 | Joint(2)，channel7 motorId=4：16384，locked-rotor |
| 21:33:58.281730 | Joint(5)，channel7 motorId=1：16384，locked-rotor |
| 21:34:01.716 | Python recovery retry 1 |
| 21:34:03.094 | Python recovery 再次开始 facedown |
| 21:34:05.968 | 用户 Ctrl+C 后推理进程停止 |
| 21:34:06.053 | 程序确认切换到 Damping |
| 至少持续到 21:38:10 | 同一 daemon 持续上报三个关节的 locked-rotor |

关节名按本仓库 K1 SERIAL 顺序：2=`aaleft_shoulder_pitch_joint`，
5=`left_elbow_yaw_joint`，9=`right_elbow_yaw_joint`。
另有同样被固件标注 locked-rotor 的组合错误码 16400、81920；未自行解释未核实的附加位。

原始证据来自 `/var/log/syslog` 中 `booster-daemon[2632]` 的
`joint_manager.cpp:363`；首次命中、末次命中和相关事件已提取到
`logs/recovery_20260927_fault_summary.json`。这里的首次时间是检索窗口内的首次，
不推断更早历史中从未发生过故障。重复日志数量不是独立故障次数。

## 只读实机采样

通过 `/booster_rpc_service` 仅调用 2018 GetStatus，返回：

```json
{"current_actions":[],"current_body_control":2,"current_mode":1}
```

采样时已经是 Prepare，不能沿用退出日志断言机器人仍处于 Damping。
`/joint_ctrl` 的外部发布者列表为空。
约 3 秒窗口获得 1405 条 low_state；最新 RPY 约 `[0.0215, 1.55995, -0.1354]`，
与俯卧一致。串行关节 2、5、9 的 q/dq/tau_est 均为 0，其他多数关节反馈非零。
该异常与日志中的三个故障关节吻合，但仅凭零值不能确认固件内部具体停机/失效机制。

此接口所有关节 temperature/lost/reserve 都为零；尤其日志明确报故障时
`reserve[0]` 仍为零，因此不能把此版本 ROS 消息的零错误位作为健康证明。
排查没有发布 joint_ctrl、切换模式、清除故障或重新运行起身。

## 代码核查与尚未确定的根因

1. 观测来源沿用 loco，实测姿态与 facedown 选择一致；运行频率约 50.023 Hz。
   现有证据不指向模型未加载、未运行或误选仰卧轨迹。
2. recovery 的手臂 kp 为 53/40/40/55，kd 为 2.5/3/2.5/2.5，
   与 `/opt/booster/Gait/configs/K1/common_module_options.lua` 中 rl_traj_fdr 一致。
   不能仅凭增益数值偏大便断言配置抄错。
3. MuJoCo 在 `MujocoController.ctrl_step()` 中对 PD 力矩做 effort_limit 裁剪，
   recovery 手臂配置为 14 Nm。实机 `BoosterRobotController.ctrl_step()`
   只下发 q/kp/kd，没有应用该 effort_limit；MotorCmd 也没有力矩上限字段。
   因而仿真与实机的饱和行为尚未对齐；实机底层最终限幅/保护值仍需核实，
   不能描述成实机完全没有任何保护。
4. 当前 low_state 回调仅传递 q/dq/tau_est 和 IMU；recovery 仅检查数值有限性、
   姿态和超时，未接入可靠的电机错误信号。因此在堵转发生后仍继续输出并重试，
   是迁移中必须补齐的实机故障处理缺口。
5. 未记录当时每帧 target/q/dq/tau、接触状态和有效故障位，尚不能区分
   具体是撑地姿态/接触阻挡、轨迹跟踪误差、输出饱和差异或其他因素首先引发堵转。

## 后续处置

暂停重复起身，先按厂家流程处理堵转故障并确认三个关节状态和反馈恢复正常。
不要用反复切换 Walking/Prepare 或加大增益替代故障诊断。
再次实机验证前，应核实 Custom 路径的力矩饱和行为、接入本机实际有效的故障状态，
实现故障即中止且禁止自动重试，并记录恢复阶段的关节目标、反馈和跟踪误差。
不能只检测当前 ROS 的 reserve[0]，因为此次采样已经证明它未反映固件错误。

本次仅完成只读诊断与证据保存；未修改运动参数或宣称故障已修复。
