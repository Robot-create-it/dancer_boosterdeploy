# Recovery / loco 输出链路与固件拒绝条件核查

日期：2026-09-28。只读分析代码、配置、已保存日志与安装的固件共享库；没有操作运动或修改保护门槛。

更新：此处低电量拒绝对应20:47一轮。后来21:17新增测试电量约52%，没有低电量拒绝，
仍出现持续承重后关节2/9堵转，见[k1_recovery_hardware_diagnosis_20260928_2117.md](k1_recovery_hardware_diagnosis_20260928_2117.md)。
不能把本文门槛分析当成所有软臂现象的根因。

## 明确的低电量门槛

分析本机 `/opt/booster/Gait/bin/lib/librsm_modules.so`，SHA256
`333290291b41784dad912deeebbd3baee11e80f506482fcdfb2d7a3f701aceda`。
这是本机版本的静态二进制分析，不是通用于所有固件版本的厂家规格。

`BoosterApiMsgServer::BatteryStateHandler`（0x2c96f0）的普通有限数值路径：

```text
若 soc <= 0.5 或 average_voltage <= 0.5：走无效数据处理分支
否则，若 soc < 15.0 或 average_voltage < float32(42.8)：
    打印 Motion module received low battery
    将 ModeManager + 0x289 的低电量标志置为1
```

关键位置：0x2c9768装载15.0，0x2c976c比较soc，0x2c9770低于分支；
0x2c977c/0x2c9784构造0x422b3333，即42.79999923706055；0x2c978c比较average_voltage。
0x2c9a2c–0x2c9a34向ModeManager+0x289写1。该回调的健康数据路径未见清除此标志的写入，
所以不能假设电压回升就自动解除；全局恢复流程仍需按厂家说明执行。

20:47:39.874192的实际日志：soc=30.7，average_voltage=42.6829。
电量未低于15%，但平均电压已低于42.8V，明确命中第二个条件。
此时瞬时电压41.6V，下一秒39.5V；触发比较用的是平均电压，不是把39.5V当作阈值。

## 拒绝发生在哪里

1. `LowCmdHandler`在0x2c901c调用`ModeManager::ExecTask(BodyControlTask)`。
2. `ValidateBodyControlTask`在0x16c190–0x16c1a0读取上述同一标志，置位则校验失败。
3. `ExecTask(BodyControlTask)`在0x1715b0校验；失败走0x1715bc开始的日志/替换路径，
   打印`Low battery, replace task with BodyControlMoveTask`，用新任务替代原来的Custom关节任务。
4. `ExecTask(UpdateActionTask)`在0x170770也检查同一标志，拒绝并打印
   `Low battery, refused to update action`。
5. `CustomBodyControl::ExecTask`在0x204dcc动态转换为BodyControlCustomTask；
   类型不匹配则不走0x204dfc之后更新关节命令快照的代码。
   GOT重定位0x458340/0x458410分别对应BodyControlCustomTask/BodyControlTask的typeinfo。

因此ROS仍可接收LowCmd，但新的Custom关节目标不会沿正常路径更新快照。
这与现场“Python目标继续变化、髋膝几乎不动、左肩持续承重”一致。
门槛不是按Python task名称或模型输出计算，loco通过同一固件接口也受这个标志约束。
仅凭loco先前正常，无法证明它是在相同电压、相同固件标志与承重条件下测试。

## 输出链路对照

以下数值由两任务配置构造后调用`booster.apply_to_robot`取得，是实机最终增益，
不是只看仿真配置。loco另有实机增益覆盖。

| 项目 | k1_loco | k1_recovery |
|---|---|---|
| 传感器来源 | low_state.motor_state_serial、IMU；共用read_proprioception | 相同 |
| 输出语义 | 20维归一化动作，映射到22个串行关节 | 22维轨迹残差，已是串行顺序 |
| 动作裁剪 | 原始动作±100 | 特定残差槽清零，轨迹+残差后裁剪各关节角度上下限 |
| 目标生成 | default + 0.25 × action | trajectory[row] + residual，无0.25缩放 |
| 平滑 | 0.8×新目标 + 0.2×上一滤波目标 | 无此指数滤波 |
| 额外手臂约束 | 无recovery专用约束 | policy阶段及发布前按新反馈约束q±14/kp |
| 手臂实机kp | 每关节20 | 每臂53/40/40/55 |
| 手臂实机kd | 每关节2 | 每臂2.5/3/2.5/2.5 |
| 髋pitch/roll/yaw、膝kp | 100/100/100/100 | 90/70/70/90 |
| 髋、膝kd | 2 | 5 |
| 踝kp/kd | 50/1 | 30/5 |
| 头部kp/kd | 4/1，默认weight=0 | 70/1.5，weight=1 |
| 发布频率、类型 | 50Hz，22维LowCmd，CMD_TYPE_SERIAL，/joint_ctrl | 相同 |
| 目标速度、前馈力矩 | dq=0、tau=0初始化后保留 | 每次显式设dq=0、tau=0 |
| 发布函数 | ctrl_step普通分支 | ctrl_step调用_publish_recovery_target，含检查与再次手臂约束 |

观测原始数据路径相同不意味着观测张量/模型语义相同；recovery为100维，loco为历史观测。
recovery的“轨迹+残差、不乘0.25、不加loco滤波”与robocupdemo的getup_policy.cpp:212–230相符。
不应把两者所有后处理机械统一，否则会改变起身模型训练/迁移所依赖的动作定义。

## 实际含义与边界

- loco正常支持串行下发接口可用，不能排除recovery的大负载、不同PD增益和供电压降。
- 本轮固件拒绝的条件已经明确；压降为何如此大（电池状态、连接、负载、既有故障等）仍需进一步验证。
- 原始任务替换可导致目标冻结；不要据此宣称已完整逆向所有驱动行为或只有一个堵转原因。
- 本次没有改动运动代码。应先恢复供电与电机故障，再补充电源/固件状态监测；不移除低电量保护。

## 证据文件

- `logs/recovery_firmware_battery_gate_disassembly.txt`
- `logs/recovery_fixed_real01_evidence.log`
- `logs/recovery_fixed_real01_analysis.json`
- `logs/recovery_fixed_real01_after.json`
- `tasks/locomotion/locomotion.py`：inference、_action_to_targets
- `tasks/locomotion/k1_recovery.py`：inference、_inference
- `booster_deploy/controllers/booster_robot_controller.py`：ctrl_step、_publish_recovery_target
