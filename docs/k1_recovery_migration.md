# K1 recovery 迁移

入口为 `k1_recovery`，沿用 `k1_loco` 的 `BoosterRobotPortal`、
`BoosterRobotController.update_state()` 和 SERIAL 关节输出。
模型与两组轨迹复制自 `dancer-robocupdemo/models/recovery/`，
文件校验值位于 `tasks/locomotion/robots/k1/models/recovery/sha256.json`。
运行时不依赖旁边的 demo checkout。

```bash
python scripts/deploy.py --task k1_recovery
```

按 X/x 进入 Custom 并保持当前实测姿态，按 A/r 启动恢复策略。
此任务使用新增的 `prepare_mode="hold"`，不运行 walking 准备策略，
也不执行 standing 准备流程的站姿插值。头部跟随轨迹，发送 `weight=1`；
不启用视觉追球。

安装 README 中的 MuJoCo 与 booster_assets 依赖后，可分别启动两种倒地姿态：

```bash
python scripts/deploy.py --task k1_recovery --mujoco --recovery-posture faceup
python scripts/deploy.py --task k1_recovery --mujoco --recovery-posture facedown
```

未指定姿态时仿真从俯卧开始；姿态参数只影响仿真初始化。
实机由稳定后的实测投影重力 x 分量选择轨迹：负值仰卧，非负值俯卧。

## 观测来源严格对齐 loco

两种策略调用同一个 `booster_deploy/utils/proprioception.py:read_proprioception()`。

| 数据 | loco 与 recovery 共用的来源/处理 |
|---|---|
| 关节位置 q | `/low_state.motor_state_serial[i].q` → `robot.data.joint_pos`，rad |
| 关节速度 dq | `/low_state.motor_state_serial[i].dq` → `robot.data.joint_vel`，rad/s |
| 机身角速度 | `/low_state.imu_state.gyro` → `robot.data.root_ang_vel_b`，直接使用 rad/s |
| 姿态 | `/low_state.imu_state.rpy` → controller 的 `quat_from_euler_xyz` → wxyz `root_quat_w` |
| 投影重力 g | 对 `[0,0,-1]` 调用同一个 `quat_apply_inverse(root_quat_w, ...)`，不加 offset |

recovery 不直接读取额外 DDS/姿态接口，不用仿真真值高度或线速度替代观测。
demo C++ 的 perception 接口使用度，需要乘 π/180；本仓库 loco 的输入已经是弧度，
因此 recovery 不重复转换。MuJoCo 则和 loco 一样从通用控制器填充 RobotData。

输入为单帧 float32 的 100 维向量，无 loco 的十帧堆叠；22 关节保持 SERIAL 顺序，
不使用 loco 的 20 关节 `webots_to_lab_idx` 重排。

| 切片 | 含义 |
|---|---|
| `[0:1]` | 仰卧 1 / 俯卧 0 |
| `[1:2]` | phase |
| `[2:5]` | 当前轨迹重力减实测 g |
| `[5:6]` | 当前轨迹参考高度，不是机器人实测高度 |
| `[6:28]` | 轨迹 q − 实测 q，头部两项置零 |
| `[28:31]` | 实测投影重力 g |
| `[31:34]` | 实测机身角速度 |
| `[34:56]` | 实测 q − recovery reference，头部两项置零 |
| `[56:78]` | `clip(dq, -5, 5) * 0.1`，头部两项置零 |
| `[78:100]` | 上一次模型 residual；索引 0、1、12、15、18、21 置零 |

整体观测 clip 到 ±100。速度裁剪、reference 和掩码是 recovery 模型的专用预处理；
其他关节（包括 hip yaw / ankle roll）的实时状态保留。

## 轨迹与输出

语义以 demo 的 `src/sim_adapter/sim/src/policy/getup_policy.cpp` 为准。
原 `k1_runtime_config.json` 的旧文字描述中，status_flag、trajectory_gravity 和 phase
与该 C++ 的实测修正不一致；本迁移采用 C++ 中的姿态标志、重力误差和 1 秒 phase 偏移。

- 控制周期 0.02 s。角速度范数连续 0.7 s 小于 0.25 rad/s 才锁定姿态；此前保持实测关节姿态。
- 轨迹时间只在稳定后推进。保持源实现的首个推理帧 row=1；
  `row=min(N-1, floor(t/0.02+0.5))`。
- `phase=clip((1+t)/(1+N*0.02), 0, 1)`。1 秒是源实现的 phase 偏移，
  不额外增加一秒等待。
- 仰卧 213 帧，俯卧 243 帧；模型输出 22 维 residual。
  对禁用关节清零后，`target=clip(trajectory_q+residual, q_min, q_max)`。
  不套用 loco 的 action_scale=0.25、滤波或右臂修正。
- kp/kd、reference、限位和 effort_limit 来自 recovery 配置。
  与 loco 相同，硬件路径发送 q/kp/kd，dq/tau 为零；`effort_limit` 用于 MuJoCo PD 力矩裁剪，
  当前 LowCmd 路径不下发新的固件力矩上限。

## 生命周期

单独运行的任务成功后保持最后目标，状态为 `succeeded`，不自动切换为 loco。
成功条件保留源实现：轨迹完成且 `projected_gravity.z < -0.5`。
轨迹完成后仍倒地超过 2 s，清空 residual、重新等待稳定和选择姿态。
与 demo 仿真内部无限重试不同，仿真配置默认最多重试两次。
2026-09-27 实机发现堵转后，`scripts/deploy.py` 的实机 recovery 入口将重试次数设为 0，
首次恢复超时即停止控制器，避免带故障重复起身；这不等于已实现堵转即时中止。
任务默认退出模式为 damping。外部管理器可读取 policy 的 `state` / `retries`，
在新会话调用 `reset()`，但本次未把自动恢复加入 loco。

## 验证

```bash
python -m unittest discover -s tests -p 'test_k1_recovery.py' -v
python -m unittest discover -s tests -p 'test_k1_loco_robot.py' -v
python -m unittest discover -s tests -v
```

测试覆盖完整两组轨迹的真实 ONNX 推理与独立 NumPy 参考逐帧比对、
与 loco 的共同观测字段对齐、速度裁剪、头部/动作掩码、目标限位、稳定门槛、
成功/重试/失败/复位、非有限值拒绝，以及模拟 low_state 经真实控制器到 SERIAL 输出的链路。
传输测试不创建 ROS 节点、不切实机模式。

本次在 booster_assets `12a97516f57469078707afb587a5f84c1cb110d1` 的 K1 模型上，
使用通用 `MujocoController` 完成闭环起身：仰卧 259 个控制周期（5.18 s），
俯卧 294 个周期（5.88 s），均零重试，结束时根节点高度约 0.55 m。
对应自动测试为 `test_k1_recovery_sim.py`。这些结果不替代实机起身验证；本次未操作实机。

后续实机失败记录见 `k1_recovery_hardware_diagnosis_20260927.md`，
继续调试前按 `k1_recovery_debugging.md` 完成输出和故障通道核实。
