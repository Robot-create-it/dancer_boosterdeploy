# 2026-09-28 recovery 17:25 实机排查

## 观测核对

核对依据为 demo 的 `src/sim_adapter/sim/src/policy/getup_policy.cpp` 实际代码，
不是 runtime JSON 中未更新的描述性文字。ONNX 输入名 `obs`，float32 `[batch,100]`，
单机 `[1,100]`；输出 `raw_action`，float32 `[batch,22]`。
当前模型与 demo 模型 SHA-256 同为
`21e099ee44d819de5f8cbe60ba4feeb765b4d450f8ae1fa4691791f0f2284139`。
轨迹 JSON 两份内容完全相同；reference、q_min/q_max、kp_0/kd、zero_joint_ids 逐项相同。

设 q/dq 为 SERIAL 22 关节实测位置/速度，g 为机身投影重力，
q_ref/g_ref/h_ref 为当前轨迹帧，q_default 为 recovery reference。

| 0-based 半开切片 | 内容 |
|---|---|
| `[0:1]` | faceup=1，facedown=0 |
| `[1:2]` | `clip((1+t)/(1+N*0.02),0,1)` |
| `[2:5]` | `g_ref-g` |
| `[5:6]` | `h_ref`，不是实测高度 |
| `[6:28]` | `q_ref-q`，关节0、1置零 |
| `[28:31]` | g |
| `[31:34]` | 机身角速度，rad/s |
| `[34:56]` | `q-q_default`，关节0、1置零 |
| `[56:78]` | `clip(dq,-5,5)*0.1`，关节0、1置零 |
| `[78:100]` | 上一帧策略残差；关节0、1、12、15、18、21置零 |

整体 clip ±100，单帧无历史堆叠。SERIAL 顺序为头2、左臂4、右臂4、左腿6、右腿6；
左臂不会被掩码清零。reference 除索引3=-1.45、7=+1.45外均为0。
输出 `q_target=clip(q_ref+delta,q_min,q_max)`，不使用 loco 的0.25动作缩放。
实际观测来源仍与 loco 共用 serial feedback 和 IMU，不因本次诊断读取 parallel 温度而改变。

## 新日志证据

原始来源 `/var/log/syslog`，提取窗口17:20～17:29，保存为
`logs/recovery_20260928_fault_summary.json`。用户确认断电重启后，在17:23前后还执行过一次起身。

| 时间 | 证据 |
|---|---|
| 17:22:56～17:23:01 | Custom 控制器反复报告 `User motor command size mismatch, expected 22, got 0` |
| 17:23:05.777635 | Joint(2)，channel7/motorId4，locked-rotor |
| 17:23:05.891828 | Joint(9)，channel8/motorId1，locked-rotor |
| 17:23:06.152281 | Joint(5)，channel7/motorId1，locked-rotor |
| 17:23:09.591637 | 进入 Damping |
| 17:23:14.507750 | 进入 Prepare；错误仍持续 |
| 17:25:21.873688 | 本轮进入 Custom |
| 17:25:23.626 | 用户终端：开始 facedown |
| 17:25:30.486 | 用户终端：恢复超时，停止 |
| 17:25:30.539341 | 固件进入 Damping |

本轮开始前已存在故障。结合用户补充，今天至少在前一次起身后重新出现故障，
不能简单说成昨天故障从未清除，也不能把17:25执行后手感异常解释成17:25才首次堵转。
17:22～17:23的空指令/无有效指令问题还需核查接管与等待A期间的持续发布；
仅凭该错误无法判定是实际发了空数组、固件缓冲被清空还是当时尚未收到有效命令。
17:25这次窗口中未检出同一 command-size 错误，不把先前事件混作本轮直接原因。

俯卧轨迹243帧×0.02s=4.86s，加2s失败等待=6.86s，
与17:25:23.626→17:25:30.486完全对应。
`max_retries=0`，日志“exhausted retries”指首次失败后没有重试额度，不代表又重复执行了多次。

## 只读状态采样

17:33:09开始的采样：GetStatus 2018返回 mode=0、body_control=1、actions=[]（Damping），
外部 joint_ctrl 发布者为空。1120帧中，索引2、5、9在 serial 和 parallel 中的
q/dq/tau_est 全部严格为0；其他手臂关节有非零反馈。
串行 temperature 全0，但 parallel 中正常臂关节为37～43℃，2、5、9仍为0。
parallel 与 serial 的 reserve 仍未反映固件故障。
保存为 `logs/recovery_20260928_readonly_state.json`。

索引2是左肩pitch，5是左肘yaw，9是右肘yaw。
因此右臂不能仅凭整体阻力判定所有电机正常；左臂“未上电”的准确电气原因，
仍需厂家有效的驱动器诊断确认，不能仅凭手感或零反馈断定整臂供电掉电。

## 处理顺序

1. 停止继续起身，按厂家流程处理2/5/9故障。在不运行起身时先确认故障清除、
   通信/反馈恢复；若无动作就再次故障，交厂家检查驱动器、供电、CAN与机械状态。
2. 使用只读检查工具确认恢复前后反馈，不切换到Walking来替代检查：
   `python scripts/check_k1_recovery_state.py --output logs/recovery_state_after_reset_01.json`。
   `suspect_arm_indices=[2,5,9]` 表示与本次失效反馈模式一致，拒绝继续测试。
   空列表只表示未发现此模式，不保证电机健康，仍要结合固件诊断。
3. 再次带负载前补齐接管命令连续性、启动故障门禁、运行故障中止、实机有效力矩限幅。
   当前只有禁用自动重试，以上几项尚未完成验收。
4. 电机和控制通道确认正常后，在现场支撑/减载条件下先验证保持与小范围跟踪，
   记录实际发出的SERIAL 22条命令与反馈；避免直接以完整起身作为第一步验收。
5. 最后单次完整起身，使用 `--recovery-log`、固件日志及视频同步记录。
   本轮用户命令未带 `--recovery-log`，没有可供还原当时100维实际数值的逐帧策略日志。
   先解决首次异常关节，按证据逐项调整，保留原始基线。

本次未切模式或发送运动命令。新增只读检查工具及4项单元测试；它是辅助诊断，
没有自动复位/上电/切模式功能，也没有把并行状态替换进策略观测。
