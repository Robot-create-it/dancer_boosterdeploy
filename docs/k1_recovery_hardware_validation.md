# Recovery 修复后的实机验证

本次修改没有操作实机。先按厂家流程解除2/6/9等电机故障，并核查已有CAN读取失败。
软件不自动清故障、重新使能电机或关闭驱动保护。空故障字段不是健康证明。
50Hz目标角约束限制手臂比例项，不是驱动器硬限流，也不能保证首次堵转不会发生。

## 1. 只读确认

```bash
python scripts/check_k1_recovery_state.py --seconds 10 --output logs/recovery_fixed_preflight01.json
```

可疑关节列表应为空，反馈有效；厂家诊断无持续电机故障，固件无未解释的通信异常。
新控制器会拒绝已出现的手臂双通道零反馈、错误长度、非有限值和过期状态。
不要拔线、堵住关节或制造实机故障来测试联锁；这些路径已经通过离线故障注入检查。

## 2. 开始记录，覆盖接管之前

终端A（停止整个测试后再Ctrl+C结束录制）：

```bash
ros2 bag record -o logs/recovery_fixed_bus01 /low_state /joint_ctrl
```

终端B：

```bash
tail -n 0 -F /var/log/syslog | grep --line-buffered 'booster-daemon' \
  | tee logs/recovery_fixed_firmware01.log
```

使用新的文件/目录名。录制不发布命令；不要在实机ROS域播放包含joint_ctrl的bag。

## 3. 有支撑的保持验证（会控制实机）

布置能够承接Damping时机体重量的支撑，手臂先不承重；保持自然倒地姿态，
不要由Prepare强行维持不合适的姿态。准备/摆放按现场厂家流程完成。

```bash
python scripts/deploy.py --task k1_recovery --recovery-hold-only 2>&1 \
  | tee logs/recovery_fixed_hold01.log
```

按X/x后，程序等待有效反馈连续稳定至少0.5s，最多等待3s；等待期间不发布保持命令、
不切Custom。`Low state subscription started`只表示订阅线程启动，不表示已经收到数据。
若看到`Waiting for continuous valid feedback before Custom`，等待检查完成即可；
实际故障或数据过期仍会立即退出，连续有效窗口一直不足则超时退出。预期日志：

```text
Custom mode started; publishing guarded hold at 50.0 Hz
RECOVERY HOLD ONLY: A/r disabled; Ctrl+C exits to Damping
```

该模式不会加载/运行起身模型，A/r无效；不使用recovery-log，保持阶段由bag记录。
保持约10s，确认没有明显跳动、软臂、持续顶住、反馈异常或新固件故障。
可在额外终端用`ros2 topic hz /joint_ctrl`检查发布频率约50Hz；
bag里每帧motor_cmd应为22，不能出现等待阶段长期停发或固件expected22/got0。
按Ctrl+C退出，预期切Damping；确保支撑已承接重量。

拒绝启动/中止日志以`K1 recovery interlock:`开头。出现它时停止验收、保存原因，
不要通过增大超时、禁用检查或重复按A绕过。

## 4. 单次起身验证（保持验证通过且现场条件允许后）

开始新一轮bag/固件日志并拍侧面视频，支撑保留双手、双脚起身所需接触；
不要完全悬空后以策略表现判断起身效果。

```bash
python scripts/deploy.py --task k1_recovery \
  --recovery-log logs/recovery_fixed_real01.jsonl 2>&1 \
  | tee logs/recovery_fixed_real01_console.log
```

X/x接管，确认保持正常后按一次A/r。模型加载阶段保持命令继续发送，
就绪后交给策略，实机自动选择faceup/facedown，零次自动重试。
成功后继续保持最终目标，不会自动切换到行走。

另一个终端只读查看：

```bash
python scripts/watch_k1_recovery.py logs/recovery_fixed_real01.jsonl --joints 2 5 6 9
```

异常顶死、失去支撑或任何新增电机故障立即按现场停机流程处理，不等待策略超时。
反馈联锁不会提前识别所有堵转，只在可观察异常/超时后停止发布并请求Damping；
切换依赖RPC/固件响应，不是硬实时急停。Ctrl+C停止查看器不会停止控制器。

## 5. 验收记录

结束动作后停止录制，再执行：

```bash
python scripts/analyze_k1_recovery.py logs/recovery_fixed_real01.jsonl
ros2 bag info logs/recovery_fixed_bus01
python scripts/check_k1_recovery_state.py --seconds 3 --output logs/recovery_fixed_postflight01.json
```

检查：

- 完整起身、无重试、固件无新增locked-rotor/读取失败；2/5/6/9无掉零。
- `published_command_frames > 0`，`published_arm_limit_violations = 0`。
- `published_arm_max_abs_p_term`对应关节2～9，每项不超过14Nm（允许1e-4数值误差）。
  它是按发布时测量值计算的比例项，不是实际电机力矩；总PD估计可能因阻尼项超过14。
- 最终bag同时含low_state和joint_ctrl，并覆盖开始接管至动作退出。

静态保持通过不能代替承重起身验收；本次离线/仿真通过也不代表实机已经修复完成。
