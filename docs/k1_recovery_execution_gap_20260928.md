# 2026-09-28 recovery 执行链路排查结论

本文保留修复前的诊断证据。后续已补齐手臂位置误差约束、Custom连续保持和
已知故障反馈联锁，实机验收尚未完成；当前操作说明见[k1_recovery_hardware_validation.md](k1_recovery_hardware_validation.md)。

## 结论与边界

已经证实本次软臂伴随2（左肩pitch）、9（右肘yaw）、6（右肩pitch）
先后发生locked-rotor，Prepare切换没有解除故障。进一步静态核对本机固件发现：
原生FallDownRecovery对手臂执行目标有限制位置误差的步骤，当前迁移没有复现。
这是明确的执行端迁移缺陷，也是撑地受力时应优先解决的问题。
缺少首次故障前的命令/电流记录，不能证明该遗漏是此次堵转的唯一原因。

## 原生输出限制的证据

只对磁盘上的 `/opt/booster/Gait/bin/lib/librsm_modules.so` 做静态反汇编；
没有附加运行进程、调用固件函数或修改二进制。
文件SHA-256：`333290291b41784dad912deeebbd3baee11e80f506482fcdfb2d7a3f701aceda`。

`FallDownRecovery::UpdateParam`：

- 地址0x20c208～0x20c220以字符串地址0x391f88（`torque_limit`）调用FindVectorOrDie<double>。
- 地址0x20c230将该vector写入对象偏移0xa0。

`FallDownRecovery::UpdateCmd`：

- 0x20e380、0x20e390、0x20e3a4读取上述torque_limit向量。
- 0x20e3a8从输出MITMotorCommand读取kp；0x20e3ac计算torque_limit/kp。
- 0x20e3bc～0x20e3dc读取MotorStateGroup的当前位置，按当前位置加减上述值裁剪候选目标。
- 0x20e3f4写出受限目标位置；0x20e40c将前馈tau置零。
- 随后的特殊关节处理包含踝关节，不能将此处手臂公式当成全部原生控制算法。

对于本次涉及的手臂，限制可写为：

```text
q_cmd = clamp(q_candidate, q_measured - torque_limit/kp,
                           q_measured + torque_limit/kp)
```

本机`common_module_options.lua`中rl_traj_fdr的手臂torque_limit=14，肩pitch kp=53，
肘yaw kp=55，对应位置误差上限约0.26415rad和0.25455rad。
这限制的是比例项，不是保证总PD力矩或驱动电流的硬上限；阻尼项和采样周期仍然重要。

当前Python仅做`clamp(trajectory + residual, q_min, q_max)`，随后发送q/kp/kd。
robocupdemo的sim_adapter getup_policy.cpp也只实现了这一层关节角限位，
仿真另外使用执行器力矩裁剪；不能据此认为复制sim_adapter即完整复制了原生实机执行链。
CustomBodyControlModule::ReadUserMotorPvtCmd的静态代码读取并传递q/kp/dq/kd/tau/weight，
该读取函数未执行上述FDR位置误差裁剪；不能外推为驱动器完全没有其他限制。

## 离线复现（没有ROS通信或实机动作）

采用现有离线控制器测试的内存消息替身，令目标比测量位置多0.6rad、速度为0：

| 关节 | 现有控制器实际构造的目标误差 | 比例项kp×误差 | 原生手臂位置误差限制 |
|---|---:|---:|---:|
| 左肩2 | 0.6rad | 31.8Nm | 0.26415rad |
| 右肩6 | 0.6rad | 31.8Nm | 0.26415rad |
| 右肘9 | 0.6rad | 33.0Nm | 0.25455rad |

这是人工构造的离线反例，不是本次事故测到的力矩。结果证明仅配置effort_limit=14
不会让实机Python路径执行该限制。保存于`logs/recovery_20260928_execution_gap_reproduction.json`。
同一次离线检查确认：等待A的4次循环中保持命令发布次数均为0。

## 模型、轨迹与观测验证

直接读取本机固件资产，不使用网上另一个版本：

- 原生`k1_fdr.pt`和迁移ONNX对64组固定随机100维输入的最大输出差：9.5367431640625e-7。
- faceup/facedown的joint、gravity、height逐元素差全部为0（height只展平Nx1维度）。
- 19项观测/策略/实机控制器离线测试通过。
- 起初MuJoCo测试因缺booster_assets跳过；随后从官方仓库在临时目录取已知资产提交
  `12a97516f57469078707afb587a5f84c1cb110d1`，通过临时PYTHONPATH运行，
  faceup/facedown两个测试均实际通过。没有永久安装依赖或启动实机控制。

上述结果降低了模型导出或轨迹复制错误的可能性，不证明事发时每帧实机观测都正确。
资产对比保存在`logs/recovery_20260928_native_asset_parity.json`。

## 实测记录与其他控制缺口

- 19:21故障时间线见`k1_recovery_hardware_diagnosis_20260928_1921.md`。
- bag trial03：19:33:30～19:33:40，4553帧；trial04：19:59:19～19:59:24，2482帧。
  两者仅有low_state，无joint_ctrl；2、6、9全程符合故障后零反馈模式。
  它们录制于故障之后，无法还原19:21前的目标角/跟踪误差。
- 20:02:31再次只读GetStatus成功，仍为Prepare，外部joint_ctrl发布者为空，
  2、6、9反馈仍全零。结果保存在`logs/recovery_20260928_2005_state.json`。
- Custom前只发送一次保持，等待A不持续发；与本轮8次expected22/got0一致。
- 故障全零值通过isfinite检查，首次故障后没有立即中止。禁止重试不等于故障联锁。
- 故障前已有间歇CAN读取失败。首次故障附近的读取失败与堵转谁先导致谁尚无法定论。

## 后续修复要求

优先补齐并验证原生手臂输出约束、Custom接管连续性和有效故障联锁；
不能将50Hz位置目标裁剪冒充驱动器硬限流，也不能仅提高kp或关闭堵转保护。
按厂家流程恢复2/6/9并检查通信后，先做有支撑的保持/小范围跟踪，
完整动作再同步记录发布命令、策略观测、反馈及固件故障。
本次任务完成的是定位和离线验证，没有修改运动控制行为，也没有宣称实机修复完成。
