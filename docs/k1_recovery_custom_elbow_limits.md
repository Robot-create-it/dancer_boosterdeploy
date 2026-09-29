# 同版sim2sim与实机的Custom肘部目标限位差异

2026-09-28：用户确认仿真与实机使用相同处理版本，排除“未同步新增手臂约束”的解释。
以下只读分析未修改实机配置、关节限位或运动代码。

## 已证实的执行差异

本机`/opt/booster/Gait/configs/K1/common_graph_define.lua:36`将`debugging_mode`映射为
`rsm::CustomBodyControlModule`，1714附近将该模块命令接入`parallel_mech_input_custom_mode`。
`common_module_options.lua:417`的`debugging_mode`配置为：

| 串行关节 | recovery策略允许的弯曲端目标 | Custom允许的弯曲端目标 |
|---|---|---|
| 5 左肘yaw | -2.4435rad | -2.1293rad |
| 9 右肘yaw | +2.4435rad | +2.1293rad |

边界相差0.3142rad，约18度。这是软件命令限位，不能等同于机械硬限位。
当前MujocoController.ctrl_step没有复现这一Custom目标角裁剪；XML自身限位也不是同一处理。

固件库SHA256为`333290291b41784dad912deeebbd3baee11e80f506482fcdfb2d7a3f701aceda`。
静态核查`CustomBodyControlModule::UpdateParam`：字符串0x391568/0x391570分别为
q_min/q_max，参数向量存入对象偏移136/160；ReadUserMotorPvtCmd在0x202c1c读取MotorCmd.q，
0x202c24–0x202c44读取上下限并比较，0x202c4c写出裁剪后的q。
因此相同Python目标在真实Custom链路还会再被限位，而直接MuJoCo控制没有这一层。

## 用21:17实际命令离线核算

逐条处理`recovery_fixed_20260928_211749.jsonl`的230条command记录：

- 左肘5：16帧越过Custom下限，轨迹row35–50，即t=0.70–1.00秒。
- 右肘9：14帧越过Custom上限，row35–48，即t=0.70–0.96秒。
- 最大目标改变量左肘0.1394rad（约8度）、右肘0.1770rad（约10.1度）。
  这与配置边界相差18度是不同的量，因为Python此前已经施加手臂P项约束。

row40（t=0.80秒）例子：

| 项目 | 左肘5 | 右肘9 |
|---|---:|---:|
| Python发布目标rad | -2.2438 | 2.2852 |
| 按Custom限制后的目标rad | -2.1293 | 2.1293 |
| 按原发布目标计算PD Nm | -13.79 | 13.02 |
| 按Custom目标计算PD Nm | -7.50 | 4.45 |
| 相邻策略帧tau_est反馈 Nm | -7.39 | 4.88 |

限位后的PD明显更接近反馈。反馈和发布快照存在少量时差，tau_est也是反馈估计量，
所以此表是旁证，不是抓到了电机端最终命令的原始记录。

## 对现象的解释范围

这项差异在手臂建立支撑的早期发生，早于约1.2秒后明显收腿，因而是用户所述
“仿真有支撑过渡、实机没有完成便动腿”的具体候选原因。
此前已确认手臂后续严重落后参考，最后关节2/9报堵转；不能把新的候选原因直接称为已证实的唯一根因。

正确验证顺序是先在同一MuJoCo模型的输出端复现当前Custom软件限位，
对比同初始状态下支撑过渡与关节跟踪是否重现实机差异。不要通过修改robot固件扩大限位验证。
若复现差异，再决定调整轨迹/策略的可行范围或经厂家确认采用适当的原生控制接口。
单纯把recovery的目标也裁到2.1293rad只会对齐限制，本身不能保证起身成功。

完整离线核算保存于`logs/recovery_2117_custom_limit_audit.json`。
