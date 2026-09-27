# Shoot 球输入：demo 仿真与 deploy 对比

本文主体为修改前的对比记录。后续实施范围仅为 pass/shoot 网络输入缓存：获得有效球后，漏检/超时沿用最后有效原始 XY，保持推理历史连续；新有效球覆盖缓存。测试方向、固定模型选择、重力观测和控制流程未迁移，首次见球前仍沿用原有保持逻辑。

对比本机 2026-09-27 的工作区文件（包括 demo 当前未提交内容），未修改控制代码或运行实机。

## 结论

**同一模型、同一球坐标、同一射门方向下，球命令的五维观测构造一致；上游球状态维护、丢球行为、射门方向和模型选择不一致。** demo 仿真没有 deploy 的“无球时每帧目标角追随实测角”分支。

## C++ 仿真主链路

根目录：`/home/booster/Workspace/dancer-robocupdemo`。

| 项目 | demo 仿真 | deploy 实机 shoot |
|---|---|---|
| 优先球来源 | `SimMotion` 接收 `/kick_ball` 的 x/y/dir/power，策略优先使用该参考 | 直接读取 `/booster_vision/detection` 的 Ball.position_projection 前两维 |
| 备用来源 | 视觉 `B` 中取最近且相机距离 <3m 的球，由相机极坐标、头部姿态与躯干姿态转换到水平机器人 yaw 坐标系 | 无第二路球估计 |
| 漏检 | 不清除上次 x/y；`ball_valid_` 一旦为真就保留 | 一帧新的检测没有合格球即写 valid=False |
| 参考过期 | `/kick_ball` 回调不检查 header 时间戳，`kick_ref_valid_` 一旦置真不会因消息停止而自动失效 | 球、图像、头位姿都要求 <=0.5 秒；重复/乱序/过远未来检测会被拒绝 |
| 完全没有参考/从未见球 | 控制选择 walk，而非让关节目标追随实测角 | 已启用 shoot 时返回当前 q，并 reset 策略 |
| 已见球后短暂丢失 | 只要 VisualKick 仍 active 且没有更高优先级状态，继续用保留参考运行 kick/shoot | 停止 shoot 模型推理，每周期返回最新 q；历史和 last_action 被清空 |
| 射门方向 | `/kick_ball.dir`，代表独立目标方向；调试 `--test-kick-aim` 也指定独立方向 | 每帧 `atan2(raw_ball_y, raw_ball_x)`，机器人到球的方向 |
| 路由 | 默认按目标方向与球方向的差选模型；已选路由在球距离 <1m 时锁定；可用 K1_KICK_MODEL 固定 | `--shoot-policy` 固定一个模型，缺省后缀 2 |

注意：仿真的 `ball_valid_` 表示曾获得球位置，不表示本帧新鲜；头部调试跟踪另用 `ball_stamp_` 判断新鲜度。这两个语义不能混用。仿真保留的是旧机器人系 x/y，不会仅凭缓存自动补偿后续机器人运动。

策略中 `kick_ref_valid` 成立时优先采用 `/kick_ball` 参考；因此“参考消息停止就自动切到新视觉球”并不符合当前实现。brain 可通过停止 VisualKick 结束该策略，但参考缓存本身没有过期处理。

补充核对上层会话限制：**缓存无限期保留不代表完整 demo 会无限期射门。** `brain_tree.cpp::visualKickBallUsable` 与 `RLVisionKick::onRunning` 会判断是否还能继续，依据时间而非固定消失帧数。当前 `src/brain/config/config.yaml:89` 配置为真实球新鲜度 650ms、真实球/预测记忆延续上限 1000ms；超过新鲜度后还必须满足预测或记忆可用条件，且无检测近球（<=0.45m）可能被提前拒绝。`lose_ball` 标志也会直接拒绝；守门员有效拦截目标另有分支。无可用球会进入退出流程，退出减速配置为 140ms；无进展 3500ms、总会话 14000ms 等条件也会结束动作。强制 kick 的调试模式/独立纯仿真脚本不能套用这组 brain 会话限制。

源码定位：

- `src/sim_adapter/sim/src/motion.cpp:93`：参考订阅；`:244`：视觉球选择/坐标变换；`:343`：调试覆盖；`:369`：kick/walk 选择。
- `src/sim_adapter/sim/include/sim/motion.h:143`：明确说明球位置无限期保留。
- `src/sim_adapter/sim/src/policy/kick_policy.cpp:190`：路由；`:277`：参考优先与观测构造；`:294`：推理和历史更新。
- deploy `booster_deploy/controllers/booster_robot_controller.py:268`、`:848`：球写入/读取有效性；`tasks/locomotion/k1_shoot.py:37`、`:54`：方向和无球分支。

brain 上游也不同：`src/brain/src/brain.cpp:509` 的 `pubKickMsg()` 按条件选择真实球、预测球或记忆球；参数控制是否允许预测/记忆，VisualKick 近距离时还有额外限制。当前代码默认真实球年龄上限 650ms、记忆上限 1000ms、无检测拒绝距离阈值 0.45m；实际使用值由配置决定。deploy 没有这层。

## 相同的命令观测与不同的方向

两边均先按所选模型加球偏移，之后计算：

```text
b = raw_ball + model_ball_offset
scale = 2 / max(2, norm(b))
command = [cos(dir + yaw_offset), sin(dir + yaw_offset),
           b.x * scale, b.y * scale, alternating_flag]
```

shoot 的 speed 恒为 1，不把 power=6 直接写进方向向量。每帧 71 维，堆叠 10 帧，初始历史重复当前帧。连续有效输入时交替标志和历史推进方式一致；deploy 丢球 reset 会打断连续性。

离线验证：五个模型的 SHA-256 均与 demo 一致；shoot 的模型列表、球偏移、方向偏移、clip_action 配置一致。对五条路由分别输入 (0.6,0.2) 与 (3,4)，让 C++ 公式的 dir 等于 deploy 的 atan2，实际 Python 五维观测与按 C++ 源码计算的参考值在 float32 下最大误差为 0。此验证没有编译运行 C++，也不证明闭环一致。

方向差异示例：固定后缀 2，球 (0.6,0.2)，第一帧：

- 仿真要求正前方射门 dir=0：`[0.995004, 0.099833, 0.6, 0.15, 1]`。
- deploy 自动指向球：`[0.912374, 0.409358, 0.6, 0.15, 1]`。

另有一项不属于球字段的整体观测差异：C++ 加 `gravity_offset=[0.015,0,0]`；当前 Python locomotion 观测没有该加法。因此不能由五维球命令一致推断完整 710 维输入完全一致。

## 独立纯 MuJoCo 测试脚本

`scripts/pure_mujoco_kick_sweep.py:403` 的普通模式每周期直接读取仿真真值球坐标；match-like 模式用简化前向视野判定更新 `last_visible_ball`，不可见时继续用该缓存。两种模式都继续调用 policy.step，不会执行 deploy 的无球追随 q 分支。该脚本 `ball_robot_xy()` 使用完整躯干旋转逆变换，区别于 C++ 备用视觉路径显式去除 roll/pitch 的水平 yaw 坐标。

## 对软掉排查的意义

本次对比进一步确认：**deploy 的丢球支撑行为不是 demo 仿真中的同等实现**，仿真 shoot 正常不能验证该分支安全。此前离线复现的追随下沉问题应优先修复；但事故日志缺少逐帧有效性记录，仍不能据此断言本次一定因丢球触发。

若目标是对齐，应明确独立射门方向、球估计/缓存与过期策略、无球时平衡控制、策略历史生命周期。直接照搬仿真的无限期旧球缓存会引入过时目标，不等于完整的实机修复。
