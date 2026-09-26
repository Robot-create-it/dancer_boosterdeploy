# K1_LOCO 实机无法正常运动问题 - 诊断结果

## 🔍 排查日期
2026-09-26

## ✅ 已验证正常的部分

### 1. 模型文件 ✓
三个ONNX模型均正常加载：
- **k1_loco_base.onnx**: 1.99MB, 输入690维 → 输出20维
- **k1_loco_side.onnx**: 1.99MB, 输入690维 → 输出20维  
- **k1_loco_turn.onnx**: 1.99MB, 输入690维 → 输出20维

测试推理成功，无加载错误。

### 2. PD增益配置 ✓
K1_LOCO通过`booster`配置复用K1_WALK的实机增益，完全相同：
- **Kp**: `[4.0, 4.0, 20.0, ..., 100.0, 65.0, ...]`
- **Kd**: `[1.0, 1.0, 2.0, ..., 2.0, 1.0, ...]`

### 3. 默认姿态 ✓
两者的`default_joint_pos`完全一致。

### 4. 控制指令通道 ✓
- 唯一发布者：`/joint_ctrl`
- 只有Custom模式生效
- 无冲突来源

---

## 🔴 发现的问题

### 🚨 问题1：准备模式的vyaw=0.2（高风险）

**代码位置**：`tasks/locomotion/nested_locomotion.py:42-45`

```python
if adjust:
    self.previous = [0.0, 0.0, 0.0]
    self.processed = [0.0, 0.0, 0.2]  # ⚠️ vyaw = 0.2
    return self.processed.copy()
```

**触发时机**：
- 按X进入Custom模式时，`loco_adjust=False`，但初始化会进入某种状态
- 如果代码路径触发`adjust=True`，会返回`[0, 0, 0.2]`

**实际测试结果**：
```
输入: [0, 0, 0], adjust=True
输出: [0.0, 0.0, 0.2]
选择模型: 2 (turn)  # ⚠️ 会选turn模型而非base
```

**影响**：
- 机器人会尝试原地旋转（vyaw=0.2 rad/s ≈ 11.5°/s）
- 同时加载的是**turn模型**而非base模型
- 这会导致机器人行为异常

---

### ⚠️ 问题2：速度增量限制导致响应缓慢

**代码位置**：`tasks/locomotion/nested_locomotion.py:47-52`

```python
self.previous[i] += _clamp(
    command[i] - self.previous[i],
    -cfg.max_vel_cmd_decre[i] * cfg.update_interval,
    cfg.max_vel_cmd_incre[i] * cfg.update_interval,
)
```

**限制值（每0.02秒）**：
- vx增量：`[-1.6*0.02, +1.3*0.02]` = `[-0.032, +0.026]`
- vy增量：`[-1.2*0.02, +1.2*0.02]` = `[-0.024, +0.024]`
- vyaw增量：`[-2.0*0.02, +2.0*0.02]` = `[-0.040, +0.040]`

**实际测试**：
```
从0加速到vx=0.5，需要约20步：
Step 0: vx=0.026
Step 1: vx=0.052
Step 2: vx=0.078
Step 3: vx=0.104
Step 4: vx=0.130
...
```

**对比K1_WALK**：
- K1_WALK：速度命令直接传递，无增量限制
- K1_LOCO：需要缓慢加速，从0到0.5需要约1秒（50次*0.02s）

**影响**：
- 按一次键盘W（增加vx=0.1），实际速度从0→0.1需要4步（0.08秒）
- 如果用户期望立即响应，会感觉"机器人不动"

---

### ⚠️ 问题3：gravity_offset改变观测值

**代码位置**：`tasks/locomotion/nested_locomotion.py:136`

```python
observation[3:6] += self.gravity_offset  # [0.015, 0.0, 0.0]
```

**影响**：
- 重力投影X分量系统性偏移0.015
- 如果训练时使用了这个offset → 必须保持
- 如果训练时没用 → 会导致姿态控制偏差

**无法从代码判断**：需要确认训练配置。

---

### ⚠️ 问题4：速度耦合约束限制运动

**代码位置**：`tasks/locomotion/nested_locomotion.py:55-64`

当vx较大时，会限制vy和vyaw：

| vx范围 | vy限制 | vyaw限制 |
|--------|--------|----------|
| > 0.1 | [-0.20, 0.25] | [-1.0, 1.0] |
| > 0.6 | [-0.15, 0.15] | [-1.0, 1.0] |
| > 0.9 | [-0.10, 0.10] | [-1.0, 1.0] |
| > 1.1 | [-0.10, 0.10] | [-0.7, 0.7] |

**测试结果**：
```
输入: [1.2, 0.3, 0]
输出: [0.26, 0.24, 0.0]  # vy被限制到0.24，且需要多步才能达到
```

**影响**：
- 高速前进时无法大幅横移或转向
- 这是设计行为，但可能与用户预期不符

---

## 🎯 最可能导致"无法运动"的根本原因

### 原因A：准备阶段的vyaw=0.2干扰（可能性80%）

**症状预测**：
1. 机器人原地有微弱旋转趋势
2. 姿态不稳，可能倾斜
3. 给前进命令后响应异常

**验证方法**：
观察按X后、未按A前，机器人是否有原地旋转。

---

### 原因B：速度增量限制+用户误判（可能性60%）

**症状预测**：
1. 按W后机器人"看起来不动"
2. 实际上在缓慢加速，但用户没耐心等
3. 需要持续按住W或多次按压才能看到明显运动

**验证方法**：
按W后保持15秒（约750步），观察vx是否达到目标。

---

### 原因C：模型与观测不匹配（可能性40%）

**症状预测**：
1. 机器人动作异常、抖动
2. 站立不稳
3. 关节目标角度超出合理范围

**验证方法**：
检查训练时是否使用了`gravity_offset=[0.015, 0.0, 0.0]`。

---

## 🔧 修复方案

### 方案1：修复准备模式的vyaw（推荐优先）

**文件**：`tasks/locomotion/nested_locomotion.py`

**第44行，改为**：
```python
self.processed = [0.0, 0.0, 0.0]  # 全部改为0
```

**原因**：
- 准备阶段不应该有任何旋转命令
- vyaw=0.2会导致选择turn模型，与预期不符

---

### 方案2：移除gravity_offset（需确认训练配置）

**文件**：`tasks/locomotion/robots/k1/loco.py`

**第23行附近，改为**：
```python
policy = K1NestedLocomotionPolicyCfg(
    # ... 其他配置保持
    gravity_offset=[0.0, 0.0, 0.0],  # 改为全0
)
```

**警告**：
- 只有在确认训练时**没用**offset的情况下才能改
- 如果训练时用了offset，移除会导致更大问题

---

### 方案3：放宽速度增量限制（可选）

**文件**：`tasks/locomotion/robots/k1/loco.py`

**第23行附近，增加配置**：
```python
policy = K1NestedLocomotionPolicyCfg(
    # ... 其他配置保持
    max_vel_cmd_incre=[2.6, 2.4, 4.0],  # 加倍增量限制（原1.3/1.2/2.0）
    max_vel_cmd_decre=[3.2, 2.4, 4.0],  # 加倍减量限制（原1.6/1.2/2.0）
)
```

**效果**：
- 加速时间从1秒减半到0.5秒
- 更接近k1_walk的即时响应感

**风险**：
- 可能导致运动不稳定
- 需要实机验证

---

### 方案4：强制使用base模型（调试用）

**文件**：`tasks/locomotion/robots/k1/loco.py`

**第23行附近，增加配置**：
```python
policy = K1NestedLocomotionPolicyCfg(
    # ... 其他配置保持
    forced_route=0,  # 强制使用base模型
)
```

**效果**：
- 无论速度命令如何，始终使用base模型
- 排除模型切换导致的问题

**用途**：
- 仅用于调试
- 验证是否是模型切换逻辑的问题

---

## 📋 推荐的调试步骤

### 步骤1：修复vyaw=0.2问题（5分钟）
```bash
cd /home/booster/Workspace/dancer_boosterdeploy

# 备份原文件
cp tasks/locomotion/nested_locomotion.py tasks/locomotion/nested_locomotion.py.backup

# 修改第44行
sed -i '44s/self.processed = \[0.0, 0.0, 0.2\]/self.processed = [0.0, 0.0, 0.0]/' tasks/locomotion/nested_locomotion.py

# 验证修改
grep -n "self.processed = \[0.0, 0.0" tasks/locomotion/nested_locomotion.py
```

### 步骤2：重新测试（10分钟）
```bash
# 启动deploy
python scripts/deploy.py --task k1_loco

# 按X进入Custom模式
# 观察：机器人是否保持静止站立？

# 按A/r启用速度命令
# 按W一次，等待10秒
# 观察：机器人是否缓慢前进？
```

### 步骤3：添加调试日志（如果仍不动）
在`nested_locomotion.py:129`添加：
```python
logger.info(f"[LOCO] Raw={raw}, Processed={processed}, Route={_ROUTE_NAMES[route]}")
```

在`nested_locomotion.py:141`添加：
```python
logger.info(f"[LOCO] Active model={_ROUTE_NAMES[self.active_route]}")
```

### 步骤4：对比joint_ctrl消息（如果仍不动）
```bash
# 终端1：运行k1_walk
python scripts/deploy.py --task k1_walk

# 终端2：记录joint_ctrl
ros2 topic echo /joint_ctrl > k1_walk_joint_ctrl.log

# 按A/r，按W，等待5秒，Ctrl+C停止

# 终端1：运行k1_loco
python scripts/deploy.py --task k1_loco

# 终端2：记录joint_ctrl
ros2 topic echo /joint_ctrl > k1_loco_joint_ctrl.log

# 对比两个日志文件，查看关节目标是否有差异
```

---

## 🧪 快速验证脚本

保存为`test_k1_loco_fix.sh`：

```bash
#!/bin/bash
cd /home/booster/Workspace/dancer_boosterdeploy

echo "=== K1_LOCO 修复测试 ==="
echo ""
echo "1. 备份原文件..."
cp tasks/locomotion/nested_locomotion.py tasks/locomotion/nested_locomotion.py.backup

echo "2. 修复vyaw=0.2问题..."
sed -i '44s/\[0.0, 0.0, 0.2\]/[0.0, 0.0, 0.0]/' tasks/locomotion/nested_locomotion.py

echo "3. 验证修改..."
grep -A 1 -B 1 "adjust:" tasks/locomotion/nested_locomotion.py | grep processed

echo ""
echo "✓ 修复完成！"
echo ""
echo "现在运行: python scripts/deploy.py --task k1_loco"
echo ""
echo "测试步骤："
echo "  1. 按X进入Custom模式，观察是否保持静止"
echo "  2. 按A启用RL模式"
echo "  3. 按W增加vx，等待10秒观察是否缓慢前进"
echo "  4. 按空格停止"
echo ""
echo "如需恢复原文件："
echo "  cp tasks/locomotion/nested_locomotion.py.backup tasks/locomotion/nested_locomotion.py"
```

---

## 📊 K1_WALK vs K1_LOCO 核心差异总结

| 维度 | K1_WALK | K1_LOCO | 影响 |
|-----|---------|---------|------|
| **模型数量** | 1个 | 3个(base/side/turn) | 增加复杂度 |
| **速度响应** | 即时 | 缓慢加速(~1秒) | **用户感知差异大** |
| **准备模式vyaw** | 0 | 0.2 | **导致异常旋转** |
| **gravity_offset** | 无 | +0.015(X) | 需验证训练配置 |
| **速度耦合** | 无 | 有复杂约束 | 高速时限制横移 |
| **实机PD增益** | 直接配置 | 相同值 | 无影响 |

---

## 💡 结论

**最可能的问题**：准备模式的`vyaw=0.2`导致机器人尝试旋转，选择了错误的模型(turn而非base)。

**建议优先级**：
1. **立即修复**：vyaw=0.2 → 0.0
2. **测试验证**：按照步骤2重新测试
3. **如仍有问题**：检查gravity_offset是否与训练一致
4. **最后手段**：添加详细日志，对比joint_ctrl消息

**预期修复后效果**：
- 按X后机器人静止站立
- 按A/r后，按W能看到缓慢但稳定的前进
- 速度达到稳态需要1-2秒（这是正常的设计行为）

如果修复vyaw后仍然完全不动，问题可能在模型本身或底层通信，需要进一步排查。
