# K1_LOCO 实机运动问题排查报告

## 执行日期
2026-09-26

## 问题描述
k1_loco任务在机器人实机测试时无法正常运动，需要与已验证可用的k1_walk对照排查原因。

## 对比分析结果

### ✅ 1. 实机PD增益 - 完全相同
k1_loco通过`booster.joint_stiffness/damping`覆盖机制，**复用了k1_walk的实机增益**：
- **Kp**: `[4.0, 4.0, 20.0, 20.0, ..., 100.0, 100.0, 65.0, 65.0, ...]`
- **Kd**: `[1.0, 1.0, 2.0, 2.0, ..., 2.0, 2.0, 1.0, 1.0, ...]`

**代码证据**：
```python
# tasks/locomotion/robots/k1/loco.py:27-30
booster = BoosterRobotControllerCfg(
    joint_stiffness=list(_ROBOT.joint_stiffness),  # 复用k1_walk增益
    joint_damping=list(_ROBOT.joint_damping),
)
```

**结论**：增益配置不是问题根源。

---

### ✅ 2. 默认姿态 - 完全相同
两者的`default_joint_pos`完全一致：
```
[0, 0, 0.2, -1.25, 0, -0.5, 0.2, 1.25, 0, 0.5, -0.15, 0, 0, 0.3, -0.15, 0, -0.15, 0, 0, 0.3, -0.15, 0]
```

**结论**：基准姿态不是问题。

---

### ⚠️ 3. 控制指令来源 - 唯一且互斥

**Deploy程序的控制指令**：
- Topic: `/joint_ctrl`
- 发布者: `booster_deploy_low_cmd_pub`
- 频率: ~50Hz (policy_dt=0.02)
- QoS: RELIABLE + KEEP_LAST

**原厂固件控制**：
- 通过RPC服务`booster_rpc_service`切换模式
- 三种模式：Damping(0) / Walking(2) / Custom(3)
- **只有Custom模式**下deploy的`/joint_ctrl`才生效

**结论**：控制权清晰，不存在冲突。

---

### ⚠️ 4. 速度命令处理 - 有差异

**K1_WALK**：
- 直接使用用户输入（手柄/键盘）
- 范围：vx=[−0.3, 1.6], vy=[−0.3, 0.3], vyaw=[−1.8, 1.8]

**K1_LOCO**：
- 经过`LocoCommandProcessor`处理
- 包含**速度限制、增量限制、耦合约束**
- 初始化时有特殊处理：`adjust=True`时返回`[0.0, 0.0, 0.2]`（vyaw=0.2）

**关键代码**：
```python
# nested_locomotion.py:42-45
if adjust:
    self.previous = [0.0, 0.0, 0.0]
    self.processed = [0.0, 0.0, 0.2]  # 注意：vyaw=0.2
    return self.processed.copy()
```

**潜在问题**：初始vyaw=0.2可能导致机器人原地小幅旋转。

---

### 🔴 5. 观测处理 - 关键差异

**K1_WALK**：
- 标准locomotion观测：`[angvel(3), gravity(3), cmd(3), dof_pos_err(20), dof_vel(20), last_action(20)]`
- 总维度：69 × 10帧 = 690

**K1_LOCO**：
- **增加了gravity_offset**: `observation[3:6] += [0.015, 0.0, 0.0]`
- **替换速度命令**：用`LocoCommandProcessor`处理后的值覆盖

```python
# nested_locomotion.py:136-138
observation = super().compute_observation()
observation[3:6] += self.gravity_offset  # ⚠️ 修改重力投影
observation[6:9] = torch.tensor(processed, ...)  # ⚠️ 替换命令
```

**分析**：
- `gravity_offset=[0.015, 0.0, 0.0]`会系统性地偏移投影重力的X分量
- 如果模型训练时使用了这个offset，实机必须也用
- 如果训练时没用，实机用了就会出问题

---

### 🔴 6. 模型选择逻辑 - 三模型切换

K1_LOCO使用三个独立模型：
- **base**: 前进/后退主控
- **side**: 横向移动（`vy > 0.1 且 vx < 0.2 且 wz < 0.2`）
- **turn**: 原地旋转（`wz > 0.1 且 vx < 0.1 且 vy < 0.1`）

**初始状态**：
```python
self.active_route = 0  # 默认base模型
self.loco_adjust = False
```

**潜在问题**：
1. 如果初始命令为0，可能选错模型
2. 模型切换瞬间可能有不连续

---

### 🔴 7. 准备模式 - 可能的问题点

**两者都使用`prepare_mode="walking"`**，但行为略有不同：

**K1_WALK**：
1. 按X → 读当前姿态 → 发一次PD保持 → 切Custom → 启动策略（0速命令）
2. 按A/r → 启用速度命令

**K1_LOCO**：
1. 按X → 读当前姿态 → 发一次PD保持 → 切Custom → 启动base模型（loco_adjust=False, 初始0速）
2. 按A/r → 设置`task_start_event` → **但模型不会重新加载**

**关键差异**：
- K1_LOCO的准备阶段已经加载了最终的三模型策略
- A/r只是启用速度命令，不会触发策略重置

---

## 🎯 最可能的问题根源

### 问题1：gravity_offset导致观测偏差
**症状**：机器人姿态不稳，倾向某个方向
**原因**：训练时的重力投影可能没用offset，但部署时加了
**验证方法**：
```python
# 检查loco_config.json的gravity_offset是否与训练一致
"gravity_offset": [0.015, 0.0, 0.0]
```
**解决方案**：确认训练配置，如果不一致则修改为`[0.0, 0.0, 0.0]`

---

### 问题2：LocoCommandProcessor的初始vyaw=0.2
**症状**：机器人启动后会原地缓慢旋转
**原因**：`adjust=True`时返回`[0.0, 0.0, 0.2]`
**验证方法**：观察机器人是否有原地旋转趋势
**解决方案**：
```python
# nested_locomotion.py:44行，改为
self.processed = [0.0, 0.0, 0.0]  # 初始所有命令为0
```

---

### 问题3：模型文件加载失败（静默）
**症状**：程序启动不报错，但动作异常
**原因**：ONNX模型加载可能部分失败
**验证方法**：
```bash
python3 -c "
import onnxruntime as ort
sess = ort.InferenceSession('tasks/locomotion/robots/k1/models/loco/k1_loco_base.onnx')
print('Input:', sess.get_inputs()[0].shape)
print('Output:', sess.get_outputs()[0].shape)
"
```
**解决方案**：检查三个ONNX文件完整性，对比sha256

---

### 问题4：速度命令处理器的耦合约束
**症状**：给vx命令但机器人不动
**原因**：`LocoCommandProcessor`有复杂的速度耦合限制
**示例**：
```python
# nested_locomotion.py:55-64
if vx > threshold:
    vy = _clamp(vy, low_y, high_y)
    wz = _clamp(wz, -max_yaw, max_yaw)
```
**验证方法**：打印处理前后的命令值
**解决方案**：降低初始vx测试，避免触发过多约束

---

## 🔧 推荐排查步骤

### 步骤1：验证模型加载（最优先）
```bash
cd /home/booster/Workspace/dancer_boosterdeploy
python3 << 'EOF'
import sys
sys.path.append('.')
from tasks.locomotion.robots.k1.loco import K1LocoTaskCfg
import onnxruntime as ort
from pathlib import Path

cfg = K1LocoTaskCfg()
for i, path in enumerate(cfg.policy.model_paths):
    full_path = Path('tasks/locomotion') / path
    print(f'Model {i}: {path}')
    print(f'  Exists: {full_path.exists()}')
    if full_path.exists():
        sess = ort.InferenceSession(str(full_path))
        print(f'  Input: {sess.get_inputs()[0].shape}')
        print(f'  Output: {sess.get_outputs()[0].shape}')
EOF
```

### 步骤2：添加调试日志
在`nested_locomotion.py:129`添加：
```python
print(f"[DEBUG] Raw cmd: {raw}, Processed: {processed}, Route: {route}")
```

### 步骤3：对比观测值
在`compute_observation()`返回前打印：
```python
print(f"[DEBUG] Gravity with offset: {observation[3:6]}")
print(f"[DEBUG] Velocity command: {observation[6:9]}")
```

### 步骤4：测试零命令行为
按X启动后，**不要按A/r**，观察机器人是否能保持站立姿态10秒。

### 步骤5：测试最小速度命令
按A/r后，按一次w（vx=0.1），观察是否有响应。

---

## 📋 配置验证清单

- [x] 实机PD增益与k1_walk完全一致
- [x] 默认姿态与k1_walk完全一致
- [x] 控制指令来源唯一（/joint_ctrl）
- [ ] 三个ONNX模型文件完整且可加载
- [ ] gravity_offset与训练配置一致
- [ ] LocoCommandProcessor初始状态合理
- [ ] 速度命令处理逻辑与预期一致
- [ ] 模型切换逻辑不会引起跳变

---

## 🚨 与k1_walk的核心差异总结

| 项目 | K1_WALK | K1_LOCO | 影响等级 |
|-----|---------|---------|---------|
| 实机PD增益 | 直接配置 | booster覆盖（值相同） | 🟢 无影响 |
| 默认姿态 | 固定值 | 固定值（相同） | 🟢 无影响 |
| 模型数量 | 1个PT | 3个ONNX | 🟡 需验证 |
| 观测gravity | 直接投影 | 投影+offset | 🔴 **高风险** |
| 速度命令 | 直接使用 | 经过处理器 | 🟡 中风险 |
| 初始vyaw | 0 | 0.2 (adjust时) | 🟡 中风险 |
| 模型切换 | 无 | 三选一 | 🟡 需验证 |

---

## 💡 快速修复建议

如果需要快速让机器人动起来，尝试以下最小改动：

### 修改1：移除gravity_offset
```python
# tasks/locomotion/robots/k1/loco.py:23
policy = K1NestedLocomotionPolicyCfg(
    # ... 其他配置保持不变
    gravity_offset=[0.0, 0.0, 0.0],  # 改为全0
)
```

### 修改2：修复初始vyaw
```python
# tasks/locomotion/nested_locomotion.py:44
self.processed = [0.0, 0.0, 0.0]  # 改为全0
```

### 修改3：强制使用base模型（调试用）
```python
# tasks/locomotion/robots/k1/loco.py:59
policy = K1NestedLocomotionPolicyCfg(
    # ... 其他配置
    forced_route=0,  # 添加此行，强制base模型
)
```

---

## 📞 需要进一步确认的信息

1. **训练时是否使用了gravity_offset=0.015?**
2. **三个模型是否都在同样的观测下训练?**
3. **实机测试时的具体现象是什么？**
   - 完全不动？
   - 动作很小？
   - 姿态不稳？
   - 原地旋转？
4. **是否有报错日志？**
5. **`/low_state`和`/joint_ctrl`的发布频率是否正常？**

---

## 附录：关键代码位置

- 配置定义：`tasks/locomotion/robots/k1/loco.py`
- 三模型策略：`tasks/locomotion/nested_locomotion.py`
- 速度处理器：`tasks/locomotion/nested_locomotion.py:24-93`
- 实机控制器：`booster_deploy/controllers/booster_robot_controller.py`
- 增益覆盖：`booster_deploy/controllers/controller_cfg.py:42-52`
