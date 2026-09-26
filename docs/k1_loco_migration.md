# K1 loco 三模型迁移说明

本任务将 `dancer-robocupdemo/models/loco/` 的三个 ONNX 和配套运控逻辑迁入
`dancer_boosterdeploy`，注册为 `k1_loco`。原 `k1_walk` 任务继续保留。

## 运行环境和启动

运行链路为：

```text
速度命令 → 命令处理 → base/side/turn 选择 → 690 维观测
        → ONNX Runtime → 20 维动作 → 22 维关节目标
        → deploy 的 MujocoController → MuJoCo / booster_assets K1
```

仿真不依赖 demo 的 brain、ROS 2 shim、比赛服务器或 `dancer-rcssservermj`。
模型已复制到本仓库，运行时也不读取相邻 demo 目录。
仿真仍使用 `{BOOSTER_ASSETS_DIR}/robots/K1/K1_22dof.xml`，没有迁入足球场景。

当前 Windows 工作区已创建 `.venv`，复用本机已有 PyTorch/NumPy，另外安装
ONNX Runtime、MuJoCo、SciPy 和 `booster_assets`。在本仓库根目录执行：

```powershell
.\.venv\Scripts\python.exe -X utf8 scripts/deploy.py --list
.\.venv\Scripts\python.exe -X utf8 scripts/deploy.py --task k1_loco --mujoco
```

仿真窗口启动后，在**终端**输入三个数并回车，例如：

```text
0.4 0 0       # 前进
0 0.2 0       # 左侧移
0 0 0.4       # 左转
-0.2 0 0      # 后退
0 0 0         # 停止速度命令
```

实际输入时只输入数字，不输入注释。平移单位 m/s，转向单位 rad/s。
点击 MuJoCo 窗口后按空格暂停/继续；关闭窗口结束仿真。
MuJoCo 终端输入方式与实机的手柄/键盘遥控方式不同。

在其他 Linux/WSL 环境：先按仓库 README 安装 `requirements.txt`，再按官方
`booster_assets` README 安装其 Python helper，然后运行同样的 `--task k1_loco --mujoco`。
Windows 仿真需要 torch、numpy、scipy、onnxruntime、mujoco 和 booster_assets；
`requirements.txt` 中的 Linux 手柄依赖 evdev 不用于该仿真入口。

## 迁移来源与文件

迁移时源仓库 HEAD 为 `45c663bedc8a1a9eba7ef9ffd4e931dcc50e9026`；
模型按迁移时工作区实际文件复制，SHA-256 记录于
`tasks/locomotion/robots/k1/models/loco/sha256.json`，并已与源文件核对一致。

| 文件 | 作用 |
|---|---|
| `tasks/locomotion/robots/k1/models/loco/*.onnx` | 原样复制 base、side、turn |
| `tasks/locomotion/robots/k1/loco_config.json` | 原 JSON 的完整 `loco` 字段 |
| `tasks/locomotion/nested_locomotion.py` | Python 命令处理、模型选择、共享历史策略 |
| `tasks/locomotion/robots/k1/loco.py` | 从 JSON 构造策略和仿真参数；实机 PD 沿用 walk，注册 `k1_loco` |
| `booster_deploy/utils/policy_runner.py` | 修复动态输出形状下的旧数组缓存，以及 TorchScript 中文路径加载 |
| `booster_deploy/controllers/booster_robot_controller.py` | 实机 PD 覆盖、遥控输入、loco 准备到正式运行的连续交接 |
| `booster_deploy/controllers/controller_cfg.py` | 可选实机 PD 参数及长度、数值检查 |
| `booster_deploy/controllers/mujoco_controller.py` | Windows 中文模型路径和终端输入兼容；三轴输入限幅 |
| `tests/test_k1_loco.py` | 命令、推理、映射和启动配置回归验证 |
| `tests/test_k1_loco_robot.py` | 实机控制器离线测试：状态回调、遥控、三模型推理、关节发布和交接 |
| `scripts/check_k1_loco.py` | 无窗口 MuJoCo 闭环验证 |

核心对应关系：源 `loco_policy.cpp::processCmd()` → `LocoCommandProcessor.process()`；
`selectRoute()` → `select_route()`；`buildFrame()` 和 `step()` 的观测历史及后处理，
由新策略继承现有 `LocomotionPolicy` 并补充差异实现。
`scripts/deploy.py` 本来就递归扫描任务模块，因此无需修改入口即可发现新任务。

## 命令处理和路由

每 0.02 秒执行一次命令处理，按顺序保留源代码的非有限数处理、每轴变化率限制、
随前进速度变化的侧移/转向包络、最终限幅和耦合约束。
内部限速状态与最终命令分开保存，最终耦合限幅不反写内部状态。

- 最大命令 `[1.5, 0.4, 1.8]`，最小命令 `[-0.7, -0.4, -1.8]`。
- 正向变化率上限 `[1.3, 1.2, 2.0]`，负向变化率上限 `[1.6, 1.2, 2.0]`。
- 模型选择和观测使用同一个处理结果，每周期只处理一次。

以下条件使用处理后速度的绝对值，按顺序判断：

| 模型 | 条件 |
|---|---|
| side，编号 1 | `vy > 0.1 and vx < 0.2 and wz < 0.2` |
| turn，编号 2 | 未选 side，且 `wz > 0.1 and vx < 0.1 and vy < 0.1` |
| base，编号 0 | 其余情况 |

模型在初始化时加载，每周期只推理其中一个。调试固定模型可在构造控制器前设置
`cfg.policy.forced_route = 0/1/2`，默认 `None` 自动选择。本移植没有读取
`K1_LOCO_MODEL` 环境变量；用配置字段显式指定。
`policy.loco_adjust = True` 会注入 `(0, 0, 0.2)` 并选择 turn；默认关闭，reset 后关闭。
与源代码相同，显式固定模型的优先级高于 adjust。

## 观测、动作和 PD

三个模型输入为 float32 `[batch, 690]`，输出为 float32 `[batch, 20]`，运行 batch=1。
每帧 69 维，按以下顺序：

```text
角速度3，投影重力3，处理后命令3，关节位置偏差20，关节速度20，上一次动作20
```

保留 10 帧，从旧到新排列；首帧重复填满。关节速度缩放为 0.1，重力观测加
`[0.015, 0, 0]`，观测裁剪到 ±100。deploy 状态已是弧度制，不重复做角度转换。
关节顺序根据 JSON 的 `body_dof_indices_20` 和 `webots_to_lab_idx` 生成名称列表。

三个模型共享观测历史、上一动作和目标位置滤波状态，路由切换不 reset。
模型输出先裁剪到 ±100，再作为下一周期的上一动作；这一点以源 `step()` 的赋值为准。
动作按关节名称映射回 22 维：

```text
target = default + 0.25 * mapped_action
filtered = 0.8 * target + 0.2 * previous_filtered
```

新任务关闭旧 `k1_walk` 的右肘额外动作修正。默认姿态及仿真 KP/KD 从 JSON 读取：

| 关节组 | KP | KD |
|---|---:|---:|
| 头部、双臂 | 20 | 2 |
| 髋、膝 | 100 | 2 |
| 踝 pitch/roll | 50 | 1 |

这里的头部采用源 loco JSON 基线并保持默认姿态；没有迁入 demo 的上层头部追踪，
demo 的 `SimMotion` 会在策略之后另行覆盖头部命令及增益。
关节力矩限制沿用 deploy 原 K1WalkControllerCfg；它不属于源 loco JSON。
MuJoCo 继续使用 deploy 默认的 10 倍 decimation：策略 50 Hz、物理步长 0.002 秒。

## 实机部署

实机与仿真共用 `K1NestedLocomotionPolicy`，没有另写一套模型选择逻辑。
实机链路为：

```text
/low_state 的 IMU、串联关节状态 + 手柄/键盘的归一化速度命令
  → BoosterRobotController（命令换算为 m/s、rad/s）
  → 命令限幅与斜坡 → base/side/turn → 690 维观测 → 20 维动作
  → 22 个关节目标位置及实机 Kp/Kd → /joint_ctrl
```

### 启动

将仓库代码和 `tasks/locomotion/robots/k1/models/loco/` 下的三个 ONNX 文件、
配套 `loco_config.json` 一起复制到机器人；不要复制 Windows 的 `.venv`。
在机器人的项目根目录、与预装 ROS 2 匹配的 Python 环境中运行：

```bash
# 首次创建环境时执行；已有匹配的环境则跳过创建。
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
source /opt/booster/BoosterRos2Interface/install/setup.bash
python scripts/deploy.py --task k1_loco
```

这里不加 `--mujoco`。实机策略入口不需要 `booster_assets` 或 MuJoCo 窗口。
依赖文件包含仿真包，但实机控制链路不导入 MuJoCo。

默认 `prepare_mode="walking"` 的操作顺序：

1. 按 `X`（键盘 `x`）：等待有效状态、按当前关节位置发送保持命令、进入 Custom。
   随后加载三个 ONNX，以零速度命令运行 base 准备策略，不读取旧 `k1_walk.pt`。
2. 按 `A`（键盘 `r`）：解除速度输入屏蔽；继续使用同一个控制器、三个模型会话、
   10 帧历史、上一动作及滤波状态，不再重载或 reset。调试用 `forced_route`
   在准备阶段清除，在正式运行时恢复。
3. 使用左右摇杆或 `w/s`、`a/d`、`q/e` 控制速度，空格将输入命令清零。
   loco 的减速斜坡仍生效；清零输入不是立刻切换阻尼模式。
4. `Ctrl+C` 结束，按 `booster.exit_mode` 切换模式；默认 `walking`，也可用
   `--exit-mode damping` 指定阻尼模式。

实机键盘每次增减的是归一化输入 `0.1`；控制器再按任务速度上限换算。
正向满输入为 `1.5 m/s`，反向满输入为 `-0.7 m/s`，侧向为 `±0.4 m/s`，
转向为 `±1.8 rad/s`；然后继续经过 loco 的组合速度限制。
模型切换时日志显示 `K1 loco policy=base/side/turn` 和处理后的命令。

`prepare_mode="standing"` 仍先插值到准备姿态，按 `A/r` 后启动三模型。
其他任务沿用各自原来的准备策略和交接方式。

### 实机 PD 来源

`k1_loco` 在 `booster.joint_stiffness/joint_damping` 中复制当前
`K1WalkControllerCfg` 的运行参数。Portal 在构造实机控制器前应用这些覆盖；
仿真仍使用 JSON 中的 PD，不会因实机覆盖发生改变。

| 关节组 | 实机 Kp/Kd（沿用 walk） | 仿真 Kp/Kd（loco JSON） |
|---|---|---|
| 头部 | 4 / 1 | 20 / 2 |
| 双臂 | 20 / 2 | 20 / 2 |
| 髋、膝 | 100 / 2 | 100 / 2 |
| 踝 pitch/roll | 65 / 1 | 50 / 1 |

关节顺序、默认姿态和配置中的力矩限制在两个任务中一致。这里只复用 walk 的 PD，
不会启用 walk 的模型、观测排列或右肘动作修正。模式切换时的首次位置保持仍使用
K1 配置中的 `prepare_state.stiffness/damping`，与策略运行 PD 分开。
`effort_limit` 在现有实机发布函数中不做 Python 侧力矩裁剪。

当前仓库未提供这些 walk 参数的实机验证记录；沿用它们不等于 loco 已通过实机验证。
本次未连接机器人，电机侧阻尼、固件模式切换、ROS/DDS 传输与原有多进程运行方式
仍需在目标机器人上核对。离线检查不验证这些接口或机械稳定性。

## 验证与结果

```powershell
.\.venv\Scripts\python.exe -X utf8 -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -X utf8 scripts/check_k1_loco.py --seconds 5 --output logs/k1_loco_smoke.json
.\.venv\Scripts\python.exe -X utf8 scripts/check_k1_loco.py --seconds 5 --real-robot-gains --output logs/k1_loco_walk_pd_smoke.json
```

9 项回归检查已通过，覆盖：源测试中的速度斜坡/耦合实例、路由边界、adjust、非有限数、
三个真实模型的连续推理、根据源索引与方程独立构造的观测/动作参考、跨模型历史连续性、
reset、新旧任务准备配置、模型 SHA-256，以及 Windows 终端输入和三轴限幅。
数值参考对照容差为 3e-5；没有编译运行源 C++。

实机接入另增加 7 项检查，合计 16 项通过。使用实际实机控制器代码及三个真实 ONNX，
以内存消息/发布器替代 ROS 和共享内存：覆盖 `/low_state` 回调、RPY 转四元数、
归一化遥控缩放、零命令准备、三模型自动切换、22 关节输出及 PD、A/r 连续交接、
无效 PD 拒绝和停止后不继续发布。160 个连续策略周期与共享仿真策略的观测、
关节目标逐步对照，容差 3e-5；这不是 ROS 网络或实机闭环测试。

`--real-robot-gains` 仅让 MuJoCo 使用实机配置的 PD 做闭环检查，不连接机器人，
也不模拟固件内部的并联电机转换。
沿用 walk PD 的 40 秒检查也已通过全部 8 个阶段，最低基座高度约 0.528 m，
最低直立余弦约 0.996，报告见 `logs/k1_loco_walk_pd_smoke.json`。
原 loco 仿真 PD 的本次回归报告见 `logs/k1_loco_sim2real_regression.json`。

2026-09-26 已完成连续 40 秒仿真（2000 个策略周期、20000 个物理步）：
站立、前进、左侧移、左转、后退、右侧移、右转、停止，各 5 秒。
8 个阶段均满足有限值、直立和末尾路由检查，最低基座高度约 0.532 m，
最低直立余弦约 0.995。报告见 `logs/k1_loco_smoke.json`。
该检查验证基本闭环和切换，不代表全速度范围、复杂地形或精确速度跟踪已经验证。

本次测试使用 ONNX Runtime 1.30.0、MuJoCo 3.14.0；BoosterAssets 提交为
`3c2dfa99e09beddf092e0d6521dbbcec7e7903ed`。资产源码保存在本地
`.venv/booster_assets`（仅检出 K1 及 Python helper，未作为项目文件提交）。
当前 Python 3.11 在中文 Windows 路径下读取 editable 安装的 `.pth` 存在编码问题，
本地 `.pth` 已改用 ASCII 转义的导入行。新机器安装资产时，优先使用纯 ASCII 路径。
MuJoCo 的中文路径读取则已在控制器内兼容，并在加载结束后恢复工作目录。
