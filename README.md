# Booster Deploy（Booster 部署框架）

K1 三模型行走任务：`python scripts/deploy.py --task k1_loco --mujoco`。
模型来源、迁移实现、Windows 启动方式与验证结果见 [K1 loco 迁移说明](docs/k1_loco_migration.md)。
K1 实机三模型部署：在机器人 ROS 2 环境执行 `python scripts/deploy.py --task k1_loco`，
具体启动步骤、PD 参数和离线验证范围见 [K1 loco 实机部署](docs/k1_loco_migration.md#实机部署)。

中文源码导读：[代码解读与运控数据流](docs/代码解读与运控数据流.md)，包含阅读路线、实机与仿真控制链路、策略观测与关节映射、启动切换流程及调试定位。

Booster Deploy 是轻量级策略部署框架，支持在 Booster 实机上运行控制策略（仿真到实机），也支持在 MuJoCo 中运行策略（仿真到仿真）。框架借鉴 IsaacLab 的模块化设计，使同一套策略执行流程适用于仿真和实机。


## 环境要求

| 环境 | 说明 |
|-------------|-------|
| Booster 固件 ≥ v1.7.2 | 实机部署需要。 |
| Python 3.10 及以上 | 机器人上已安装。 |
| ROS 2 与 `booster_interface` | `/low_state`、`/joint_ctrl` 和 RPC 接口通过 DDS 通信，机器人上已安装所需组件。 |
| MuJoCo | 本地运行仿真时安装。 |


## 运行部署任务

### K1 视觉传球

本地 sim2sim 使用与实机相同的 `k1_pass` 策略和 50 Hz 关节控制链路：

```bash
python scripts/deploy.py --task k1_pass --mujoco
python scripts/deploy.py --task k1_pass --mujoco --ball-pos 0.5 0.2
```

仿真场景使用 `assets/soccer` 中的球场与足球（来自 `dancer-rcssservermj` 的
`resources/environments/soccer`）。默认球心位于世界坐标 `(0.8, 0, 0.11)` 米，
`--ball-pos` 修改水平初始位置。策略从 MuJoCo 的机器人和足球位姿计算机器人坐标系下的球位置，
仅在球心投影落入当前头部相机视野时更新网络的球坐标；获得过有效球后，出视野继续使用最后一次原始坐标推理，头部按实机的丢球保持与扫描逻辑找球。
默认仿真相机内参取自 `vision_ws/src/vision/config/vision.yaml`，图像尺寸暂设为 512×480，
可通过任务配置中的 `mujoco.pass_camera` 修改。此过滤不模拟遮挡或识别误差。
球可与机器人碰撞。此入口无需相机、ROS 或视觉节点。
窗口中按空格暂停/继续。
无窗口对照测试（持续可见、开局不可见、运行 1 秒后丢球）：
`python scripts/check_k1_pass_visibility.py --output logs/k1_pass_visibility.json`。

在机器人上加载 Booster ROS 2 接口，编译并加载本仓库 `vision_ws` 后，运行
`python scripts/start_k1_pass.py --vision-config /opt/booster`。
入口负责启动或复用视觉、检查相机及 `/head_pose`，再进入 deploy。
头部在准备阶段开始找球/跟球，并通过现有 `/joint_ctrl` 合入。
先用 `--vision-only` 检查球坐标，再用 `--head-only` 验证头部与零速度 loco，最后启用 pass。
完整操作和验证步骤见 [相机与头部测试指南](docs/k1_pass_camera_head_testing.md)。
通过 SSH 在电脑浏览器看实时识别画面：转发 `8080:127.0.0.1:8080`，
在机器人运行 `/usr/bin/python3 scripts/view_vision.py`，电脑打开 `http://127.0.0.1:8080`。
查看器只读订阅现有视觉；支持 NV12、识别框及原图切换，详细步骤见上述指南。

任务从有效的 `Ball` 检测结果中选取置信度最高的球，读取其机器人坐标系下的 `position_projection`，将机器人指向球的方向设为传球方向，并以固定力度 2 运行 `k1_passing_policy_2.onnx`。网络输入按 demo 仿真运动层保留最后一次有效原始球坐标：漏检或视觉/头部位姿超时后仍继续推理，观测历史与交替标志连续；新有效球到来时更新坐标。缓存不设消失帧数或时间上限，`ball_max_age=0.5` 仅用于接纳新观测；策略 reset 不清除缓存，新建策略实例才从无球开始。首次获得有效球前仍沿用现有实测关节姿态保持。头部仍使用实时检测，不使用身体策略缓存的旧球框。Kp/Kd 和传球模型来自 RoboCup 示例工程；策略推理和 `joint_ctrl` 控制循环沿用本工程的 K1 行走部署流程。

### K1 视觉射门

shoot 使用与上述 pass 相同的相机、足球场景、50 Hz 控制循环、球坐标缓存和头部跟踪。通过 `--shoot-policy` 固定选择 RoboCup 示例工程的五个 shoot 模型之一：`2`、`264`、`192`、`0109_0`、`0109_2`；缺省为 `2`。参数与模型文件后缀相同，例如 `264` 对应 `k1_shoot_policy_264.onnx`。策略把机器人指向球的方向作为测试方向，并应用所选模型在比赛配置中的球位置及方向偏移。缓存的是偏移和归一化之前的原始坐标，丢球时不会重复叠加偏移。比赛端以 power 6 选择 shoot 动作族；shoot 模型实际接收的速度观测固定为 1.0。pass 继续固定使用 power 2。本次仅对齐球观测缓存，不迁移 brain、模型自动路由或其他控制流程。

```bash
# 本地 sim2sim；也可加 --ball-pos 0.5 0.2
python scripts/deploy.py --task k1_shoot --mujoco --shoot-policy 264

# 实机 sim2real：先用 --vision-only 和 --head-only 检查，再启动策略
python scripts/start_k1_shoot.py --vision-only --vision-config /opt/booster
python scripts/start_k1_shoot.py --head-only --vision-config /opt/booster
python scripts/start_k1_shoot.py --vision-config /opt/booster --shoot-policy 264
```

实机按与 pass 相同的 X/A 控制流程启动零速度行走准备与 shoot。实机运行需要机器人 ROS 2 和本仓库的 `vision_ws`。上述自动化测试可用 `python -m unittest tests.test_k1_shoot tests.test_k1_shoot_sim` 运行。

### Python 环境

创建并激活本地虚拟环境，然后安装依赖：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

在 Debian/Ubuntu 上，如有需要，先安装 `python3-venv`。在机器人上，应先激活 `.venv`，再加载 ROS 2 环境；`booster_interface` 由机器人提供：

```bash
source .venv/bin/activate
source /opt/booster/BoosterRos2Interface/install/setup.bash
```

### 添加和查看任务

1. 在 `tasks/` 下为任务创建子目录。
2. 实现 `Policy`/`PolicyCfg`，并在 `ControllerCfg` 中指定该策略。
3. 将策略模型文件放在 `models/` 下，并在配置中填写路径。
4. 将 `ControllerCfg` 注册到任务注册表；注册方式可参考现有任务。
5. 查看全部可用任务：

   ```bash
   python scripts/deploy.py --list
   ```

### 策略推理后端

框架根据模型文件后缀自动选择推理后端：

- `.pt`、`.jit`、`.torchscript`：TorchScript
- `.onnx`：ONNX Runtime，使用 CPU 执行提供程序

部署环境中需要安装 `onnxruntime`。

### 运行仿真到仿真（MuJoCo）

- 下载并安装 BoosterAssets：
   - 克隆包含 Booster 机器人模型和资源的 [booster_assets](https://github.com/BoosterRobotics/booster_assets) 仓库。
   - 按仓库说明安装 `booster_assets` Python 辅助包。

- 在已激活的虚拟环境中安装 Python 依赖：
   ```
   python -m pip install -r requirements.txt
   ```

- 在 MuJoCo 中启动任务：
   ```bash
   python scripts/deploy.py --task <TASK_NAME> --mujoco
   ```

### 运行仿真到实机（实体机器人）

**注意**：继续操作前，确认机器人已安装 v1.4 或更高版本的 [Booster 固件](https://booster.feishu.cn/wiki/E3q5wF5SnitXZgkY18Uc8odBnXb)。

- 在本地完成仿真到仿真测试后，将项目复制到机器人。

- 在机器人已激活的虚拟环境中安装 Python 依赖：
   ```
   python -m pip install -r requirements.txt
   ```

- 通过 SSH 登录机器人，并加载提供的初始化脚本以启用 ROS 2 环境：
   ```bash
   source /opt/booster/BoosterRos2Interface/install/setup.bash
   ```

- 在机器人上启动任务，并按命令行提示操作：
   ```bash
   python scripts/deploy.py --task <TASK_NAME>
   ```

#### 实机 PD 阻尼参数（`Kd`）

可通过 `booster.joint_stiffness` 和 `booster.joint_damping` 覆盖仅用于实机的增益，数值顺序须与 `robot.joint_names` 一致。未设置覆盖值时，使用 `robot` 中对应的增益。MuJoCo 始终使用 `robot.joint_stiffness` 和 `robot.joint_damping`。

`k1_loco` 在实机上以现有 `k1_walk` 增益为基准，在仿真中保留原始 loco 增益。这不代表这些增益已经过三模型 loco 策略的实机验证。

对于并联驱动关节，应用 `booster.joint_damping` 覆盖值后，`robot.joint_damping` 会直接发送给电机，因此不要直接沿用训练仿真器中的 `Kd`。电机侧数值可按下式计算：

```text
Kd = 2 * zeta * J_eq * (2 * pi * f_n)
```

其中，`J_eq` 为并联驱动关节的等效转动惯量，`f_n` 为固有频率，`zeta` 为阻尼比。


#### 控制器退出模式

`booster.exit_mode` 控制自定义控制器退出后机器人切换到的模式。此设置适用于固件支持相应 DDS RPC 模式切换接口的机器人：

- `"damping"`：切换到阻尼模式
- `"walking"`：切换到行走模式（默认）

可在任务控制器配置中设置该值，例如：

```python
booster = BoosterRobotControllerCfg(exit_mode="damping")
```

如需以阻尼模式退出，也可在启动时覆盖任务配置：

```bash
python3 scripts/deploy.py --task <TASK_NAME> --exit-mode damping
```

Python 配置中也接受 `"walk"`，它是 `"walking"` 的别名。


#### 机器人准备模式

`robot.prepare_mode` 控制按下 `X` 进入 `Custom` 模式后的准备流程。可分别在 T1、T2、K1 的机器人配置中设置：

- `"walking"`（默认）：从 `/low_state` 读取当前关节位置，使用机器人的 `prepare_state` `kp/kd` 发布一次位置保持指令，切换到 `Custom` 模式，然后启动与机器人对应的行走策略，并将速度指令全部置零。按遥控器 `A`（或键盘 `r`）结束准备阶段，启动 `--task` 指定的任务。对于 `k1_loco`，准备阶段已运行所选的三模型策略；按 `A`/`r` 后只开放速度指令，不重新加载模型，也不重置观测历史和动作滤波状态。
- `"standing"`：发布当前关节位置保持指令，切换到 `Custom` 模式，再用约一秒插值过渡到配置的 `prepare_state.joint_pos`。按 `A`/`r` 启动所选任务策略。
- `"hold"`：发布当前实测关节位置保持指令并进入 `Custom`，按 `A`/`r` 启动任务，不做站姿插值。`k1_recovery` 使用此准备方式。

可在机器人配置中设置准备模式，例如：

```python
robot = T2_31DOF_CFG.replace(prepare_mode="walking")
```

### 遥控器与键盘

部署程序支持遥控器和键盘输入：

- GameSir 遥控器：自动检测。
- Booster 遥控器：通过 `/remote_controller_state` 接入。
- 键盘：可直接使用。

<table>
  <tr>
    <th align="left">GameSir</th>
    <th align="left">Booster 遥控器</th>
  </tr>
  <tr>
    <td valign="top"><img src="docs/images/gamesir.jpg" alt="GameSir 遥控器" width="320"></td>
    <td valign="top"><img src="docs/images/booster_remote.jpg" alt="Booster 遥控器" width="320"></td>
  </tr>
</table>

两种遥控器均使用左摇杆控制前进、后退和横移，右摇杆控制旋转；按 `X` 进入 `Custom` 模式，按 `A` 启动强化学习（RL）策略。

| 操作 | 功能 |
|---------|--------|
| 左摇杆前／后 | 增大／减小前向速度（`vx`） |
| 左摇杆左／右 | 增大／减小横向速度（`vy`） |
| 右摇杆左／右 | 向左／右旋转（`vyaw`） |
| 遥控器 `X` | 进入 `Custom` 模式 |
| 遥控器 `A` | 启动 RL 策略 |

键盘操作：

| 按键 | 功能 |
|-----|--------|
| `w` / `s` | 将 `vx` 增大／减小 `0.1` |
| `a` / `d` | 将 `vy` 增大／减小 `0.1` |
| `q` / `e` | 将 `vyaw` 增大／减小 `0.1` |
| `x` | 进入 `Custom` 模式 |
| `r` | 启动 RL 策略 |
| 空格键 | 将全部速度指令置零 |

当 `prepare_mode="walking"` 时，按 `X` 启动零速度指令的行走准备，按 `A`/`r` 启动所选任务策略。当 `prepare_mode="standing"` 时，按 `X` 先进行约一秒的 `prepare_state.joint_pos` 姿态过渡，再按 `A`/`r` 启动所选任务策略。按 `Ctrl+C` 停止部署程序。


### K1 倒地恢复

`k1_recovery` 使用 demo 的 FDR 模型和仰卧/俯卧轨迹，观测传感器来源与
`k1_loco` 共用同一读取函数。实机入口：

```bash
python scripts/deploy.py --task k1_recovery
```

按 X/x 保持当前实测姿态，按 A/r 开始恢复。成功后保持末帧目标；默认最多重试两次，
失败或退出时进入 damping。模型观测、仿真命令和验证范围见
[recovery 迁移说明](docs/k1_recovery_migration.md)。

## 仓库结构

```
booster_deploy/
├─ booster_deploy/           # 控制器、策略及工具模块
│  └─ robots/                # 机器人模型配置
│     ├─ k1.py               # K1 配置
│     ├─ t1.py               # T1 23 自由度配置
│     ├─ t2.py               # T2 31 自由度配置
│     ├─ __init__.py         # 对外导出的配置
│     └─ booster.py          # 兼容旧代码的导入路径
├─ scripts/                  # 启动脚本（deploy.py）
├─ tasks/                    # 任务注册表和配置
└─ requirements.txt          # Python 依赖
```

主要模块：

- `booster_deploy/`：核心模块，为 MuJoCo 仿真和实体机器人提供统一接口。实机通信使用 ROS 2 DDS，包括 `/low_state` 订阅者、`/joint_ctrl` 发布者和 RPC 客户端。
- `booster_deploy/robots/`：机器人配置模块。每种机器人都有独立的 `RobotCfg` 配置：
    - `k1.py`：`K1_CFG`
    - `t1.py`：`T1_23DOF_CFG`
    - `t2.py`：`T2_31DOF_CFG`
    - 关节名称与机身部件名称
    - 默认关节位置
    - 默认关节刚度（`joint_stiffness`）和阻尼（`joint_damping`）
    - 力矩上限
    - 用于加载 MuJoCo 模型的 `mjcf_path`
    - `prepare_state`：进入 `Custom` 模式时使用的准备姿态、刚度和阻尼

  可从软件包或对应机器人的模块导入配置：

  ```python
  from booster_deploy.robots import K1_CFG
  # 等价写法：
  from booster_deploy.robots.k1 import K1_CFG
  ```

  为兼容现有部署代码，仍可通过 `booster_deploy.robots.booster` 导入。

- `tasks/`：用户任务的定义与实现。每个任务模块包含：
    - 实现推理逻辑的 `Policy`/`PolicyCfg` 类；
    - 包含策略配置的 `ControllerCfg` 类；
    - 将 `ControllerCfg` 实例注册为任务的代码。

  典型任务目录示例：

   ```text
   tasks/my_task/
   ├─ __init__.py        # 调用注册函数注册任务
   ├─ task.py            # Policy 和 ControllerCfg 的实现
   ├─ models/            # 可选：策略模型文件
   └─ motions/           # 可选：动作原语或动作数据
   ```
