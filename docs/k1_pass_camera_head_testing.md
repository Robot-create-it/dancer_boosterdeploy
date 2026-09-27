# K1 pass：相机、视觉与头部控制迁入和测试

本次迁入保留 deploy 的 50 Hz 运控：`update_state → policy_step → ctrl_step → /joint_ctrl`。
头部目标在策略输出之后覆盖两个头关节，20 个身体关节仍由 loco/pass 控制。
pass 保持 10×71 维观测、power=2、`dir=atan2(ball_y, ball_x)`。

## 实现与依赖

- `vision_ws/src/vision`、`vision_interface` 来自 dancer-robocupdemo，源提交
  `da8d68db78886de3eb35019a83839e78b753531e`。识别、投影及消息定义沿用 demo。
- `scripts/start_k1_pass.py` 管理自己启动的相机/视觉进程，退出时只清理这些进程。
  已有视觉节点会复用；其标定必须与 `--vision-config` 相同。
- 默认 D-Robotics 相机由机器人的平台服务启动；本仓库未包含它的驱动或服务实现。
  程序检查其图像/深度，不重启平台服务。RealSense 可通过
  `--camera-driver realsense` 启动 `realsense2_camera`。
- `/head_pose` 必须来自机器人平台，并在 Custom 模式下持续反映实测头部位姿。
  本迁入没有生成近似的 `/head_pose`。缺少它时启动检查会明确报错，pass 也不会消费球坐标。
- 视觉节点默认不加载分割模型，关闭图像存盘；修复了 demo 关闭存盘后访问空日志对象的问题。
  图像与头部位姿/深度相差超过 0.2 秒时，不输出该帧检测。
- 头部跟踪沿用 demo 的图像偏差和 3.5 平滑系数；每张图只更新一次目标。
  短时丢球保持，超过 0.5 秒开始六点扫描。
  扫描点间隔至少 800 ms，并等待限速后的命令到达该点再切换，避免扫不到两侧。
  默认 yaw 范围 ±1.0 rad、pitch 范围 0.2–0.85 rad、速度限制 0.6 rad/s。
  这些是保守的软件范围，需在第 5 步核对本机限位及实际运动方向。
  头部 Kp=20、Kd=2，准备阶段与 pass 阶段相同。
  两个头关节的外部控制 `weight=1`；身体关节沿用原 deploy 的 weight。
- 球坐标、检测框来自同一个 Ball；低于 60 分、无效坐标、延迟或重复检测帧不会刷新球的位置。
  新图明确无球会立即清除当前实时检测。图像或 `/head_pose` 超过 0.5 秒不更新时，不再接纳新的球坐标。身体策略另行保留已获得的最近有效原始 XY，供网络继续推理；头部不会把该缓存当作新检测。

## 1. 离线测试（不连接机器人控制）

在仓库根目录、安装好 requirements 的 Python 环境中执行：

```bash
python -m unittest discover -s tests -v
python -m compileall -q scripts booster_deploy tasks
```

检查：71 维观测和原 loco 回归通过；头部方向、限速、丢球、过期图像、身体关节保持、
切换时保留 tracker、head-only 不响应 A/r 等测试通过。
缺少 MuJoCo/BoosterAssets 的两个已有控制台测试允许显示 skipped。

## 2. 在机器人上编译视觉包

首先激活部署 Python 环境，加载机器人原有的 ROS 2/Booster 接口环境。
下面假设当前目录为本仓库根目录：

```bash
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source /opt/booster/BoosterRos2Interface/install/setup.bash
cd vision_ws
colcon build --symlink-install --packages-select vision_interface vision \
  --cmake-args -DBUILD_TESTING=OFF -DBUILD_CALIBRATION=OFF
source install/setup.bash
cd ..
ros2 pkg prefix vision
python -c 'from vision_interface.msg import Detections; from booster_interface.msg import LowState'
```

`ros2 pkg prefix vision` 应指向本仓库 `vision_ws/install/vision`。
构建需要 CUDA/nvcc、TensorRT、OpenCV、PCL、yaml-cpp、image_transport、tf2。
随包迁入两份检测 engine；TensorRT engine 必须与机器人的硬件和 TensorRT 版本兼容。
配置中的模型不存在时程序会报出完整路径，不会偷偷改用另一份模型。

每个新终端都需要加载同一套 ROS/DDS 环境和 `vision_ws/install/setup.bash`。
已编译好视觉包时，在仓库根目录依次运行：

```bash
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source /opt/booster/BoosterRos2Interface/install/setup.bash
source vision_ws/install/local_setup.bash
python -c 'from booster_deploy.utils.robot_runtime import require_robot_interface; require_robot_interface(); from vision_interface.msg import Detections; print("ROS interfaces OK")'
```

这里使用 `local_setup.bash` 将视觉包叠加到刚加载的 Booster 环境。
若出现 `ModuleNotFoundError: No module named 'booster_interface'`，说明当前 Python
没有加载机器人运控接口；即使相机和检测话题正常，也仍需加载 Booster 的 setup。
该接口由机器人提供，无需通过 pip 安装。实机入口会在启动视觉或加载策略之前检查它。

使用机器人自身 `/opt/booster/vision.yaml` 和可选的 `vision_local.yaml`；不要直接将仓库里的
示例外参当作本机标定。程序会合并两个文件，从中取得相机类型、fx/fy，图像宽高来自实时图像。

## 3. 只启动视觉（不下发关节命令）

```bash
python scripts/start_k1_pass.py --vision-only --vision-config /opt/booster
```

默认 D-Robotics + use_depth=true 时，检查这四个输入持续更新：

```text
/StereoNetNode/rectified_image
/StereoNetNode/stereonet_depth
/head_pose
/booster_vision/detection
```

预期：先显示 `Inputs ready` 和图像分辨率，然后每秒打印机器人坐标系的球位置（米）。
没有球时显示 `ball=NONE/STALE`。将球放到机器人前方约 0.5–1 米处，左右移动：
前方 x 应为正，左侧 y 应为正，距离应接近实测值。

Ctrl+C 退出。脚本启动的视觉会退出；复用的视觉和平台相机不会被终止。
启动日志：`logs/pass_vision.log`，自行启动 RealSense 时还有 `logs/pass_camera.log`。

如果输入已由别的终端运行，可单独执行只读检查：

```bash
python scripts/start_k1_pass.py --check-only --vision-config /opt/booster
```

这条命令不会启动相机、视觉或运动进程。必须收到多帧新鲜消息才通过，不是仅检查话题名称。

使用 RealSense 时，在上述启动命令中增加 `--camera-driver realsense`，并提供其自身标定目录。
该模式使用 `/boostercamera/head/color/image_raw` 和
`/boostercamera/head/aligned_depth_to_color/image_raw`，启动入口会同步配置视觉和 deploy 的话题。

## 4. 检查头部位姿输入（仍不由本工具下发运动）

在视觉已运行的终端环境下：

```bash
ros2 topic hz /head_pose
ros2 topic echo /head_pose --once
ros2 topic hz /booster_vision/detection
```

确认 `/head_pose` 不是固定的占位消息。若它没有发布源，先恢复本机平台的头部位姿服务；
不能通过伪造固定姿态绕过检查，否则球坐标会随转头发生错误。
图像 timestamp 必须与 ROS 时钟一致。`Missing/stale inputs` 会列出具体缺失的话题。

## 通过 SSH 在电脑浏览器实时看识别画面

使用 `scripts/view_vision.py`，无需 X11、rqt 或机器人本地桌面。
它只订阅图像和已有检测，不启动第二个识别模型，也不发送机器人控制命令。
默认网页服务只监听机器人 `127.0.0.1:8080`，通过 SSH 转发访问。

1. 在**你的电脑**终端（Windows PowerShell、macOS 或 Linux 均可）执行，替换 ROBOT_IP：

   ```bash
   ssh -L 8080:127.0.0.1:8080 booster@ROBOT_IP
   ```

2. 登录后在这个终端的**机器人 shell**中执行：

   ```bash
   cd ~/Workspace/dancer_boosterdeploy
   source /opt/ros/humble/setup.bash
   source vision_ws/install/setup.bash
   /usr/bin/python3 scripts/view_vision.py --vision-config /opt/booster
   ```

3. 保持终端运行，在**你的电脑浏览器**打开 `http://127.0.0.1:8080`。
   绿色框为 Ball，显示置信度和机器人坐标；蓝色框为其他目标。
   点击“隐藏识别框”可比较原始画面。顶部显示最近图像/检测接收时间。
   查看器按图像与检测的时间戳匹配；没有匹配结果时显示原图，不把旧框画到新图上。

如果 head-only/pass 测试正在运行，直接复用它的检测话题即可。
如果没有视觉节点，在另一个机器人终端按第 3 步启动 `--vision-only`，再打开查看器。
Ctrl+C 只停止查看器，原来的运控和视觉进程继续运行。

图像解码使用系统 OpenCV，支持 K1 的 NV12 和常见 RGB/BGR 格式。
当前 `.venv` 未安装 `cv2`，因此查看器明确使用 `/usr/bin/python3`，不依赖运控虚拟环境。
默认 10 FPS、JPEG 质量 80；带宽较低时可使用 `--fps 5 --quality 60`。
如果电脑的 8080 端口已占用，将 SSH 参数改为 `-L 8082:127.0.0.1:8080`，
然后浏览器访问 `http://127.0.0.1:8082`；机器人查看器命令不变。
网页能打开但无图时，检查查看器与相机使用相同 ROS_DOMAIN_ID/DDS 环境；
可用 `--image-topic /实际相机话题` 覆盖配置中的相机话题。

## 5. 只联调头部（会运行 loco 支撑身体，不运行 pass）

先保证机器人处于可进行原 loco 测试的受控环境，有人能够随时停止。
本模式是“头部加零速度 loco”，不是仅给两个头部电机上电。

```bash
python scripts/start_k1_pass.py --head-only --vision-config /opt/booster
```

1. 完成输入检查后，沿用 deploy 的 **X / 键盘 x** 进入 Custom 和行走准备。
2. 此时头部功能启用。**A / r 被禁用，不会切入 pass**；身体速度命令维持零。
3. 遮住球：超过短时保持后，日志应进入 `Head tracker: scan`，头部平滑扫描。
4. 在画面边缘出现球：日志应进入 `track`。球在画面右边时头向右，球在下方时头向下。
   球进入中心容差区域后，头部保持；demo 的容差较宽，不要求球总在精确中心。
5. 遮球短于 0.5 秒不应立即大幅扫头；持续遮挡后应重新扫描。
6. 固定机器人和球，观察转头期间球坐标。允许识别抖动，但不应随头部转动出现明显系统性漂移。
7. 通过 `/head_pose` 再次确认 Custom 模式下位姿持续更新；确认未顶到机械限位。
8. Ctrl+C 退出，沿用 deploy 的退出模式处理，默认回 walking。

如方向、坐标、限位不对，先修正标定/头部配置再进行下一步。

## 6. pass 联调（会启用踢球策略）

```bash
python scripts/start_k1_pass.py --vision-config /opt/booster
```

1. **X / x**：进入零速度 loco 准备，头部开始找球/跟球。
2. 确认球位置正确、头部正常后，**A / r**：切入 pass。
3. pass 使用固定 power=2，方向沿机器人指向球的连线，Kp/Kd 沿用已有 pass 配置。
4. 首次获得有效球前，沿用现有实测姿态保持；获得过有效球后，无球、球数据过期、相机或头部位姿中断时，pass/shoot actor 继续使用最后有效原始 XY，缓存没有过期时间，且不会因漏检重置观测历史。新有效观测到来后更新缓存。
   图像仍正常但丢球时，头部继续搜索；相机图像中断时，头部保持最近命令。
5. 重新看到球且姿态输入恢复后，pass 重新建立历史观测并恢复推理。
6. Ctrl+C 退出。不要同时运行 demo brain 或另一套 `/joint_ctrl` 控制器。

当前 pass 的生命周期仍是既有的“有有效球就运行模型”，未增加单次踢球完成检测。

## 常见问题

| 现象 | 排查 |
|---|---|
| `vision_interface` 无法导入 | 检查当前 Python 与 ROS Python 兼容，并 source 新 workspace |
| 视觉立即退出 | 查看 `logs/pass_vision.log`；核对 engine 路径、TensorRT 版本和标定配置 |
| 相机话题存在但检查失败 | 检查是否真有新图、图像时间戳与 ROS 时钟是否一致 |
| 一直提示等待同步 | `/head_pose` 或深度时间差超过 0.2 秒；检查平台时间同步及发布源 |
| 视觉有球但 pass 不运行 | 检查置信度是否 ≥60、投影坐标有效、图像和 head_pose 是否新鲜 |
| 头部不动 | 检查是否按 X 进入运控、是否有新图；中心容差内不转头是预期行为 |
| 转头时球坐标漂移 | 核对相机外参、内参分辨率和实测 head_pose；不要用目标角生成假姿态 |

验证边界：离线测试验证 Python 逻辑、真实 ONNX 推理和模拟 ROS 下发；
它们不能证明本机相机标定、驱动、头部运动方向或实机稳定性正确，必须按上述步骤逐项验证。

本次迁入的验证记录：30 项 Python 测试中 28 项通过，2 项已有 MuJoCo/BoosterAssets
测试跳过；两个 ROS 包构建成功。隔离 ROS 域内，用合成图像、深度和头部位姿验证了
TensorRT 加载、推理及 Detections 发布；未向实机发送运动命令。

## 头部不动的复测（2026-09-27 修复）

初版只更新头部 q/kp/kd，遗漏 MotorCmd.weight，保持默认 0。
本机 SDK `example/low_level/low_level_publisher.cpp` 的头部例子明确使用 weight=1；
K1 平台的 `intercept_motor_cmd` 配置包含头部索引 0、1。
已在跟踪器启用时为两个头关节设置 weight=1，并增加每秒一次的 `Head detail` 日志。
原日志中的 `track` 只说明看到了球，不代表电机已执行命令。

重新运行第 5 步，按 X 后先遮球两三秒，检查：

- `state=scan`，`sent` 持续变化，`weight=(1.0,1.0)`，`measured` 应随之变化。
- 再将球放到画面边缘，检查 `reason=pixel_error` 和 measured 的跟随。
- `reason=within_deadband` 表示球在中心容差内，头部保持是预期行为。544×448 图像的
  中心容差约为 x=109–435、y=90–358；球需要越出这个区域才触发跟踪转头。
- 如果 sent 已变化、weight=1，但 measured 始终不变，保留这些日志，继续检查固件
  控制仲裁或其他头部控制源；仅凭 track/hold/scan 状态无法确认电机执行。

同时修复了启动入口 Ctrl+C 时重复 rclpy.shutdown 的异常；该异常发生在退出阶段，
不是此前运行期间头部不动的原因。
