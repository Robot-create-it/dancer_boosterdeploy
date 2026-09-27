from __future__ import annotations
import json
import logging
import signal
import time
import threading
import multiprocessing as mp
from copy import deepcopy
from multiprocessing import synchronize

import numpy as np
import torch

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from booster_interface.msg import BoosterApiReqMsg, LowState, LowCmd, MotorCmd
from booster_interface.srv import RpcService


class _RobotModeInt:
    """Mode values used by the Booster Loco RPC API."""

    kDamping = 0
    kWalking = 2
    kCustom = 3


_LOC_API_CHANGE_MODE = 2000
_LOC_API_GET_STATUS = 2018

from .controller_cfg import ControllerCfg
from .base_controller import BaseController, BoosterRobot
from ..utils.synced_array import SyncedArray
from ..utils.metrics import SyncedMetrics
from ..utils.isaaclab import math as lab_math
from ..utils.remote_control_service import RemoteControlService
from ..utils.vision_ball import BallObservation, select_ball_observation
from ..utils.head_ball_tracker import HeadBallTracker


logger = logging.getLogger("booster_deploy")
logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s %(message)s")


class BoosterRobotPortal:
    synced_state: SyncedArray
    synced_command: SyncedArray
    synced_action: SyncedArray
    exit_event: synchronize.Event

    def __init__(self, cfg: ControllerCfg) -> None:
        # Resolve motor-side gains before preparing or publishing any commands.
        # Keep the registered task's simulation configuration intact.
        self.cfg = deepcopy(cfg)
        self.cfg.robot = self.cfg.booster.apply_to_robot(self.cfg.robot)

        self.robot = BoosterRobot(self.cfg.robot)

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        if not rclpy.ok():
            rclpy.init()
        self.remoteControlService = RemoteControlService()
        # Use multiprocessing.Event for inter-process communication
        self.exit_event = mp.Event()
        self.velocity_commands_enabled_event = mp.Event()
        self.task_start_event = mp.Event()
        self.low_state_received_event = threading.Event()
        self.is_running = True
        def signal_handler(sig, frame):
            if mp.current_process().name == "MainProcess":
                print("\nKeyboard interrupt received. Shutting down...")
            self.exit_event.set()

        # Register signal handler
        signal.signal(signal.SIGINT, signal_handler)

        self._init_synced_buffer()
        self._init_metrics()

        self._cleanup_done = False
        self._safety_abort = False
        self.inference_process = None  # Inference process reference
        self.low_cmd_publisher: rclpy.publisher.Publisher = None
        self.low_state_thread = None
        self.low_cmd_process: mp.Process | None = None

        # Initialize communication. Callbacks may start immediately and
        # reference `is_running` and `exit_event`, so ensure those are set.
        self._init_communication()

    def _init_synced_buffer(self):
        from tasks.locomotion.k1_pass import K1PassPolicyCfg
        from tasks.locomotion.k1_shoot import K1ShootPolicyCfg
        self._vision_pass_enabled = isinstance(self.cfg.policy, (K1PassPolicyCfg, K1ShootPolicyCfg))
        self.head_tracker = None
        if self.cfg.booster.head_tracking.enabled:
            self.head_tracker = HeadBallTracker(self.cfg.booster.head_tracking)
        self._last_detection_stamp = 0.0
        self.synced_ball = SyncedArray(
            "ball", shape=(1,),
            dtype=np.dtype([("x", float), ("y", float), ("stamp", float),
                            ("image_stamp", float), ("confidence", float),
                            ("bbox", float, (4,)), ("bbox_valid", bool), ("valid", bool)]),
        ) if self._vision_pass_enabled else None
        if self.synced_ball is not None:
            self.synced_ball.write(np.zeros((1,), dtype=self.synced_ball.dtype))
        self.synced_vision_state = SyncedArray(
            "vision_state", shape=(1,),
            dtype=np.dtype([("width", int), ("height", int),
                            ("image_received", float), ("pose_received", float)]),
        ) if self._vision_pass_enabled else None
        if self.synced_vision_state is not None:
            self.synced_vision_state.write(np.zeros((1,), dtype=self.synced_vision_state.dtype))
        action_dtype = np.dtype(
            [
                ("dof_target", float, (self.robot.num_joints,)),
                ("stiffness", float, (self.robot.num_joints,)),
                ("damping", float, (self.robot.num_joints,)),
            ]
        )
        self.synced_action = SyncedArray(
            "action",
            shape=(1,),
            dtype=action_dtype,
        )
        self._action_buf = np.ndarray((1,), dtype=action_dtype)

        state_dtype = np.dtype(
            [
                ("root_rpy_w", float, (3,)),
                ("root_ang_vel_b", float, (3,)),
                ("root_pos_w", float, (3,)),
                ("root_lin_vel_w", float, (3,)),
                ("joint_pos", float, (self.robot.num_joints,)),
                ("joint_vel", float, (self.robot.num_joints,)),
                ("feedback_torque", float, (self.robot.num_joints,)),
            ]
        )
        self.synced_state = SyncedArray(
            "state",
            shape=(1,),
            dtype=state_dtype
        )
        self._state_buf = np.zeros((1,), dtype=state_dtype)

        command_dtype = np.dtype(
            [
                ("vx", float),
                ("vy", float),
                ("vyaw", float),
            ]
        )
        self.synced_command = SyncedArray(
            "command",
            shape=(1,),
            dtype=command_dtype,
        )

    def _init_metrics(self):
        # initialize cross-process synced metrics
        max_events = self.cfg.booster.metrics_max_events
        self.metrics = {
            "low_state_handler": SyncedMetrics(
                "low_state_handler", max_events=max_events
            ),
            "policy_step": SyncedMetrics(
                "policy_step", max_events=max_events
            ),
        }

    def _init_communication(self) -> None:
        try:
            if self._vision_pass_enabled:
                try:
                    from vision_interface.msg import Detections
                except ImportError as exc:
                    raise RuntimeError(
                        "K1 visual kick requires vision_interface; source vision_ws/install/setup.bash"
                    ) from exc
                self._detections_type = Detections
            self.create_low_cmd_publisher("booster_deploy_low_cmd_pub")
            self._start_low_state_subscription()
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise

    def _start_low_state_subscription(self) -> None:
        """Start ROS 2 subscription loop on a dedicated thread.

        The subscription is run on a dedicated thread and spins a
        SingleThreadedExecutor for the `/low_state` topic.
        """

        def low_state_service_executor():
            self.logger.info("Low state subscription started")
            low_state_node = rclpy.create_node("booster_deploy_low_state_sub")
            low_state_node.create_subscription(
                LowState,
                "/low_state",
                self._low_state_handler,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                ),
            )
            if self._vision_pass_enabled:
                from sensor_msgs.msg import Image
                from geometry_msgs.msg import Pose
                self._vision_node = low_state_node
                low_state_node.create_subscription(
                    Image, self.cfg.booster.head_tracking.color_topic,
                    self._image_handler, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                low_state_node.create_subscription(
                    Pose, "/head_pose", self._head_pose_handler,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                low_state_node.create_subscription(
                    self._detections_type,
                    "/booster_vision/detection",
                    self._vision_handler,
                    QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                    ),
                )

            executor = SingleThreadedExecutor()
            executor.add_node(low_state_node)

            try:
                # loop: check exit_event and rclpy.ok()
                while rclpy.ok() and not self.exit_event.is_set():
                    executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                pass
            except Exception as exc:
                # Suppress RCLError if we are shutting down
                is_rcl_error = "RCLError" in type(exc).__name__
                is_shutting_down = self.exit_event.is_set() or not rclpy.ok()

                if is_rcl_error and is_shutting_down:
                    pass
                else:
                    self.logger.error(
                        "Low state subscription executor stopped: %s",
                        exc,
                        exc_info=True
                    )
            finally:
                executor.shutdown()
                low_state_node.destroy_node()
            self.logger.info("Low state subscription stopped")

        self.low_state_thread = threading.Thread(
            target=low_state_service_executor,
            name="low_state_executor",
            daemon=True,
        )
        self.low_state_thread.start()

    def _vision_handler(self, detections):
        ros_now = self._vision_node.get_clock().now().nanoseconds * 1e-9
        image_stamp = detections.header.stamp.sec + detections.header.stamp.nanosec * 1e-9
        # Reject repeated, out-of-order and future frames without refreshing their age.
        if image_stamp <= self._last_detection_stamp or image_stamp > ros_now + 0.05:
            return
        self._last_detection_stamp = image_stamp
        ball = select_ball_observation(detections, time.monotonic(), ros_now,
                                       self.cfg.policy.ball_max_age)
        sample = np.zeros((1,), dtype=self.synced_ball.dtype)
        if ball is not None:
            for key in ("x", "y", "stamp", "image_stamp", "confidence"):
                sample[0][key] = getattr(ball, key)
            sample[0]["valid"] = True
            if ball.bbox is not None:
                sample[0]["bbox"] = ball.bbox
                sample[0]["bbox_valid"] = True
        self.synced_ball.write(sample)

    def _image_handler(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self._vision_node.get_clock().now().nanoseconds * 1e-9 - stamp
        if msg.width <= 0 or msg.height <= 0 or not -0.05 <= age <= 0.5:
            return
        state = self.synced_vision_state.read()
        state[0]["width"], state[0]["height"] = msg.width, msg.height
        state[0]["image_received"] = time.monotonic() - max(0.0, age)
        self.synced_vision_state.write(state)

    def _head_pose_handler(self, msg):
        q = msg.orientation
        values = [msg.position.x, msg.position.y, msg.position.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(values).all() or not 0.9 <= np.linalg.norm(values[3:]) <= 1.1:
            return
        state = self.synced_vision_state.read()
        state[0]["pose_received"] = time.monotonic()
        self.synced_vision_state.write(state)

    def _low_state_handler(self, low_state_msg: LowState):
        self.metrics["low_state_handler"].mark()
        try:
            if not self.is_running or self.exit_event.is_set():
                return

            # collect state data
            rpy = np.array(low_state_msg.imu_state.rpy, dtype=np.float32)
            gyro = np.array(low_state_msg.imu_state.gyro, dtype=np.float32)
            dof_pos = np.zeros(self.robot.num_joints, dtype=np.float32)
            dof_vel = np.zeros(self.robot.num_joints, dtype=np.float32)
            fb_torque = np.zeros(self.robot.num_joints, dtype=np.float32)

            for i, motor in enumerate(low_state_msg.motor_state_serial):
                dof_pos[i] = motor.q
                dof_vel[i] = motor.dq
                fb_torque[i] = motor.tau_est

            self._state_buf[0]["root_rpy_w"][:] = rpy
            self._state_buf[0]["root_ang_vel_b"][:] = gyro
            self._state_buf[0]["root_pos_w"][:] = np.zeros(
                3, dtype=np.float32
            )
            self._state_buf[0]["root_lin_vel_w"][:] = np.zeros(
                3, dtype=np.float32
            )
            self._state_buf[0]["joint_pos"][:] = dof_pos
            self._state_buf[0]["joint_vel"][:] = dof_vel
            self._state_buf[0]["feedback_torque"][:] = fb_torque
            self.synced_state.write(self._state_buf)
            self.low_state_received_event.set()

            # update velocity commands to synced_command
            cmd = np.zeros((1,), dtype=self.synced_command.dtype)
            cmd[0]["vx"] = self.remoteControlService.get_vx_cmd()
            cmd[0]["vy"] = self.remoteControlService.get_vy_cmd()
            cmd[0]["vyaw"] = self.remoteControlService.get_vyaw_cmd()
            self.synced_command.write(cmd)

        except Exception as e:
            self.logger.error(f"Error in _low_state_handler: {e}")
            self.running = False
            self.exit_event.set()

    def create_low_cmd_publisher(self, name):
        self.publish_node = rclpy.create_node(name)
        publisher = self.publish_node.create_publisher(
            LowCmd,
            "joint_ctrl",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            )
        )
        self.low_cmd_publisher = publisher

        # construct low_cmd struct
        self.low_cmd = LowCmd()  # type: ignore
        self.low_cmd.cmd_type = LowCmd.CMD_TYPE_SERIAL   # type: ignore
        motor_cmd_buf = [
            MotorCmd() for _ in range(self.robot.num_joints)
        ]  # type: ignore
        for i in range(self.robot.num_joints):
            motor_cmd_buf[i].q = 0.0
            motor_cmd_buf[i].dq = 0.0
            motor_cmd_buf[i].tau = 0.0
            motor_cmd_buf[i].kp = 0.0
            motor_cmd_buf[i].kd = 0.0
            motor_cmd_buf[i].weight = 0.0
        self.low_cmd.motor_cmd.extend(motor_cmd_buf)
        self.motor_cmd = self.low_cmd.motor_cmd

        self.rpc_service_client = self.publish_node.create_client(
            RpcService, "booster_rpc_service"
        )

        return publisher

    def _call_booster_rpc(
        self,
        api_id: int,
        body: dict[str, object] | None = None,
    ) -> tuple[bool, dict | None]:
        """Call Booster's ROS2 Loco RPC and parse its JSON response."""
        if self.rpc_service_client is None:
            self.logger.error("booster_rpc_service client is not initialized")
            return False, None

        try:
            request = RpcService.Request()
            request.msg = BoosterApiReqMsg()
            request.msg.api_id = api_id
            request.msg.body = json.dumps(body) if body is not None else ""

            future = self.rpc_service_client.call_async(request)
            rclpy.spin_until_future_complete(self.publish_node, future)
            result = future.result()
        except Exception as exc:
            self.logger.error("booster_rpc_service call failed: %s", exc)
            return False, None

        if result is None:
            self.logger.error("booster_rpc_service returned no result")
            return False, None

        parsed = None
        if result.msg.body:
            try:
                parsed = json.loads(result.msg.body)
            except json.JSONDecodeError:
                self.logger.warning(
                    "booster_rpc_service returned non-JSON body: %s",
                    result.msg.body,
                )

        if result.msg.status != 0:
            self.logger.warning(
                "booster_rpc_service status=%s body=%s",
                result.msg.status,
                result.msg.body,
            )
            return False, parsed
        return True, parsed

    def _change_robot_mode(self, mode_name: str) -> bool:
        mode_values = {
            "damping": _RobotModeInt.kDamping,
            "walking": _RobotModeInt.kWalking,
            "walk": _RobotModeInt.kWalking,
            "custom": _RobotModeInt.kCustom,
        }
        normalized = mode_name.strip().lower()
        if normalized not in mode_values:
            raise ValueError(
                f"Unsupported robot mode {mode_name!r}; "
                "expected 'damping', 'walking' (or 'walk'), or 'custom'"
            )

        if not self.rpc_service_client.wait_for_service(timeout_sec=15.0):
            self.logger.error("booster_rpc_service is unavailable")
            return False

        expected_mode = mode_values[normalized]
        display_name = (
            "Walking" if expected_mode == _RobotModeInt.kWalking
            else normalized.capitalize()
        )
        for _ in range(20):
            ok, _ = self._call_booster_rpc(
                _LOC_API_CHANGE_MODE,
                {"mode": expected_mode},
            )
            if ok:
                status_ok, status = self._call_booster_rpc(
                    _LOC_API_GET_STATUS
                )
                if (
                    status_ok
                    and status is not None
                    and int(status.get("current_mode", -1)) == expected_mode
                ):
                    self.logger.info("Robot switched to %s mode", display_name)
                    return True
            time.sleep(0.5)

        self.logger.error("Failed to switch robot to %s mode", display_name)
        return False

    def _prime_custom_command(self) -> None:
        """Preload the PVT-style PD hold command before leaving PVT."""
        state = self.synced_state.read()[0]
        joint_pos = state["joint_pos"]
        prepare_state = self.robot.cfg.prepare_state
        for i in range(self.robot.num_joints):
            self.motor_cmd[i].q = float(joint_pos[i])
            self.motor_cmd[i].kp = float(prepare_state.stiffness[i])
            self.motor_cmd[i].kd = float(prepare_state.damping[i])
            self.motor_cmd[i].tau = 0.0
        # The low-level controller keeps this command when Custom is entered,
        # until the locomotion policy publishes its first action.
        self.low_cmd_publisher.publish(self.low_cmd)

    def start_custom_mode_conditionally(self):
        print(f"{self.remoteControlService.get_custom_mode_operation_hint()}")
        while not self.exit_event.is_set():
            if self.remoteControlService.start_custom_mode():
                break
            time.sleep(0.1)

        if self.exit_event.is_set():
            return False

        while (
            rclpy.ok()
            and not self.exit_event.is_set()
            and not self.low_state_received_event.wait(timeout=0.5)
        ):
            self.logger.info("Waiting for first '/low_state' message")

        if not self.low_state_received_event.is_set():
            self.logger.error("No valid '/low_state'; refusing Custom mode")
            return False

        # Python-side walking preparation is allowed only from an upright
        # posture.  This is a single check at the X trigger; interpolation
        # itself intentionally has no additional posture checks.
        if self.cfg.robot.prepare_mode.strip().lower() == "walking":
            state = self.synced_state.read()[0]
            rpy = torch.from_numpy(state["root_rpy_w"]).to(dtype=torch.float32)
            projected_gravity = lab_math.quat_apply_inverse(
                lab_math.quat_from_euler_xyz(*rpy).squeeze(),
                torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32),
            )
            if projected_gravity[2] > -0.5:
                self.logger.error(
                    "Refusing walk preparation: robot posture is unsafe "
                    "(projected_gravity[2]=%.3f). Switching to damping mode.",
                    float(projected_gravity[2]),
                )
                self._safety_abort = True
                self._change_robot_mode("damping")
                return False

            while rclpy.ok() and self.low_cmd_publisher.get_subscription_count() == 0:
                self.logger.info("Waiting for '/joint_ctrl' subscriber, retry in 0.5s")
                time.sleep(0.5)

            # Match standing preparation's hold position and PVT-style PD
            # gains.  Locomotion takes over only after Custom is entered.
            # Send exactly one command while PVT still owns the robot; the
            # low-level controller retains it across the Custom transition.
            self._prime_custom_command()
            # Give the low-level/PVT path time to receive and retain the
            # non-zero-gain hold command before relinquishing control.
            time.sleep(0.1)

            # The Python locomotion policy publishes joint_ctrl commands.  The
            # robot must be in Custom mode for those commands to be accepted;
            # merely starting the inference process leaves a robot that was in
            # PVT mode under the previous controller.
            if not self._change_robot_mode("custom"):
                self.logger.error(
                    "Failed to switch to Custom mode for walking preparation"
                )
                return False

            # Walking preparation uses the Python locomotion policy directly;
            # do not run the PVT interpolation.
            self.logger.info(
                "Walking preparation accepted; starting zero-command locomotion"
            )
            return True

        while rclpy.ok() and self.low_cmd_publisher.get_subscription_count() == 0:
            self.logger.info("Waiting for '/joint_ctrl' subscriber, retry in 0.5s")
            time.sleep(0.5)

        self.logger.info("Subscriber found, starting control loop")        

        prepare_state = self.robot.cfg.prepare_state
        init_joint_pos = self.synced_state.read()[0]['joint_pos']
        for i in range(self.robot.num_joints):
            self.motor_cmd[i].q = init_joint_pos[i]
            self.motor_cmd[i].kp = float(prepare_state.stiffness[i])
            self.motor_cmd[i].kd = float(prepare_state.damping[i])
        if self.cfg.booster.control_head:
            for name in ("aahead_yaw_joint", "aahead_pitch_joint"):
                self.motor_cmd[self.robot.cfg.joint_names.index(name)].weight = 1.0

        self.low_cmd_publisher.publish(self.low_cmd)
        time.sleep(0.1)

        if not self._change_robot_mode("custom"):
            self.logger.error("Failed to switch to Custom mode")
            return False

        if self.cfg.robot.prepare_mode.strip().lower() == "hold":
            self.logger.info("Custom mode started; holding measured pose until A/r")
            return True

        trans = np.linspace(init_joint_pos, prepare_state.joint_pos, num=500)
        start_time = time.perf_counter()
        for i in range(500):
            for j in range(self.robot.num_joints):
                self.motor_cmd[j].q = trans[i][j]
            self.low_cmd_publisher.publish(self.low_cmd)
            while time.perf_counter() < start_time + (i + 1) * 0.002:
                time.sleep(0.0002)
        if self.cfg.robot.prepare_mode.strip().lower() == "walking":
            self.logger.info(
                "Prepare pose reached; starting Python RL walking with zero commands"
            )
        self.logger.info("Custom mode started, initialized with prepare pose")
        return True

    def _build_prepare_cfg(self):
        """Build the matching robot locomotion config for walking preparation."""
        robot_name = self.cfg.robot.name.lower()
        if "t2" in robot_name:
            from tasks.locomotion.robots.t2 import T2WalkTaskCfg
            walk_cfg = T2WalkTaskCfg()
        elif "t1" in robot_name:
            from tasks.locomotion.robots.t1 import T1WalkControllerCfg1
            walk_cfg = T1WalkControllerCfg1()
        elif "k1" in robot_name:
            from tasks.locomotion.nested_locomotion import K1NestedLocomotionPolicyCfg
            from tasks.locomotion.k1_pass import K1PassPolicyCfg
            from tasks.locomotion.k1_shoot import K1ShootPolicyCfg
            if isinstance(self.cfg.policy, K1NestedLocomotionPolicyCfg):
                # Keep the selected loco models, pose and gains during zero-command
                # preparation; do not silently load the old k1_walk.pt checkpoint.
                walk_cfg = deepcopy(self.cfg)
                walk_cfg.policy.forced_route = None
            elif isinstance(self.cfg.policy, (K1PassPolicyCfg, K1ShootPolicyCfg)):
                from tasks.locomotion.robots.k1.loco import K1LocoTaskCfg
                walk_cfg = K1LocoTaskCfg()
            else:
                from tasks.locomotion.robots.k1 import K1WalkTaskCfg
                walk_cfg = K1WalkTaskCfg()
        else:
            raise RuntimeError(f"No locomotion preparation policy for robot {self.cfg.robot.name!r}")
        prepare_cfg = deepcopy(self.cfg)
        # The hand-off command and the prepare controller must use the
        # matching locomotion robot gains, not gains inherited from a task
        # policy that happens to run on the same robot.
        prepare_cfg.robot = deepcopy(walk_cfg.robot)
        prepare_cfg.policy = deepcopy(walk_cfg.policy)
        prepare_cfg.vel_command = deepcopy(walk_cfg.vel_command)
        prepare_cfg.robot.prepare_mode = "walking"
        if self.cfg.booster.head_tracking.enabled:
            for name in ("aahead_yaw_joint", "aahead_pitch_joint"):
                i = prepare_cfg.robot.joint_names.index(name)
                j = self.cfg.robot.joint_names.index(name)
                prepare_cfg.robot.joint_stiffness[i] = self.cfg.robot.joint_stiffness[j]
                prepare_cfg.robot.joint_damping[i] = self.cfg.robot.joint_damping[j]
        return prepare_cfg

    def start_rl_gait_conditionally(self, wait_for_trigger: bool = True):
        """Start RL mode and spawn inference process and publisher thread."""
        if wait_for_trigger:
            print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
            while not self.exit_event.is_set():
                if self.remoteControlService.start_rl_gait():
                    break
                time.sleep(0.1)

        if self.exit_event.is_set():
            return False

        # In walking preparation, start the matching zero-command locomotion
        # policy immediately; the child switches to the task policy after A/r.
        process_cfg = self.cfg
        prepare_then_task = (
            self.cfg.robot.prepare_mode.strip().lower() == "walking"
        )
        if prepare_then_task:
            process_cfg = self._build_prepare_cfg()
        self.inference_process = mp.Process(
            target=BoosterRobotPortal.inference_process_func,
            args=(process_cfg, self, prepare_then_task),
            daemon=True,
        )
        self.inference_process.start()
        self.logger.info("Inference process started")

        # In walking preparation the locomotion policy is command-free, so do
        # not advertise velocity controls until the task policy is enabled.
        if not prepare_then_task and self.cfg.vel_command is not None:
            print(f"{self.remoteControlService.get_operation_hint()}")
        return True

    def cleanup(self) -> None:
        """Clean up resources (idempotent)."""
        if self._cleanup_done:
            return
        self._cleanup_done = True

        self.logger.info("Doing cleanup...")

        # stop threads and processes
        self.is_running = False
        self.exit_event.set()

        # wait for inference process
        if (
            self.inference_process is not None
            and self.inference_process.is_alive()
        ):
            self.logger.info("Waiting for inference process...")
            self.inference_process.join(timeout=2.0)
            if self.inference_process.is_alive():
                self.logger.warning(
                    "Inference process did not stop, terminating...")
                self.inference_process.terminate()
                self.inference_process.join(timeout=1.0)

        # close communications
        try:
            self.remoteControlService.close()
        except Exception as e:
            self.logger.error(f"Error closing remote control: {e}")

        if self.low_cmd_process is not None and self.low_cmd_process.is_alive():
            self.logger.info("Waiting for low cmd publisher process...")
            self.low_cmd_process.join(timeout=2.0)
            if self.low_cmd_process.is_alive():
                self.logger.warning(
                    "Low cmd publisher process did not stop, terminating...")
                self.low_cmd_process.terminate()
                self.low_cmd_process.join(timeout=1.0)

        try:
            thread = self.low_state_thread
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)

        except Exception as e:
            self.logger.error(f"Error waiting for low state thread: {e}")

        if rclpy.ok():
            rclpy.shutdown()

        self.logger.info("Cleanup complete")

        # Print synced metrics summary to stdout
        for name, metric in self.metrics.items():
            stats = metric.compute()
            print(
                f"METRICS {name}: count={stats['count']}, "
                f"freq={stats['freq_hz']:.3f}Hz, "
                f"mean_period={stats['mean_period_s']}, "
                f"min={stats['min_period_s']}, max={stats['max_period_s']}"
            )

    def run(self):
        """Main loop: monitor inference process and diagnostics (10Hz)."""

        print("Initialization complete.")

        prepare_mode = self.cfg.robot.prepare_mode.strip().lower()
        if prepare_mode not in ("walking", "standing", "hold"):
            raise ValueError(
                f"Unsupported prepare_mode {self.cfg.robot.prepare_mode!r}; "
                "expected 'walking', 'standing', or 'hold'"
            )

        # Walking preparation starts the Python locomotion policy immediately
        # with zero commands; A/r then unlocks its velocity input.  Other
        # policies, and standing preparation, wait for A/r before inference.
        if not self.start_custom_mode_conditionally():
            print("Custom mode initialization cancelled.")
        elif not self.start_rl_gait_conditionally(
            wait_for_trigger=prepare_mode in ("standing", "hold")
        ):
            print("RL mode initialization cancelled.")
        else:
            if prepare_mode == "walking":
                if self.cfg.booster.head_only:
                    print("Head test active: zero-command loco; A/r is disabled. Ctrl+C exits.")
                else:
                    print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
            # main loop: wait for exit signal
            while self.is_running and not self.exit_event.is_set():
                if (
                    prepare_mode == "walking"
                    and not self.cfg.booster.head_only
                    and not self.task_start_event.is_set()
                    and self.remoteControlService.start_rl_gait()
                ):
                    self.task_start_event.set()
                    self.velocity_commands_enabled_event.set()
                    self.logger.info("Task policy enabled after A trigger")
                    if self.cfg.vel_command is not None:
                        print(f"{self.remoteControlService.get_operation_hint()}")
                # check whether the inference process is alive
                if self.inference_process is not None:
                    inference_process_alive = self.inference_process.is_alive()
                    if not inference_process_alive:
                        self.logger.error("Inference process died unexpectedly")
                        self.is_running = False
                        self.exit_event.set()
                        break
                time.sleep(0.1)

        exit_mode = "damping" if self._safety_abort else self.cfg.booster.exit_mode.strip().lower()
        exit_mode_display = "Walking" if exit_mode == "walk" else exit_mode.capitalize()
        self.logger.info(
            "Custom mode ended; explicitly switching to %s mode...",
            exit_mode_display,
        )
        try:
            if not self._change_robot_mode(exit_mode):
                self.logger.error(
                    "Custom mode ended, but switching to %s mode failed",
                    exit_mode_display,
                )
        except ValueError as exc:
            self.logger.error("Custom mode exit configuration is invalid: %s", exc)

    def __enter__(self) -> BoosterRobotPortal:
        return self

    def __exit__(self, *args) -> None:
        self.cleanup()

    @staticmethod
    def inference_process_func(
        cfg: ControllerCfg,
        portal: BoosterRobotPortal,
        prepare_then_task: bool = False,
    ) -> None:
        controller = BoosterRobotController(cfg, portal)
        controller.run(stop_event=portal.task_start_event if prepare_then_task else None)
        if prepare_then_task and not portal.exit_event.is_set():
            from tasks.locomotion.nested_locomotion import K1NestedLocomotionPolicyCfg

            if isinstance(cfg.policy, K1NestedLocomotionPolicyCfg) and isinstance(
                portal.cfg.policy, K1NestedLocomotionPolicyCfg
            ):
                # _build_prepare_cfg copies this task, only clearing forced_route.
                # Keep the three sessions, observation history and action filter
                # alive across A/r instead of loading and resetting them again.
                controller.policy.cfg.forced_route = portal.cfg.policy.forced_route
                controller._velocity_commands_enabled = True
                portal.logger.info("K1 loco enabled; continuing the prepared policy")
                controller.run(reset_policy=False)
            else:
                BoosterRobotController(portal.cfg, portal).run()
        portal.logger.info("Inference process stopped.")


class BoosterRobotController(BaseController):
    '''Controller for Booster robots. Note that this controller runs in a
    separate process forked by BoosterRobotPortal.
    '''
    def __init__(self, cfg: ControllerCfg, portal: BoosterRobotPortal) -> None:
        super().__init__(cfg)
        self.portal = portal
        self._last_head_diagnostic = -float("inf")
        # Walking preparation starts inference before A/r, but keeps velocity
        # commands masked until that trigger is received.
        self._velocity_commands_enabled = not (
            cfg.robot.prepare_mode.strip().lower() == "walking"
            and cfg.vel_command is not None
        )

    def get_ball_position(self, max_age: float):
        ball = self.get_ball_observation(max_age)
        return (ball.x, ball.y) if ball is not None else None

    def get_ball_observation(self, max_age: float):
        if self.portal.synced_ball is None:
            return None
        now = time.monotonic()
        vision = self.portal.synced_vision_state.read()[0]
        if any(not 0 <= now - float(vision[key]) <= max_age
               for key in ("image_received", "pose_received")):
            return None
        sample = self.portal.synced_ball.read()[0]
        stamp = float(sample["stamp"])
        age = now - stamp
        if not sample["valid"] or stamp <= 0 or age < 0 or age > max_age:
            return None
        return BallObservation(float(sample["x"]), float(sample["y"]),
                               float(sample["confidence"]),
                               tuple(sample["bbox"]) if sample["bbox_valid"] else None,
                               stamp, float(sample["image_stamp"]))

    def update_vel_command(self):
        if not self._velocity_commands_enabled:
            self.vel_command.lin_vel_x = 0.0
            self.vel_command.lin_vel_y = 0.0
            self.vel_command.ang_vel_yaw = 0.0
            return
        cmd = self.portal.synced_command.read()[0]
        normalized = np.array([cmd["vx"], cmd["vy"], cmd["vyaw"]], dtype=float)
        if not np.isfinite(normalized).all():
            normalized.fill(0.0)
        normalized = np.clip(normalized, -1.0, 1.0)
        self.vel_command.lin_vel_x = self.vel_command.scale_vx(normalized[0])
        self.vel_command.lin_vel_y = normalized[1] * self.vel_command.vy_max
        self.vel_command.ang_vel_yaw = normalized[2] * self.vel_command.vyaw_max

    def update_state(self) -> None:
        state = self.portal.synced_state.read()[0]

        self.robot.data.joint_pos = torch.from_numpy(
            state["joint_pos"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        self.robot.data.joint_vel = torch.from_numpy(
            state["joint_vel"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        self.robot.data.feedback_torque = torch.from_numpy(
            state["feedback_torque"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        self.robot.data.root_pos_w = torch.from_numpy(
            state["root_pos_w"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        rpy_t = torch.from_numpy(state["root_rpy_w"]).to(
            dtype=torch.float32).to(self.robot.data.device)
        self.robot.data.root_quat_w = lab_math.quat_from_euler_xyz(
            *rpy_t
        ).squeeze()
        self.robot.data.root_lin_vel_b = lab_math.quat_apply_inverse(
            self.robot.data.root_quat_w,
            torch.from_numpy(
                state["root_lin_vel_w"]).to(dtype=torch.float32).to(
                    self.robot.data.device)
        )
        self.robot.data.root_ang_vel_b = torch.from_numpy(
            state["root_ang_vel_b"]).to(dtype=torch.float32).to(
                self.robot.data.device)

    def ctrl_step(self, dof_targets: torch.Tensor) -> None:
        tracker = getattr(self.portal, "head_tracker", None)
        if tracker is not None:
            now = time.monotonic()
            state = self.portal.synced_vision_state.read()[0]
            size = (int(state["width"]), int(state["height"]))
            if not 0 <= now - float(state["image_received"]) <= 0.5:
                size = (0, 0)
            indices = [self.cfg.robot.joint_names.index(name) for name in
                       ("aahead_yaw_joint", "aahead_pitch_joint")]
            measured = [float(self.robot.data.joint_pos[i]) for i in indices]
            previous_state = tracker.state
            ball = self.get_ball_observation(tracker.cfg.detection_max_age)
            target = tracker.update(now, self.cfg.policy_dt, measured, ball, size)
            if tracker.state != previous_state:
                logger.info("Head tracker: %s", tracker.state)
            dof_targets = dof_targets.clone()
            for i, value in zip(indices, target):
                dof_targets[i] = value
        for i in range(self.robot.num_joints):
            self.portal.motor_cmd[i].q = float(dof_targets[i].item())
            kp_val = float(self.robot.joint_stiffness[i].item())
            kd_val = float(self.robot.joint_damping[i].item())
            self.portal.motor_cmd[i].kp = kp_val
            self.portal.motor_cmd[i].kd = kd_val
        if tracker is not None or self.cfg.booster.control_head:
            # The K1 firmware routes the head through its upper-body command
            # interceptor even when Custom supplies the body targets. Match the
            # SDK low_level_publisher example: an externally controlled head
            # joint must carry weight=1; the message default is 0.
            for name in ("aahead_yaw_joint", "aahead_pitch_joint"):
                i = self.cfg.robot.joint_names.index(name)
                self.portal.motor_cmd[i].weight = 1.0
        self.portal.low_cmd_publisher.publish(self.portal.low_cmd)
        if tracker is not None and now - self._last_head_diagnostic >= 1.0:
            self._last_head_diagnostic = now
            logger.info(
                "Head detail: state=%s reason=%s measured=(%.3f,%.3f) "
                "desired=(%.3f,%.3f) sent=(%.3f,%.3f) weight=(%.1f,%.1f) "
                "bbox=%s image_size=%s image_age=%.3f pose_age=%.3f",
                tracker.state, tracker.reason, *measured, *tracker.target,
                *(self.portal.motor_cmd[i].q for i in indices),
                *(self.portal.motor_cmd[i].weight for i in indices),
                ball.bbox if ball is not None else None, size,
                now - float(state["image_received"]), now - float(state["pose_received"]),
            )

    def stop(self):
        super().stop()
        self.portal.exit_event.set()

    def run(self, stop_event=None, *, reset_policy: bool = True):
        self.update_state()
        if self.vel_command is not None:
            self.update_vel_command()
        if reset_policy:
            self.start()
        next_inference_time = time.perf_counter()
        while (
            self.is_running
            and not self.portal.exit_event.is_set()
            and not (stop_event is not None and stop_event.is_set())
        ):
            if (
                stop_event is None
                and not self._velocity_commands_enabled
                and self.portal.velocity_commands_enabled_event.is_set()
            ):
                self._velocity_commands_enabled = True
            if time.perf_counter() < next_inference_time:
                time.sleep(0.0002)
                continue
            next_inference_time += self.cfg.policy_dt

            self.update_state()
            if self.vel_command is not None:
                self.update_vel_command()
            self.portal.metrics["policy_step"].mark()
            dof_targets = self.policy_step()
            if not self.is_running or self.portal.exit_event.is_set():
                break
            self.ctrl_step(dof_targets)

        if stop_event is None:
            self.portal.exit_event.set()
