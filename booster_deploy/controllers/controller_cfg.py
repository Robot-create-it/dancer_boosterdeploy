from typing import Callable, List, Optional
from dataclasses import MISSING
import math
import torch

from ..utils.isaaclab.configclass import configclass


@configclass
class PrepareStateCfg:
    stiffness: List[float] = MISSING
    damping: List[float] = MISSING
    joint_pos: List[float] = MISSING


@configclass
class PassCameraCfg:
    """Pinhole camera used to gate privileged pass ball observations."""

    # Intrinsics match vision_ws/src/vision/config/vision.yaml. The image size
    # is configurable because the YAML contains intrinsics but no dimensions.
    width: int = 512
    height: int = 480
    fx: float = 203.239578
    fy: float = 203.239578
    cx: float = 259.408417
    cy: float = 214.265747


@configclass
class MujocoControllerCfg:
    init_pos: List[float] = [0.0, 0.0, 0.6]
    init_quat: List[float] = [1.0, 0.0, 0.0, 0.0]
    # Used by the K1 pass soccer scene; the ball centre is 0.11 m above the field.
    ball_init_xy: List[float] = [0.8, 0.0]
    print_speed: bool = False  # report measured planar velocity every 0.5 simulated seconds
    pass_camera: PassCameraCfg = PassCameraCfg()
    decimation: int = 10
    # physics_dt will automatically be set by ControllerCfg
    physics_dt: float = None  # type: ignore
    log_states: Optional[str] = None
    log_joint_torque_csv: Optional[str] = None
    log_joint_velocity_csv: Optional[str] = None
    log_joint_position_csv: Optional[str] = None
    visualize_reference_ghost: bool = False
    ghost_rgba: List[float] = [0.2, 0.8, 0.2, 0.25]
    show_left_ui: bool = False
    show_right_ui: bool = False


@configclass
class HeadTrackingCfg:
    enabled: bool = False
    # Filled from the robot's calibrated vision.yaml by scripts/deploy.py.
    fx: float = 0.0
    fy: float = 0.0
    color_topic: str = "/StereoNetNode/rectified_image"
    yaw_min: float = -1.0
    yaw_max: float = 1.0
    pitch_min: float = 0.2
    pitch_max: float = 0.85
    max_speed: float = 0.6  # rad/s, applied in the 50 Hz control loop
    smoother: float = 3.5
    center_tolerance: float = 0.1  # fraction of image size, as in demo
    detection_max_age: float = 0.5
    lost_hold_s: float = 0.5
    scan_interval_s: float = 0.8


@configclass
class BoosterRobotControllerCfg:
    metrics_max_events: int = 2000
    # Mode to enter after Custom control exits. Supported values: "walking", "damping".
    exit_mode: str = "walking"
    # Optional motor-side gains for the real robot; simulation uses RobotCfg.
    joint_stiffness: Optional[List[float]] = None
    joint_damping: Optional[List[float]] = None
    head_tracking: HeadTrackingCfg = HeadTrackingCfg()
    head_only: bool = False  # hold default stance and track the head without an actor
    control_head: bool = False  # task supplies head targets; send weight=1
    recovery_safety: bool = False  # K1-specific feedback and command checks
    recovery_hold_only: bool = False  # validation: A/r cannot start the policy
    recovery_state_max_age: float = 0.1
    recovery_ready_duration: float = 0.5
    recovery_command_max_age: float = 0.2

    def apply_to_robot(self, robot: "RobotCfg") -> "RobotCfg":
        overrides = {}
        for name in ("joint_stiffness", "joint_damping"):
            values = getattr(self, name)
            if values is not None:
                if len(values) != len(robot.joint_names) or any(
                    not math.isfinite(value) or value < 0 for value in values
                ):
                    raise ValueError(f"booster.{name} must contain one finite, non-negative gain per joint")
                overrides[name] = list(values)
        return robot.replace(**overrides)


@configclass
class RobotCfg:
    name: str = MISSING
    # Preparation after X: "walking", "standing", or "hold" (current pose).
    prepare_mode: str = "walking"

    joint_names: list[str] = MISSING
    body_names: list[str] = MISSING

    sim_joint_names: list[str] = MISSING
    sim_body_names: list[str] = MISSING

    joint_stiffness: List[float] = MISSING
    joint_damping: List[float] = MISSING

    default_joint_pos: List[float] = MISSING
    effort_limit: List[float] = MISSING

    mjcf_path: str = MISSING

    prepare_state: PrepareStateCfg = MISSING

    def __post_init__(self):
        assert (
            len(self.joint_names)
            == len(self.joint_stiffness)
            == len(self.joint_damping)
            == len(self.default_joint_pos)
            == len(self.effort_limit)
        )


@configclass
class VelocityCommandCfg:
    vx_max: float = 1.0
    # Direction-specific forward velocity limits.  Keeping these at the
    # default value preserves the historical symmetric +/-vx_max behavior.
    vx_forward_max: Optional[float] = None
    vx_backward_max: Optional[float] = None
    vy_max: float = 1.0
    vyaw_max: float = 1.0


@configclass
class PolicyCfg:
    constructor: Callable = MISSING
    checkpoint_path: str = MISSING
    enable_safety_fallback: bool = True
    device: str | torch.device = "cpu"


@configclass
class EvaluatorCfg:
    constructor: Callable = MISSING
    # Rendering
    render: bool = True


@configclass
class ControllerCfg:
    """Controller configuration class.
    """

    policy_dt: float = 0.02
    robot: RobotCfg = MISSING
    vel_command: Optional[VelocityCommandCfg] = None
    policy: PolicyCfg = MISSING

    mujoco: MujocoControllerCfg = MujocoControllerCfg()
    booster: BoosterRobotControllerCfg = BoosterRobotControllerCfg()
    evaluator: Optional[EvaluatorCfg] = None

    def __post_init__(self):
        self.mujoco.physics_dt = self.policy_dt / self.mujoco.decimation
