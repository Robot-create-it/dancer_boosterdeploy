"""K1 shoot task: match power 6, selectable fixed policy, vision ball."""

import json
from pathlib import Path

from booster_deploy.controllers.controller_cfg import BoosterRobotControllerCfg, ControllerCfg, HeadTrackingCfg
from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.registry import register_task

from ...k1_shoot import K1ShootPolicyCfg
from . import K1WalkControllerCfg


# The demo uses the same 20-body-joint runtime parameters for pass and shoot.
_KICK = json.loads(Path(__file__).with_name("pass_config.json").read_text(encoding="utf-8"))
_SHOOT = json.loads(Path(__file__).with_name("shoot_config.json").read_text(encoding="utf-8"))
SHOOT_POLICY_IDS = tuple(
    Path(filename).stem.removeprefix("k1_shoot_policy_")
    for filename in _SHOOT["model_files"]
)
_ROBOT = K1WalkControllerCfg().robot
_ORDER = _KICK["webots_to_lab_idx"]
_BODY = _KICK["body_dof_indices_20"]


@configclass
class K1ShootTaskCfg(ControllerCfg):
    policy_dt = 0.02
    booster = BoosterRobotControllerCfg(
        joint_stiffness=_KICK["kp_22"],
        joint_damping=_KICK["kd_22"],
        head_tracking=HeadTrackingCfg(enabled=True),
    )
    robot = _ROBOT.replace(
        default_joint_pos=_KICK["default_dof_pos_22"],
        joint_stiffness=_KICK["kp_22"],
        joint_damping=_KICK["kd_22"],
        prepare_state=_ROBOT.prepare_state.replace(joint_pos=_KICK["default_dof_pos_22"]),
    )
    policy = K1ShootPolicyCfg(
        checkpoint_path="robots/k1/models/" + _SHOOT["model_files"][0],
        policy_joint_names=[_ROBOT.joint_names[_BODY[i]] for i in _ORDER],
        actor_obs_history_length=10,
        action_scale=_KICK["action_scale"],
        action_filter=_KICK["action_filter"],
        obs_dof_vel_scale=_KICK["dof_velocity_scale"],
        clip_action=_SHOOT["clip_action"],
        clip_observation=_KICK["clip_observation"],
    )


def select_shoot_policy(cfg: K1ShootTaskCfg, policy_id: str) -> None:
    """Pin one shoot model and its matching demo observation offsets."""
    try:
        index = SHOOT_POLICY_IDS.index(policy_id)
    except ValueError as exc:
        raise ValueError(
            f"Unknown shoot policy {policy_id!r}; choose one of {SHOOT_POLICY_IDS}"
        ) from exc
    cfg.policy.checkpoint_path = "robots/k1/models/" + _SHOOT["model_files"][index]
    cfg.policy.ball_pos_offset = tuple(_SHOOT["ball_pos_offset"][index])
    cfg.policy.kick_yaw_offset = _SHOOT["kick_yaw_offset"][index]


register_task("k1_shoot", K1ShootTaskCfg())
