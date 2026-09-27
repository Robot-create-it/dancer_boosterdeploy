"""K1 pass task: power 2, vision ball, loco-style joint control."""

import json
from pathlib import Path

from booster_deploy.controllers.controller_cfg import BoosterRobotControllerCfg, ControllerCfg, HeadTrackingCfg
from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.registry import register_task

from ...k1_pass import K1PassPolicyCfg
from . import K1WalkControllerCfg


_PASS = json.loads(Path(__file__).with_name("pass_config.json").read_text(encoding="utf-8"))
_ROBOT = K1WalkControllerCfg().robot
_ORDER = _PASS["webots_to_lab_idx"]
_BODY = _PASS["body_dof_indices_20"]


@configclass
class K1PassTaskCfg(ControllerCfg):
    policy_dt = 0.02
    booster = BoosterRobotControllerCfg(
        joint_stiffness=_PASS["kp_22"],
        joint_damping=_PASS["kd_22"],
        head_tracking=HeadTrackingCfg(enabled=True),
    )
    robot = _ROBOT.replace(
        default_joint_pos=_PASS["default_dof_pos_22"],
        joint_stiffness=_PASS["kp_22"],
        joint_damping=_PASS["kd_22"],
        prepare_state=_ROBOT.prepare_state.replace(joint_pos=_PASS["default_dof_pos_22"]),
    )
    policy = K1PassPolicyCfg(
        checkpoint_path="robots/k1/models/" + _PASS["pass"]["model_file"],
        policy_joint_names=[_ROBOT.joint_names[_BODY[i]] for i in _ORDER],
        actor_obs_history_length=10,
        action_scale=_PASS["action_scale"],
        action_filter=_PASS["action_filter"],
        obs_dof_vel_scale=_PASS["dof_velocity_scale"],
        clip_action=_PASS["pass"]["clip_action"],
        clip_observation=_PASS["clip_observation"],
    )


register_task("k1_pass", K1PassTaskCfg())
