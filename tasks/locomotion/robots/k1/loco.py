"""Standalone K1 loco task using the demo models and walk-style inputs."""

import json
from pathlib import Path

from booster_deploy.controllers.controller_cfg import (
    BoosterRobotControllerCfg, ControllerCfg, VelocityCommandCfg,
)
from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.registry import register_task
from ...nested_locomotion import K1NestedLocomotionPolicyCfg
from . import K1WalkControllerCfg


_LOCO = json.loads(Path(__file__).with_name("loco_config.json").read_text(encoding="utf-8"))
_ROBOT = K1WalkControllerCfg().robot
_BODY = _LOCO["body_dof_indices_20"]
_ORDER = _LOCO["webots_to_lab_idx"]
_PATHS = ["robots/k1/models/" + path for path in _LOCO["model_files"]]


@configclass
class K1LocoTaskCfg(ControllerCfg):
    policy_dt = _LOCO["update_interval"]
    # Match the loco ankle gains on hardware; retain walk gains elsewhere.
    booster = BoosterRobotControllerCfg(
        joint_stiffness=[
            _LOCO["kp_22"][i] if "_ankle_" in name else _ROBOT.joint_stiffness[i]
            for i, name in enumerate(_ROBOT.joint_names)
        ],
        joint_damping=[
            _LOCO["kd_22"][i] if "_ankle_" in name else _ROBOT.joint_damping[i]
            for i, name in enumerate(_ROBOT.joint_names)
        ],
    )
    robot = _ROBOT.replace(
        default_joint_pos=_LOCO["default_dof_pos_22"],
        joint_stiffness=_LOCO["kp_22"],
        joint_damping=_LOCO["kd_22"],
        prepare_state=_ROBOT.prepare_state.replace(joint_pos=_LOCO["default_dof_pos_22"]),
    )
    vel_command = VelocityCommandCfg(
        vx_forward_max=_LOCO["max_vel_cmd"][0],
        vx_backward_max=-_LOCO["min_vel_cmd"][0],
        vy_max=_LOCO["max_vel_cmd"][1],
        vyaw_max=_LOCO["max_vel_cmd"][2],
    )
    policy = K1NestedLocomotionPolicyCfg(
        checkpoint_path=_PATHS[0],
        model_paths=_PATHS,
        policy_joint_names=[_ROBOT.joint_names[_BODY[i]] for i in _ORDER],
        actor_obs_history_length=_LOCO["num_obs_stacking"],
        action_scale=_LOCO["action_scale"],
        action_filter=_LOCO["action_filter"],
        obs_dof_vel_scale=_LOCO["dof_velocity_scale"],
        clip_action=_LOCO["clip_action"],
        clip_observation=_LOCO["clip_observation"],
        max_vel_cmd=_LOCO["max_vel_cmd"],
        min_vel_cmd=_LOCO["min_vel_cmd"],
        update_interval=_LOCO["update_interval"],
    )


register_task("k1_loco", K1LocoTaskCfg())
