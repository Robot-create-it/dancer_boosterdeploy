"""Slow student task with pass/shoot camera and head control."""

import json
from pathlib import Path

from booster_deploy.controllers.controller_cfg import BoosterRobotControllerCfg, ControllerCfg, HeadTrackingCfg
from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.registry import register_task

from ...k1_student import K1StudentPolicyCfg
from . import K1WalkControllerCfg


_STUDENT = json.loads(Path(__file__).with_name("student_config.json").read_text(encoding="utf-8"))
_ROBOT = K1WalkControllerCfg().robot


@configclass
class K1StudentTaskCfg(ControllerCfg):
    policy_dt = 0.02
    booster = BoosterRobotControllerCfg(
        joint_stiffness=_STUDENT["kp_22"],
        joint_damping=_STUDENT["kd_22"],
        head_tracking=HeadTrackingCfg(enabled=True),
    )
    robot = _ROBOT.replace(
        default_joint_pos=_STUDENT["default_dof_pos_22"],
        joint_stiffness=_STUDENT["kp_22"],
        joint_damping=_STUDENT["kd_22"],
        prepare_state=_ROBOT.prepare_state.replace(joint_pos=_STUDENT["default_dof_pos_22"]),
    )
    policy = K1StudentPolicyCfg(
        checkpoint_path="robots/k1/models/student/model_10000_student_body20_slow.onnx",
        policy_joint_names=_ROBOT.joint_names[2:],
        actor_obs_history_length=_STUDENT["hist_len"],
        obs_dof_vel_scale=_STUDENT["dof_velocity_scale"],
        action_scale=_STUDENT["action_scale"],
        action_filter=_STUDENT["action_filter"],
        clip_action=_STUDENT["clip_actions"],
        command=tuple(_STUDENT["default_command"]),
        gait_frequency=_STUDENT["default_gait_frequency"],
        ball_pos_scale=_STUDENT["ball_pos_scale"],
        goal_pos_scale=_STUDENT["goal_pos_scale"],
        target_distance=_STUDENT["target_distance"],
    )


register_task("k1_student", K1StudentTaskCfg())
