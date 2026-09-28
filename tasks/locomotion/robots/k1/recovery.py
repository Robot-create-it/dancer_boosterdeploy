"""Standalone K1 recovery task, with the same controller as loco."""

import json
import math
from pathlib import Path

from booster_deploy.controllers.controller_cfg import BoosterRobotControllerCfg, ControllerCfg, MujocoControllerCfg
from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.registry import register_task
from ...k1_recovery import K1RecoveryPolicyCfg
from . import K1WalkControllerCfg


_RECOVERY = json.loads(Path(__file__).with_name("recovery_config.json").read_text(encoding="utf-8"))
_ROBOT = K1WalkControllerCfg().robot


@configclass
class K1RecoveryTaskCfg(ControllerCfg):
    policy_dt = _RECOVERY["fdr_control_dt"]
    booster = BoosterRobotControllerCfg(exit_mode="damping", control_head=True, recovery_safety=True)
    mujoco = MujocoControllerCfg(
        init_pos=[0.0, 0.0, 0.25],
        init_quat=[math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0],
    )
    robot = _ROBOT.replace(
        prepare_mode="hold",
        default_joint_pos=_RECOVERY["reference_22"],
        joint_stiffness=_RECOVERY["kp_0_22"],
        joint_damping=_RECOVERY["kd_22"],
        effort_limit=_RECOVERY["torque_limit_22"],
        prepare_state=_ROBOT.prepare_state.replace(
            joint_pos=_RECOVERY["reference_22"],
            stiffness=_RECOVERY["kp_0_22"],
            damping=_RECOVERY["kd_22"],
        ),
    )
    policy = K1RecoveryPolicyCfg(
        checkpoint_path="robots/k1/models/" + _RECOVERY["model_file"],
        trajectory_path="robots/k1/models/" + _RECOVERY["trajectory_file"],
        reference=_RECOVERY["reference_22"],
        q_min=_RECOVERY["q_min_22"],
        q_max=_RECOVERY["q_max_22"],
        zero_joint_ids=_RECOVERY["zero_joint_ids"],
        control_dt=_RECOVERY["fdr_control_dt"],
        dof_velocity_clip=_RECOVERY["dof_velocity_clip"],
        dof_velocity_scale=_RECOVERY["dof_velocity_scale"],
        clip_observation=_RECOVERY["clip_observation"],
        settle_ang_vel=_RECOVERY["thresholds"]["fallen_ang_vel_threshold"],
        settle_duration=_RECOVERY["thresholds"]["is_ready_cumulative_time_threshold"],
        phase_pre_roll_s=_RECOVERY["phase_pre_roll_s"],
        retry_margin_s=_RECOVERY["retry_margin_s"],
        max_retries=_RECOVERY["max_retries"],
    )


register_task("k1_recovery", K1RecoveryTaskCfg())
