"""K1 visual pass policy, using the normal locomotion control loop."""

from __future__ import annotations

import math

import torch

from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.vision_ball import MotionBallMemory
from .locomotion import K1LocomotionPolicyCfg, LocomotionPolicy


class K1PassPolicy(LocomotionPolicy):
    """Run power-2 pass with the demo's last-valid-ball network input."""

    def __init__(self, cfg: K1PassPolicyCfg, controller):
        if len(cfg.policy_joint_names) != 20 or cfg.actor_obs_history_length != 10:
            raise ValueError("K1 pass requires 20 policy joints and 10 history frames")
        super().__init__(cfg, controller)
        self.frame_count = 0
        self._current_ball = None
        self.ball_memory = MotionBallMemory()

    def reset(self) -> None:
        super().reset()
        self.frame_count = 0
        self._current_ball = None
        # As in SimMotion, policy reset does not erase the last ball reference.

    def _get_ball_reference(self):
        return self.ball_memory.update(
            self.controller.get_ball_position(self.cfg.ball_max_age))

    def compute_observation(self) -> torch.Tensor:
        # loco supplies IMU, gravity, joint offsets/velocities and previous action.
        loco = super().compute_observation()
        ball = self._current_ball
        if ball is None:
            ball = self._get_ball_reference()
        if ball is None:
            raise RuntimeError("K1 pass observation requires an initial valid ball")
        bx, by = ball
        direction = math.atan2(by, bx)  # robot-to-ball heading in the robot frame
        scale = 2.0 / max(2.0, math.hypot(bx, by))
        command = torch.tensor([
            2.0 * math.cos(direction),  # fixed passing power
            2.0 * math.sin(direction),
            bx * scale,
            by * scale,
            1.0 if self.frame_count % 2 == 0 else -1.0,
        ], dtype=loco.dtype, device=self.device)
        self.frame_count += 1
        # Kick layout: angvel[3], gravity[3], joints[20+20], previous[20], command[5].
        return torch.cat((loco[:6], loco[9:], command))

    def inference(self) -> torch.Tensor:
        ball = self._get_ball_reference()
        if ball is None:
            # Before the first valid sighting, retain the existing startup hold.
            # Once acquired, missing frames use the cache and keep inference,
            # history and the phase flag running normally.
            LocomotionPolicy.compute_observation(self)  # retain loco fall detection
            self.reset()
            return self.robot.data.joint_pos.clone()
        if self.obs_history is None:
            self.filtered_dof_target.copy_(self.robot.data.joint_pos)
        self._current_ball = ball
        try:
            return super().inference()
        finally:
            self._current_ball = None


@configclass
class K1PassPolicyCfg(K1LocomotionPolicyCfg):
    constructor = K1PassPolicy
    ball_max_age: float = 0.5  # new observations only; cached XY does not expire
    clip_action: float = 10.0
    arm_action_fix_joint_name: str | None = None
    arm_action_fix_offset: float = 0.0
