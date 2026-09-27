"""K1 visual shoot policy, using the pass deployment control loop."""

from __future__ import annotations

import math

import torch

from booster_deploy.utils.isaaclab.configclass import configclass
from .locomotion import K1LocomotionPolicyCfg, LocomotionPolicy


class K1ShootPolicy(LocomotionPolicy):
    """Run the selected shoot actor on fresh robot-frame ball coordinates."""

    def __init__(self, cfg: K1ShootPolicyCfg, controller):
        if len(cfg.policy_joint_names) != 20 or cfg.actor_obs_history_length != 10:
            raise ValueError("K1 shoot requires 20 policy joints and 10 history frames")
        if cfg.kick_power != 6.0:
            raise ValueError("K1 shoot test uses match power 6")
        super().__init__(cfg, controller)
        self.frame_count = 0
        self._current_ball = None

    def reset(self) -> None:
        super().reset()
        self.frame_count = 0
        self._current_ball = None

    def compute_observation(self) -> torch.Tensor:
        loco = super().compute_observation()
        ball = self._current_ball
        if ball is None:
            ball = self.controller.get_ball_position(self.cfg.ball_max_age)
        if ball is None:
            raise RuntimeError("K1 shoot observation requires a fresh vision ball")
        raw_bx, raw_by = ball
        # Match pass's ball-facing test direction. Each selected shoot route
        # adds its own yaw and ball offsets before building the actor frame.
        direction = math.atan2(raw_by, raw_bx) + self.cfg.kick_yaw_offset
        bx = raw_bx + self.cfg.ball_pos_offset[0]
        by = raw_by + self.cfg.ball_pos_offset[1]
        scale = 2.0 / max(2.0, math.hypot(bx, by))
        command = torch.tensor([
            math.cos(direction),  # shoot BuildObs always uses speed 1, even for power 6
            math.sin(direction),
            bx * scale,
            by * scale,
            1.0 if self.frame_count % 2 == 0 else -1.0,
        ], dtype=loco.dtype, device=self.device)
        self.frame_count += 1
        return torch.cat((loco[:6], loco[9:], command))

    def inference(self) -> torch.Tensor:
        ball = self.controller.get_ball_position(self.cfg.ball_max_age)
        if ball is None:
            LocomotionPolicy.compute_observation(self)
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
class K1ShootPolicyCfg(K1LocomotionPolicyCfg):
    constructor = K1ShootPolicy
    ball_max_age: float = 0.5
    kick_power: float = 6.0
    # Default route 0 of the demo's shoot family is k1_shoot_policy_2.onnx.
    ball_pos_offset: tuple[float, float] = (0.0, -0.05)
    kick_yaw_offset: float = 0.1
    clip_action: float = 10.0
    arm_action_fix_joint_name: str | None = None
    arm_action_fix_offset: float = 0.0
