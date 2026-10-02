"""K1 slow student body policy on the pass/shoot sensor and control path."""

from __future__ import annotations

import math

import torch

from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.proprioception import read_proprioception
from booster_deploy.utils.vision_ball import MotionBallMemory
from .locomotion import K1LocomotionPolicyCfg, LocomotionPolicy


class K1StudentPolicy(LocomotionPolicy):
    """Build the demo's 81-value frame and 50-frame zero-initialized history."""

    def __init__(self, cfg: K1StudentPolicyCfg, controller):
        if len(cfg.policy_joint_names) != 20 or cfg.actor_obs_history_length != 50:
            raise ValueError("K1 student requires 20 body joints and 50 history frames")
        super().__init__(cfg, controller)
        self.ball_memory = MotionBallMemory()
        self.head_action = torch.zeros(2, device=self.device)
        self.phase = 0.0
        self.frame_count = 0

    def reset(self) -> None:
        super().reset()
        self.head_action.zero_()
        self.phase = 0.0
        self.frame_count = 0
        # The last visible ball remains available for the goal approximation.

    def set_head_target(self, target) -> None:
        """Feed the external head command into the next previous-action slot."""
        head = torch.as_tensor(target, dtype=torch.float32, device=self.device)
        self.head_action.copy_((head - self.default_joint_pos[:2]) / self.cfg.action_scale)

    def compute_observation(self) -> torch.Tensor:
        q, dq, gyro, gravity = read_proprioception(self.robot.data)
        if self.cfg.enable_safety_fallback and gravity[2] > -0.5:
            self.controller.stop()

        self.phase = (self.phase + self.controller.cfg.policy_dt * self.cfg.gait_frequency) % 1.0
        angle = 2.0 * math.pi * self.phase
        command = torch.tensor([
            *self.cfg.command, math.cos(angle), math.sin(angle),
        ], dtype=torch.float32, device=self.device)

        fresh_ball = self.controller.get_ball_position(self.cfg.ball_max_age)
        remembered_ball = self.ball_memory.update(fresh_ball)
        ball = torch.zeros(2, dtype=torch.float32, device=self.device)
        if fresh_ball is not None:
            ball[:] = torch.as_tensor(fresh_ball, dtype=torch.float32, device=self.device)
        goal = torch.zeros_like(ball)
        if remembered_ball is not None:
            bx, by = remembered_ball
            # Match pass/shoot: aim along the robot-to-ball ray in body XY.
            bearing = math.atan2(by, bx)
            goal[:] = torch.tensor([
                self.cfg.target_distance * math.cos(bearing),
                self.cfg.target_distance * math.sin(bearing),
            ], dtype=torch.float32, device=self.device)

        frame = torch.cat((
            gravity, gyro, command,
            q - self.default_joint_pos,
            dq * self.cfg.obs_dof_vel_scale,
            self.head_action, self.last_action,
            ball * self.cfg.ball_pos_scale,
            goal * self.cfg.goal_pos_scale,
        ))
        if frame.numel() != 81:
            raise RuntimeError(f"K1 student frame has {frame.numel()} values; expected 81")
        return frame

    def inference(self) -> torch.Tensor:
        fresh_ball = self.controller.get_ball_position(self.cfg.ball_max_age)
        if self.ball_memory.update(fresh_ball) is None:
            # Match pass/shoot startup: hold until the first valid camera ball.
            self.reset()
            return self.robot.data.joint_pos.clone()
        if self.obs_history is None:
            self.obs_history = torch.zeros(50, 81, dtype=torch.float32, device=self.device)
            self.filtered_dof_target.copy_(self.robot.data.joint_pos)
        else:
            frame = self.compute_observation().clamp(
                -self.cfg.clip_observation, self.cfg.clip_observation)
            self.obs_history = self.obs_history.roll(-1, 0)
            self.obs_history[-1] = frame
        with torch.no_grad():
            action = self._forward_model(self.obs_history).clamp(
                -self.cfg.clip_action, self.cfg.clip_action)
        if action.numel() != 20:
            raise RuntimeError(f"K1 student returned {action.numel()} actions; expected 20")
        self.last_action.copy_(action)
        self.frame_count += 1
        return self._action_to_targets(action)


@configclass
class K1StudentPolicyCfg(K1LocomotionPolicyCfg):
    constructor = K1StudentPolicy
    actor_obs_history_length: int = 50
    command: tuple[float, float, float] = (1.0, 0.0, 1.0)
    gait_frequency: float = 1.8
    target_distance: float = 6.0
    ball_pos_scale: float = 1.0
    goal_pos_scale: float = 0.2
    ball_max_age: float = 0.5
    clip_action: float = 1.0
    action_scale: float = 1.0
    action_filter: float = 1.0
    arm_action_fix_joint_name: str | None = None
    arm_action_fix_offset: float = 0.0
