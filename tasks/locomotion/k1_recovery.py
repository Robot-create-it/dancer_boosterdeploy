"""K1 FDR trajectory plus residual policy, using loco's sensor data path."""

from __future__ import annotations

from dataclasses import MISSING
import json
import logging
import math
from pathlib import Path

import torch

from booster_deploy.controllers.base_controller import Policy
from booster_deploy.controllers.controller_cfg import PolicyCfg
from booster_deploy.robots.k1 import K1_CFG
from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.policy_runner import create_policy_runner
from booster_deploy.utils.proprioception import read_proprioception
from booster_deploy.utils.recovery_safety import limit_arm_targets


logger = logging.getLogger(__name__)


class K1RecoveryPolicy(Policy):
    def __init__(self, cfg: K1RecoveryPolicyCfg, controller):
        super().__init__(cfg, controller)
        self.robot = controller.robot
        if self.robot.cfg.joint_names != K1_CFG.joint_names:
            raise ValueError("K1 recovery requires the canonical 22 serial joints")
        if not math.isclose(controller.cfg.policy_dt, cfg.control_dt, abs_tol=1e-9):
            raise ValueError("policy_dt must match recovery control_dt (0.02 s)")
        self.device = torch.device(cfg.device)
        self.robot.data.to(self.device)

        def tensor(values):
            return torch.as_tensor(values, dtype=torch.float32, device=self.device)

        def resource(path):
            return str(Path(self.task_path) / path)

        self.reference = tensor(cfg.reference)
        self.q_min, self.q_max = tensor(cfg.q_min), tensor(cfg.q_max)
        if any(t.shape != (22,) or not torch.isfinite(t).all()
               for t in (self.reference, self.q_min, self.q_max)):
            raise ValueError("Recovery reference and limits must contain 22 finite values")
        if torch.any(self.q_min > self.q_max):
            raise ValueError("Invalid recovery joint limits")
        self.zero_joint_ids = torch.tensor(cfg.zero_joint_ids, device=self.device, dtype=torch.long)
        self.trajectories = {}
        data = json.loads(Path(resource(cfg.trajectory_path)).read_text(encoding="utf-8"))
        for name in ("faceup", "facedown"):
            traj = {key: tensor(data[name][key]) for key in ("joint", "gravity", "height")}
            rows = len(traj["joint"])
            if (rows == 0 or traj["joint"].shape != (rows, 22)
                    or traj["gravity"].shape != (rows, 3)
                    or traj["height"].shape != (rows,)
                    or any(not torch.isfinite(t).all() for t in traj.values())):
                raise ValueError(f"Invalid {name} recovery trajectory")
            self.trajectories[name] = traj
        self._model = create_policy_runner(resource(cfg.checkpoint_path), self.device)
        self.last_action = torch.zeros(22, device=self.device)
        self.reset()
        self.trace = None
        if cfg.trace_path is not None:
            from booster_deploy.utils.recovery_trace import RecoveryTrace
            self.trace = RecoveryTrace(cfg.trace_path, self)

    def reset(self):
        self.retries = 0
        self._reset_attempt()

    def _reset_attempt(self):
        self.state = "settling"
        self.settle_time = 0.0
        self.elapsed = 0.0
        self.posture = None
        self.row = 0
        self.phase = 0.0
        self.last_action.zero_()
        self.last_observation = None
        self._last_target = None
        self.target_before_arm_limit = None

    def compute_observation(self):
        if self.posture is None:
            raise RuntimeError("Recovery posture has not been selected")
        q, dq, gyro, gravity = read_proprioception(self.robot.data)
        traj = self.trajectories[self.posture]
        q_error = traj["joint"][self.row] - q
        q_offset = q - self.reference
        velocity = dq.clamp(-self.cfg.dof_velocity_clip, self.cfg.dof_velocity_clip) * self.cfg.dof_velocity_scale
        # Only head state slots are masked. Hip yaw / ankle roll stay observable.
        q_error[:2] = 0
        q_offset[:2] = 0
        velocity[:2] = 0
        previous = self.last_action.clone()
        previous[self.zero_joint_ids] = 0
        return torch.cat((
            torch.tensor([float(self.posture == "faceup"), self.phase], device=self.device),
            traj["gravity"][self.row] - gravity,
            traj["height"][self.row].reshape(1),
            q_error, gravity, gyro, q_offset, velocity, previous,
        )).clamp(-self.cfg.clip_observation, self.cfg.clip_observation)

    def inference(self):
        try:
            target = self._inference()
            self.target_before_arm_limit = target.clone()
            target = torch.as_tensor(limit_arm_targets(
                target.detach().cpu().numpy(), self.robot.data.joint_pos.detach().cpu().numpy(),
                self.robot.joint_stiffness.cpu().numpy(), self.robot.effort_limit.cpu().numpy(),
            ), dtype=target.dtype, device=target.device)
            if self.trace is not None:
                self.trace.record(self, target)
            return target
        except Exception as exc:
            if self.trace is not None:
                self.trace.write({'type': 'error', 'message': str(exc)})
            raise

    def _inference(self):
        q, dq, gyro, gravity = read_proprioception(self.robot.data)
        if any(not torch.isfinite(value).all() for value in (q, dq, gyro, gravity)):
            raise RuntimeError("Non-finite recovery sensor input")
        if self.state in ("succeeded", "failed"):
            return self._last_target.clone()
        if self.posture is None:
            self.settle_time = (self.settle_time + self.cfg.control_dt
                                if torch.linalg.vector_norm(gyro) < self.cfg.settle_ang_vel else 0.0)
            if self.settle_time + 1e-9 < self.cfg.settle_duration:
                return q.clone()
            # gravity.x == sin(pitch), from the exact quaternion path loco uses.
            self.posture = "faceup" if gravity[0] < 0 else "facedown"
            self.state = "executing"
            logger.info("K1 recovery start: %s", self.posture)

        traj = self.trajectories[self.posture]
        rows = len(traj["joint"])
        self.elapsed += self.cfg.control_dt
        # C++ llround for nonnegative t, followed by clamping to the last row.
        self.row = min(rows - 1, int(math.floor(self.elapsed / self.cfg.control_dt + 0.5)))
        self.phase = min(1.0, (self.cfg.phase_pre_roll_s + self.elapsed) /
                         (self.cfg.phase_pre_roll_s + rows * self.cfg.control_dt))
        self.last_observation = self.compute_observation()
        with torch.inference_mode():
            action = self._model(self.last_observation).reshape(-1)
        if action.shape != (22,) or not torch.isfinite(action).all():
            raise RuntimeError("Recovery model must return 22 finite residuals")
        self.last_action.copy_(action)
        self.last_action[self.zero_joint_ids] = 0
        target = (traj["joint"][self.row] + self.last_action).clamp(self.q_min, self.q_max)
        self._last_target = target.clone()
        if self.phase >= 1.0 and gravity[2] < -0.5:
            self.state = "succeeded"
            logger.info("K1 recovery succeeded; holding final target")
        elif self.elapsed > rows * self.cfg.control_dt + self.cfg.retry_margin_s:
            if self.retries >= self.cfg.max_retries:
                self.state = "failed"
                logger.error("K1 recovery exhausted retries; stopping controller")
                self.controller.stop()
            else:
                self.retries += 1
                logger.info("K1 recovery retry %d", self.retries)
                self._reset_attempt()
                return q.clone()
        return target


@configclass
class K1RecoveryPolicyCfg(PolicyCfg):
    constructor = K1RecoveryPolicy
    # Falling is the expected input; locomotion's upright-only fallback does not apply.
    enable_safety_fallback: bool = False
    trajectory_path: str = MISSING
    reference: list[float] = MISSING
    q_min: list[float] = MISSING
    q_max: list[float] = MISSING
    zero_joint_ids: list[int] = [0, 1, 12, 15, 18, 21]
    control_dt: float = 0.02
    dof_velocity_clip: float = 5.0
    dof_velocity_scale: float = 0.1
    clip_observation: float = 100.0
    settle_ang_vel: float = 0.25
    settle_duration: float = 0.7
    phase_pre_roll_s: float = 1.0
    retry_margin_s: float = 2.0
    max_retries: int = 2
    trace_path: str | None = None
