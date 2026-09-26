"""K1 base/side/turn locomotion ported from dancer-robocupdemo's loco_policy.cpp."""

from __future__ import annotations

import math
import logging
from pathlib import Path

import torch

from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.policy_runner import create_policy_runner
from .locomotion import K1LocomotionPolicyCfg, LocomotionPolicy


logger = logging.getLogger(__name__)
_ROUTE_NAMES = ("base", "side", "turn")


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


class LocoCommandProcessor:
    """Stateful ProcessCmd/SelectModel; Python floats preserve C++ double state."""

    def __init__(self, cfg: K1NestedLocomotionPolicyCfg):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.previous = [0.0, 0.0, 0.0]
        self.processed = [0.0, 0.0, 0.0]

    def process(self, command, adjust: bool = False) -> list[float]:
        cfg = self.cfg
        command = [float(v) for v in command]
        if not all(math.isfinite(v) for v in command):
            command = [0.0, 0.0, 0.0]
        if not all(math.isfinite(v) for v in self.previous):
            self.previous = [0.0, 0.0, 0.0]
        if adjust:
            self.previous = [0.0, 0.0, 0.0]
            self.processed = [0.0, 0.0, 0.2]
            return self.processed.copy()

        for i in range(3):
            self.previous[i] += _clamp(
                command[i] - self.previous[i],
                -cfg.max_vel_cmd_decre[i] * cfg.update_interval,
                cfg.max_vel_cmd_incre[i] * cfg.update_interval,
            )
        vx, vy, wz = self.previous
        # These constraints are cumulative, not mutually exclusive bands.
        for threshold, low_y, high_y, max_yaw in (
            (0.1, -0.20, 0.25, 1.0),
            (0.6, -0.15, 0.15, 1.0),
            (0.9, -0.10, 0.10, 1.0),
            (1.1, -0.10, 0.10, 0.7),
        ):
            if vx > threshold:
                vy = _clamp(vy, low_y, high_y)
                wz = _clamp(wz, -max_yaw, max_yaw)
        self.previous[1:] = [vy, wz]

        ax, ay = abs(vx), abs(vy)
        lateral_cap = math.inf if ax < 0.001 else (0.20 if vx >= 0 else 0.05) / ax
        yaw_cap = min(
            math.inf if ax < 0.001 else (0.8 if vx >= 0 else 0.4) / ax,
            math.inf if ay < 0.001 else 0.4 / ay,
        )
        # Final clamps deliberately do not overwrite the rate-limiter state.
        self.processed = [
            _clamp(vx, cfg.min_vel_cmd[0], cfg.max_vel_cmd[0]),
            _clamp(vy, max(-lateral_cap, cfg.min_vel_cmd[1]),
                   min(lateral_cap, cfg.max_vel_cmd[1])),
            _clamp(wz, max(-yaw_cap, cfg.min_vel_cmd[2]),
                   min(yaw_cap, cfg.max_vel_cmd[2])),
        ]
        return self.processed.copy()

    def select_route(self, adjust: bool = False) -> int:
        if self.cfg.forced_route is not None:
            return self.cfg.forced_route
        if adjust:
            return 2
        vx, vy, wz = map(abs, self.processed)
        if vy > 0.1 and vx < 0.2 and wz < 0.2:
            return 1
        if wz > 0.1 and vx < 0.1 and vy < 0.1:
            return 2
        return 0


class K1NestedLocomotionPolicy(LocomotionPolicy):
    """Three actors sharing one observation history, previous action and filter."""

    def __init__(self, cfg: K1NestedLocomotionPolicyCfg, controller):
        if cfg.forced_route not in (None, 0, 1, 2):
            raise ValueError("forced_route must be None, 0 (base), 1 (side), or 2 (turn)")
        if not math.isclose(controller.cfg.policy_dt, cfg.update_interval, abs_tol=1e-9):
            raise ValueError("policy_dt must match loco update_interval (0.02 s)")
        if len(cfg.policy_joint_names) != 20 or cfg.actor_obs_history_length != 10:
            raise ValueError("K1 loco requires 20 policy joints and 10 history frames")
        if len(cfg.model_paths) != 3 or cfg.checkpoint_path != cfg.model_paths[0]:
            raise ValueError("Configure model_paths as [base, side, turn] and checkpoint_path as base")
        super().__init__(cfg, controller)  # Loads base and initializes shared state.
        self.models = [self._model]
        for path in cfg.model_paths[1:]:
            model_path = Path(path)
            if not model_path.is_absolute():
                model_path = Path(self.task_path) / model_path
            self.models.append(create_policy_runner(str(model_path), self.device))
        self.command_processor = LocoCommandProcessor(cfg)
        self.gravity_offset = torch.tensor(cfg.gravity_offset, device=self.device)
        self.reset()
        logger.info("Loaded K1 loco base/side/turn policies; initial route=base")

    def reset(self) -> None:
        super().reset()
        self.command_processor.reset()
        self.active_route = 0
        self.loco_adjust = False

    def compute_observation(self) -> torch.Tensor:
        command = self.controller.vel_command
        raw = ([command.lin_vel_x, command.lin_vel_y, command.ang_vel_yaw]
               if command is not None else [0.0, 0.0, 0.0])
        processed = self.command_processor.process(raw, self.loco_adjust)
        route = self.command_processor.select_route(self.loco_adjust)
        if route != self.active_route:
            logger.info("K1 loco policy=%s, processed command=(%.3f, %.3f, %.3f)",
                        _ROUTE_NAMES[route], *processed)
        self.active_route = route
        observation = super().compute_observation()
        observation[3:6] += self.gravity_offset
        observation[6:9] = torch.tensor(processed, dtype=observation.dtype, device=self.device)
        return observation

    def _forward_model(self, obs_history: torch.Tensor) -> torch.Tensor:
        return self.models[self.active_route](obs_history.flatten()).reshape(-1)


@configclass
class K1NestedLocomotionPolicyCfg(K1LocomotionPolicyCfg):
    constructor = K1NestedLocomotionPolicy
    model_paths: list[str] = []
    use_actor_module: bool = False
    arm_action_fix_offset: float = 0.0
    arm_action_fix_joint_name: str | None = None
    gravity_offset: list[float] = [0.015, 0.0, 0.0]
    max_vel_cmd: list[float] = [1.5, 0.4, 1.8]
    min_vel_cmd: list[float] = [-0.7, -0.4, -1.8]
    max_vel_cmd_incre: list[float] = [1.3, 1.2, 2.0]
    max_vel_cmd_decre: list[float] = [1.6, 1.2, 2.0]
    update_interval: float = 0.02
    forced_route: int | None = None
