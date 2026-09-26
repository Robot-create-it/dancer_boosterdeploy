"""Run deterministic, headless K1 loco closed-loop checks in deploy's MuJoCo."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from booster_deploy.controllers.mujoco_controller import MujocoController
from tasks.locomotion.robots.k1.loco import K1LocoTaskCfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.0, help='Seconds per command phase')
    parser.add_argument('--output', type=Path, help='Optional JSON report path')
    parser.add_argument('--real-robot-gains', action='store_true',
                        help='Exercise the configured hardware PD gains in MuJoCo (no robot connection)')
    args = parser.parse_args()
    if args.seconds < 1:
        parser.error('--seconds must be at least 1')
    cfg = K1LocoTaskCfg()
    if args.real_robot_gains:
        cfg.robot = cfg.booster.apply_to_robot(cfg.robot)
    controller = MujocoController(cfg)
    # Commands are supplied below; do not poll the interactive terminal.
    controller.update_vel_command = lambda: None
    controller.update_state()
    controller.start()
    phases = [('stand', (0, 0, 0), 0), ('forward', (0.4, 0, 0), 0),
              ('side_left', (0, 0.2, 0), 1), ('turn_left', (0, 0, 0.4), 2),
              ('backward', (-0.2, 0, 0), 0), ('side_right', (0, -0.2, 0), 1),
              ('turn_right', (0, 0, -0.4), 2), ('stop', (0, 0, 0), 0)]
    results = []
    passed = True
    for name, cmd, expected_route in phases:
        velocity = controller.vel_command
        velocity.lin_vel_x, velocity.lin_vel_y, velocity.ang_vel_yaw = cmd
        min_height = float('inf')
        min_up = 1.0
        targets_finite = True
        for _ in range(round(args.seconds / cfg.policy_dt)):
            controller.update_state()
            targets = controller.policy_step()
            targets_finite = bool(np.isfinite(targets.cpu().numpy()).all())
            if not controller.is_running or not targets_finite:
                passed = False
                break
            controller.ctrl_step(targets)
            min_height = min(min_height, float(controller.mj_data.qpos[2]))
            w, x, y, z = controller.mj_data.qpos[3:7]
            min_up = min(min_up, float(1 - 2 * (x*x + y*y)))
            if not np.isfinite(controller.mj_data.qpos).all() or min_height < 0.25 or min_up < 0.5:
                passed = False
                break
        route = controller.policy.active_route
        passed = passed and targets_finite and route == expected_route
        result = dict(phase=name, command=cmd, route=route,
                      min_base_height=min_height, min_upright_cosine=min_up, passed=passed)
        results.append(result)
        print(json.dumps(result), flush=True)
        if not passed:
            break
    controller.stop()
    report = dict(passed=passed, seconds_per_phase=args.seconds,
                  gain_profile='real_robot' if args.real_robot_gains else 'simulation',
                  results=results)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
