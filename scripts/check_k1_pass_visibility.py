"""Compare K1 pass balance with a visible ball, no ball, and ball loss."""

import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
import numpy as np

from booster_deploy.controllers.k1_pass_mujoco_controller import K1PassMujocoController
from tasks.locomotion.robots.k1.passing import K1PassTaskCfg


def run_scenario(name: str, seconds: float, loss_at: float) -> dict:
    cfg = K1PassTaskCfg()
    cfg.mujoco.ball_init_xy = [-2.0, 0.0] if name == "never_visible" else [0.8, 0.0]
    controller = K1PassMujocoController(cfg)
    ball_joint = controller.mj_model.joint("ball-root")
    ball_qpos = int(ball_joint.qposadr[0])
    ball_qvel = int(ball_joint.dofadr[0])
    controller.update_state()
    controller.start()

    visible_steps = hold_steps = 0
    min_height, min_upright = float("inf"), 1.0
    first_height_below_035 = first_upright_below_07 = None
    samples = []
    for step in range(round(seconds / cfg.policy_dt)):
        now = float(controller.mj_data.time)
        if name == "lost_after" and step == round(loss_at / cfg.policy_dt):
            # Move the ball behind the robot and clear its momentum.
            controller.mj_data.qpos[ball_qpos:ball_qpos + 7] = [-2, 0, .11, 1, 0, 0, 0]
            controller.mj_data.qvel[ball_qvel:ball_qvel + 6] = 0
            mujoco.mj_forward(controller.mj_model, controller.mj_data)

        controller.update_state()
        visible = controller.get_ball_position(cfg.policy.ball_max_age) is not None
        visible_steps += visible
        targets = controller.policy_step()
        hold_steps += controller.policy.obs_history is None
        controller.ctrl_step(targets)

        qpos = controller.mj_data.qpos
        height = float(qpos[2])
        upright = float(1 - 2 * (qpos[4] ** 2 + qpos[5] ** 2))
        elapsed = float(controller.mj_data.time)
        min_height = min(min_height, height)
        min_upright = min(min_upright, upright)
        if height < .35 and first_height_below_035 is None:
            first_height_below_035 = elapsed
        if upright < .7 and first_upright_below_07 is None:
            first_upright_below_07 = elapsed
        if (step + 1) % round(1 / cfg.policy_dt) == 0:
            samples.append({"time": round(elapsed, 2), "height": round(height, 3),
                            "upright": round(upright, 3), "ball_visible": visible})
        if not np.isfinite(qpos).all():
            raise RuntimeError(f"Non-finite MuJoCo state in {name} at {elapsed:.2f} s")
        if not controller.is_running:
            break

    safety_stopped = not controller.is_running
    controller.stop()
    return {
        "scenario": name,
        "simulated_seconds": round(elapsed, 3),
        "visible_steps": int(visible_steps),
        "hold_steps": int(hold_steps),
        "first_height_below_0_35": (
            round(first_height_below_035, 3) if first_height_below_035 is not None else None),
        "first_upright_below_0_7": (
            round(first_upright_below_07, 3) if first_upright_below_07 is not None else None),
        "minimum_height": round(min_height, 3),
        "minimum_upright_cosine": round(min_upright, 3),
        "safety_stopped": safety_stopped,
        "samples": samples,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--loss-at", type=float, default=1.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if (not math.isfinite(args.seconds) or not math.isfinite(args.loss_at)
            or args.seconds < 1 or not .02 <= args.loss_at < args.seconds):
        parser.error("Require finite --seconds >= 1 and 0.02 <= --loss-at < --seconds")
    results = [run_scenario(name, args.seconds, args.loss_at)
               for name in ("never_visible", "lost_after", "visible")]
    report = {"seconds_requested": args.seconds, "loss_at": args.loss_at,
              "results": results}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
