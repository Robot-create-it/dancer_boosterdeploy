"""Vision and actor contract checks for the K1 shoot deployment."""

import math
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import ast

import torch

from booster_deploy.controllers.base_controller import BaseController
from booster_deploy.utils.vision_ball import select_ball
from tasks.locomotion.locomotion import LocomotionPolicy
from tasks.locomotion.robots.k1.shooting import (
    K1ShootTaskCfg, SHOOT_POLICY_IDS, select_shoot_policy,
)


class ShootTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = BaseController(K1ShootTaskCfg())

    def setUp(self):
        self.controller.robot.data.joint_pos = self.controller.robot.default_joint_pos.clone()
        self.controller.robot.data.joint_vel.zero_()
        self.controller.robot.data.root_quat_w = torch.tensor([1., 0., 0., 0.])
        self.controller.robot.data.root_ang_vel_b.zero_()
        self.ball = (0.6, 0.8)
        self.controller.get_ball_position = lambda max_age: self.ball
        self.controller.start()

    def test_same_vision_selection_as_pass(self):
        def obj(label, confidence, projection):
            return SimpleNamespace(label=label, confidence=confidence,
                                   position_projection=projection)
        detections = SimpleNamespace(detected_objects=[
            obj("Ball", 90, [float("nan"), 0]),
            obj("Goalpost", 99, [0.1, 0.2]),
            obj("Ball", 59, [0.1, 0.2]),
            obj("Ball", 70, [0.4, -0.2]),
            obj("Ball", 95, [16, 0]),
        ])
        self.assertEqual(select_ball(detections), (0.4, -0.2))

    def test_71_dim_frame_uses_shoot_route_zero_offsets_and_speed_one(self):
        policy = self.controller.policy
        policy.robot.data.root_ang_vel_b[:] = torch.tensor([0.1, 0.2, 0.3])
        loco = LocomotionPolicy.compute_observation(policy)
        frame = policy.compute_observation()
        self.assertEqual(frame.numel(), 71)
        torch.testing.assert_close(frame[:66], torch.cat((loco[:6], loco[9:])))
        direction = math.atan2(0.8, 0.6) + 0.1
        expected = torch.tensor([math.cos(direction), math.sin(direction),
                                 0.6, 0.75, 1.0])
        torch.testing.assert_close(frame[66:], expected)
        self.assertEqual(policy.compute_observation()[-1].item(), -1.0)

    def test_power_six_actor_and_lost_ball_hold(self):
        cfg = self.controller.cfg
        self.assertEqual(cfg.policy.kick_power, 6.0)
        self.assertEqual(cfg.robot.joint_stiffness[10], 100.0)
        self.assertEqual(cfg.robot.joint_damping[14], 1.0)
        self.assertTrue(cfg.policy.checkpoint_path.endswith("k1_shoot_policy_2.onnx"))
        targets = self.controller.policy_step()
        self.assertEqual(targets.numel(), 22)
        self.assertTrue(torch.isfinite(targets).all())
        self.assertEqual(tuple(self.controller.policy.obs_history.shape), (10, 71))
        self.ball = None
        held = self.controller.policy_step()
        torch.testing.assert_close(held, self.controller.robot.data.joint_pos)
        self.assertIsNone(self.controller.policy.obs_history)

    def test_each_shoot_model_uses_its_own_offsets_and_runs(self):
        expected = {
            "2": ((0.0, -0.05), 0.1),
            "264": ((0.0, 0.05), -0.1),
            "192": ((0.0, 0.08), 0.0),
            "0109_0": ((0.05, -0.05), 0.1),
            "0109_2": ((0.05, 0.02), 0.2),
        }
        self.assertEqual(tuple(expected), SHOOT_POLICY_IDS)
        for policy_id, (ball_offset, yaw_offset) in expected.items():
            with self.subTest(policy=policy_id):
                cfg = K1ShootTaskCfg()
                select_shoot_policy(cfg, policy_id)
                self.assertTrue(cfg.policy.checkpoint_path.endswith(
                    f"k1_shoot_policy_{policy_id}.onnx"))
                self.assertEqual(cfg.policy.ball_pos_offset, ball_offset)
                self.assertEqual(cfg.policy.kick_yaw_offset, yaw_offset)
                self.assertEqual(cfg.policy.kick_power, 6.0)
                controller = BaseController(cfg)
                controller.get_ball_position = lambda max_age: (0.6, 0.8)
                controller.start()
                frame = controller.policy.compute_observation()
                direction = math.atan2(0.8, 0.6) + yaw_offset
                expected_command = torch.tensor([
                    math.cos(direction), math.sin(direction),
                    0.6 + ball_offset[0], 0.8 + ball_offset[1], 1.0,
                ])
                torch.testing.assert_close(frame[66:], expected_command)
                targets = controller.policy_step()
                self.assertEqual(targets.numel(), 22)
                self.assertTrue(torch.isfinite(targets).all())

    def test_unknown_shoot_model_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unknown shoot policy"):
            select_shoot_policy(K1ShootTaskCfg(), "unknown")

    def test_walking_preparation_uses_nested_loco(self):
        source = (Path(__file__).resolve().parents[1] /
                  "booster_deploy/controllers/booster_robot_controller.py")
        tree = ast.parse(source.read_text(encoding="utf-8"))
        portal = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                      and n.name == "BoosterRobotPortal")
        method = next(n for n in portal.body if isinstance(n, ast.FunctionDef)
                      and n.name == "_build_prepare_cfg")
        namespace = {"deepcopy": deepcopy}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        prepared = namespace["_build_prepare_cfg"](
            SimpleNamespace(cfg=self.controller.cfg))
        self.assertEqual(len(prepared.policy.model_paths), 3)
        self.assertTrue(prepared.policy.checkpoint_path.endswith("k1_loco_base.onnx"))
        self.assertEqual(prepared.robot.joint_stiffness[10], 100.0)


if __name__ == "__main__":
    unittest.main()
