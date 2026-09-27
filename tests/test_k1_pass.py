"""Vision and actor contract checks for the K1 pass deployment."""

import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import unittest

import torch

from booster_deploy.controllers.base_controller import BaseController
from booster_deploy.utils.vision_ball import select_ball
from tasks.locomotion.locomotion import LocomotionPolicy
from tasks.locomotion.robots.k1.passing import K1PassTaskCfg


class VisionTests(unittest.TestCase):
    def test_demo_detection_selects_best_projected_ball(self):
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


class PassTests(unittest.TestCase):
    def setUp(self):
        self.controller = BaseController(K1PassTaskCfg())
        self.controller.robot.data.joint_pos = self.controller.robot.default_joint_pos.clone()
        self.controller.robot.data.joint_vel.zero_()
        self.controller.robot.data.root_quat_w = torch.tensor([1., 0., 0., 0.])
        self.controller.robot.data.root_ang_vel_b.zero_()
        self.ball = (0.6, 0.8)
        self.controller.get_ball_position = lambda max_age: self.ball
        self.controller.start()

    def test_71_dim_frame_uses_loco_sensors_and_five_visual_features(self):
        policy = self.controller.policy
        policy.robot.data.root_ang_vel_b[:] = torch.tensor([0.1, 0.2, 0.3])
        loco = LocomotionPolicy.compute_observation(policy)
        frame = policy.compute_observation()
        self.assertEqual(frame.numel(), 71)
        torch.testing.assert_close(frame[:66], torch.cat((loco[:6], loco[9:])))
        expected = torch.tensor([1.2, 1.6, 0.6, 0.8, 1.0])
        torch.testing.assert_close(frame[66:], expected)
        self.assertEqual(policy.compute_observation()[-1].item(), -1.0)

    def test_power_two_actor_continues_with_last_ball(self):
        cfg = self.controller.cfg
        self.assertEqual(cfg.robot.joint_stiffness[10], 100.0)
        self.assertEqual(cfg.robot.joint_damping[14], 1.0)
        self.assertTrue(cfg.policy.checkpoint_path.endswith("k1_passing_policy_2.onnx"))
        targets = self.controller.policy_step()
        self.assertEqual(targets.numel(), 22)
        self.assertTrue(torch.isfinite(targets).all())
        self.assertEqual(tuple(self.controller.policy.obs_history.shape), (10, 71))
        self.ball = None
        targets = self.controller.policy_step()
        self.assertTrue(torch.isfinite(targets).all())
        self.assertEqual(self.controller.policy.frame_count, 2)
        torch.testing.assert_close(self.controller.policy.obs_history[-1, 66:],
                                   torch.tensor([1.2, 1.6, 0.6, 0.8, -1.0]))

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
