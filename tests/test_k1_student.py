"""Slow student ONNX contract and pass/shoot sensor-path checks."""

import math
import unittest
from unittest.mock import patch

import torch

from booster_deploy.controllers.base_controller import BaseController
from booster_deploy.utils.proprioception import read_proprioception
from tasks.locomotion.robots.k1.student import K1StudentTaskCfg


class StudentTests(unittest.TestCase):
    def setUp(self):
        self.controller = BaseController(K1StudentTaskCfg())
        robot = self.controller.robot
        robot.data.joint_pos = robot.default_joint_pos.clone()
        robot.data.joint_vel.zero_()
        robot.data.root_quat_w = torch.tensor([1., 0., 0., 0.])
        robot.data.root_ang_vel_b = torch.tensor([0.1, 0.2, 0.3])
        self.ball = (0.6, 0.8)
        self.controller.get_ball_position = lambda max_age: self.ball
        self.controller.start()

    def test_config_and_frame_layout(self):
        cfg = self.controller.cfg
        self.assertEqual(cfg.policy.command, (1.0, 0.0, 1.0))
        self.assertEqual(cfg.policy.gait_frequency, 1.8)
        self.assertEqual(cfg.robot.joint_stiffness[10], 200.0)
        self.assertEqual(cfg.robot.joint_damping[10], 5.0)
        self.assertEqual(cfg.booster.joint_stiffness, cfg.robot.joint_stiffness)
        self.assertEqual(cfg.booster.joint_damping, cfg.robot.joint_damping)
        self.assertEqual(cfg.policy.policy_joint_names, cfg.robot.joint_names[2:])

        policy = self.controller.policy
        frame = policy.compute_observation()
        q, dq, gyro, gravity = read_proprioception(self.controller.robot.data)
        self.assertEqual(frame.numel(), 81)
        torch.testing.assert_close(frame[:3], gravity)
        torch.testing.assert_close(frame[3:6], gyro)
        torch.testing.assert_close(frame[6:9], torch.tensor([1., 0., 1.]))
        phase = 0.02 * 1.8 * 2 * math.pi
        torch.testing.assert_close(frame[9:11], torch.tensor([math.cos(phase), math.sin(phase)]))
        torch.testing.assert_close(frame[11:33], q - self.controller.robot.default_joint_pos)
        torch.testing.assert_close(frame[33:55], dq * 0.1)
        torch.testing.assert_close(frame[55:77], torch.zeros(22))
        torch.testing.assert_close(frame[77:79], torch.tensor([0.6, 0.8]))
        torch.testing.assert_close(frame[79:81], torch.tensor([0.72, 0.96]))

    def test_zero_first_history_head_feedback_and_missing_ball(self):
        policy = self.controller.policy
        targets = self.controller.policy_step()
        self.assertEqual(tuple(targets.shape), (22,))
        self.assertTrue(torch.isfinite(targets).all())
        self.assertEqual(tuple(policy.obs_history.shape), (50, 81))
        self.assertEqual(torch.count_nonzero(policy.obs_history).item(), 0)
        policy.set_head_target((0.2, 0.3))
        self.ball = None
        targets = self.controller.policy_step()
        self.assertTrue(torch.isfinite(targets).all())
        self.assertEqual(policy.frame_count, 2)
        torch.testing.assert_close(policy.obs_history[-1, 55:57],
                                   torch.tensor([0.1988, 0.2986]))
        torch.testing.assert_close(policy.obs_history[-1, 77:79], torch.tensor([0.6, 0.8]))
        torch.testing.assert_close(policy.obs_history[-1, 79:81], torch.tensor([0.72, 0.96]))

    def test_ball_reference_survives_long_loss_invalid_updates_and_reset(self):
        policy = self.controller.policy
        with patch.object(policy, '_forward_model', return_value=torch.zeros(20)) as actor:
            self.controller.policy_step()
            self.controller.policy_step()
            reference = policy.obs_history[-1, 77:81].clone()
            phase = policy.phase
            self.ball = None
            # Outlast both the 0.5 s freshness window and all 50 history frames.
            for _ in range(60):
                history = policy.obs_history.clone()
                self.controller.policy_step()
                torch.testing.assert_close(policy.obs_history[:-1], history[1:])
                torch.testing.assert_close(policy.obs_history[-1, 77:81], reference)
            self.assertEqual(actor.call_count, 62)
            self.assertEqual(policy.frame_count, 62)
            self.assertNotAlmostEqual(policy.phase, phase)
            for invalid in ((float('nan'), 0.), (0., float('inf'))):
                self.ball = invalid
                self.controller.policy_step()
                torch.testing.assert_close(policy.obs_history[-1, 77:81], reference)
            self.ball = None
            self.controller.start()
            self.controller.policy_step()  # preserve the zero first-history contract
            self.controller.policy_step()
            torch.testing.assert_close(policy.obs_history[-1, 77:81], reference)
            self.ball = (0.8, -0.6)
            self.controller.policy_step()
            torch.testing.assert_close(policy.obs_history[-1, 77:81],
                                       torch.tensor([0.8, -0.6, 0.96, -0.72]))

    def test_hold_before_first_ball(self):
        self.ball = None
        targets = self.controller.policy_step()
        torch.testing.assert_close(targets, self.controller.robot.data.joint_pos)
        self.assertIsNone(self.controller.policy.obs_history)


if __name__ == "__main__":
    unittest.main()
