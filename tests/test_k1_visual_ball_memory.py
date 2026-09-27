"""Exercise the network-input lifecycle shared by pass and shoot."""

import unittest
from unittest.mock import patch

import torch

from booster_deploy.controllers.base_controller import BaseController
from tasks.locomotion.robots.k1.passing import K1PassTaskCfg
from tasks.locomotion.robots.k1.shooting import K1ShootTaskCfg


class VisualBallMemoryTests(unittest.TestCase):
    def make_controller(self, cfg_type):
        controller = BaseController(cfg_type())
        controller.robot.data.joint_pos = controller.robot.default_joint_pos.clone()
        controller.robot.data.joint_vel.zero_()
        controller.robot.data.root_quat_w = torch.tensor([1., 0., 0., 0.])
        controller.robot.data.root_ang_vel_b.zero_()
        self.ball = None
        controller.get_ball_position = lambda max_age: self.ball
        controller.start()
        return controller

    def test_missing_before_first_ball_keeps_existing_startup_behavior(self):
        for cfg_type in (K1PassTaskCfg, K1ShootTaskCfg):
            with self.subTest(task=cfg_type.__name__):
                c = self.make_controller(cfg_type)
                with patch.object(c.policy, '_forward_model') as actor:
                    for _ in range(3):
                        torch.testing.assert_close(c.policy_step(), c.robot.data.joint_pos)
                    actor.assert_not_called()
                self.assertIsNone(c.policy.obs_history)
                self.ball = (.6, .2)
                self.assertTrue(torch.isfinite(c.policy_step()).all())
                self.assertEqual(c.policy.frame_count, 1)

    def test_long_loss_keeps_inference_history_phase_and_raw_ball(self):
        for cfg_type in (K1PassTaskCfg, K1ShootTaskCfg):
            with self.subTest(task=cfg_type.__name__):
                c = self.make_controller(cfg_type)
                self.ball = (3., 4.)  # exercise offsets before the 2 m normalisation
                c.policy_step()
                command = c.policy.obs_history[-1, 66:70].clone()
                self.ball = None
                with patch.object(c.policy, '_forward_model',
                                  wraps=c.policy._forward_model) as actor:
                    # Longer than the 0.5 s admission window; no cache expiry.
                    for i in range(1, 61):
                        history = c.policy.obs_history.clone()
                        targets = c.policy_step()
                        self.assertTrue(torch.isfinite(targets).all())
                        torch.testing.assert_close(c.policy.obs_history[:-1], history[1:])
                        torch.testing.assert_close(c.policy.obs_history[-1, 66:70], command)
                        self.assertEqual(c.policy.obs_history[-1, 70].item(),
                                         1. if i % 2 == 0 else -1.)
                    self.assertEqual(actor.call_count, 60)
                self.assertEqual(c.policy.frame_count, 61)
                self.assertFalse(torch.allclose(targets, c.robot.data.joint_pos))
                self.ball = (.4, -.2)
                previous = c.policy.obs_history.clone()
                c.policy_step()
                torch.testing.assert_close(c.policy.obs_history[:-1], previous[1:])
                offset = getattr(c.cfg.policy, 'ball_pos_offset', (0., 0.))
                torch.testing.assert_close(c.policy.obs_history[-1, 68:70],
                                           torch.tensor([.4 + offset[0], -.2 + offset[1]]))
                self.assertEqual(c.policy.frame_count, 62)

    def test_invalid_updates_and_policy_reset_do_not_erase_reference(self):
        for cfg_type in (K1PassTaskCfg, K1ShootTaskCfg):
            with self.subTest(task=cfg_type.__name__):
                c = self.make_controller(cfg_type)
                self.ball = (.6, .2)
                c.policy_step()
                command = c.policy.obs_history[-1, 66:70].clone()
                for invalid in ((float('nan'), 0.), (0., float('inf')), None):
                    self.ball = invalid
                    c.policy_step()
                    torch.testing.assert_close(c.policy.obs_history[-1, 66:70], command)
                c.start()  # policy reset, not a new motion-layer instance
                c.policy_step()
                torch.testing.assert_close(c.policy.obs_history[-1, 66:70], command)
                self.assertEqual(c.policy.frame_count, 1)


if __name__ == '__main__':
    unittest.main()
