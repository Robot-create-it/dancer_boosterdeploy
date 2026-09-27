"""End-to-end FDR getup using the same MuJoCo state/control loop as loco."""

import importlib.util
import unittest

import torch

from tasks.locomotion.robots.k1.recovery import K1RecoveryTaskCfg


@unittest.skipUnless(
    importlib.util.find_spec('mujoco') and importlib.util.find_spec('booster_assets'),
    'MuJoCo and booster_assets are required',
)
class RecoverySimTests(unittest.TestCase):
    def run_getup(self, posture):
        from booster_deploy.controllers.mujoco_controller import MujocoController
        from booster_deploy.utils.proprioception import read_proprioception

        cfg = K1RecoveryTaskCfg()
        if posture == 'faceup':
            cfg.mujoco.init_quat[2] *= -1
        c = MujocoController(cfg)
        c.update_state()
        c.start()
        for _ in range(450):
            c.update_state()
            targets = c.policy_step()
            self.assertTrue(c.is_running)
            self.assertTrue(torch.isfinite(targets).all())
            c.ctrl_step(targets)
            if c.policy.state == 'succeeded':
                break
        self.assertEqual(c.policy.posture, posture)
        self.assertEqual(c.policy.state, 'succeeded')
        self.assertEqual(c.policy.retries, 0)
        c.update_state()
        self.assertLess(read_proprioception(c.robot.data)[3][2].item(), -0.5)
        self.assertGreater(c.robot.data.root_pos_w[2].item(), 0.4)
        # A finished standalone task keeps commanding its final pose.
        torch.testing.assert_close(c.policy_step(), targets)

    def test_faceup_getup(self):
        self.run_getup('faceup')

    def test_facedown_getup(self):
        self.run_getup('facedown')


if __name__ == '__main__':
    unittest.main()
