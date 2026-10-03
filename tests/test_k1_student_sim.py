"""MuJoCo scene and policy smoke for the slow student."""

import unittest

import numpy as np
import torch

from booster_deploy.controllers.k1_student_mujoco_controller import K1StudentMujocoController
from tasks.locomotion.robots.k1.student import K1StudentTaskCfg


class StudentSimTests(unittest.TestCase):
    def test_leg_torque_limits_match_demo(self):
        controller = K1StudentMujocoController(K1StudentTaskCfg())
        targets = torch.as_tensor(
            controller.mj_data.qpos[7:29].copy() + 10, dtype=torch.float32
        )
        controller.ctrl_step(targets)
        np.testing.assert_allclose(
            controller.mj_data.ctrl[10:], [30, 35, 20, 40, 20, 20] * 2
        )

    def test_scene_policy_and_head_control(self):
        controller = K1StudentMujocoController(K1StudentTaskCfg())
        self.assertEqual((controller.mj_model.nq, controller.mj_model.nu), (36, 22))
        np.testing.assert_allclose(controller.get_ball_position(0.5), (0.8, 0.0))
        controller.update_state()
        controller.start()
        for _ in range(10):
            controller.update_state()
            targets = controller.policy_step()
            self.assertTrue(np.isfinite(targets.numpy()).all())
            controller.ctrl_step(targets)
        self.assertEqual(tuple(controller.policy.obs_history.shape), (50, 81))
        self.assertGreater(controller.mj_data.qpos[8], 0.0)
        self.assertTrue(np.isfinite(controller.mj_data.qpos).all())


if __name__ == "__main__":
    unittest.main()
