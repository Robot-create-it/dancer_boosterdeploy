"""MuJoCo integration checks for the K1 pass scene and policy."""

import math
import unittest

import mujoco
import numpy as np

from booster_deploy.controllers.k1_pass_mujoco_controller import K1PassMujocoController
from tasks.locomotion.robots.k1.passing import K1PassTaskCfg


class PassSimTests(unittest.TestCase):
    def setUp(self):
        self.controller = K1PassMujocoController(K1PassTaskCfg())

    def test_scene_policy_and_head_control(self):
        c = self.controller
        self.assertEqual((c.mj_model.nq, c.mj_model.nu), (36, 22))
        np.testing.assert_allclose(c.get_ball_position(0.5), (0.8, 0.0), atol=1e-6)
        self.assertAlmostEqual(c.mj_model.geom("ball").size[0], 0.11)
        c.update_state()
        c.start()
        for _ in range(10):
            c.update_state()
            targets = c.policy_step()
            self.assertEqual(tuple(targets.shape), (22,))
            self.assertTrue(np.isfinite(targets.numpy()).all())
            c.ctrl_step(targets)
        self.assertEqual(tuple(c.policy.obs_history.shape), (10, 71))
        self.assertGreater(c.mj_data.qpos[8], 0.0)  # head pitches down to the ball
        self.assertTrue(np.isfinite(c.mj_data.qpos).all())

    def test_ball_coordinates_follow_robot_yaw(self):
        c = self.controller
        c.mj_data.qpos[3:7] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
        mujoco.mj_forward(c.mj_model, c.mj_data)
        np.testing.assert_allclose(c.get_ball_position(0.5), (0.0, -0.8), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
