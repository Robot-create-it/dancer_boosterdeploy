"""MuJoCo integration checks for the K1 shoot scene and policy."""

import math
import unittest

import mujoco
import numpy as np

from booster_deploy.controllers.k1_shoot_mujoco_controller import K1ShootMujocoController
from tasks.locomotion.robots.k1.shooting import K1ShootTaskCfg


class ShootSimTests(unittest.TestCase):
    def setUp(self):
        self.controller = K1ShootMujocoController(K1ShootTaskCfg())

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
        self.assertGreater(c.mj_data.qpos[8], 0.0)
        self.assertTrue(np.isfinite(c.mj_data.qpos).all())

    def test_ball_coordinates_follow_robot_yaw(self):
        c = self.controller
        c.mj_data.qpos[3:7] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
        mujoco.mj_forward(c.mj_model, c.mj_data)
        np.testing.assert_allclose(
            c._ball_in_robot_frame(c.mj_data.xpos[c._trunk_body_id])[:2],
            (0.0, -0.8), atol=1e-6)
        self.assertIsNone(c.get_ball_position(0.5))

    def test_ball_outside_fov_holds_policy_and_starts_head_scan(self):
        c = self.controller
        ball_qpos = c.mj_model.joint("ball-root").qposadr[0]
        c.mj_data.qpos[ball_qpos:ball_qpos + 3] = [-0.8, 0, 0.11]
        mujoco.mj_forward(c.mj_model, c.mj_data)
        self.assertIsNone(c.get_ball_position(0.5))
        c.update_state()
        c.start()
        np.testing.assert_allclose(c.policy_step().numpy(), c.robot.data.joint_pos.numpy())
        self.assertIsNone(c.policy.obs_history)
        c.ctrl_step(c.robot.data.joint_pos.clone())
        self.assertEqual(c._head_tracker.state, "hold")
        for _ in range(30):
            c.update_state()
            c.ctrl_step(c.robot.data.joint_pos.clone())
        self.assertEqual(c._head_tracker.state, "scan")

    def test_turning_head_brings_side_ball_into_fov(self):
        c = self.controller
        ball_qpos = c.mj_model.joint("ball-root").qposadr[0]
        c.mj_data.qpos[ball_qpos:ball_qpos + 3] = [0.8, 1.5, 0.11]
        mujoco.mj_forward(c.mj_model, c.mj_data)
        self.assertIsNone(c.get_ball_position(0.5))
        c.mj_data.qpos[7:9] = [0.8, 0.5]
        mujoco.mj_forward(c.mj_model, c.mj_data)
        np.testing.assert_allclose(c.get_ball_position(0.5), (0.8, 1.5), atol=1e-4)


if __name__ == "__main__":
    unittest.main()
