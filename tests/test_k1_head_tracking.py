"""Exercise vision freshness and head targets without ROS or physical motion."""

from dataclasses import replace
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock

import torch

from booster_deploy.controllers.controller_cfg import HeadTrackingCfg
from booster_deploy.utils.head_ball_tracker import HeadBallTracker
from booster_deploy.utils.vision_ball import BallObservation, select_ball_observation
from booster_deploy.utils.vision_config import load_vision_config, camera_topics
from tasks.locomotion.robots.k1.passing import K1PassTaskCfg
from test_k1_loco_robot import ROBOT, make_portal


def detection(stamp=100., objects=None):
    if objects is None:
        objects = [SimpleNamespace(label="Ball", confidence=90.,
                    position_projection=[0.6, 0.2], xmin=590, xmax=630, ymin=440, ymax=470)]
    sec = int(stamp)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=sec, nanosec=round((stamp-sec)*1e9))), detected_objects=objects)


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = HeadBallTracker(HeadTrackingCfg(fx=320., fy=320., enabled=True))
        self.ball = BallObservation(.6, .2, 90., (590, 440, 630, 470), 100., 100.)

    def test_direction_and_rate_limit(self):
        yaw, pitch = self.tracker.update(100., .02, (0., .4), self.ball, (640, 480))
        self.assertLess(yaw, 0.)  # ball right -> turn right
        self.assertGreater(pitch, .4)  # ball below -> look down
        self.assertLessEqual(abs(yaw), .012001)
        self.assertLessEqual(abs(pitch-.4), .012001)
        desired = self.tracker.target.copy()
        self.tracker.update(100.02, .02, (yaw, pitch), self.ball, (640, 480))
        self.assertEqual(self.tracker.target, desired)  # no re-integration of one image

    def test_centred_detection_cancels_scan(self):
        self.tracker.update(99., .02, (0., .4), None, (640, 480))
        self.tracker.update(100., .02, (0., .4), None, (640, 480))
        self.assertEqual(self.tracker.state, "scan")
        centered = replace(self.ball, bbox=(300, 220, 340, 260))
        self.tracker.update(100.02, .02, (0., .4), centered, (640, 480))
        self.assertEqual(self.tracker.state, "track")
        self.assertEqual(self.tracker.target, [0., .4])

    def test_hold_scan_limits_and_camera_loss(self):
        self.tracker.update(100., .02, (0., .4), self.ball, (640, 480))
        self.tracker.update(100.2, .02, (0., .4), None, (640, 480))
        self.assertEqual(self.tracker.state, "hold")
        for n in range(300):
            command = self.tracker.update(101.+n*.02, .02, (0., .4), None, (640, 480))
            self.assertTrue(-1 <= command[0] <= 1)
            self.assertTrue(.2 <= command[1] <= .85)
        self.assertEqual(self.tracker.state, "scan")
        self.assertEqual(command, self.tracker.update(108., .02, (0., .4), None, (0, 0)))


class VisionFreshnessTests(unittest.TestCase):
    def test_delayed_future_and_invalid_frames(self):
        self.assertIsNone(select_ball_observation(detection(99.), 20., 100.))
        self.assertIsNone(select_ball_observation(detection(101.), 20., 100.))
        ball = select_ball_observation(detection(99.8), 20., 100.)
        self.assertAlmostEqual(ball.stamp, 19.8)
        self.assertEqual(ball.bbox, (590., 440., 630., 470.))
        self.assertIsNone(select_ball_observation(detection(objects=[]), 20., 100.))

    def test_camera_config_merges_local_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)
            (p/'vision.yaml').write_text('camera:\n  type: d-robotics\n  intrin: {fx: 200, fy: 201}\nuse_depth: true\n')
            (p/'vision_local.yaml').write_text('camera:\n  intrin: {fx: 202}\n')
            cfg = load_vision_config(p)
            self.assertEqual(cfg['camera']['intrin'], {'fx': 202, 'fy': 201})
            self.assertEqual(camera_topics(cfg)[0], '/StereoNetNode/rectified_image')


class HeadControllerTests(unittest.TestCase):
    def setUp(self):
        cfg = K1PassTaskCfg()
        cfg.booster.head_tracking.fx = cfg.booster.head_tracking.fy = 320.
        self.portal = make_portal(cfg)
        self.portal._vision_node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
        self.controller = ROBOT.BoosterRobotController(self.portal.cfg, self.portal)
        self.controller.update_state()
        self.controller.robot.data.joint_pos[1] = .4
        state = self.portal.synced_vision_state.read()
        state[0]['width'], state[0]['height'] = 640, 480
        state[0]['image_received'] = state[0]['pose_received'] = 20.
        self.portal.synced_vision_state.write(state)

    def tearDown(self):
        self.portal._cleanup_done = True

    def test_detection_to_motor_command_and_stale_pose(self):
        with patch.object(ROBOT.time, 'monotonic', return_value=20.):
            self.portal._vision_handler(detection())
            self.assertEqual(self.controller.get_ball_position(.5), (.6, .2))
            targets = torch.arange(22, dtype=torch.float32) * .01
            original = targets.clone()
            self.controller.ctrl_step(targets)
            torch.testing.assert_close(targets, original)
            self.assertLess(self.portal.motor_cmd[0].q, 0.)
            self.assertGreater(self.portal.motor_cmd[1].q, .4)
            self.assertEqual(self.portal.motor_cmd[0].kp, 20.)
            self.assertEqual(self.portal.motor_cmd[0].kd, 2.)
            self.assertEqual([self.portal.motor_cmd[i].weight for i in (0, 1)], [1., 1.])
            for i in range(2, 22):
                self.assertAlmostEqual(self.portal.motor_cmd[i].q, float(targets[i]))
                self.assertEqual(self.portal.motor_cmd[i].weight, 0.)
            state = self.portal.synced_vision_state.read()
            state[0]['pose_received'] = 19.
            self.portal.synced_vision_state.write(state)
            self.assertIsNone(self.controller.get_ball_position(.5))

    def test_replayed_detection_does_not_refresh_and_empty_frame_clears(self):
        with patch.object(ROBOT.time, 'monotonic', return_value=20.):
            self.portal._vision_handler(detection(99.9))
            first = self.portal.synced_ball.read()[0]['stamp']
        with patch.object(ROBOT.time, 'monotonic', return_value=20.2):
            self.portal._vision_handler(detection(99.9))
            self.assertEqual(self.portal.synced_ball.read()[0]['stamp'], first)
            self.portal._vision_handler(detection(100., []))
            self.assertIsNone(self.controller.get_ball_position(.5))

    def test_motion_retains_ball_while_head_rejects_missing_and_stale_data(self):
        with patch.object(ROBOT.time, 'monotonic', return_value=20.):
            self.portal._vision_handler(detection(99.9))
            self.controller.start()
            self.controller.policy_step()
            command = self.controller.policy.obs_history[-1, 66:70].clone()
        with patch.object(ROBOT.time, 'monotonic', return_value=20.2):
            self.portal._vision_handler(detection(100., []))
            self.assertIsNone(self.controller.get_ball_observation(.5))
            self.controller.policy_step()
        # Even far past image/pose timeout, the actor gets its retained XY.
        with patch.object(ROBOT.time, 'monotonic', return_value=200.):
            self.assertIsNone(self.controller.get_ball_observation(.5))
            targets = self.controller.policy_step()
            self.assertTrue(torch.isfinite(targets).all())
            torch.testing.assert_close(self.controller.policy.obs_history[-1, 66:70], command)
            self.assertEqual(self.controller.policy.frame_count, 3)

    def test_head_continues_while_pass_holds_body_and_across_handoff(self):
        with patch.object(ROBOT.time, 'monotonic', return_value=20.):
            self.controller.start()
            targets = self.controller.policy_step()  # no ball: hold measured body
            self.controller.ctrl_step(targets)
        state = self.portal.synced_vision_state.read()
        state[0]['image_received'] = state[0]['pose_received'] = 21.
        self.portal.synced_vision_state.write(state)
        with patch.object(ROBOT.time, 'monotonic', return_value=21.):
            self.controller.ctrl_step(self.controller.policy_step())
        self.assertEqual(self.portal.head_tracker.state, 'scan')
        self.assertGreater(self.portal.motor_cmd[0].q, 0.)
        self.assertEqual(self.portal.motor_cmd[0].weight, 1.)
        for i in range(2, 22):
            self.assertAlmostEqual(self.portal.motor_cmd[i].q, float(self.controller.robot.data.joint_pos[i]))
        tracker = self.portal.head_tracker
        prepared = self.portal._build_prepare_cfg()
        ROBOT.BoosterRobotController(prepared, self.portal)
        self.assertIs(self.portal.head_tracker, tracker)
        self.assertEqual(prepared.robot.joint_stiffness[:2], [20., 20.])

    def test_head_only_does_not_accept_a_trigger(self):
        portal = self.portal
        portal.cfg.booster.head_only = True
        portal.start_custom_mode_conditionally = Mock(return_value=True)
        portal.start_rl_gait_conditionally = Mock(return_value=True)
        portal._change_robot_mode = Mock(return_value=True)
        portal.inference_process = Mock()
        portal.inference_process.is_alive.return_value = True
        portal.remoteControlService.start_rl_gait.return_value = True
        def finish(_):
            portal.exit_event.set()
        with patch.object(ROBOT.time, 'sleep', side_effect=finish):
            portal.run()
        self.assertFalse(portal.task_start_event.is_set())
        self.assertFalse(portal.velocity_commands_enabled_event.is_set())


if __name__ == '__main__':
    unittest.main()
