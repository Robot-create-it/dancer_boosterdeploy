"""Recovery fault injection and command tests with in-memory transport only."""

from pathlib import Path
import tempfile
from threading import Event, Timer
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

from booster_deploy.utils.recovery_safety import feedback_fault, limit_arm_targets
from booster_deploy.utils.recovery_trace import RecoveryTrace, summarize_trace
from tasks.locomotion.robots.k1.recovery import K1RecoveryTaskCfg
from tests.test_k1_loco_robot import ROBOT, make_portal


def sample():
    def motor():
        return SimpleNamespace(q=0.1, dq=0., tau_est=0., temperature=40)
    return SimpleNamespace(imu_state=SimpleNamespace(rpy=[0., 1.5, 0.], gyro=[0., 0., 0.]),
                           motor_state_serial=[motor() for _ in range(22)],
                           motor_state_parallel=[motor() for _ in range(22)])


class RecoverySafetyTests(unittest.TestCase):
    def setUp(self):
        self.portal = make_portal(K1RecoveryTaskCfg())

    def tearDown(self):
        self.portal.exit_event.set()
        self.portal._stop_recovery_hold()

    def test_native_arm_bound_preserves_other_joints(self):
        cfg = self.portal.cfg.robot
        q = np.zeros(22)
        target = np.linspace(-1, 1, 22)
        actual = limit_arm_targets(target, q, cfg.joint_stiffness, cfg.effort_limit)
        p_term = np.abs(actual[2:10] * np.array(cfg.joint_stiffness)[2:10])
        self.assertTrue((p_term <= 14.00001).all())
        np.testing.assert_equal(actual[:2], target[:2])
        np.testing.assert_equal(actual[10:], target[10:])
        self.assertRaises(ValueError, limit_arm_targets, target, q, np.zeros(22), cfg.effort_limit)

    def test_fault_detector_requires_both_feedback_paths(self):
        msg = sample()
        for i in (2, 6, 9):
            for group in (msg.motor_state_serial, msg.motor_state_parallel):
                group[i].q = group[i].dq = group[i].tau_est = 0.
            msg.motor_state_parallel[i].temperature = 0
            self.assertEqual(feedback_fault(msg), (3, i))
            msg.motor_state_parallel[i].temperature = 40
        self.assertEqual(feedback_fault(msg), (0, -1))
        msg.motor_state_serial.pop()
        self.assertEqual(feedback_fault(msg), (1, -1))

    def test_nonfinite_imu_and_motor_feedback_rejected(self):
        msg = sample()
        msg.imu_state.gyro[0] = float('nan')
        self.assertEqual(feedback_fault(msg), (2, -1))
        msg = sample()
        msg.motor_state_parallel[9].tau_est = float('inf')
        self.assertEqual(feedback_fault(msg), (2, 9))

    def test_startup_fault_refuses_custom_and_publication(self):
        p = self.portal
        msg = sample()
        for group in (msg.motor_state_serial, msg.motor_state_parallel):
            group[2].q = group[2].dq = group[2].tau_est = group[2].temperature = 0
        p._low_state_handler(msg)
        # Even if an earlier healthy sample set the event, the latest sample must be checked.
        p.low_state_received_event.set()
        p.remoteControlService.start_custom_mode.return_value = True
        with patch.object(p, '_change_robot_mode') as mode:
            self.assertFalse(p.start_custom_mode_conditionally())
        mode.assert_not_called()
        p.low_cmd_publisher.publish.assert_not_called()

    def test_continuous_valid_window_resets_after_gap(self):
        p = self.portal
        with patch.object(ROBOT.time, 'monotonic', return_value=100.):
            p._low_state_handler(sample())
        with patch.object(ROBOT.time, 'monotonic', return_value=100.6):
            p._low_state_handler(sample())
            self.assertFalse(p._recovery_guard(before_start=True))

    def test_startup_waits_for_half_second_before_hold_or_custom(self):
        p = self.portal
        clock = [100.335]
        state = p.synced_state.read()
        state[0]['state_received'] = clock[0]
        state[0]['recovery_valid_since'] = 100.
        p.synced_state.write(state)
        p.low_state_received_event.set()
        p.remoteControlService.start_custom_mode.return_value = True

        def receive(timeout):
            p.low_cmd_publisher.publish.assert_not_called()
            clock[0] += timeout
            state[0]['state_received'] = clock[0]
            p.synced_state.write(state)

        def hold():
            self.assertGreaterEqual(clock[0] - 100., .5)
            return True

        with patch.object(ROBOT.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(p.exit_event, 'wait', side_effect=receive), \
                patch.object(p, '_start_recovery_hold', side_effect=hold) as start, \
                patch.object(p, '_change_robot_mode', return_value=True) as mode, \
                patch.object(ROBOT.time, 'sleep'):
            self.assertTrue(p.start_custom_mode_conditionally())
        start.assert_called_once()
        mode.assert_called_once_with('custom')
        self.assertFalse(p.exit_event.is_set())

    def test_startup_wait_stops_when_feedback_becomes_stale(self):
        p = self.portal
        clock = [100.]
        state = p.synced_state.read()
        state[0]['state_received'] = state[0]['recovery_valid_since'] = clock[0]
        p.synced_state.write(state)

        def advance(timeout):
            clock[0] += timeout

        with patch.object(ROBOT.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(p.exit_event, 'wait', side_effect=advance):
            self.assertFalse(p._wait_recovery_ready())
        self.assertTrue(p.exit_event.is_set())
        self.assertIn('stale', p.logger.error.call_args.args[1])
        p.low_cmd_publisher.publish.assert_not_called()

    def test_startup_wait_has_bounded_deadline_even_if_window_keeps_resetting(self):
        p = self.portal
        clock = [100.]
        state = p.synced_state.read()
        state[0]['state_received'] = state[0]['recovery_valid_since'] = clock[0]
        p.synced_state.write(state)

        def receive(timeout):
            clock[0] += timeout
            state[0]['state_received'] = state[0]['recovery_valid_since'] = clock[0]
            p.synced_state.write(state)

        with patch.object(ROBOT.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(p.exit_event, 'wait', side_effect=receive):
            self.assertFalse(p._wait_recovery_ready())
        self.assertTrue(p.exit_event.is_set())
        self.assertIn('timed out', p.logger.error.call_args.args[1])
        self.assertLess(clock[0], 103.03)
        p.low_cmd_publisher.publish.assert_not_called()

    def test_fault_arriving_during_startup_wait_aborts(self):
        p = self.portal
        state = p.synced_state.read()
        state[0]['recovery_valid_since'] = time.monotonic()
        p.synced_state.write(state)

        def fault(timeout):
            state[0]['recovery_fault_code'] = 3
            state[0]['recovery_fault_joint'] = 6
            p.synced_state.write(state)

        with patch.object(p.exit_event, 'wait', side_effect=fault):
            self.assertFalse(p._wait_recovery_ready())
        self.assertTrue(p.exit_event.is_set())
        self.assertIn('joint=6', p.logger.error.call_args.args[1])
        p.low_cmd_publisher.publish.assert_not_called()

    def test_fault_after_arm_latches_stop_before_next_publish(self):
        p = self.portal
        p.recovery_active_event.set()
        msg = sample()
        for group in (msg.motor_state_serial, msg.motor_state_parallel):
            group[6].q = group[6].dq = group[6].tau_est = group[6].temperature = 0
        p._low_state_handler(msg)
        self.assertTrue(p.exit_event.is_set())
        self.assertFalse(p._publish_recovery_target(np.zeros(22), [53]*22, [2.5]*22))
        p.low_cmd_publisher.publish.assert_not_called()

    def test_stale_feedback_blocks_publishing(self):
        p = self.portal
        stamp = float(p.synced_state.read()[0]['state_received'])
        with patch.object(ROBOT.time, 'monotonic', return_value=stamp + .101):
            self.assertFalse(p._publish_recovery_target(np.zeros(22), [53]*22, [2.5]*22))
        p.low_cmd_publisher.publish.assert_not_called()

    def test_publish_uses_latest_feedback_and_records_limited_command(self):
        p = self.portal
        state = p.synced_state.read()
        state[0]['joint_pos'][:] = 0.1
        p.synced_state.write(state)
        trace = Mock()
        self.assertTrue(p._publish_recovery_target(np.ones(22), [53]*22, [2.5]*22, trace))
        for i in range(2, 10):
            self.assertAlmostEqual(p.motor_cmd[i].q, .1 + 14/53)
        self.assertGreater(p.synced_action.read()[0]['published_at'], 0)
        trace.record_command.assert_called_once()

    def test_hold_keeps_publishing_frozen_pose_while_feedback_changes(self):
        p = self.portal
        done = Event()
        outputs = []
        initial = p.synced_state.read()[0]['joint_pos'].copy()

        def published(msg):
            outputs.append([m.q for m in msg.motor_cmd])
            state = p.synced_state.read()
            state[0]['joint_pos'] += .005
            state[0]['state_received'] = time.monotonic()
            p.synced_state.write(state)
            if len(outputs) >= 4:
                p._recovery_hold_stop.set()
                done.set()

        p.low_cmd_publisher.publish.side_effect = published
        self.assertTrue(p._start_recovery_hold())
        self.assertTrue(done.wait(timeout=1.0))
        p._stop_recovery_hold()
        self.assertEqual(len(outputs), 4)
        for output in outputs:
            np.testing.assert_allclose(output, initial)

    def test_handoff_stops_hold_before_child_go(self):
        p = self.portal
        self.assertTrue(p._start_recovery_hold())
        process = Mock()
        process.start.side_effect = p.recovery_ready_event.set
        stop = p._stop_recovery_hold

        def stop_and_check():
            self.assertFalse(p.recovery_go_event.is_set())
            return stop()

        with patch.object(ROBOT.mp, 'Process', return_value=process), \
                patch.object(p, '_stop_recovery_hold', side_effect=stop_and_check):
            self.assertTrue(p.start_rl_gait_conditionally(wait_for_trigger=False))
        self.assertFalse(p._recovery_hold_thread.is_alive())
        self.assertTrue(p.recovery_go_event.is_set())

    def test_hold_only_never_starts_inference_even_if_a_pressed(self):
        p = self.portal
        p.cfg.booster.recovery_hold_only = True
        p.remoteControlService.start_rl_gait.return_value = True
        p.exit_event.set()
        with patch.object(ROBOT.mp, 'Process') as process:
            self.assertFalse(p.start_rl_gait_conditionally())
        process.assert_not_called()
        p.remoteControlService.start_rl_gait.assert_not_called()

    def test_hold_continues_during_model_loading(self):
        p = self.portal
        received = []

        def publish(_):
            received.append(time.monotonic())
            state = p.synced_state.read()
            state[0]['state_received'] = time.monotonic()
            p.synced_state.write(state)

        p.low_cmd_publisher.publish.side_effect = publish
        self.assertTrue(p._start_recovery_hold())
        ready = Timer(.12, p.recovery_ready_event.set)
        process = Mock()
        process.start.side_effect = ready.start
        try:
            with patch.object(ROBOT.mp, 'Process', return_value=process):
                self.assertTrue(p.start_rl_gait_conditionally(wait_for_trigger=False))
            self.assertGreaterEqual(len(received), 3)
            self.assertTrue(p.recovery_go_event.is_set())
        finally:
            ready.cancel()
            ready.join()

    def test_failed_model_loading_never_grants_child_publication(self):
        p = self.portal
        self.assertTrue(p._start_recovery_hold())
        process = Mock()
        process.is_alive.return_value = False
        with patch.object(ROBOT.mp, 'Process', return_value=process):
            self.assertFalse(p.start_rl_gait_conditionally(wait_for_trigger=False))
        self.assertTrue(p.exit_event.is_set())
        self.assertFalse(p.recovery_go_event.is_set())

    def test_blocked_hold_publisher_cannot_hang_inference_startup(self):
        p = self.portal
        p._recovery_publish_lock = Mock()
        p._recovery_publish_lock.acquire.return_value = False
        with patch.object(ROBOT.mp, 'Process') as process:
            self.assertFalse(p.start_rl_gait_conditionally(wait_for_trigger=False))
        process.return_value.start.assert_not_called()
        self.assertTrue(p.exit_event.is_set())

    def test_command_timeout_requests_damping(self):
        p = self.portal
        with patch.object(p, 'start_custom_mode_conditionally', return_value=True), \
                patch.object(p, 'start_rl_gait_conditionally', return_value=True), \
                patch.object(p, '_change_robot_mode', return_value=True) as mode:
            p.run()
        self.assertTrue(p.exit_event.is_set())
        mode.assert_called_once_with('damping')

    def test_rpc_timeout_is_bounded_and_cancelled(self):
        p = self.portal
        p.rpc_service_client = Mock()
        p.publish_node = Mock()
        future = p.rpc_service_client.call_async.return_value
        future.done.return_value = False
        with patch.object(ROBOT.rclpy, 'spin_until_future_complete', create=True) as spin, \
                patch.object(ROBOT, 'RpcService', SimpleNamespace(Request=Mock)):
            self.assertEqual(p._call_booster_rpc(2018), (False, None))
        self.assertEqual(spin.call_args.kwargs['timeout_sec'], .5)
        future.cancel.assert_called_once()

    def test_command_trace_checks_proportional_limit_separately_from_damping(self):
        p = self.portal
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'trace.jsonl'
            # Use the real command writer without constructing/loading another policy.
            trace = RecoveryTrace.__new__(RecoveryTrace)
            trace.file = path.open('w', buffering=1)
            try:
                trace.write({'type':'metadata', 'joint_names':p.cfg.robot.joint_names,
                             'effort_limit':p.cfg.robot.effort_limit})
                self.assertTrue(p._publish_recovery_target(np.ones(22), [53]*22, [2.5]*22, trace))
                result = summarize_trace(path)
                self.assertEqual(result['published_command_frames'], 1)
                self.assertEqual(result['published_arm_limit_violations'], 0)
                np.testing.assert_allclose(result['published_arm_max_abs_p_term'], 14.)
            finally:
                trace.close()


if __name__ == '__main__':
    unittest.main()
