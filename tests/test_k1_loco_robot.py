"""Offline real-controller checks: actual ONNX inference, in-memory ROS transport.

No ROS nodes, multiprocessing workers, or physical robot are started. These
checks exercise the real deployment module but do not validate DDS or firmware.
"""

from copy import deepcopy
import importlib.util
from pathlib import Path
from threading import Event
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import torch

from booster_deploy.controllers.base_controller import BaseController
from tasks.locomotion.robots.k1 import K1WalkTaskCfg
from tasks.locomotion.robots.k1.loco import K1LocoTaskCfg
from tasks.locomotion.robots.k1.recovery import K1RecoveryTaskCfg


ROOT = Path(__file__).resolve().parents[1]


class MemoryArray:
    def __init__(self, name, shape, dtype):
        self.dtype = np.dtype(dtype)
        self.data = np.zeros(shape, dtype=self.dtype)

    def read(self):
        return self.data.copy()

    def write(self, data):
        self.data[:] = data


def load_robot_module():
    """Replace only unavailable OS/ROS transport dependencies during import."""
    attributes = {
        'rclpy': {'ok': lambda: True},
        'rclpy.executors': {
            'SingleThreadedExecutor': Mock,
            'ExternalShutdownException': RuntimeError,
        },
        'rclpy.qos': {
            'QoSProfile': Mock, 'ReliabilityPolicy': Mock(), 'HistoryPolicy': Mock(),
        },
        'booster_interface': {},
        'booster_interface.msg': {
            name: Mock for name in ('BoosterApiReqMsg', 'LowState', 'LowCmd', 'MotorCmd')
        },
        'booster_interface.srv': {'RpcService': Mock},
        'booster_deploy.utils.remote_control_service': {'RemoteControlService': Mock},
        'booster_deploy.utils.synced_array': {'SyncedArray': MemoryArray},
        'booster_deploy.utils.metrics': {
            'SyncedMetrics': lambda *args, **kwargs: SimpleNamespace(mark=Mock()),
        },
    }
    modules = {}
    for name, members in attributes.items():
        module = ModuleType(name)
        module.__dict__.update(members)
        modules[name] = module
    spec = importlib.util.spec_from_file_location(
        'booster_deploy.controllers._offline_robot_test',
        ROOT / 'booster_deploy/controllers/booster_robot_controller.py',
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


ROBOT = load_robot_module()


def make_portal(cfg):
    with patch.object(ROBOT.BoosterRobotPortal, '_init_communication'), \
            patch.object(ROBOT.signal, 'signal'), patch.object(ROBOT.mp, 'Event', Event):
        portal = ROBOT.BoosterRobotPortal(cfg)
    portal.logger = Mock()
    portal.remoteControlService.get_vx_cmd.return_value = 0.0
    portal.remoteControlService.get_vy_cmd.return_value = 0.0
    portal.remoteControlService.get_vyaw_cmd.return_value = 0.0
    portal.motor_cmd = [SimpleNamespace(q=0., dq=0., tau=0., kp=0., kd=0., weight=0.)
                        for _ in portal.robot.cfg.joint_names]
    portal.low_cmd = SimpleNamespace(motor_cmd=portal.motor_cmd)
    portal.low_cmd_publisher = Mock()
    state = portal.synced_state.read()
    state[0]['joint_pos'] = cfg.robot.default_joint_pos
    portal.synced_state.write(state)
    return portal


class RobotConfigurationTests(unittest.TestCase):
    def test_hardware_uses_loco_ankle_gains_without_mutating_other_tasks(self):
        cfg, walk = K1LocoTaskCfg(), K1WalkTaskCfg()
        portal = make_portal(cfg)
        expected_kp = list(walk.robot.joint_stiffness)
        for i in (14, 15, 20, 21):
            expected_kp[i] = 50.
            self.assertEqual(walk.robot.joint_stiffness[i], 65.)
            self.assertEqual(portal.cfg.robot.joint_damping[i], 1.)
        self.assertEqual(portal.cfg.robot.joint_stiffness, expected_kp)
        self.assertEqual(portal.cfg.robot.joint_damping, walk.robot.joint_damping)
        self.assertEqual(cfg.robot.joint_stiffness[0], 20.)
        self.assertEqual(cfg.robot.joint_stiffness[14], 50.)
        self.assertEqual(cfg.robot.joint_damping[0], 2.)
        prepared = portal._build_prepare_cfg()
        self.assertEqual(prepared.robot.joint_stiffness, expected_kp)
        self.assertEqual(prepared.policy.model_paths, cfg.policy.model_paths)
        self.assertIsNone(prepared.policy.forced_route)
        self.assertEqual(prepared.robot.default_joint_pos, cfg.robot.default_joint_pos)
        # Tasks without a hardware override retain their previous configuration.
        self.assertEqual(walk.booster.apply_to_robot(walk.robot), walk.robot)

    def test_invalid_motor_gains_are_rejected_before_ros_setup(self):
        for gains in ([1.], [float('nan')] * 22, [-1.] * 22):
            cfg = K1LocoTaskCfg()
            cfg.booster.joint_damping = gains
            with patch.object(ROBOT, 'RemoteControlService') as remote:
                with self.assertRaises(ValueError):
                    ROBOT.BoosterRobotPortal(cfg)
                remote.assert_not_called()


class RobotInferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.portal = make_portal(K1LocoTaskCfg())
        cls.controller = ROBOT.BoosterRobotController(cls.portal.cfg, cls.portal)
        cls.reference = BaseController(K1LocoTaskCfg())

    def setUp(self):
        p, c = self.portal, self.controller
        p.exit_event.clear()
        p.task_start_event.clear()
        p.velocity_commands_enabled_event.clear()
        p.low_cmd_publisher.reset_mock()
        p.synced_command.write(np.zeros(1, dtype=p.synced_command.dtype))
        state = np.zeros(1, dtype=p.synced_state.dtype)
        state[0]['joint_pos'] = c.cfg.robot.default_joint_pos
        p.synced_state.write(state)
        c._velocity_commands_enabled = False
        c.policy.cfg.forced_route = None
        c.update_state()
        c.start()

    def command(self, values):
        cmd = self.portal.synced_command.read()
        cmd[0] = tuple(values)
        self.portal.synced_command.write(cmd)
        self.controller.update_vel_command()

    def test_remote_scaling_preparation_mask_and_invalid_input(self):
        c = self.controller
        self.command([1, 1, 1])
        self.assertEqual([c.vel_command.lin_vel_x, c.vel_command.lin_vel_y,
                          c.vel_command.ang_vel_yaw], [0, 0, 0])
        c._velocity_commands_enabled = True
        for raw, expected in [([2, -2, 2], [1.5, -0.4, 1.8]),
                              ([-1, 0, 0], [-0.7, 0, 0]),
                              ([float('nan'), 1, 1], [0, 0, 0])]:
            self.command(raw)
            np.testing.assert_allclose([c.vel_command.lin_vel_x, c.vel_command.lin_vel_y,
                                        c.vel_command.ang_vel_yaw], expected)

    def test_low_state_callback_through_all_routes_to_motor_commands(self):
        p, c, ref = self.portal, self.controller, self.reference
        c._velocity_commands_enabled = True
        rpy = np.array([0.06, -0.04, 0.1])
        gyro = np.array([0.03, -0.02, 0.01])
        q = np.array(c.cfg.robot.default_joint_pos) + np.linspace(-0.01, 0.01, 22)
        dq = np.linspace(-0.1, 0.1, 22)
        message = SimpleNamespace(
            imu_state=SimpleNamespace(rpy=rpy, gyro=gyro),
            motor_state_serial=[SimpleNamespace(q=a, dq=b, tau_est=0.2)
                                for a, b in zip(q, dq)],
        )
        # Independent Euler-to-quaternion reference, wxyz ordering.
        cr, cp, cy = np.cos(rpy / 2)
        sr, sp, sy = np.sin(rpy / 2)
        quat = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]
        ref.robot.data.root_quat_w = torch.tensor(quat, dtype=torch.float32)
        ref.robot.data.joint_pos = torch.tensor(q, dtype=torch.float32)
        ref.robot.data.joint_vel = torch.tensor(dq, dtype=torch.float32)
        ref.robot.data.root_ang_vel_b = torch.tensor(gyro, dtype=torch.float32)
        ref.start()
        routes = set()
        for raw, physical, route in [((0.2, 0, 0), (0.3, 0, 0), 0),
                                     ((0, 0.5, 0), (0, 0.2, 0), 1),
                                     ((0, 0, 0.25), (0, 0, 0.45), 2),
                                     ((-0.5, 0, 0), (-0.35, 0, 0), 0)]:
            p.remoteControlService.get_vx_cmd.return_value = raw[0]
            p.remoteControlService.get_vy_cmd.return_value = raw[1]
            p.remoteControlService.get_vyaw_cmd.return_value = raw[2]
            ref.vel_command.lin_vel_x, ref.vel_command.lin_vel_y, ref.vel_command.ang_vel_yaw = physical
            for _ in range(40):
                p._low_state_handler(message)
                c.update_state()
                c.update_vel_command()
                target = c.policy_step()
                expected = ref.policy_step()
                c.ctrl_step(target)
                np.testing.assert_allclose(c.policy.obs_history, ref.policy.obs_history, atol=3e-5)
                np.testing.assert_allclose(target, expected, atol=3e-5)
                np.testing.assert_allclose([m.q for m in p.motor_cmd], expected, atol=3e-5)
                self.assertTrue(torch.isfinite(target).all())
                routes.add(c.policy.active_route)
            self.assertEqual(c.policy.active_route, route)
        self.assertEqual(routes, {0, 1, 2})
        self.assertTrue(p.low_state_received_event.is_set())
        self.assertEqual(p.low_cmd_publisher.publish.call_count, 160)
        expected_kp = list(K1WalkTaskCfg().robot.joint_stiffness)
        for i in (14, 15, 20, 21):
            expected_kp[i] = 50.
        np.testing.assert_allclose([m.kp for m in p.motor_cmd], expected_kp)
        np.testing.assert_allclose([m.kd for m in p.motor_cmd], K1WalkTaskCfg().robot.joint_damping)

    def test_stop_during_inference_does_not_publish_another_command(self):
        c = self.controller
        def stop_and_return():
            c.stop()
            return torch.zeros(22)
        with patch.object(c, 'policy_step', side_effect=stop_and_return):
            c.run()
        self.portal.low_cmd_publisher.publish.assert_not_called()


class HandoffTests(unittest.TestCase):
    def test_loco_handoff_keeps_sessions_history_and_filter(self):
        cfg = K1LocoTaskCfg()
        cfg.policy.forced_route = 2
        portal = make_portal(cfg)
        prepared = portal._build_prepare_cfg()
        controller = ROBOT.BoosterRobotController(prepared, portal)
        controller.start()
        sessions = tuple(id(m) for m in controller.policy.models)
        histories, step_counts, routes, commands = [], [], [], []
        cmd = portal.synced_command.read()
        cmd[0] = (0., 0.5, 0.)
        portal.synced_command.write(cmd)
        # Even if command-enable arrives early, preparation must stay at zero.
        portal.velocity_commands_enabled_event.set()

        def published(message):
            histories.append(controller.policy.obs_history.clone())
            step_counts.append(controller._step_count)
            routes.append(controller.policy.active_route)
            commands.append(controller.policy.command_processor.processed.copy())
            if len(histories) == 3:
                portal.task_start_event.set()
            if len(histories) == 7:
                portal.exit_event.set()

        portal.low_cmd_publisher.publish.side_effect = published
        with patch.object(ROBOT, 'BoosterRobotController', return_value=controller) as factory, \
                patch.object(controller.policy, 'reset', wraps=controller.policy.reset) as reset:
            ROBOT.BoosterRobotPortal.inference_process_func(prepared, portal, True)
            factory.assert_called_once()
            reset.assert_called_once()
        self.assertEqual(tuple(id(m) for m in controller.policy.models), sessions)
        self.assertEqual(step_counts, list(range(1, 8)))
        self.assertEqual(routes[:3], [0, 0, 0])
        self.assertEqual(routes[3:], [2, 2, 2, 2])
        self.assertEqual(commands[:3], [[0, 0, 0]] * 3)
        self.assertAlmostEqual(commands[3][1], 0.2)
        np.testing.assert_allclose(histories[3][:-1], histories[2][1:])
        # Last-action observation survives the A/r transition.
        np.testing.assert_allclose(histories[3][-1, 49:69],
                                   controller.policy.models[0](histories[2].flatten()).flatten(), atol=3e-5)

    def test_other_tasks_still_construct_the_selected_policy_after_preparation(self):
        portal = SimpleNamespace(cfg=K1WalkTaskCfg(), task_start_event=Event(),
                                 exit_event=Event(), logger=Mock())
        prepared = deepcopy(portal.cfg)
        first, second = Mock(), Mock()
        with patch.object(ROBOT, 'BoosterRobotController', side_effect=[first, second]) as factory:
            ROBOT.BoosterRobotPortal.inference_process_func(prepared, portal, True)
        self.assertEqual(factory.call_count, 2)
        first.run.assert_called_once_with(stop_event=portal.task_start_event)
        second.run.assert_called_once_with()


class RecoveryRobotTests(unittest.TestCase):
    def test_recovery_uses_loco_low_state_path_and_serial_commands(self):
        cfg = K1RecoveryTaskCfg()
        portal = make_portal(cfg)
        controller = ROBOT.BoosterRobotController(portal.cfg, portal)
        q = np.array(cfg.robot.default_joint_pos) + np.linspace(-0.02, 0.02, 22)
        dq = np.linspace(-7., 7., 22)
        gyro = [0.03, -0.04, 0.05]
        message = SimpleNamespace(
            imu_state=SimpleNamespace(rpy=[0.1, -1.4, 0.2], gyro=gyro),
            motor_state_serial=[SimpleNamespace(q=a, dq=b, tau_est=0.) for a, b in zip(q, dq)],
        )
        portal._low_state_handler(message)
        controller.update_state()
        controller.start()
        controller.policy.settle_time = 1.
        target = controller.policy_step()
        controller.ctrl_step(target)
        obs = controller.policy.last_observation
        np.testing.assert_allclose(obs[31:34], gyro, atol=1e-7)  # no degrees conversion
        np.testing.assert_allclose(obs[36:56], (q - cfg.policy.reference)[2:], atol=1e-7)
        np.testing.assert_allclose(obs[58:78], np.clip(dq[2:], -5, 5) * .1, atol=1e-7)
        np.testing.assert_allclose([m.q for m in portal.motor_cmd], target)
        np.testing.assert_allclose([m.kp for m in portal.motor_cmd], cfg.robot.joint_stiffness)
        np.testing.assert_allclose([m.kd for m in portal.motor_cmd], cfg.robot.joint_damping)
        self.assertEqual([m.weight for m in portal.motor_cmd[:2]], [1., 1.])
        self.assertTrue(all(m.dq == 0 and m.tau == 0 for m in portal.motor_cmd))
        self.assertFalse(portal.head_tracker)
        self.assertEqual(portal.cfg.booster.exit_mode, 'damping')

    def test_hold_preparation_accepts_fallen_pose_without_interpolation(self):
        portal = make_portal(K1RecoveryTaskCfg())
        state = portal.synced_state.read()
        state[0]['root_rpy_w'] = [0., -1.5, 0.]
        state[0]['joint_pos'] += .01
        portal.synced_state.write(state)
        portal.low_state_received_event.set()
        portal.remoteControlService.start_custom_mode.return_value = True
        portal.low_cmd_publisher.get_subscription_count.return_value = 1
        with patch.object(portal, '_change_robot_mode', return_value=True) as mode, \
                patch.object(ROBOT.time, 'sleep'), patch.object(ROBOT.np, 'linspace') as interpolate:
            self.assertTrue(portal.start_custom_mode_conditionally())
        mode.assert_called_once_with('custom')
        interpolate.assert_not_called()
        portal.low_cmd_publisher.publish.assert_called_once()
        np.testing.assert_allclose([m.q for m in portal.motor_cmd], state[0]['joint_pos'])
        self.assertEqual([m.weight for m in portal.motor_cmd[:2]], [1., 1.])

    def test_hold_mode_waits_for_task_trigger(self):
        portal = make_portal(K1RecoveryTaskCfg())
        with patch.object(portal, 'start_custom_mode_conditionally', return_value=True), \
                patch.object(portal, 'start_rl_gait_conditionally', return_value=False) as start, \
                patch.object(portal, '_change_robot_mode', return_value=True):
            portal.run()
        start.assert_called_once_with(wait_for_trigger=True)


if __name__ == '__main__':
    unittest.main()
