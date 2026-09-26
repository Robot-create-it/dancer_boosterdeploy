"""Run with: python -m unittest discover -s tests -v (requires onnxruntime)."""

import ast
from copy import deepcopy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import torch

from booster_deploy.controllers.base_controller import BaseController
from tasks.locomotion.nested_locomotion import LocoCommandProcessor
from tasks.locomotion.robots.k1 import K1WalkTaskCfg
from tasks.locomotion.robots.k1.loco import K1LocoTaskCfg


ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "tasks/locomotion/robots/k1/loco_config.json").read_text())


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.cfg = K1LocoTaskCfg().policy
        self.processor = LocoCommandProcessor(self.cfg)

    def test_demo_ramp_and_coupling_examples(self):
        p = self.processor
        self.assertAlmostEqual(p.process([1.5, 0, 0])[0], 0.026)
        for _ in range(9):
            p.process([1.5, 0, 0])
        self.assertAlmostEqual(p.processed[0], 0.26)
        self.assertAlmostEqual(p.process([0, 0, 0])[0], 0.228)
        for _ in range(300):
            p.process([1.5, 0, 1.8])
        np.testing.assert_allclose(p.processed, [1.5, 0, 0.8 / 1.5])
        self.assertEqual(p.select_route(), 0)
        # Negative vx uses a tighter lateral coupling coefficient of 0.05.
        p.reset()
        for _ in range(300):
            p.process([-0.5, 0.4, 1.8])
        np.testing.assert_allclose(p.processed, [-0.5, 0.1, 0.8])

    def test_demo_routes_after_ramping(self):
        cases = [([0.8, 0, 0.05], 0), ([0, 0.3, 0], 1),
                 ([0, -0.3, 0], 1), ([0, 0, 0.5], 2),
                 ([0, 0, -0.5], 2), ([-0.5, 0, 0], 0),
                 ([0.5, 0.3, 0], 0), ([0, 0.3, 0.5], 0)]
        for cmd, expected in cases:
            with self.subTest(cmd=cmd):
                self.processor.reset()
                for _ in range(120):
                    self.processor.process(cmd)
                self.assertEqual(self.processor.select_route(), expected)

    def test_strict_thresholds_adjust_and_nonfinite(self):
        p = self.processor
        for cmd, route in [([0, 0.1, 0], 0), ([0.2, 0.3, 0], 0),
                           ([0, 0.3, 0.2], 0), ([0, 0, 0.1], 0),
                           ([0.1, 0, 0.5], 0), ([0, 0.1, 0.5], 0),
                           ([0, 0.15, 0.15], 1)]:
            p.processed = cmd
            self.assertEqual(p.select_route(), route)
        self.assertEqual(p.process([1, 0, 0], adjust=True), [0, 0, 0.2])
        self.assertEqual(p.previous, [0, 0, 0])
        self.assertEqual(p.select_route(adjust=True), 2)
        self.cfg.forced_route = 0
        self.assertEqual(p.select_route(adjust=True), 0)
        p.reset()
        self.assertEqual(p.process([float('nan'), 0.3, 0]), [0, 0, 0])

    def test_config_and_preparation_preserve_old_task(self):
        new, old = K1LocoTaskCfg(), K1WalkTaskCfg()
        self.assertEqual(new.robot.joint_stiffness[14:16], [50, 50])
        self.assertEqual(old.robot.joint_stiffness[14:16], [65, 65])
        self.assertIsNone(new.policy.arm_action_fix_joint_name)
        self.assertEqual(old.policy.arm_action_fix_offset, 0.2)
        # Exercise the actual preparation method without importing ROS 2 on this
        # offline test host. Its body only depends on deepcopy and task configs.
        tree = ast.parse((ROOT / 'booster_deploy/controllers/booster_robot_controller.py').read_text(encoding='utf-8'))
        portal = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BoosterRobotPortal')
        method = next(n for n in portal.body if isinstance(n, ast.FunctionDef) and n.name == '_build_prepare_cfg')
        namespace = {'deepcopy': deepcopy}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<prepare method>', 'exec'), namespace)
        new.policy.forced_route = 1
        prepare = namespace['_build_prepare_cfg'](SimpleNamespace(cfg=new))
        self.assertEqual(prepare.policy.model_paths, new.policy.model_paths)
        self.assertIsNone(prepare.policy.forced_route)
        self.assertEqual(new.policy.forced_route, 1)
        self.assertEqual(prepare.robot.joint_stiffness, new.robot.joint_stiffness)
        legacy = namespace['_build_prepare_cfg'](SimpleNamespace(cfg=old))
        self.assertEqual(legacy.policy.checkpoint_path, old.policy.checkpoint_path)


class InferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = BaseController(K1LocoTaskCfg())

    def setUp(self):
        c = self.controller
        c.robot.data.joint_pos = c.robot.default_joint_pos.clone()
        c.robot.data.joint_vel.zero_()
        c.robot.data.root_quat_w = torch.tensor([1., 0., 0., 0.])
        c.robot.data.root_ang_vel_b.zero_()
        c.vel_command.lin_vel_x = c.vel_command.lin_vel_y = c.vel_command.ang_vel_yaw = 0
        c.policy.cfg.forced_route = None
        c.start()

    def test_model_hashes_and_dynamic_runner_freshness(self):
        models = ROOT / 'tasks/locomotion/robots/k1/models/loco'
        for name, digest in json.loads((models / 'sha256.json').read_text()).items():
            self.assertEqual(hashlib.sha256((models / name).read_bytes()).hexdigest(), digest)
        rng = np.random.default_rng(17)
        for runner in self.controller.policy.models:
            previous = None
            for _ in range(3):
                obs = rng.normal(0, 0.2, (1, 690)).astype(np.float32)
                reference = runner.session.run([runner.output_name], {runner.input_name: obs})[0]
                result = runner(torch.from_numpy(obs[0])).clone().numpy()
                np.testing.assert_allclose(result, reference, atol=1e-6)
                if previous is not None:
                    self.assertFalse(np.array_equal(previous, result))
                previous = result.copy()

    def test_all_actors_against_demo_observation_and_target_equations(self):
        c, p = self.controller, self.controller.policy
        # Reference follows source JSON index arrays rather than deploy's name mapping.
        body = np.array(CONFIG['body_dof_indices_20'])
        order = np.array(CONFIG['webots_to_lab_idx'])
        inverse = np.array(CONFIG['lab_to_webots_idx'])
        default = np.array(CONFIG['default_dof_pos_22'])
        last_action = np.zeros(20)
        last_target = default.copy()
        history = None
        rng = np.random.default_rng(29)
        for route in [0, 1, 2, 0, 2, 1]:
            p.cfg.forced_route = route
            q = default + rng.normal(0, 0.04, 22)
            dq = rng.normal(0, 0.1, 22)
            quat = np.array([0.98, 0.04, -0.03, 0.05])
            quat /= np.linalg.norm(quat)
            gyro = rng.normal(0, 0.1, 3)
            c.robot.data.joint_pos = torch.tensor(q, dtype=torch.float32)
            c.robot.data.joint_vel = torch.tensor(dq, dtype=torch.float32)
            c.robot.data.root_quat_w = torch.tensor(quat, dtype=torch.float32)
            c.robot.data.root_ang_vel_b = torch.tensor(gyro, dtype=torch.float32)
            w, x, y, z = quat
            gravity = np.array([2*(w*y-x*z), -2*(w*x+y*z), 2*(x*x+y*y)-1])
            frame = np.concatenate([gyro, gravity + CONFIG['gravity_offset'], [0, 0, 0],
                                    (q-default)[body][order], dq[body][order]*0.1, last_action])
            frame = np.clip(frame, -100, 100).astype(np.float32)
            history = np.tile(frame, (10, 1)) if history is None else np.vstack([history[1:], frame])
            runner = p.models[route]
            raw = runner.session.run([runner.output_name], {runner.input_name: history.reshape(1, 690)})[0].ravel()
            action = np.clip(raw, -100, 100)
            target = default.copy()
            target[body] += 0.25 * action[inverse]
            target = 0.8 * target + 0.2 * last_target
            actual = c.policy_step().numpy()
            np.testing.assert_allclose(p.obs_history.numpy(), history, atol=3e-5)
            np.testing.assert_allclose(actual, target, atol=3e-5)
            self.assertEqual(p.active_route, route)
            self.assertTrue(np.isfinite(actual).all())
            last_action, last_target = action.copy(), target.copy()
        p.reset()
        self.assertIsNone(p.obs_history)
        self.assertEqual(p.command_processor.previous, [0, 0, 0])
        np.testing.assert_array_equal(p.last_action.numpy(), np.zeros(20))
        np.testing.assert_allclose(p.filtered_dof_target.numpy(), default)

    def test_processed_command_is_used_once_in_observation_and_route(self):
        c, p = self.controller, self.controller.policy
        c.vel_command.lin_vel_y = 0.3
        c.policy_step()
        self.assertAlmostEqual(p.command_processor.processed[1], 0.024)
        self.assertEqual(p.active_route, 0)
        for _ in range(4):
            c.policy_step()
        self.assertAlmostEqual(p.command_processor.processed[1], 0.12)
        self.assertEqual(p.active_route, 1)
        np.testing.assert_allclose(p.obs_history[-1, 6:9].numpy(), [0, 0.12, 0], atol=1e-7)


@unittest.skipUnless(importlib.util.find_spec('mujoco') and importlib.util.find_spec('booster_assets'),
                     'MuJoCo and BoosterAssets required for console checks')
class ConsoleTests(unittest.TestCase):
    def test_terminal_clamps_all_axes_and_rejects_nan(self):
        from booster_deploy.controllers.mujoco_controller import MujocoController
        from booster_deploy.controllers.base_controller import VelocityCommand
        c = MujocoController.__new__(MujocoController)
        c.vel_command = VelocityCommand(K1LocoTaskCfg().vel_command)
        c._read_command_line = lambda: '-2 3 4'
        with patch('sys.stdout', new_callable=io.StringIO):
            c.update_vel_command()
            self.assertEqual([c.vel_command.lin_vel_x, c.vel_command.lin_vel_y,
                              c.vel_command.ang_vel_yaw], [-0.7, 0.4, 1.8])
            c._read_command_line = lambda: 'nan 0 0'
            c.update_vel_command()
            self.assertEqual(c.vel_command.lin_vel_x, -0.7)

    def test_windows_console_partial_line_and_backspace(self):
        from collections import deque
        from booster_deploy.controllers import mujoco_controller as module
        c = module.MujocoController.__new__(module.MujocoController)
        chars = deque('0.4 0 0.5')
        console = SimpleNamespace(kbhit=lambda: bool(chars), getwch=lambda: chars.popleft())
        with patch.object(module.os, 'name', 'nt'), patch.dict('sys.modules', {'msvcrt': console}), \
                patch('sys.stdin', SimpleNamespace(isatty=lambda: True)), \
                patch('sys.stdout', new_callable=io.StringIO):
            self.assertIsNone(c._read_command_line())
            chars.extend('\b4\r')
            self.assertEqual(c._read_command_line(), '0.4 0 0.4')
            self.assertEqual(c._command_line_buffer, '')


if __name__ == '__main__':
    unittest.main()
