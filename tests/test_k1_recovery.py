"""Recovery model/trajectory contract and loco observation-source parity."""

import hashlib
import json
import math
from pathlib import Path
import unittest
import tempfile
from unittest.mock import Mock

import numpy as np
import torch

from booster_deploy.controllers.base_controller import BaseController
from booster_deploy.utils.isaaclab import math as lab_math
from booster_deploy.utils.recovery_safety import limit_arm_targets
from tasks.locomotion.robots.k1.loco import K1LocoTaskCfg
from tasks.locomotion.robots.k1.recovery import K1RecoveryTaskCfg


class RecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = BaseController(K1RecoveryTaskCfg())
        loco_cfg = K1LocoTaskCfg()
        loco_cfg.policy.enable_safety_fallback = False
        cls.loco = BaseController(loco_cfg)

    def setUp(self):
        c = self.controller
        c.robot.data.joint_pos = c.robot.default_joint_pos.clone()
        c.robot.data.joint_vel.zero_()
        c.robot.data.root_quat_w = torch.tensor([math.sqrt(0.5), 0., -math.sqrt(0.5), 0.])
        c.robot.data.root_ang_vel_b.zero_()
        c.start()

    def test_release_hashes(self):
        folder = Path(__file__).resolve().parents[1] / 'tasks/locomotion/robots/k1/models/recovery'
        for name, digest in json.loads((folder / 'sha256.json').read_text()).items():
            self.assertEqual(hashlib.sha256((folder / name).read_bytes()).hexdigest(), digest)

    def test_all_observation_slots_and_loco_sensor_parity(self):
        c, loco = self.controller, self.loco
        p = c.policy
        c.robot.data.joint_pos += torch.linspace(-0.5, 0.5, 22)
        c.robot.data.joint_vel = torch.linspace(-8., 8., 22)
        c.robot.data.root_ang_vel_b = torch.tensor([0.3, -0.7, 1.2])
        c.robot.data.root_quat_w = lab_math.quat_from_euler_xyz(
            *torch.tensor([0.23, -1.1, 0.48])).squeeze()
        loco.robot.data = c.robot.data
        loco.start()
        lo = loco.policy.compute_observation()
        p.posture, p.row, p.phase = 'faceup', 37, 0.4
        p.last_action[:] = torch.linspace(-0.2, 0.2, 22)
        obs = p.compute_observation()
        self.assertEqual(tuple(obs.shape), (100,))
        torch.testing.assert_close(obs[28:31], lo[3:6])
        torch.testing.assert_close(obs[31:34], lo[:3])
        mapping = loco.policy.real2sim_joint_map
        torch.testing.assert_close(
            obs[34:56][mapping] + p.reference[mapping],
            lo[9:29] + loco.policy.default_joint_pos[mapping])
        torch.testing.assert_close(obs[56:78][mapping], lo[29:49].clamp(-0.5, 0.5))
        trajectory = p.trajectories['faceup']
        expected = torch.zeros(100)
        expected[:2] = torch.tensor([1., 0.4])
        expected[2:5] = trajectory['gravity'][37] - lo[3:6]
        expected[5] = trajectory['height'][37]
        expected[6:28] = trajectory['joint'][37] - c.robot.data.joint_pos
        expected[28:31] = lo[3:6]
        expected[31:34] = c.robot.data.root_ang_vel_b
        expected[34:56] = c.robot.data.joint_pos - p.reference
        expected[56:78] = c.robot.data.joint_vel.clamp(-5, 5) * 0.1
        expected[78:] = p.last_action
        expected[[6, 7, 34, 35, 56, 57]] = 0
        expected[78 + p.zero_joint_ids] = 0
        torch.testing.assert_close(obs, expected)
        self.assertTrue(torch.all(obs[34 + torch.tensor([12, 15, 18, 21])] != 0))

    def test_settle_gate_is_continuous_and_selects_both_postures(self):
        c, p = self.controller, self.controller.policy
        for pitch, posture in [(-1.5, 'faceup'), (1.5, 'facedown')]:
            c.start()
            c.robot.data.root_quat_w = lab_math.quat_from_euler_xyz(*torch.tensor([0., pitch, 0.])).squeeze()
            for _ in range(20):
                torch.testing.assert_close(c.policy_step(), c.robot.data.joint_pos)
            c.robot.data.root_ang_vel_b[0] = 0.3
            c.policy_step()
            self.assertEqual(p.settle_time, 0.)
            c.robot.data.root_ang_vel_b.zero_()
            for _ in range(34):
                c.policy_step()
            self.assertIsNone(p.posture)
            result = c.policy_step()
            self.assertEqual(p.posture, posture)
            self.assertEqual(p.row, 1)
            rows = len(p.trajectories[posture]['joint'])
            self.assertAlmostEqual(p.phase, 1.02 / (1 + rows * 0.02))
            self.assertTrue(torch.isfinite(result).all())

    def test_residual_mask_previous_action_limits_and_reset(self):
        c, p = self.controller, self.controller.policy
        original = p._model
        p._model = Mock(return_value=torch.linspace(-4, 4, 22))
        try:
            p.settle_time = 1.0
            target = c.policy_step()
            residual = torch.linspace(-4, 4, 22)
            residual[p.zero_joint_ids] = 0
            expected = (p.trajectories['faceup']['joint'][1] + residual).clamp(p.q_min, p.q_max)
            expected = torch.tensor(limit_arm_targets(expected, c.robot.data.joint_pos,
                                    c.robot.joint_stiffness, c.robot.effort_limit), dtype=expected.dtype)
            torch.testing.assert_close(target, expected)
            c.policy_step()
            torch.testing.assert_close(p.last_observation[78:], residual)
            c.start()
            self.assertEqual(p.state, 'settling')
            self.assertIsNone(p.posture)
            self.assertEqual(p.elapsed, 0)
            self.assertTrue(torch.all(p.last_action == 0))
        finally:
            p._model = original

    def test_real_onnx_full_trajectories_against_numpy_reference(self):
        c, p = self.controller, self.controller.policy
        for posture, sign in [('faceup', -1.), ('facedown', 1.)]:
            c.start()
            c.robot.data.root_quat_w = torch.tensor([math.sqrt(0.5), 0., sign * math.sqrt(0.5), 0.])
            p.settle_time = 1.
            traj = p.trajectories[posture]
            previous = np.zeros(22, dtype=np.float32)
            for step in range(1, len(traj['joint']) + 1):
                row = min(step, len(traj['joint']) - 1)
                # A changing measured state makes serial permutation errors visible.
                q = traj['joint'][row].numpy() + np.linspace(-0.02, 0.02, 22, dtype=np.float32)
                dq = np.linspace(-6., 6., 22, dtype=np.float32)
                c.robot.data.joint_pos = torch.from_numpy(q)
                c.robot.data.joint_vel = torch.from_numpy(dq)
                target = c.policy_step()
                gravity = np.array([sign, 0., 0.], dtype=np.float32)
                expected = np.concatenate((
                    [float(posture == 'faceup'), min(1., (1 + step * .02) / (1 + len(traj['joint']) * .02))],
                    traj['gravity'][row].numpy() - gravity,
                    [traj['height'][row].item()], traj['joint'][row].numpy() - q,
                    gravity, [0., 0., 0.], q - p.reference.numpy(), np.clip(dq, -5, 5) * .1,
                    previous,
                )).astype(np.float32)
                expected[[6, 7, 34, 35, 56, 57]] = 0
                np.testing.assert_allclose(p.last_observation.numpy(), expected, atol=3e-6)
                runner = p._model
                raw = runner.session.run([runner.output_name], {runner.input_name: expected[None]})[0].reshape(-1)
                raw[p.cfg.zero_joint_ids] = 0
                fused = np.clip(traj['joint'][row].numpy() + raw, p.q_min.numpy(), p.q_max.numpy())
                fused = limit_arm_targets(fused, q, c.robot.joint_stiffness, c.robot.effort_limit)
                np.testing.assert_allclose(target.numpy(), fused, atol=2e-5)
                previous = raw.copy()

    def test_completion_retry_and_failure(self):
        c, p = self.controller, self.controller.policy
        p.posture, p.state = 'faceup', 'executing'
        duration = len(p.trajectories['faceup']['joint']) * .02
        p.elapsed = duration
        c.robot.data.root_quat_w = torch.tensor([1., 0., 0., 0.])
        final = c.policy_step()
        self.assertEqual(p.state, 'succeeded')
        torch.testing.assert_close(c.policy_step(), final)
        c.start()
        c.robot.data.root_quat_w = torch.tensor([math.sqrt(.5), 0., -math.sqrt(.5), 0.])
        for attempt in range(p.cfg.max_retries + 1):
            p.posture, p.state = 'faceup', 'executing'
            p.elapsed = duration + p.cfg.retry_margin_s
            c.policy_step()
            if attempt < p.cfg.max_retries:
                self.assertEqual(p.state, 'settling')
                self.assertEqual(p.retries, attempt + 1)
        self.assertEqual(p.state, 'failed')
        self.assertFalse(c.is_running)

    def test_bad_model_output_or_sensor_is_rejected(self):
        p = self.controller.policy
        original = p._model
        try:
            for output in (torch.zeros(20), torch.full((22,), float('nan'))):
                self.controller.start()
                p.settle_time = 1.
                p._model = Mock(return_value=output)
                with self.assertRaisesRegex(RuntimeError, '22 finite'):
                    self.controller.policy_step()
        finally:
            p._model = original
        self.controller.robot.data.joint_pos[3] = float('nan')
        with self.assertRaisesRegex(RuntimeError, 'sensor input'):
            self.controller.policy_step()

    def test_trace_records_proposals_without_changing_inference(self):
        from booster_deploy.utils.recovery_trace import RecoveryTrace, summarize_trace
        c, p = self.controller, self.controller.policy
        p.settle_time = 1.
        expected = c.policy_step().clone()
        c.start()
        p.settle_time = 1.
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'trial.jsonl'
            p.trace = RecoveryTrace(path, p)
            try:
                actual = c.policy_step()
                torch.testing.assert_close(actual, expected)
                # The last frame must already be on disk before close/exit.
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                self.assertEqual(len(rows), 2)
                frame = rows[-1]
                np.testing.assert_allclose(frame['target_proposed'], actual)
                np.testing.assert_allclose(frame['pd_estimate'],
                    c.robot.joint_stiffness * (actual - c.robot.data.joint_pos))
                self.assertEqual(len(frame['observation']), 100)
                summary = summarize_trace(path)
                self.assertEqual(summary['executing_frames'], 1)
                self.assertEqual(len(summary['joints']), 22)
                with self.assertRaises(FileExistsError):
                    RecoveryTrace(path, p)
                c.robot.data.joint_pos[0] = float('nan')
                with self.assertRaises(RuntimeError):
                    c.policy_step()
                self.assertEqual(json.loads(path.read_text().splitlines()[-1])['type'], 'error')
            finally:
                p.trace.close()
                p.trace = None


if __name__ == '__main__':
    unittest.main()
