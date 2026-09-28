"""Flushed per-step recovery proposals; never sends robot commands."""

import json
import hashlib
from pathlib import Path
import time


def _values(tensor):
    return tensor.detach().cpu().tolist()


class RecoveryTrace:
    def __init__(self, path, policy):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Do not overwrite a previous hardware trial.
        self.file = self.path.open('x', encoding='utf-8', buffering=1)
        self.start = time.monotonic()
        self.step = 0
        robot = policy.robot
        self.write({
            'type': 'metadata', 'schema': 1,
            'controller': type(policy.controller).__name__,
            'joint_names': robot.cfg.joint_names,
            'kp': _values(robot.joint_stiffness),
            'kd': _values(robot.joint_damping),
            'effort_limit': _values(robot.effort_limit),
            'policy_dt': policy.cfg.control_dt,
            'max_retries': policy.cfg.max_retries,
            'model': policy.cfg.checkpoint_path,
            'model_sha256': hashlib.sha256(
                (Path(policy.task_path) / policy.cfg.checkpoint_path).read_bytes()
            ).hexdigest(),
            'q_min': _values(policy.q_min), 'q_max': _values(policy.q_max),
            'note': 'Targets are policy proposals, not proof of publication or '
                    'motor acceptance. pd_estimate is not measured torque or '
                    'a hardware torque limiter. ROS zero fault fields may be invalid.',
        })

    def write(self, row):
        self.file.write(json.dumps(row, allow_nan=False) + '\n')

    def record(self, policy, target):
        from .proprioception import read_proprioception
        q, dq, gyro, gravity = read_proprioception(policy.robot.data)
        kp = policy.robot.joint_stiffness.to(q.device)
        kd = policy.robot.joint_damping.to(q.device)
        self.step += 1
        trajectory = (policy.trajectories[policy.posture]['joint'][policy.row]
                      if policy.posture is not None else None)
        self.write({
            'type': 'step', 'step': self.step,
            'wall_time': time.time(), 'elapsed': time.monotonic() - self.start,
            'state': policy.state, 'posture': policy.posture,
            'row': policy.row, 'phase': policy.phase,
            'trajectory_time': policy.elapsed, 'retries': policy.retries,
            'controller_running': policy.controller.is_running,
            'q': _values(q), 'dq': _values(dq),
            'tau_feedback': _values(policy.robot.data.feedback_torque),
            'gyro': _values(gyro), 'gravity': _values(gravity),
            'trajectory_q': _values(trajectory) if trajectory is not None else None,
            'residual': _values(policy.last_action),
            'target_proposed': _values(target),
            'pd_estimate': _values(kp * (target - q) - kd * dq),
            'observation': (_values(policy.last_observation)
                            if policy.last_observation is not None else None),
        })

    def close(self):
        self.file.close()


def summarize_trace(path):
    """Summarize executing frames without interpreting motor fault codes."""
    metadata = None
    stats = []
    count = 0
    final_state = None
    with Path(path).open(encoding='utf-8') as source:
        for number, line in enumerate(source, 1):
            try:
                frame = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f'Invalid trace JSON on line {number}') from exc
            if frame['type'] == 'metadata':
                metadata = frame
                stats = [dict(joint=name, index=i, max_error=0.,
                              max_error_time=0., max_abs_dq=0.,
                              max_abs_feedback_torque=0., max_abs_pd_estimate=0.,
                              estimated_over_limit_frames=0)
                         for i, name in enumerate(frame['joint_names'])]
                continue
            if frame['type'] != 'step':
                continue
            final_state = frame['state']
            if metadata is None:
                raise ValueError('Trace is missing metadata')
            if frame['state'] != 'executing':
                continue
            count += 1
            for i, stat in enumerate(stats):
                error = abs(frame['target_proposed'][i] - frame['q'][i])
                if error > stat['max_error']:
                    stat['max_error'] = error
                    stat['max_error_time'] = frame['trajectory_time']
                for field, key in [('dq', 'max_abs_dq'),
                                   ('tau_feedback', 'max_abs_feedback_torque'),
                                   ('pd_estimate', 'max_abs_pd_estimate')]:
                    stat[key] = max(stat[key], abs(frame[field][i]))
                if abs(frame['pd_estimate'][i]) > metadata['effort_limit'][i]:
                    stat['estimated_over_limit_frames'] += 1
    if metadata is None:
        raise ValueError('Trace is missing metadata')
    return {'executing_frames': count, 'final_state': final_state,
            'note': 'PD estimate exceeding the configured simulation limit does '
                    'not prove measured hardware over-torque or a stall.',
            'joints': sorted(stats, key=lambda item: item['max_error'], reverse=True)}
