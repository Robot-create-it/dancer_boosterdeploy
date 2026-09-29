"""Read-only K1 /joint_ctrl versus /low_state serial-joint monitor.

ROS messages have no common hardware timestamp. Pairing uses local reception
order, so a displayed error is a tracking observation, not motor acceptance.
"""

import argparse
import json
import math
from pathlib import Path
import time


# K1 serial order, matching booster_deploy/robots/k1.py and LowCmd.
JOINT_NAMES = (
    'head_yaw', 'head_pitch',
    'left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow_pitch', 'left_elbow_yaw',
    'right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow_pitch', 'right_elbow_yaw',
    'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw', 'left_knee_pitch',
    'left_ankle_pitch', 'left_ankle_roll',
    'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw', 'right_knee_pitch',
    'right_ankle_pitch', 'right_ankle_roll',
)
N_JOINTS = len(JOINT_NAMES)


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def fmt(value):
    return '   n/a' if value is None else f'{value:+7.3f}'


class JointMonitor:
    def __init__(self, node, output, joints, period, max_command_age):
        from booster_interface.msg import LowCmd, LowState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        self.node = node
        self.output = output
        self.joints = joints
        self.max_command_age_ns = int(max_command_age * 1e9)
        self.command_count = 0
        self.feedback_count = 0
        self.latest_command = None
        self.latest_feedback = None
        self.last_printed_feedback = 0
        self.last_warning = set()

        # Best effort accepts both reliable and best-effort publishers. Volatile
        # avoids replaying an old motor target when the monitor starts.
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(LowCmd, '/joint_ctrl', self.on_command, qos)
        node.create_subscription(LowState, '/low_state', self.on_feedback, qos)
        node.create_timer(period, self.show)
        self.write({'type': 'metadata', 'wall_time_ns': time.time_ns(),
                    'joint_names': JOINT_NAMES, 'joint_order': 'K1 serial',
                    'pairing': 'latest command received before feedback, by local monotonic clock',
                    'max_command_age_ms': max_command_age * 1000})

    def write(self, record):
        if self.output is not None:
            self.output.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
            self.output.flush()

    def warn_once(self, key, message):
        if key not in self.last_warning:
            self.last_warning.add(key)
            self.node.get_logger().warning(message)

    def on_command(self, msg):
        stamp_ns = time.monotonic_ns()
        self.command_count += 1
        serial = msg.cmd_type == msg.CMD_TYPE_SERIAL
        valid = serial and len(msg.motor_cmd) == N_JOINTS
        if not serial:
            self.warn_once('parallel', 'Ignoring non-serial /joint_ctrl for angle comparison')
        elif not valid:
            self.warn_once('command_length', f'Expected {N_JOINTS} commands, got {len(msg.motor_cmd)}')

        command = {
            'type': 'command', 'seq': self.command_count,
            'wall_time_ns': time.time_ns(), 'monotonic_ns': stamp_ns,
            'cmd_type': int(msg.cmd_type), 'valid_serial_22': valid,
            'q': [finite(m.q) for m in msg.motor_cmd],
            'dq': [finite(m.dq) for m in msg.motor_cmd],
            'tau': [finite(m.tau) for m in msg.motor_cmd],
            'kp': [finite(m.kp) for m in msg.motor_cmd],
            'kd': [finite(m.kd) for m in msg.motor_cmd],
            'weight': [finite(m.weight) for m in msg.motor_cmd],
        }
        self.latest_command = command
        self.write(command)

    def on_feedback(self, msg):
        stamp_ns = time.monotonic_ns()
        self.feedback_count += 1
        motors = msg.motor_state_serial
        valid = len(motors) == N_JOINTS
        if not valid:
            self.warn_once('feedback_length', f'Expected {N_JOINTS} serial feedback motors, got {len(motors)}')
        q = [finite(m.q) for m in motors]
        command = self.latest_command
        age_ns = stamp_ns - command['monotonic_ns'] if command else None
        fresh = (valid and command is not None and command['valid_serial_22']
                 and age_ns is not None and 0 <= age_ns <= self.max_command_age_ns)
        target = command['q'] if fresh else None
        error = ([None if a is None or b is None else a - b for a, b in zip(target, q)]
                 if target is not None else None)
        feedback = {
            'type': 'feedback', 'seq': self.feedback_count,
            'wall_time_ns': time.time_ns(), 'monotonic_ns': stamp_ns,
            'valid_serial_22': valid, 'q': q,
            'dq': [finite(m.dq) for m in motors],
            'tau_est': [finite(m.tau_est) for m in motors],
            'command_seq': command['seq'] if fresh else None,
            'command_age_ms': age_ns / 1e6 if fresh else None,
            'target_q': target, 'target_minus_feedback_q': error,
        }
        self.latest_feedback = feedback
        self.write(feedback)

    def show(self):
        sample = self.latest_feedback
        if sample is None:
            print(f'Waiting for /low_state; /joint_ctrl commands seen: {self.command_count}', flush=True)
            return
        if sample['seq'] == self.last_printed_feedback:
            print('No new /low_state feedback', flush=True)
            return
        self.last_printed_feedback = sample['seq']
        age = sample['command_age_ms']
        pairing = (f'command #{sample["command_seq"]}, age {age:.1f} ms'
                   if age is not None else 'no fresh serial command')
        print(f'\ncmd={self.command_count} feedback={self.feedback_count}  {pairing}', flush=True)
        print(' id joint                    target     actual  target-actual     dq_fb   tau_est', flush=True)
        for i in self.joints:
            if i >= len(sample['q']):
                continue
            target = sample['target_q'][i] if sample['target_q'] is not None else None
            error = (sample['target_minus_feedback_q'][i]
                     if sample['target_minus_feedback_q'] is not None else None)
            print(f'{i:3d} {JOINT_NAMES[i]:23s} {fmt(target)} {fmt(sample["q"][i])} '
                  f'{fmt(error)} {fmt(sample["dq"][i])} {fmt(sample["tau_est"][i])}',
                  flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Write every received command and feedback to a new JSONL file')
    parser.add_argument('--seconds', type=float, default=0,
                        help='Stop after this many seconds; 0 means until Ctrl+C')
    parser.add_argument('--period', type=float, default=1,
                        help='Seconds between terminal tables (default: 1)')
    parser.add_argument('--joints', nargs='+', type=int,
                        help='Joint indices to print (default: all 0..21); JSONL always saves all')
    parser.add_argument('--max-command-age', type=float, default=0.2,
                        help='Maximum receipt gap for pairing target with feedback, seconds (default: 0.2)')
    args = parser.parse_args()
    if args.seconds < 0 or not math.isfinite(args.seconds):
        parser.error('--seconds must be finite and nonnegative')
    if args.period <= 0 or not math.isfinite(args.period):
        parser.error('--period must be finite and positive')
    if args.max_command_age <= 0 or not math.isfinite(args.max_command_age):
        parser.error('--max-command-age must be finite and positive')
    joints = args.joints if args.joints is not None else range(N_JOINTS)
    if any(i < 0 or i >= N_JOINTS for i in joints):
        parser.error('--joints must be indices from 0 to 21')

    import rclpy

    output = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        try:
            output = args.output.open('x', encoding='utf-8')
        except FileExistsError:
            parser.error('--output must name a new file')
    rclpy.init()
    node = rclpy.create_node('k1_joint_ctrl_feedback_monitor')
    try:
        monitor = JointMonitor(node, output, joints, args.period, args.max_command_age)
        print('Read-only: /joint_ctrl targets and /low_state serial feedback; '
              'angles are radians. No motor command is published.', flush=True)
        if args.output:
            print(f'JSONL: {args.output}', flush=True)
        deadline = time.monotonic() + args.seconds if args.seconds else None
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if 'monitor' in locals() and monitor.latest_feedback is not None:
            if monitor.latest_feedback['seq'] != monitor.last_printed_feedback:
                monitor.show()
        print(f'Observed {monitor.command_count if "monitor" in locals() else 0} commands and '
              f'{monitor.feedback_count if "monitor" in locals() else 0} feedback messages.', flush=True)
        node.destroy_node()
        rclpy.shutdown()
        if output is not None:
            output.close()


if __name__ == '__main__':
    main()
