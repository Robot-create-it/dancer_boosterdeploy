"""Follow a recovery JSONL file without ROS, motor commands, or mode changes."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import time


def show_frame(frame, joints, names, observation):
    stamp = datetime.fromtimestamp(frame['wall_time']).isoformat(timespec='milliseconds')
    gravity = ','.join(f'{v:+.3f}' for v in frame['gravity'])
    print(f"\n{stamp} step={frame['step']} state={frame['state']} "
          f"posture={frame['posture']} row={frame['row']} phase={frame['phase']:.3f} "
          f"gravity=({gravity})", flush=True)
    print('joint                         q(rad)  target    error   dq(rad/s)  tau_fb(Nm)  PD_est(Nm)')
    for i in joints:
        q, target = frame['q'][i], frame['target_proposed'][i]
        name = names[i] if names else f'joint_{i}'
        print(f"{i:2d} {name:26s} {q:+7.3f} {target:+7.3f} {target-q:+7.3f} "
              f"{frame['dq'][i]:+9.3f} {frame['tau_feedback'][i]:+11.3f} "
              f"{frame['pd_estimate'][i]:+11.3f}")
    if observation:
        print('obs100=' + json.dumps(frame['observation']))
    print(flush=True)


def watch(path, joints, every, observation, follow):
    if not path.exists():
        if not follow:
            raise FileNotFoundError(path)
        print(f'Waiting for {path}; this viewer does not start recovery.', flush=True)
        while not path.exists():
            time.sleep(0.1)
    names = []
    count = 0
    previous_state = None
    last_record = time.monotonic()
    idle_reported = False
    with path.open(encoding='utf-8') as source:
        while True:
            position = source.tell()
            line = source.readline()
            # A writer can be interrupted halfway through a JSON record.
            # Re-read it only once its terminating newline has arrived.
            if not line or not line.endswith('\n'):
                source.seek(position)
                if not follow:
                    if line:
                        print('Incomplete trailing record ignored.', flush=True)
                    return
                if not idle_reported and time.monotonic() - last_record >= 2:
                    print('No new complete records for 2 s; policy may be stopped. '
                          'This is not a motor health diagnosis.', flush=True)
                    idle_reported = True
                time.sleep(0.1)
                continue
            last_record = time.monotonic()
            idle_reported = False
            frame = json.loads(line)
            if frame['type'] == 'metadata':
                names = frame['joint_names']
                print(f"controller={frame['controller']} policy_dt={frame['policy_dt']} "
                      f"model_sha256={frame['model_sha256']}", flush=True)
                print('Targets are proposals; PD_est is a calculation, not measured torque. '
                      'Display only: no automatic stop or fault detection.', flush=True)
            elif frame['type'] == 'error':
                print('POLICY ERROR: ' + frame['message'], flush=True)
            elif frame['type'] == 'step':
                if count % every == 0 or frame['state'] != previous_state:
                    show_frame(frame, joints, names, observation)
                count += 1
                previous_state = frame['state']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--joints', nargs='+', type=int, default=[2, 6, 9])
    parser.add_argument('--every', type=int, default=50,
                        help='Display one out of N step records (default: 50); file stays unchanged')
    parser.add_argument('--observation', action='store_true', help='Also print the full obs100')
    parser.add_argument('--no-follow', action='store_true', help='Read an existing file and exit at EOF')
    args = parser.parse_args()
    if args.every < 1 or any(i < 0 or i >= 22 for i in args.joints):
        parser.error('--every must be positive; joint indices must be between 0 and 21')
    try:
        watch(args.trace, args.joints, args.every, args.observation, not args.no_follow)
    except KeyboardInterrupt:
        print('\nViewer stopped; robot/controller state was not changed.')
    except (OSError, ValueError, KeyError, IndexError, TypeError) as exc:
        parser.exit(1, f'Cannot read recovery trace: {exc}\n')


if __name__ == '__main__':
    main()
