"""K1 recovery feedback and state checks.

These checks are not a motor current limiter or a complete firmware fault decoder.
"""

import numpy as np


FAULTS = {
    1: 'expected 22 serial and parallel joints',
    2: 'non-finite or malformed feedback',
    3: 'arm serial/parallel q,dq,tau and parallel temperature are all zero',
}


def feedback_fault(message):
    """Return (reason code, joint index); ROS reserve=0 is not a health proof."""
    serial = message.motor_state_serial
    parallel = getattr(message, 'motor_state_parallel', [])
    if len(serial) != 22 or len(parallel) != 22:
        return 1, -1
    imu = message.imu_state
    if (len(imu.rpy) != 3 or len(imu.gyro) != 3
            or not np.isfinite([*imu.rpy, *imu.gyro]).all()):
        return 2, -1
    for i, (s, p) in enumerate(zip(serial, parallel)):
        values = [s.q, s.dq, s.tau_est, p.q, p.dq, p.tau_est, p.temperature]
        if not np.isfinite(values).all():
            return 2, i
        if 2 <= i < 10 and all(v == 0 for v in values):
            return 3, i
    return 0, -1


def state_fault(state, now, max_age, ready_duration=0.0):
    received = float(state['state_received'])
    if received <= 0 or not 0 <= now - received <= max_age:
        return 'low_state missing or stale'
    code = int(state['recovery_fault_code'])
    if code:
        return f"{FAULTS.get(code, 'unknown feedback fault')}; joint={int(state['recovery_fault_joint'])}"
    valid_since = float(state['recovery_valid_since'])
    if valid_since <= 0 or now - valid_since < ready_duration:
        return f'need {ready_duration:.2f}s of continuous valid feedback before Custom'
    return None
