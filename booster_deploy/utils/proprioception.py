"""Common locomotion/recovery sensor inputs, in the robot's serial order."""

import torch

from .isaaclab import math as lab_math


def read_proprioception(robot_data):
    """Return q, dq, body gyro (rad/s), and gravity from the same state sample.

    Hardware populates this data from low_state motor_state_serial and IMU;
    root_quat_w is wxyz, converted from the IMU RPY by the controller.
    No additional frame rotation, gyro conversion, or gravity offset is applied.
    """
    gravity_w = torch.tensor(
        [0.0, 0.0, -1.0], dtype=torch.float32,
        device=robot_data.root_quat_w.device,
    )
    return (
        robot_data.joint_pos,
        robot_data.joint_vel,
        robot_data.root_ang_vel_b,
        lab_math.quat_apply_inverse(robot_data.root_quat_w, gravity_w),
    )
