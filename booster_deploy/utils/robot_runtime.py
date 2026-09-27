"""Read-only dependency checks for real-robot entry points."""

import sys


def require_robot_interface():
    """Check message imports and native type support without creating a node."""
    try:
        from booster_interface.msg import BoosterApiReqMsg, LowState, LowCmd, MotorCmd
        from booster_interface.srv import RpcService
        from rclpy.type_support import check_for_type_support

        for message_type in (BoosterApiReqMsg, LowState, LowCmd, MotorCmd, RpcService):
            check_for_type_support(message_type)
    except (ImportError, OSError) as exc:
        raise RuntimeError(
            f"Booster ROS 2 interface unavailable for {sys.executable}: {exc}\n"
            "In the same terminal, from the repository root, run:\n"
            "  source .venv/bin/activate\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source /opt/booster/BoosterRos2Interface/install/setup.bash\n"
            "  source vision_ws/install/local_setup.bash\n"
            "Then retry. booster_interface is supplied by the robot runtime, not pip."
        ) from exc
