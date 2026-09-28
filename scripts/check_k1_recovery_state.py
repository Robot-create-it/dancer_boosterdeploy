"""Read K1 status and feedback only; never publishes commands or changes mode."""

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from booster_deploy.utils.recovery_state_check import summarize_samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 30:
        parser.error('--seconds must be between 1 and 30')
    if args.output and args.output.exists():
        parser.error('--output must name a new file')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from booster_interface.msg import LowState, BoosterApiReqMsg
    from booster_interface.srv import RpcService

    rclpy.init()
    node = rclpy.create_node('k1_recovery_readonly_check')
    samples = []
    report = {'sampled_at': time.strftime('%Y-%m-%d %H:%M:%S%z')}
    try:
        node.create_subscription(LowState, '/low_state', samples.append,
                                 qos_profile_sensor_data)
        client = node.create_client(RpcService, '/booster_rpc_service')
        if client.wait_for_service(timeout_sec=2):
            request = RpcService.Request()
            request.msg = BoosterApiReqMsg()
            request.msg.api_id = 2018  # GetStatus, read-only
            request.msg.body = ''
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if future.done() and future.result() is not None:
                response = future.result().msg
                report['status'] = {'code': int(response.status), 'body': response.body}
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report.update(summarize_samples(samples))
        if samples:
            report['imu_rpy'] = [float(v) for v in samples[-1].imu_state.rpy]
        report['joint_ctrl_publishers'] = [
            {'name': item.node_name, 'namespace': item.node_namespace}
            for item in node.get_publishers_info_by_topic('/joint_ctrl')]
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'groups'}, indent=2))
        for group, motors in report.get('groups', {}).items():
            print(group)
            for motor in motors[2:10]:
                print(json.dumps(motor))
        return 2 if (not samples or report['suspect_arm_indices']) else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
