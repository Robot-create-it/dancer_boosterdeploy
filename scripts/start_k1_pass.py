"""Start/reuse camera and migrated vision, check inputs, then enter deploy.

Only the final deploy subprocess can command joints. --check-only and
--vision-only never construct a robot controller.
"""

import argparse
from pathlib import Path
import os
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from booster_deploy.utils.vision_config import load_vision_config, camera_topics


class InputProbe:
    def __init__(self, config, color, depth, task="k1_pass"):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import Pose
        from vision_interface.msg import Detections
        from booster_deploy.utils.vision_ball import select_ball_observation
        self.rclpy = rclpy
        self.node = rclpy.create_node(f"deploy_{task.removeprefix('k1_')}_input_check")
        self.seen = {}
        self.count = {}
        self.ball = None
        self.image_size = None
        def mark(name):
            self.seen[name] = time.monotonic()
            self.count[name] = self.count.get(name, 0) + 1
        def image(msg, name):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
            if msg.width > 0 and msg.height > 0 and -0.05 <= age <= 0.5:
                mark(name)
                if name == color:
                    self.image_size = (msg.width, msg.height)
        def pose(msg):
            import math
            q = msg.orientation
            values = (msg.position.x, msg.position.y, msg.position.z, q.x, q.y, q.z, q.w)
            if all(map(math.isfinite, values)) and 0.81 <= sum(v*v for v in values[3:]) <= 1.21:
                mark("/head_pose")
        def detection(msg):
            now = self.node.get_clock().now().nanoseconds * 1e-9
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if -0.05 <= now - stamp <= 0.5:
                mark("/booster_vision/detection")
                self.ball = select_ball_observation(msg, time.monotonic(), now)
        self.node.create_subscription(Image, color, lambda m: image(m, color), qos_profile_sensor_data)
        self.required = [color, "/head_pose", "/booster_vision/detection"]
        if config.get("use_depth", False):
            self.node.create_subscription(Image, depth, lambda m: image(m, depth), qos_profile_sensor_data)
            self.required.append(depth)
        self.node.create_subscription(Pose, "/head_pose", pose, qos_profile_sensor_data)
        self.node.create_subscription(Detections, "/booster_vision/detection", detection, qos_profile_sensor_data)

    def wait(self, timeout, processes=()):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError("A camera/vision process exited; inspect the task's camera/vision logs")
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()
            if all(self.count.get(name, 0) >= 3 and now - self.seen.get(name, 0) < 0.5
                   for name in self.required):
                print(f"Inputs ready: {self.required}; image size={self.image_size}", flush=True)
                return
        missing = [name for name in self.required if self.count.get(name, 0) < 3
                   or time.monotonic() - self.seen.get(name, 0) >= 0.5]
        raise RuntimeError(f"Missing/stale inputs: {missing}. Check platform camera/head_pose, ROS_DOMAIN_ID and image timestamps.")


def main(task="k1_pass"):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vision-config", default="/opt/booster")
    parser.add_argument("--camera-driver", choices=("platform", "realsense"), default="platform")
    parser.add_argument("--color-topic")
    parser.add_argument("--depth-topic")
    parser.add_argument("--timeout", type=float, default=30.)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--exit-mode", choices=("walking", "damping"), default="walking")
    if task == "k1_shoot":
        parser.add_argument("--shoot-policy", choices=("2", "264", "192", "0109_0", "0109_2"),
                            default="2", help="Choose the fixed shoot model by filename suffix")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true", help="Read-only check; starts no camera, vision or motion processes")
    mode.add_argument("--vision-only", action="store_true", help="Start/reuse vision and print ball positions; no motion")
    mode.add_argument("--head-only", action="store_true", help="Hold the default body stance and track the head; no locomotion/kick model")
    args = parser.parse_args()
    if not (args.check_only or args.vision_only):
        from booster_deploy.utils.robot_runtime import require_robot_interface
        try:
            require_robot_interface()
        except RuntimeError as exc:
            parser.exit(1, f"ERROR: {exc}\n")
    config = load_vision_config(args.vision_config)
    color, depth = camera_topics(config)
    if args.camera_driver == "realsense":
        if config["camera"]["type"] != "realsense":
            parser.error("--camera-driver realsense requires a realsense calibration configuration")
        color, depth = "/boostercamera/head/color/image_raw", "/boostercamera/head/aligned_depth_to_color/image_raw"
    color, depth = args.color_topic or color, args.depth_topic or depth

    import rclpy
    rclpy.init()
    probe = InputProbe(config, color, depth, task)
    processes, files = [], []
    def launch(command, name):
        (ROOT / "logs").mkdir(exist_ok=True)
        log = (ROOT / "logs" / f"{task.removeprefix('k1_')}_{name}.log").open("w")
        files.append(log)
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        print(f"Started {name}, pid={process.pid}; log={log.name}", flush=True)
        return process
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        # Allow DDS discovery before deciding whether a producer already exists.
        for _ in range(15):
            rclpy.spin_once(probe.node, timeout_sec=0.1)
        producers = probe.node.count_publishers("/booster_vision/detection")
        if producers > 1:
            raise RuntimeError("Multiple detection publishers found; use one vision instance")
        if not args.check_only:
            if args.camera_driver == "realsense" and probe.node.count_publishers(color) == 0:
                launch(["ros2", "launch", "realsense2_camera", "rs_launch.py",
                        "camera_namespace:=boostercamera", "camera_name:=head", "align_depth.enable:=true"], "camera")
            if producers == 0:
                launch(["ros2", "launch", "vision", "launch.py",
                        f"vision_config_path:={Path(args.vision_config).resolve()}",
                        "save_data:=false", "show_det:=false",
                        f"color_topic:={color}", f"depth_topic:={depth}"], "vision")
            else:
                print("Reusing existing vision publisher; ensure its calibration matches --vision-config.", flush=True)
        probe.wait(args.timeout, processes)
        if args.check_only:
            print("Read-only check passed; no motion commands sent.")
            return 0
        if args.vision_only:
            print("Vision only; Ctrl+C exits. Ball coordinates are robot-frame metres.", flush=True)
            next_print = 0.
            while True:
                rclpy.spin_once(probe.node, timeout_sec=0.1)
                if time.monotonic() >= next_print:
                    ball = probe.ball
                    fresh = ball is not None and 0 <= time.monotonic() - ball.stamp <= 0.5
                    print(f"ball=({ball.x:.3f}, {ball.y:.3f}), confidence={ball.confidence:.1f}"
                          if fresh else "ball=NONE/STALE", flush=True)
                    next_print = time.monotonic() + 1.
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError("Vision/camera exited; inspect logs")
        command = [sys.executable, "scripts/deploy.py", "--task", task,
                   "--vision-config", str(Path(args.vision_config).resolve()),
                   "--color-topic", color, "--device", args.device, "--exit-mode", args.exit_mode]
        if task == "k1_shoot":
            command.extend(("--shoot-policy", args.shoot_policy))
        if args.head_only:
            command.append("--head-only")
        # Keep deploy interactive, including its existing X/A controls.
        controller = subprocess.Popen(command, cwd=ROOT, start_new_session=True)
        processes.append(controller)
        return controller.wait()
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        # Only terminate process groups started by this invocation. Platform
        # camera, reused vision and unrelated ROS processes remain owned by their callers.
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=2.)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
        for log in files:
            log.close()
        probe.node.destroy_node()
        # SIGINT may already have shut down the context in rclpy's handler.
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
