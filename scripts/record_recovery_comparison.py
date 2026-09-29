"""Record two read-only K1 recovery trials and compare their measured motion.

Record before triggering either recovery. This script never publishes ROS
messages or invokes a robot RPC. ROS bag timestamps are recorder receipt times;
they are not synchronized hardware timestamps. Firmware events come from the
local syslog and retain the daemon's own timestamp.
"""

from __future__ import annotations

import argparse
import bisect
from collections import deque
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import threading
import time


TOPICS = {
    "/low_state": "booster_interface/msg/LowState",
    "/joint_ctrl": "booster_interface/msg/LowCmd",
    "/fall_down": "booster_interface/msg/FallDownState",
    "/robot_states": "booster_interface/msg/RobotStatesMsg",
    "/prone_body_control_status": "booster_interface/msg/ProneBodyControlStatus",
    "/fall_down_recovery_state": "booster_interface/msg/RawBytesMsg",
    "/LocoApiTopicReq": "booster_msgs/msg/RpcReqMsg",
    "/LocoApiTopicResp": "booster_msgs/msg/RpcRespMsg",
    "/odometer_state": "booster_interface/msg/Odometer",
    "/send_imu": "sensor_msgs/msg/Imu",
    "/booster/ros2_k2_imu": "sensor_msgs/msg/Imu",
    "/joint_states": "sensor_msgs/msg/JointState",
    # Captured for investigation; its relationship to the final motor command
    # is not established. Do not treat it as the actuator-side target.
    "/booster/ros2_k2_joint_cmd": "sensor_msgs/msg/JointState",
    "/booster/ros2_k2_joint_states": "sensor_msgs/msg/JointState",
}
REQUIRED = {"/low_state"}
JOINTS = (2, 5, 6, 9, 10, 13, 16, 19)
JOINT_NAMES = {
    2: "left_shoulder_pitch", 5: "left_elbow_yaw",
    6: "right_shoulder_pitch", 9: "right_elbow_yaw",
    10: "left_hip_pitch", 13: "left_knee_pitch",
    16: "right_hip_pitch", 19: "right_knee_pitch",
}
CONFIG_FILES = (
    "/opt/booster/Gait/configs/K1/common_module_options.lua",
    "tasks/locomotion/robots/k1/recovery_config.json",
    "tasks/locomotion/k1_recovery.py",
    "booster_deploy/controllers/booster_robot_controller.py",
)


def utc_now():
    return datetime.now().astimezone().isoformat(timespec="microseconds")


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def discover_topics():
    from rosidl_runtime_py.utilities import get_message

    discovered = {}
    for _ in range(3):
        result = subprocess.run(
            ["ros2", "topic", "list", "--no-daemon", "--spin-time", "3", "-t"],
            capture_output=True, text=True, timeout=15,
        )
        if result.returncode:
            raise RuntimeError(f"Cannot discover ROS topics: {result.stderr.strip()}")
        for line in result.stdout.splitlines():
            match = re.fullmatch(r"(\S+) \[(.+)\]", line.strip())
            if match:
                discovered[match.group(1)] = match.group(2)
        if REQUIRED <= discovered.keys():
            break
    selected, skipped = [], {}
    for topic, expected in TOPICS.items():
        actual = discovered.get(topic)
        if topic == "/joint_ctrl" and actual is None:
            # The firmware trial has no external writer; keep the subscription
            # ready so a later deploy writer is captured in the second trial.
            get_message(expected)
            selected.append(topic)
        elif actual is None:
            skipped[topic] = "topic absent at startup"
        elif actual != expected:
            skipped[topic] = f"type {actual}, expected {expected}"
        else:
            try:
                get_message(actual)
            except Exception as exc:
                skipped[topic] = f"cannot load ROS type: {exc}"
            else:
                selected.append(topic)
    absent_required = REQUIRED - set(selected)
    if absent_required:
        raise RuntimeError(
            f"Required topics unavailable: {sorted(absent_required)}; "
            "source the robot ROS install before running this script"
        )
    return selected, skipped


def preflight_feedback():
    """Wait for DDS discovery, then require 0.5 s of valid arm feedback."""
    import rclpy
    from booster_interface.msg import LowState
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    samples = deque(maxlen=600)
    received = 0
    first_received = None

    def on_state(msg):
        nonlocal received, first_received
        stamp = time.monotonic_ns()
        received += 1
        if first_received is None:
            first_received = stamp
        samples.append((stamp, msg))

    rclpy.init()
    node = rclpy.create_node("k1_recovery_record_preflight")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(
        LowState, "/low_state", on_state, qos,
    )
    start = time.monotonic_ns()
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not samples:
                continue
            last_time = samples[-1][0]
            recent = [row for row in samples
                      if row[0] >= last_time - 500_000_000]
            if (len(recent) >= 20
                    and recent[-1][0] - recent[0][0] >= 450_000_000
                    and time.monotonic_ns() - last_time < 100_000_000):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not samples:
        raise RuntimeError(
            "Preflight received 0 /low_state samples in 8 s; "
            "check the publisher and ROS_DOMAIN_ID before recording"
        )
    last_time = samples[-1][0]
    recent = [msg for stamp, msg in samples if stamp >= last_time - 500_000_000]
    if (len(recent) < 20
            or samples[-1][0] - samples[-len(recent)][0] < 450_000_000
            or time.monotonic_ns() - last_time >= 100_000_000):
        first_delay = (first_received - start) / 1e9
        raise RuntimeError(
            f"Preflight received {received} /low_state samples in 8 s "
            f"(first after {first_delay:.2f} s), but no continuous 0.5 s "
            "window; check the publisher before recording"
        )
    faulty, malformed = set(), False
    for msg in recent:
        serial, parallel = msg.motor_state_serial, msg.motor_state_parallel
        if len(serial) != 22 or len(parallel) != 22:
            malformed = True
            continue
        for joint in range(2, 10):
            s, p = serial[joint], parallel[joint]
            if all(value == 0 for value in
                   (s.q, s.dq, s.tau_est, p.q, p.dq, p.tau_est, p.temperature)):
                faulty.add(joint)
    if malformed or faulty:
        raise RuntimeError(
            "Preflight failed: malformed 22-joint feedback"
            if malformed else
            f"Preflight failed: arm joints {sorted(faulty)} have all-zero "
            "serial/parallel feedback; clear and diagnose the drive/CAN faults "
            "before another recovery trial"
        )
    return {"samples": received, "recent_samples": len(recent),
            "first_sample_delay_s": round((first_received - start) / 1e9, 3),
            "arm_all_zero_joints": []}


def capture_syslog(process, output):
    with output.open("w", encoding="utf-8", buffering=1) as stream:
        for line in process.stdout:
            if "booster-daemon" in line:
                stream.write(line)


def stop_process_group(process, timeout=20):
    if process.poll() is not None:
        return process.returncode
    os.killpg(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            return process.wait()


def record(args):
    selected, skipped = discover_topics()
    if not os.access(args.syslog, os.R_OK):
        raise RuntimeError(f"Cannot read {args.syslog}")
    preflight = preflight_feedback()
    trial_dir = args.output.expanduser().resolve()
    trial_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "trial": args.trial,
        "start": utc_now(),
        "start_wall_time_ns": time.time_ns(),
        "topics": {name: TOPICS[name] for name in selected},
        "skipped_topics": skipped,
        "timestamp_note": "ROS bag times are local recorder receipt times",
        "final_motor_target_exposed": False,
        "contact_sensor_exposed": False,
        "preflight": preflight,
        "file_sha256": {},
    }
    repo = Path(__file__).resolve().parent.parent
    for name in CONFIG_FILES:
        path = Path(name) if name.startswith("/") else repo / name
        if path.is_file():
            manifest["file_sha256"][name] = sha256(path)
    (trial_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    qos_path = trial_dir / "qos_overrides.yaml"
    qos_path.write_text(
        "/LocoApiTopicReq:\n"
        "  reliability: reliable\n"
        "  durability: volatile\n"
        "  history: keep_last\n"
        "  depth: 100\n"
        "/LocoApiTopicResp:\n"
        "  reliability: reliable\n"
        "  durability: volatile\n"
        "  history: keep_last\n"
        "  depth: 100\n",
        encoding="utf-8",
    )

    tail = subprocess.Popen(
        ["tail", "-n", "0", "-F", str(args.syslog)],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        bufsize=1, start_new_session=True,
    )
    syslog_thread = threading.Thread(
        target=capture_syslog, args=(tail, trial_dir / "firmware.log"),
        daemon=True,
    )
    syslog_thread.start()
    bag_log = (trial_dir / "rosbag.log").open("w", encoding="utf-8")
    bag = None
    try:
        bag = subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(trial_dir / "bag"),
             "--include-unpublished-topics",
             "--qos-profile-overrides-path", str(qos_path), *selected],
            stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        time.sleep(3)
        if bag.poll() is not None:
            raise RuntimeError(
                f"ros2 bag record exited early ({bag.returncode}); "
                f"see {trial_dir / 'rosbag.log'}"
            )
        print(f"Recording {args.trial}: {trial_dir}", flush=True)
        print("Recorder ready. Trigger recovery separately. Ctrl+C stops recording.", flush=True)
        if skipped:
            print(f"Optional topics skipped: {skipped}", flush=True)
        deadline = time.monotonic() + args.seconds if args.seconds else None
        while bag.poll() is None and (deadline is None or time.monotonic() < deadline):
            time.sleep(0.2)
        if bag.poll() is not None:
            raise RuntimeError(f"ros2 bag record stopped unexpectedly ({bag.returncode})")
    except KeyboardInterrupt:
        pass
    finally:
        manifest["end"] = utc_now()
        manifest["end_wall_time_ns"] = time.time_ns()
        if bag is not None:
            manifest["rosbag_returncode"] = stop_process_group(bag)
        bag_log.close()
        if tail.poll() is None:
            tail.terminate()
        try:
            tail.wait(timeout=3)
        except subprocess.TimeoutExpired:
            tail.kill()
            tail.wait()
        syslog_thread.join(timeout=3)
        manifest["bag_metadata_exists"] = (trial_dir / "bag" / "metadata.yaml").is_file()
        (trial_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print(f"Saved bag, firmware.log and manifest.json in {trial_dir}", flush=True)
        if manifest["bag_metadata_exists"]:
            try:
                path, commands, feedback = export_trial(trial_dir)
                print(f"Saved {feedback} feedback and {commands} commands to {path}",
                      flush=True)
            except Exception as exc:
                print(f"Joint JSONL export failed; bag remains available: {exc}",
                      file=sys.stderr, flush=True)


def finite(value):
    value = float(value)
    return value if math.isfinite(value) else None


def motor_values(motors):
    return {
        "q": [finite(m.q) for m in motors],
        "dq": [finite(m.dq) for m in motors],
        "tau": [finite(m.tau_est) for m in motors],
        "mode": [int(m.mode) for m in motors],
        "lost": [int(m.lost) for m in motors],
        "reserve": [[int(x) for x in m.reserve] for m in motors],
        "temperature": [int(m.temperature) for m in motors],
    }


def read_bag(trial_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    manifest = json.loads((trial_dir / "manifest.json").read_text(encoding="utf-8"))
    bag_dir = trial_dir / "bag"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    classes = {}
    for info in reader.get_all_topics_and_types():
        if info.name in ("/low_state", "/joint_ctrl", "/fall_down",
                         "/robot_states", "/LocoApiTopicReq", "/LocoApiTopicResp",
                         "/booster/ros2_k2_joint_cmd"):
            classes[info.name] = get_message(info.type)
    result = {
        "manifest": manifest, "feedback": [], "commands": [],
        "fall": [], "modes": [], "rpc": [], "candidate_commands": [],
    }
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(raw, classes[topic])
        if topic == "/low_state":
            result["feedback"].append({
                "t": stamp,
                "serial": motor_values(msg.motor_state_serial),
                "parallel": motor_values(msg.motor_state_parallel),
                "rpy": [finite(x) for x in msg.imu_state.rpy],
                "gyro": [finite(x) for x in msg.imu_state.gyro],
                "acc": [finite(x) for x in msg.imu_state.acc],
            })
        elif topic == "/joint_ctrl":
            result["commands"].append({
                "t": stamp, "cmd_type": int(msg.cmd_type),
                "q": [finite(m.q) for m in msg.motor_cmd],
                "dq": [finite(m.dq) for m in msg.motor_cmd],
                "tau": [finite(m.tau) for m in msg.motor_cmd],
                "kp": [finite(m.kp) for m in msg.motor_cmd],
                "kd": [finite(m.kd) for m in msg.motor_cmd],
                "weight": [finite(m.weight) for m in msg.motor_cmd],
            })
        elif topic == "/fall_down":
            result["fall"].append({
                "t": stamp, "state": int(msg.fall_down_state),
                "available": bool(msg.is_recovery_available),
            })
        elif topic == "/robot_states":
            result["modes"].append({
                "t": stamp, "mode": int(msg.current_mode),
                "body_control": int(msg.current_body_control),
                "actions": [int(x) for x in msg.current_actions],
            })
        elif topic == "/booster/ros2_k2_joint_cmd":
            result["candidate_commands"].append({
                "t": stamp, "name": list(msg.name),
                "position": [finite(x) for x in msg.position],
                "velocity": [finite(x) for x in msg.velocity],
                "effort": [finite(x) for x in msg.effort],
            })
        else:
            header = {}
            try:
                header = json.loads(msg.header)
            except (ValueError, TypeError):
                pass
            result["rpc"].append({
                "t": stamp, "topic": topic, "uuid": msg.uuid,
                "api_id": header.get("api_id"), "header": msg.header,
                "body": msg.body,
            })
    return result


def export_trial(trial_dir, output=None, max_command_age_ms=200.0):
    """Write the previous monitor's per-frame target/feedback view from a bag."""
    trial_dir = Path(trial_dir)
    trial = read_bag(trial_dir)
    output = Path(output) if output is not None else trial_dir / "joint_tracking.jsonl"
    output.parent.mkdir(parents=True, exist_ok=True)
    commands = trial["commands"]
    feedback = trial["feedback"]
    for seq, row in enumerate(commands, 1):
        row["seq"] = seq
    for seq, row in enumerate(feedback, 1):
        row["seq"] = seq
    events = sorted(
        [(row["t"], 0, row) for row in commands] +
        [(row["t"], 1, row) for row in feedback],
        key=lambda item: (item[0], item[1]),
    )
    latest_command = None
    with output.open("x", encoding="utf-8", buffering=1) as stream:
        def write(record):
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

        write({
            "type": "metadata", "trial": trial["manifest"]["trial"],
            "source": "ROS bag recorder receipt timestamps",
            "joint_order": "K1 serial, indices 0..21",
            "max_command_age_ms": max_command_age_ms,
            "firmware_internal_target_available": False,
        })
        for stamp, kind, row in events:
            if kind == 0:
                latest_command = row
                write({
                    "type": "command", "seq": row["seq"],
                    "wall_time_ns": stamp, "cmd_type": row["cmd_type"],
                    "valid_serial_22": row["cmd_type"] == 1 and len(row["q"]) == 22,
                    "q": row["q"], "dq": row["dq"], "tau": row["tau"],
                    "kp": row["kp"], "kd": row["kd"], "weight": row["weight"],
                })
                continue
            serial, parallel = row["serial"], row["parallel"]
            age = ((stamp - latest_command["t"]) / 1e6
                   if latest_command is not None else None)
            fresh = (latest_command is not None and age is not None
                     and 0 <= age <= max_command_age_ms
                     and latest_command["cmd_type"] == 1
                     and len(latest_command["q"]) == 22
                     and len(serial["q"]) == 22)
            target = latest_command["q"] if fresh else None
            error = (
                [None if desired is None or actual is None else desired - actual
                 for desired, actual in zip(target, serial["q"])]
                if target is not None else None
            )
            write({
                "type": "feedback", "seq": row["seq"],
                "wall_time_ns": stamp,
                "valid_serial_22": len(serial["q"]) == 22,
                "q": serial["q"], "dq": serial["dq"],
                "tau_est": serial["tau"],
                "serial_temperature": serial["temperature"],
                "serial_lost": serial["lost"],
                "serial_reserve": serial["reserve"],
                "parallel_q": parallel["q"],
                "parallel_dq": parallel["dq"],
                "parallel_tau_est": parallel["tau"],
                "parallel_temperature": parallel["temperature"],
                "parallel_lost": parallel["lost"],
                "parallel_reserve": parallel["reserve"],
                "imu_rpy": row["rpy"], "imu_gyro": row["gyro"],
                "imu_acc": row["acc"],
                "command_seq": latest_command["seq"] if fresh else None,
                "command_age_ms": age if fresh else None,
                "target_q": target,
                "target_minus_feedback_q": error,
            })
    return output, len(commands), len(feedback)


def firmware_events(trial_dir):
    path = trial_dir / "firmware.log"
    events, batteries = [], []
    seen_faults = set()
    if not path.is_file():
        return events, batteries
    year = datetime.fromisoformat(
        json.loads((trial_dir / "manifest.json").read_text())["start"]
    ).year
    for line in path.open(encoding="utf-8", errors="replace"):
        match = re.search(r"\[(\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]", line)
        if not match:
            continue
        stamp = datetime.strptime(f"{year}-{match.group(1)}", "%Y-%m-%d %H:%M:%S.%f")
        t = int(stamp.timestamp() * 1e9)
        battery = re.search(
            r"battery soc: ([\d.]+), voltage: ([\d.]+), "
            r"average_voltage: ([\d.]+), current: ([-\d.]+)", line
        )
        if battery:
            batteries.append({
                "t": t, "soc": float(battery[1]), "voltage": float(battery[2]),
                "average_voltage": float(battery[3]), "current": float(battery[4]),
            })
        if "locked-rotor" in line:
            joint = re.search(r"Joint\((\d+)\)", line)
            kind = f"locked-rotor joint {joint[1]}" if joint else "locked-rotor"
        elif "Read joint failed" in line:
            chan = re.search(r"can chan = (\d+)", line)
            kind = f"CAN read failure channel {chan[1]}" if chan else "CAN read failure"
        elif "FDR exec succeeded" in line:
            kind = "FDR exec succeeded"
        elif "OnEnter " in line or "OnExit " in line:
            mode = re.search(r"On(?:Enter|Exit) \w+", line)
            kind = mode[0] if mode else "mode change"
        else:
            continue
        if kind.startswith(("locked-rotor", "CAN read failure")):
            if kind in seen_faults:
                continue
            seen_faults.add(kind)
        if not events or events[-1]["kind"] != kind:
            events.append({"t": t, "kind": kind})
    return events, batteries


def nearest(rows, target):
    times = [row["t"] for row in rows]
    index = bisect.bisect_left(times, target)
    index = min((max(index - 1, 0), min(index, len(rows) - 1)),
                key=lambda i: abs(times[i] - target))
    return rows[index]


def latest_before(rows, target):
    if not rows:
        return None
    index = bisect.bisect_right([row["t"] for row in rows], target) - 1
    return rows[index] if index >= 0 else None


def onset(feedback, joint, threshold):
    if not feedback or len(feedback[0]["serial"]["q"]) <= joint:
        raise ValueError("No valid 22-joint feedback")
    start = feedback[0]["t"]
    baseline = statistics.median(
        row["serial"]["q"][joint] for row in feedback
        if row["t"] < start + 500_000_000
        and row["serial"]["q"][joint] is not None
    )
    for row in feedback:
        q = row["serial"]["q"][joint]
        if q is not None and abs(q - baseline) > threshold:
            return row["t"], baseline
    raise ValueError(f"No movement onset detected for joint {joint}")


def longest_stall(feedback, joint, start, end, torque=13.5, speed=0.2):
    longest = 0.0
    run_start = run_last = None
    for row in feedback:
        if row["t"] < start or row["t"] >= end:
            continue
        tau = row["serial"]["tau"][joint]
        dq = row["serial"]["dq"][joint]
        stalled = (tau is not None and dq is not None
                   and abs(tau) >= torque and abs(dq) < speed)
        if stalled:
            if run_start is None:
                run_start = row["t"]
            run_last = row["t"]
        elif run_start is not None:
            longest = max(longest, (run_last - run_start) / 1e9)
            run_start = run_last = None
    if run_start is not None:
        longest = max(longest, (run_last - run_start) / 1e9)
    return longest


def fmt_time(stamp):
    return datetime.fromtimestamp(stamp / 1e9).strftime("%H:%M:%S.%f")[:-3]


def summarize(trial, joint, threshold):
    f = trial["feedback"]
    if not f:
        raise ValueError("Bag has no /low_state samples")
    t0, baseline = onset(f, joint, threshold)
    events, batteries = firmware_events(trial["directory"])
    faults = [
        e for e in events
        if e["kind"].startswith("locked-rotor") and e["t"] >= t0
    ]
    fault_time = faults[0]["t"] if faults else None
    stop = min(t0 + 4_000_000_000, fault_time) if fault_time else t0 + 4_000_000_000
    commands = trial["commands"]
    gaps = [
        (b["t"] - a["t"]) / 1e6 for a, b in zip(commands, commands[1:])
    ]
    mode_changes, fall_changes = [], []
    for row in trial["modes"]:
        state = (row["mode"], row["body_control"])
        if not mode_changes or mode_changes[-1]["state"] != state:
            mode_changes.append({"t": fmt_time(row["t"]), "state": state})
    for row in trial["fall"]:
        state = (row["state"], row["available"])
        if not fall_changes or fall_changes[-1]["state"] != state:
            fall_changes.append({"t": fmt_time(row["t"]), "state": state})
    zero_feedback = {}
    for j in (2, 5, 6, 9):
        for row in f:
            if row["t"] < t0:
                continue
            s, p = row["serial"], row["parallel"]
            if len(s["q"]) <= j or len(p["q"]) <= j:
                continue
            if all(value == 0 for value in
                   (s["q"][j], s["dq"][j], s["tau"][j],
                    p["q"][j], p["dq"][j], p["tau"][j],
                    p["temperature"][j])):
                zero_feedback[str(j)] = fmt_time(row["t"])
                break
    return {
        "directory": str(trial["directory"]),
        "label": trial["manifest"]["trial"],
        "feedback_count": len(f),
        "feedback_start": fmt_time(f[0]["t"]),
        "feedback_end": fmt_time(f[-1]["t"]),
        "first_feedback_rpy": f[0]["rpy"],
        "onset": fmt_time(t0),
        "onset_delay_s": round((t0 - f[0]["t"]) / 1e9, 3),
        "onset_joint": joint,
        "onset_baseline_q": baseline,
        "onset_rpy": nearest(f, t0)["rpy"],
        "rpy_at_2s": nearest(f, t0 + 2_000_000_000)["rpy"],
        "command_count": len(commands),
        "unverified_bridge_command_count": len(trial["candidate_commands"]),
        "command_max_gap_ms": round(max(gaps), 3) if gaps else None,
        "stall_seconds": {
            str(j): round(longest_stall(f, j, t0, stop), 3)
            for j in (2, 5, 6, 9)
        },
        "first_fault": faults[0] if faults else None,
        "post_fault_commands": sum(c["t"] > faults[0]["t"] for c in commands)
            if faults else None,
        "first_all_zero_arm_feedback": zero_feedback,
        "firmware_events": events[:100],
        "battery_min_voltage": min(batteries, key=lambda b: b["voltage"])
            if batteries else None,
        "battery_max_draw": min(batteries, key=lambda b: b["current"])
            if batteries else None,
        "mode_changes": mode_changes,
        "fall_changes": fall_changes,
        "rpc_samples": trial["rpc"][:100],
    }, t0


def compare(args):
    trials = []
    for path in (args.firmware, args.deploy):
        directory = path.expanduser().resolve()
        trial = read_bag(directory)
        trial["directory"] = directory
        trials.append(trial)
    summaries, onsets = zip(
        *(summarize(trial, args.onset_joint, args.threshold) for trial in trials)
    )
    print("Trial comparison (receipt timestamps; joint angles in rad)")
    for summary, trial in zip(summaries, trials):
        print(
            f"{summary['label']}: {summary['feedback_count']} feedback, "
            f"{summary['command_count']} /joint_ctrl commands, "
            f"{summary['unverified_bridge_command_count']} unverified bridge command samples; "
            f"onset {summary['onset']} "
            f"({summary['onset_delay_s']:.2f}s after recording began)"
        )
        print(
            f"  IMU rpy at onset: {summary['onset_rpy']}; "
            f"at +2s: {summary['rpy_at_2s']}"
        )
        print(
            f"  max command gap ms: {summary['command_max_gap_ms']}; "
            f"arm stall seconds: {summary['stall_seconds']}"
        )
        print(
            f"  first firmware fault: {summary['first_fault']}; "
            f"min battery voltage sample: {summary['battery_min_voltage']}"
        )
        print(
            f"  mode changes: {summary['mode_changes']}; "
            f"fall state changes: {summary['fall_changes']}"
        )
        print(
            f"  RPC requests: "
            f"{[(fmt_time(r['t']), r['api_id']) for r in trial['rpc'] if r['topic'].endswith('Req')]}; "
            f"all-zero arm feedback: {summary['first_all_zero_arm_feedback']}; "
            f"commands after first fault: {summary['post_fault_commands']}"
        )
        if summary["onset_delay_s"] < 1:
            print("  WARNING: less than 1 s was recorded before motion onset")
    if summaries[0]["command_count"]:
        print("WARNING: firmware trial had external /joint_ctrl commands")
    if not summaries[1]["command_count"]:
        print("WARNING: deploy trial had no /joint_ctrl commands")
    initial_rpy = [
        a - b for a, b in zip(
            summaries[0]["first_feedback_rpy"], summaries[1]["first_feedback_rpy"]
        ) if a is not None and b is not None
    ]
    print(f"Initial IMU rpy difference (firmware - deploy): {initial_rpy}")
    print("\nAt +2.0 s from detected arm movement:")
    print(" joint                    firmware q  deploy q  deploy published q")
    target_time = onsets[1] + 2_000_000_000
    command = latest_before(trials[1]["commands"], target_time)
    for joint in JOINTS:
        native = nearest(trials[0]["feedback"], onsets[0] + 2_000_000_000)
        deploy = nearest(trials[1]["feedback"], target_time)
        q0 = native["serial"]["q"][joint]
        q1 = deploy["serial"]["q"][joint]
        command_q = (
            command["q"][joint] if command and len(command["q"]) > joint else None
        )
        print(
            f" {joint:2d} {JOINT_NAMES[joint]:23s} "
            f"{q0:+9.3f} {q1:+9.3f} "
            f"{command_q:+9.3f}" if command_q is not None else
            f" {joint:2d} {JOINT_NAMES[joint]:23s} {q0:+9.3f} {q1:+9.3f}       n/a"
        )
    print(
        "\nInterpretation limit: no verified actuator-side final target or foot "
        "contact measurement is exposed by these topics. Use synchronized "
        "video/contact instrumentation for the latter."
    )
    report = {
        "firmware": summaries[0], "deploy": summaries[1],
        "alignment": {
            "joint": args.onset_joint, "threshold_rad": args.threshold,
            "method": "first measured deviation from first 0.5s baseline",
        },
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
        print(f"Report: {args.output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="mode", required=True)
    rec = subs.add_parser("record", help="Record one read-only trial")
    rec.add_argument("--trial", choices=("firmware", "deploy"), required=True)
    rec.add_argument("--output", type=Path, required=True, help="New trial directory")
    rec.add_argument("--seconds", type=float, default=0,
                     help="0 records until Ctrl+C; otherwise stop after N seconds")
    rec.add_argument("--syslog", type=Path, default=Path("/var/log/syslog"))
    comp = subs.add_parser("compare", help="Compare two recorded trial directories")
    comp.add_argument("firmware", type=Path)
    comp.add_argument("deploy", type=Path)
    comp.add_argument("--onset-joint", type=int, default=5)
    comp.add_argument("--threshold", type=float, default=0.1)
    comp.add_argument("--output", type=Path, help="Write a new JSON report")
    exp = subs.add_parser("export", help="Export per-frame target and feedback JSONL")
    exp.add_argument("trial_dir", type=Path)
    exp.add_argument("--output", type=Path, help="New JSONL path")
    exp.add_argument("--max-command-age-ms", type=float, default=200.0)
    args = parser.parse_args()
    if args.mode == "record":
        if not math.isfinite(args.seconds) or args.seconds < 0:
            parser.error("--seconds must be finite and nonnegative")
        record(args)
    elif args.mode == "compare":
        if args.onset_joint < 0 or args.onset_joint >= 22:
            parser.error("--onset-joint must be in 0..21")
        if not math.isfinite(args.threshold) or args.threshold <= 0:
            parser.error("--threshold must be finite and positive")
        compare(args)
    else:
        if not math.isfinite(args.max_command_age_ms) or args.max_command_age_ms <= 0:
            parser.error("--max-command-age-ms must be finite and positive")
        path, commands, feedback = export_trial(
            args.trial_dir, args.output, args.max_command_age_ms
        )
        print(f"Saved {feedback} feedback and {commands} commands to {path}")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
