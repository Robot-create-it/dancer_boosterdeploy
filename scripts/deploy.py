import argparse
import sys

sys.path.append(".")

parser = argparse.ArgumentParser()
# require either --task or --list (mutually exclusive)
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--task", type=str, help="Name of the configuration file.")
group.add_argument("-l", "--list", action="store_true", dest="list_tasks",
                   default=False, help="list available tasks")

parser.add_argument("--mujoco", action="store_true", default=False,
                    help="deploy in mujoco simulation")
parser.add_argument("--recovery-posture", choices=("faceup", "facedown"),
                    help="k1_recovery --mujoco: initial lying posture (default: facedown)")
parser.add_argument("--recovery-log", metavar="JSONL",
                    help="k1_recovery: record each policy proposal and measured state; file must be new")
parser.add_argument("--recovery-hold-only", action="store_true",
                    help="k1_recovery hardware: X starts guarded pose hold; A/r never starts getup")
parser.add_argument("--ball-pos", type=float, nargs=2, metavar=("X", "Y"),
                    help="k1_pass/k1_shoot --mujoco: initial ball centre in world XY (metres)")
parser.add_argument("--shoot-policy", choices=("2", "264", "192", "0109_0", "0109_2"),
                    help="k1_shoot: choose the fixed shoot model by filename suffix (default: 2)")
parser.add_argument(
    "--vision-config", default="/opt/booster",
    help="Directory containing the robot's vision.yaml and optional vision_local.yaml",
)
parser.add_argument("--head-only", action="store_true",
                    help="K1 visual kick: remain in zero-command loco preparation and test head tracking")
parser.add_argument("--color-topic", default=None, help="Override pass camera image topic")
parser.add_argument("--no-head-tracking", action="store_true",
                    help="Disable automatic head targets for diagnosis")
parser.add_argument(
    "--device", type=str, default="cpu",
    help="Device to run the evaluation on (e.g., 'cpu', 'cuda')")
parser.add_argument(
    "--exit-mode",
    choices=("walking", "damping"),
    default=None,
    help="Robot mode to enter after controller exit (default: task config, walking)",
)
args = parser.parse_args()


def main():
    if args.recovery_hold_only and (args.task != "k1_recovery" or args.mujoco):
        parser.error('--recovery-hold-only requires hardware --task k1_recovery')
    if args.recovery_hold_only and args.recovery_log is not None:
        parser.error('--recovery-hold-only does not run a policy; record /low_state and /joint_ctrl with ros2 bag')
    if args.task == 'k1_recovery' and not args.mujoco and args.exit_mode not in (None, 'damping'):
        parser.error('Hardware recovery must exit to damping')
    if args.recovery_log is not None and args.task != "k1_recovery":
        parser.error("--recovery-log requires --task k1_recovery")
    if args.recovery_posture is not None and (args.task != "k1_recovery" or not args.mujoco):
        parser.error("--recovery-posture requires --task k1_recovery --mujoco")
    if args.shoot_policy is not None and args.task != "k1_shoot":
        parser.error("--shoot-policy requires --task k1_shoot")
    if not (args.list_tasks or args.mujoco):
        from booster_deploy.utils.robot_runtime import require_robot_interface
        try:
            require_robot_interface()
        except RuntimeError as exc:
            parser.exit(1, f"ERROR: {exc}\n")
    # load task registry and dispatch
    import pkgutil
    import tasks as tasks_pkg

    # auto-import all submodules under tasks (recursive) so they can register themselves
    for mod_info in pkgutil.walk_packages(tasks_pkg.__path__, prefix="tasks."):
        full_name = mod_info.name
        try:
            __import__(full_name)
        except Exception as e:
            raise e
    from booster_deploy.utils.registry import get_task, list_tasks

    if args.list_tasks:
        print("Available tasks:")
        for task_name, cfg in list_tasks().items():
            cls = type(cfg)
            full_cls = f"{cls.__module__}.{cls.__qualname__}"
            print(f"  {task_name}\t:\t{full_cls}")
        sys.exit(0)

    try:
        task_cfg = get_task(args.task)
    except KeyError:
        print(f"Unknown task '{args.task}'. Available tasks: {list(list_tasks().keys())}")
        sys.exit(1)

    if args.task == "k1_shoot":
        from tasks.locomotion.robots.k1.shooting import select_shoot_policy
        selected = args.shoot_policy or "2"
        select_shoot_policy(task_cfg, selected)
        print(f"K1 shoot policy: {selected} (power 6)", flush=True)

    # Set device for policy
    if args.task == "k1_recovery":
        task_cfg.booster.recovery_hold_only = args.recovery_hold_only
        task_cfg.policy.trace_path = args.recovery_log
        if not args.mujoco:
            # Hardware faults are not reliably represented by ROS reserve[0]
            # on this robot. Do not blindly retry a failed getup.
            task_cfg.policy.max_retries = 0
        if args.recovery_log is not None:
            from pathlib import Path
            trace_path = Path(args.recovery_log)
            if trace_path.exists():
                parser.error("--recovery-log must name a new file")
            trace_path.parent.mkdir(parents=True, exist_ok=True)
    if args.task == "k1_recovery" and args.mujoco and args.recovery_posture == "faceup":
        task_cfg.mujoco.init_quat[2] *= -1
    task_cfg.policy.device = args.device
    if args.exit_mode is not None:
        task_cfg.booster.exit_mode = args.exit_mode
    if args.head_only and (args.task not in ("k1_pass", "k1_shoot") or args.mujoco):
        parser.error("--head-only requires real-robot --task k1_pass or k1_shoot")
    if args.ball_pos is not None:
        import math
        if args.task not in ("k1_pass", "k1_shoot") or not args.mujoco or not all(map(math.isfinite, args.ball_pos)):
            parser.error("--ball-pos requires k1_pass/k1_shoot --mujoco and two finite coordinates")
        task_cfg.mujoco.ball_init_xy = args.ball_pos
    task_cfg.booster.head_only = args.head_only
    if args.no_head_tracking:
        task_cfg.booster.head_tracking.enabled = False
    if args.task in ("k1_pass", "k1_shoot") and not args.mujoco:
        from booster_deploy.utils.vision_config import load_vision_config, camera_topics
        config = load_vision_config(args.vision_config)
        head = task_cfg.booster.head_tracking
        head.fx = float(config["camera"]["intrin"]["fx"])
        head.fy = float(config["camera"]["intrin"]["fy"])
        head.color_topic = camera_topics(config)[0]
        if args.color_topic:
            head.color_topic = args.color_topic
        if args.head_only:
            print("HEAD TEST: X starts zero-command loco and head tracking. A/r will NOT start the kick policy.")

    # decide how to run based on flags
    if args.mujoco:
        # run mujoco controller
        if args.task == "k1_pass":
            from booster_deploy.controllers.k1_pass_mujoco_controller import K1PassMujocoController
            K1PassMujocoController(task_cfg).run()
        elif args.task == "k1_shoot":
            from booster_deploy.controllers.k1_shoot_mujoco_controller import K1ShootMujocoController
            K1ShootMujocoController(task_cfg).run()
        else:
            from booster_deploy.controllers.mujoco_controller import MujocoController
            MujocoController(task_cfg).run()
    else:
        from booster_deploy.controllers.booster_robot_controller import BoosterRobotPortal
        with BoosterRobotPortal(task_cfg) as portal:
            portal.run()


if __name__ == "__main__":
    main()
