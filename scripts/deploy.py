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
parser.add_argument(
    "--vision-config", default="/opt/booster",
    help="Directory containing the robot's vision.yaml and optional vision_local.yaml",
)
parser.add_argument("--head-only", action="store_true",
                    help="k1_pass: remain in zero-command loco preparation and test head tracking; A/r does not enable pass")
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

    # Set device for policy
    task_cfg.policy.device = args.device
    if args.exit_mode is not None:
        task_cfg.booster.exit_mode = args.exit_mode
    if args.head_only and (args.task != "k1_pass" or args.mujoco):
        parser.error("--head-only requires real-robot --task k1_pass")
    task_cfg.booster.head_only = args.head_only
    if args.no_head_tracking:
        task_cfg.booster.head_tracking.enabled = False
    if args.task == "k1_pass" and not args.mujoco:
        from booster_deploy.utils.vision_config import load_vision_config, camera_topics
        config = load_vision_config(args.vision_config)
        head = task_cfg.booster.head_tracking
        head.fx = float(config["camera"]["intrin"]["fx"])
        head.fy = float(config["camera"]["intrin"]["fy"])
        head.color_topic = camera_topics(config)[0]
        if args.color_topic:
            head.color_topic = args.color_topic
        if args.head_only:
            print("HEAD TEST: X starts zero-command loco and head tracking. A/r will NOT start pass.")

    # decide how to run based on flags
    if args.mujoco:
        # run mujoco controller
        from booster_deploy.controllers.mujoco_controller import MujocoController

        MujocoController(task_cfg).run()
    else:
        from booster_deploy.controllers.booster_robot_controller import BoosterRobotPortal
        with BoosterRobotPortal(task_cfg) as portal:
            portal.run()


if __name__ == "__main__":
    main()
