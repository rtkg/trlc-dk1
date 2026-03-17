"""Single-arm teleoperation with impedance controller (MIT mode + gravity compensation)."""

import argparse
import json
import time
import logging

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from lerobot_robot_trlc_dk1.follower import DK1Follower, DK1FollowerConfig
from lerobot_robot_trlc_dk1.leader import DK1Leader, DK1LeaderConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def main():
    parser = argparse.ArgumentParser(description="Single-arm impedance teleop")
    parser.add_argument("--follower-port", default="/dev/ttyACM0")
    parser.add_argument("--leader-port", default="/dev/ttyACM2")
    parser.add_argument("--fps", type=int, default=200)
    parser.add_argument("--arm-kp", type=float, nargs=6, default=None,
                        help="MIT Kp gains per joint [j1..j6]")
    parser.add_argument("--arm-kd", type=float, nargs=6, default=None,
                        help="MIT Kd gains per joint [j1..j6]")
    parser.add_argument("--camera", type=str, default=None,
                        help="JSON string of camera configs, e.g. '{\"right_wrist\":{...}}'")
    parser.add_argument("--display_data", action="store_true")
    parser.add_argument("--display_url", default=None)
    parser.add_argument("--display_port", type=int, default=9876)
    args = parser.parse_args()

    init_logging()

    cameras = {}
    if args.camera:
        for name, cfg in json.loads(args.camera).items():
            cfg.pop("type", None)
            cameras[name] = OpenCVCameraConfig(**cfg)

    follower_config = DK1FollowerConfig(
        port=args.follower_port,
        control_mode="impedance",
        arm_kp=args.arm_kp,
        arm_kd=args.arm_kd,
        cameras=cameras,
    )

    leader_config = DK1LeaderConfig(
        port=args.leader_port,
    )

    leader = DK1Leader(leader_config)
    leader.connect()

    follower = DK1Follower(follower_config)
    follower.connect()

    if args.display_data:
        init_rerun(session_name="teleop", ip=args.display_url, port=args.display_port)

    logger.info(f"Teleop running at {args.fps} Hz. Press Ctrl+C to stop.")

    try:
        while True:
            action = leader.get_action()
            follower.send_action(action)
            if args.display_data:
                obs = follower.get_observation()
                log_rerun_data(observation=obs, action=action)
            time.sleep(1 / args.fps)
    except KeyboardInterrupt:
        print("\nStopping teleop...")
    finally:
        leader.disconnect()
        follower.disconnect()


if __name__ == "__main__":
    main()
