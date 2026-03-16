"""Bimanual teleoperation with impedance controller and rerun logging."""

import argparse
import time
import logging

from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from lerobot_robot_trlc_dk1.bi_follower import BiDK1Follower, BiDK1FollowerConfig
from lerobot_robot_trlc_dk1.bi_leader import BiDK1Leader, BiDK1LeaderConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def main():
    parser = argparse.ArgumentParser(description="Dual-arm impedance teleop with rerun logging")
    parser.add_argument("--left-follower-port", default="/dev/ttyACM0")
    parser.add_argument("--right-follower-port", default="/dev/ttyACM1")
    parser.add_argument("--left-leader-port", default="/dev/ttyACM2")
    parser.add_argument("--right-leader-port", default="/dev/ttyACM3")
    parser.add_argument("--fps", type=int, default=200)
    parser.add_argument("--rerun-ip", default="127.0.0.1")
    parser.add_argument("--rerun-port", type=int, default=9876)
    args = parser.parse_args()

    init_logging()
    init_rerun(session_name="bi_teleop_impedance", ip=args.rerun_ip, port=args.rerun_port)

    follower_config = BiDK1FollowerConfig(
        left_arm_port=args.left_follower_port,
        right_arm_port=args.right_follower_port,
    )

    leader_config = BiDK1LeaderConfig(
        left_arm_port=args.left_leader_port,
        right_arm_port=args.right_leader_port,
    )

    leader = BiDK1Leader(leader_config)
    leader.connect()

    follower = BiDK1Follower(follower_config)
    follower.connect()

    logger.info(f"Teleop running at {args.fps} Hz with rerun logging. Press Ctrl+C to stop.")

    try:
        while True:
            action = leader.get_action()
            follower.send_action(action)
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
