"""
Move robot to a target joint configuration via minimum jerk trajectory.

Usage:
    python examples/move_to.py --port /dev/ttyACM1 --q 0.0 0.0 0.0 0.0 0.0 0.0
"""
import argparse
import json
import time

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot_robot_trlc_dk1.follower import DK1Follower, DK1FollowerConfig

JOINT_KEYS = [f"joint_{i}.pos" for i in range(1, 7)]


def minimum_jerk(t, T):
    """Minimum jerk trajectory scalar: returns s in [0, 1]."""
    tau = min(t / T, 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def main():
    parser = argparse.ArgumentParser(description="Move to joint configuration via minimum jerk trajectory")
    parser.add_argument("--port", required=True)
    parser.add_argument("--duration", type=float, default=3.0,
                        help="Trajectory duration in seconds")
    parser.add_argument("--q", type=float, nargs=6,
                        default=[0.0, 1.46, 1.36, -1.43, 0.43, 0.0],
                        help="Target joint positions (6 floats, rad)")
    parser.add_argument("--gripper", type=float, default=None,
                        help="Target gripper position (0-1 normalized)")
    parser.add_argument("--arm_kp", type=float, nargs=6, default=None,
                        help="MIT Kp gains for joints 1-6")
    parser.add_argument("--arm_kd", type=float, nargs=6, default=None,
                        help="MIT Kd gains for joints 1-6")
    parser.add_argument("--camera", type=str, default=None,
                        help="JSON string of camera configs, e.g. '{\"right_wrist\":{...}}'")
    parser.add_argument("--display_data", action="store_true")
    parser.add_argument("--display_url", default=None)
    parser.add_argument("--display_port", type=int, default=9876)
    args = parser.parse_args()

    cameras = {}
    if args.camera:
        for name, cfg in json.loads(args.camera).items():
            cfg.pop("type", None)
            cameras[name] = OpenCVCameraConfig(**cfg)

    config = DK1FollowerConfig(
        port=args.port,
        control_mode="impedance",
        disable_torque_on_disconnect=True,
        arm_kp=args.arm_kp,
        arm_kd=args.arm_kd,
        cameras=cameras,
    )
    follower = DK1Follower(config)
    follower.connect()

    if args.display_data:
        init_rerun(session_name="move_to", ip=args.display_url, port=args.display_port)

    # Read initial positions as baseline
    obs = follower.get_observation()
    baseline = {k: v for k, v in obs.items() if k.endswith(".pos")}
    print(f"Start positions: { {k: f'{v:.3f}' for k, v in baseline.items()} }")

    # Build target
    target = dict(baseline)
    for i, key in enumerate(JOINT_KEYS):
        target[key] = args.q[i]
    if args.gripper is not None:
        target["gripper.pos"] = args.gripper

    print(f"Target positions: { {k: f'{v:.3f}' for k, v in target.items() if k in JOINT_KEYS or k == 'gripper.pos'} }")

    freq = 200  # control loop Hz
    T = args.duration

    t0 = time.time()
    try:
        # Trajectory phase
        while (t := time.time() - t0) < T:
            s = minimum_jerk(t, T)
            action = {}
            for key in baseline:
                action[key] = baseline[key] + s * (target.get(key, baseline[key]) - baseline[key])
            follower.send_action(action)
            if args.display_data:
                obs = follower.get_observation()
                log_rerun_data(observation=obs, action=action)
            time.sleep(1 / freq)

        print("Trajectory complete. Holding final position (Ctrl+C to stop)...")

        # Hold phase
        while True:
            follower.send_action(target)
            if args.display_data:
                obs = follower.get_observation()
                log_rerun_data(observation=obs, action=target)
            time.sleep(1 / freq)
    except KeyboardInterrupt:
        print("\nStopping...")
        follower.disconnect()


if __name__ == "__main__":
    main()
