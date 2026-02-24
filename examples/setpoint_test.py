"""
Standalone setpoint test — actuate a single joint with a sinusoidal trajectory.

Usage (joint_impedance, default):
    python examples/setpoint_test.py --port /dev/ttyACM1 --urdf_path /path/to/URDF --joint joint_3

Usage (pos_vel):
    python examples/setpoint_test.py --port /dev/ttyACM1 --controller_type pos_vel --joint joint_3

All other joints hold their startup position.
"""
import argparse
import math
import time

from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot_robot_trlc_dk1.follower import DK1Follower, DK1FollowerConfig


def main():
    parser = argparse.ArgumentParser(description="Single-joint setpoint test")
    parser.add_argument("--port", required=True)
    parser.add_argument("--joint", default="joint_6",
                        choices=["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","gripper"])
    parser.add_argument("--q_0", type=float, default=None,
                        help="Servo joint to this position (rad) before oscillating")
    parser.add_argument("--amplitude", type=float, default=0.3,
                        help="Sinusoid amplitude in radians (gripper: 0-1 normalized)")
    parser.add_argument("--frequency", type=float, default=0.5,
                        help="Sinusoid frequency in Hz")
    parser.add_argument("--controller_type", default="joint_impedance",
                        choices=["pos_vel", "joint_impedance"])
    parser.add_argument("--urdf_path", default=None,
                        help="Path to URDF (required for joint_impedance)")
    parser.add_argument("--controller_config_path", default="config/impedance_config.yaml")
    parser.add_argument("--velocity_scaling", type=float, default=0.2,
                        help="Only used in pos_vel mode")
    parser.add_argument("--display_data", action="store_true")
    parser.add_argument("--display_url", default=None)
    parser.add_argument("--display_port", type=int, default=9876)
    args = parser.parse_args()

    config = DK1FollowerConfig(
        port=args.port,
        controller_type=args.controller_type,
        joint_velocity_scaling=args.velocity_scaling,
        disable_torque_on_disconnect=True,
        urdf_path=args.urdf_path,
        controller_config_path=args.controller_config_path,
    )
    follower = DK1Follower(config)
    follower.connect()

    if args.display_data:
        init_rerun(session_name="setpoint_test", url=args.display_url, port=args.display_port)

    # Read initial positions as baseline
    obs = follower.get_observation()
    baseline = {k: v for k, v in obs.items() if k.endswith(".pos")}
    print(f"Baseline positions: { {k: f'{v:.3f}' for k, v in baseline.items()} }")

    freq = 200  # control loop Hz

    # Servo to q_0 if specified
    if args.q_0 is not None:
        joint_key = f"{args.joint}.pos"
        q_start = baseline[joint_key]
        servo_duration = 2.0
        print(f"Servoing {args.joint} from {q_start:.3f} to {args.q_0:.3f} rad...")
        t0 = time.time()
        while (t := time.time() - t0) < servo_duration:
            alpha = t / servo_duration
            action = dict(baseline)
            action[joint_key] = q_start + alpha * (args.q_0 - q_start)
            follower.send_action(action)
            if args.display_data:
                obs = follower.get_observation()
                log_rerun_data(observation=obs, action=action)
            time.sleep(1 / freq)
        baseline[joint_key] = args.q_0
        print("Servo complete. Starting oscillation.")

    t0 = time.time()
    pause_total = 0.0  # accumulated pause time (excluded from oscillation phase)
    prev_sign = 0

    try:
        while True:
            t = time.time() - t0 - pause_total
            action = dict(baseline)  # copy baseline

            sine_val = math.sin(2 * math.pi * args.frequency * t)
            curr_sign = 1 if sine_val >= 0 else -1

            # Pause for 1s at each zero crossing (skip the initial t≈0)
            if prev_sign != 0 and curr_sign != prev_sign:
                print(f"Zero crossing at t_osc={t:.2f}s — holding baseline for 3s")
                pause_start = time.time()
                while time.time() - pause_start < 3.0:
                    follower.send_action(dict(baseline))
                    if args.display_data:
                        obs = follower.get_observation()
                        log_rerun_data(observation=obs, action=dict(baseline))
                    time.sleep(1 / freq)
                pause_total += time.time() - pause_start

            prev_sign = curr_sign

            offset = args.amplitude * sine_val
            action[f"{args.joint}.pos"] = baseline[f"{args.joint}.pos"] + offset

            follower.send_action(action)

            if args.display_data:
                obs = follower.get_observation()
                log_rerun_data(observation=obs, action=action)

            time.sleep(1 / freq)
    except KeyboardInterrupt:
        print("\nStopping setpoint test...")
        follower.disconnect()


if __name__ == "__main__":
    main()
