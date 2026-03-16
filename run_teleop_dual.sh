#!/bin/bash
taskset -c 8-15 chrt -f 85 uv run lerobot-teleoperate \
    --fps=200 \
    --robot.type=bi_dk1_follower \
    --robot.controller_type=${1:-joint_impedance} \
    --robot.joint_velocity_scaling=${2:-1.0} \
    --robot.left_arm_port=/dev/ttyACM0 \
    --robot.right_arm_port=/dev/ttyACM1 \
    --robot.urdf_path=/home/rtkg/Coding/trlc-dk1-follower-urdf/TRLC-DK1-Follower.urdf \
    --robot.controller_config_path=config/impedance_config.yaml \
    --robot.disable_torque_on_disconnect=true \
    --teleop.type=bi_dk1_leader \
    --teleop.left_arm_port=/dev/ttyACM2 \
    --teleop.right_arm_port=/dev/ttyACM3 \
    --robot.cameras='{"left_wrist":{"type":"opencv","index_or_path":"/dev/video2","width":1280,"height":720,"fps":60,"rotation":180,"fourcc":"MJPG"},"right_wrist":{"type":"opencv","index_or_path":"/dev/video0","width":1280,"height":720,"fps":60,"rotation":180,"fourcc":"MJPG"}}' \
    --display_data=false
