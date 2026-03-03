#!/bin/bash
RERUN_FLUSH_NUM_BYTES=1000 taskset -c 8-15 chrt -f 85 uv run lerobot-teleoperate \
    --fps=200 \
    --robot.type=dk1_follower \
    --robot.controller_type=${1:-joint_impedance} \
    --robot.joint_velocity_scaling=${2:-1.0} \
    --robot.port=/dev/ttyACM0 \
    --robot.urdf_path=/home/rtkg/Coding/trlc-dk1-follower-urdf/TRLC-DK1-Follower.urdf \
    --robot.controller_config_path=config/impedance_config.yaml \
    --robot.disable_torque_on_disconnect=true \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/ttyACM2 \
    --robot.cameras='{"left_wrist":{"type":"opencv","index_or_path":"/dev/video2","width":1280,"height":720,"fps":60,"rotation":180,"fourcc":"MJPG"}}' \
    --display_data=true \
    --display_url=10.11.10.36 \
    --display_port=9876

