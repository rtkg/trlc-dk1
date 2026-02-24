#!/bin/bash
RERUN_FLUSH_NUM_BYTES=1000 uv run python examples/setpoint_test.py \
    --port /dev/ttyACM0 \
    --urdf_path /home/rtkg/Coding/trlc-dk1-follower-urdf/TRLC-DK1-Follower.urdf \
    --controller_config_path config/impedance_config.yaml \
    --joint joint_6 \
    --q_0 0.0 \
    --amplitude 0.3 \
    --frequency 0.5 \
    --display_data \
    --display_url 10.11.10.36 \
    --display_port 9876
