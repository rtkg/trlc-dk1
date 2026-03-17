#!/bin/bash
# Single-arm teleoperation with impedance controller + rerun logging
#
# Usage: ./run_teleop_rerun.sh [--arm-kp j1 j2 j3 j4 j5 j6] [--arm-kd j1 j2 j3 j4 j5 j6]
#
# Default ports are for the LEFT arm (override with env vars):
#   FOLLOWER_PORT = /dev/ttyACM0
#   LEADER_PORT   = /dev/ttyACM2

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${CAMERA_JSON+x}" ]; then
    CAMERA_JSON='{"right_wrist":{"type":"opencv","index_or_path":"/dev/video0","width":640,"height":360,"fps":60,"rotation":180,"fourcc":"MJPG"}}'
fi

exec env RERUN_FLUSH_NUM_BYTES=1000 \
    taskset -c 8-15 chrt -f 85 uv run python "${SCRIPT_DIR}/examples/teleop_impedance.py" \
    --follower-port "${FOLLOWER_PORT:-/dev/ttyACM0}" \
    --leader-port "${LEADER_PORT:-/dev/ttyACM2}" \
    --camera "${CAMERA_JSON}" \
    --display_data \
    --display_url "${DISPLAY_URL:-10.11.10.36}" \
    --display_port "${DISPLAY_PORT:-9876}" \
    "$@"
