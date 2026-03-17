#!/bin/bash
# Single-arm teleoperation with impedance controller
# Uses MIT mode + gravity compensation on one follower arm
#
# Usage: ./run_teleop.sh [--arm-kp j1 j2 j3 j4 j5 j6] [--arm-kd j1 j2 j3 j4 j5 j6]
#
# Default ports are for the LEFT arm (override with env vars):
#   FOLLOWER_PORT = /dev/ttyACM0
#   LEADER_PORT   = /dev/ttyACM2

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec taskset -c 8-15 chrt -f 85 uv run python "${SCRIPT_DIR}/examples/teleop_impedance.py" \
    --follower-port "${FOLLOWER_PORT:-/dev/ttyACM0}" \
    --leader-port "${LEADER_PORT:-/dev/ttyACM2}" \
    "$@"
