#!/bin/bash
# Dual-arm teleoperation with impedance controller
# Uses MIT mode + gravity compensation on both follower arms
#
# Usage: ./run_teleop_dual.sh
#
# Default ports (override with env vars):
#   LEFT_FOLLOWER_PORT  = /dev/ttyACM0
#   RIGHT_FOLLOWER_PORT = /dev/ttyACM1
#   LEFT_LEADER_PORT    = /dev/ttyACM2
#   RIGHT_LEADER_PORT   = /dev/ttyACM3

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec taskset -c 8-15 chrt -f 85 uv run python "${SCRIPT_DIR}/examples/bi_teleop_impedance.py" \
    --left-follower-port "${LEFT_FOLLOWER_PORT:-/dev/ttyACM0}" \
    --right-follower-port "${RIGHT_FOLLOWER_PORT:-/dev/ttyACM1}" \
    --left-leader-port "${LEFT_LEADER_PORT:-/dev/ttyACM2}" \
    --right-leader-port "${RIGHT_LEADER_PORT:-/dev/ttyACM3}" \
    "$@"
