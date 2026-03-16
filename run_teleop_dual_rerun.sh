#!/bin/bash
# Dual-arm teleoperation with impedance controller + rerun logging
#
# Usage: ./run_teleop_dual_rerun.sh [--rerun-ip IP] [--rerun-port PORT]
#
# Default ports (override with env vars):
#   LEFT_FOLLOWER_PORT  = /dev/ttyACM0
#   RIGHT_FOLLOWER_PORT = /dev/ttyACM1
#   LEFT_LEADER_PORT    = /dev/ttyACM2
#   RIGHT_LEADER_PORT   = /dev/ttyACM3
#   RERUN_IP            = 10.11.10.36 (laptop IP for rerun viewer)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RERUN_FLUSH_NUM_BYTES=1000 exec taskset -c 8-15 chrt -f 85 uv run python "${SCRIPT_DIR}/examples/bi_teleop_impedance_rerun.py" \
    --left-follower-port "${LEFT_FOLLOWER_PORT:-/dev/ttyACM0}" \
    --right-follower-port "${RIGHT_FOLLOWER_PORT:-/dev/ttyACM1}" \
    --left-leader-port "${LEFT_LEADER_PORT:-/dev/ttyACM2}" \
    --right-leader-port "${RIGHT_LEADER_PORT:-/dev/ttyACM3}" \
    --rerun-ip "${RERUN_IP:-10.11.10.36}" \
    "$@"
