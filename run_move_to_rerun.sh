#!/bin/bash
ARM_KP="80.0 70.0 60.0 20.0 20.0 10.0"
ARM_KD="5.0 5.0 4.0 1.0 1.0 1.0"

while [[ $# -gt 0 ]]; do
    case $1 in
        --arm_kp) ARM_KP="$2 $3 $4 $5 $6 $7"; shift 7 ;;
        --arm_kd) ARM_KD="$2 $3 $4 $5 $6 $7"; shift 7 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

if [ -z "${CAMERA_JSON+x}" ]; then
    CAMERA_JSON='{"right_wrist":{"type":"opencv","index_or_path":"/dev/video0","width":640,"height":360,"fps":60,"rotation":180,"fourcc":"MJPG"}}'
fi

RERUN_FLUSH_NUM_BYTES=1000 uv run python examples/move_to.py \
    --port /dev/ttyACM1 \
    --duration 3.0 \
    --q 0.0 1.46 1.36 -1.43 0.0 0.0 \
    --arm_kp $ARM_KP \
    --arm_kd $ARM_KD \
    --camera "${CAMERA_JSON}" \
    --display_data \
    --display_url 10.11.10.36 \
    --display_port 9876
