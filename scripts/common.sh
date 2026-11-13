#!/usr/bin/env bash
# Shared paths. Source this after parsing --help.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
CAMERA_WS="${CAMERA_WS:-${ROOT_DIR}/camera_ws}"
KALIBR_WS="${KALIBR_WS:-${ROOT_DIR}/third_party/kalibr}"
ALLAN_WS="${ALLAN_WS:-${ROOT_DIR}/third_party/allan}"
CALIB_DIR="${ROOT_DIR}/configs"
TARGET="${TARGET:-${ROOT_DIR}/targets/aprilgrid_6x6_25mm_spacing03.yaml}"
CAMERA_PARAM_FILE="${CAMERA_PARAM_FILE:-${CALIB_DIR}/camera.yaml}"
CAM_TOPIC="${CAM_TOPIC:-/camera/image_raw}"
IMU_TOPIC="${IMU_TOPIC:-/mavros/imu/data}"
FCU_URL="${FCU_URL:-/dev/ttyTHS0:2000000}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${ROOT_DIR}/outputs}"
mkdir -p "$OUTPUT_ROOT"
OUTPUT_ROOT="$(realpath "$OUTPUT_ROOT")"
if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS environment missing: $ROS_SETUP" >&2
    return 1
fi
source "$ROS_SETUP"
load_camera_workspace() {
    if [ ! -f "${CAMERA_WS}/devel/setup.bash" ]; then
        echo "Build camera_ws first; see README.md (or set CAMERA_WS)." >&2
        return 1
    fi
    source "${CAMERA_WS}/devel/setup.bash"
}
load_kalibr_workspace() {
    if [ ! -f "${KALIBR_WS}/devel/setup.bash" ]; then
        echo "Build Kalibr first; see docs/dependencies.md (or set KALIBR_WS)." >&2
        return 1
    fi
    source "${KALIBR_WS}/devel/setup.bash"
}
wait_for_bag() {
    local pid="$1"
    while kill -0 "$pid" 2>/dev/null; do
        wait "$pid" 2>/dev/null || true
    done
    wait "$pid" 2>/dev/null || true
}
