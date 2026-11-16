#!/usr/bin/env bash
# 标定脚本共享的路径、ROS 环境和工作空间加载函数。
# 用法：在各标定入口完成 --help 处理后 source 本文件。

####################### 参数区 #######################

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
CAMERA_WS="${CAMERA_WS:-${ROOT_DIR}/camera_ws}"
KALIBR_WS="${KALIBR_WS:-${ROOT_DIR}/third_party/kalibr}"
ALLAN_WS="${ALLAN_WS:-${ROOT_DIR}/third_party/allan}"
CALIB_DIR="${ROOT_DIR}/configs"
TARGET="${TARGET:-${ROOT_DIR}/targets/aprilgrid_6x6_25mm_spacing03.yaml}"
CAMERA_PACKAGE="${CAMERA_PACKAGE:-camera_driver_ov2710}"
CAMERA_LAUNCH="${CAMERA_LAUNCH:-parallel_mjpeg_camera.launch}"
CAMERA_PARAM_FILE="${CAMERA_PARAM_FILE:-}"  # 留空使用 launch 默认配置。
CAM_TOPIC="${CAM_TOPIC:-/camera/image_raw}"
IMU_TOPIC="${IMU_TOPIC:-/mavros/imu/data}"
FCU_URL="${FCU_URL:-/dev/ttyTHS0:2000000}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${ROOT_DIR}/outputs}"

####################### 环境变量区 #######################

# 本文件不额外导出环境变量；各路径可由调用者预先设置同名变量覆盖。

####################### 通用函数区 #######################

load_camera_workspace() {
    if [ ! -f "${CAMERA_WS}/devel/setup.bash" ]; then
        echo "Build camera_ws first; see README.md (or set CAMERA_WS)." >&2
        return 1
    fi
    source "${CAMERA_WS}/devel/setup.bash"
}
load_kalibr_workspace() {
    if [ ! -f "${KALIBR_WS}/devel/setup.bash" ]; then
        echo "Build Kalibr first; see README.md (or set KALIBR_WS)." >&2
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

####################### 启动区 #######################

mkdir -p "$OUTPUT_ROOT"
OUTPUT_ROOT="$(realpath "$OUTPUT_ROOT")"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS environment missing: $ROS_SETUP" >&2
    return 1
fi
source "$ROS_SETUP"
