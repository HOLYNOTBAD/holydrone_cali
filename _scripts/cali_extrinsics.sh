#!/usr/bin/env bash
# 采集或复用相机与 IMU rosbag，并调用 Kalibr 标定外参与时间偏移。
# 用法：修改顶部参数后运行 bash _scripts/cali_extrinsics.sh

set -e

####################### 参数区 #######################

SH_DIR=$(cd "$(dirname "$0")" && pwd)
source "${SH_DIR}/common.sh"

CAMCHAIN=""  # 填写内参 camchain.yaml 的绝对路径。
IMU="${OUTPUT_ROOT}/imu/imu_mavros.yaml"
BAG=""  # 留空录制新包；填写绝对路径则离线分析。
BAG_FREQ=10  # 0 表示处理全部图像。
TIMEOFFSET_PADDING=0.15
OUTPUT_DIR="${OUTPUT_ROOT}/extrinsics"
KALIBR="${KALIBR_WS}/devel/lib/kalibr/kalibr_calibrate_imu_camera"
PREVIEW_TOPIC="/camera/calibration_preview"
CAMERA_ARGS=()
[ -z "$CAMERA_PARAM_FILE" ] || CAMERA_ARGS=(camera_params_file:="$CAMERA_PARAM_FILE")

####################### 环境变量区 #######################

# 本脚本不额外导出环境变量；工作空间环境由 common.sh 的加载函数提供。

####################### 通用函数区 #######################

stop_preview() {
    local pid
    for pid in "$PREVIEW_THROTTLE_PID" "$PREVIEW_WINDOW_PID"; do
        [ -n "$pid" ] || continue
        kill -TERM "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    PREVIEW_THROTTLE_PID=""
    PREVIEW_WINDOW_PID=""
}

cleanup_processes() {
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
    stop_preview
    [ -n "$CAMERA_PID" ] && kill -TERM "$CAMERA_PID" 2>/dev/null || true
    [ -n "$MAVROS_PID" ] && kill -TERM "$MAVROS_PID" 2>/dev/null || true
    [ -n "$ROSCORE_PID" ] && kill -TERM "$ROSCORE_PID" 2>/dev/null || true
}

stop_recording() {
    # 录够图像和 IMU 后按 Ctrl-C，只停止 rosbag，然后继续跑 Kalibr。
    echo
    echo "Stopping recording..."
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
    stop_preview
}

####################### 启动区 #######################

RECORD_PID=""
PREVIEW_THROTTLE_PID=""
PREVIEW_WINDOW_PID=""
ROSCORE_PID=""
MAVROS_PID=""
CAMERA_PID=""
trap cleanup_processes EXIT

mkdir -p "$OUTPUT_DIR"

if [ -z "$CAMCHAIN" ] || [ ! -f "$CAMCHAIN" ]; then
    echo "Camera intrinsics camchain not found: ${CAMCHAIN}" >&2
    echo "Set CAMCHAIN at the top of this script to the intrinsics YAML." >&2
    exit 1
fi
# Kalibr 会在输出目录中运行，因此保留输入文件的绝对路径。
CAMCHAIN=$(realpath "$CAMCHAIN")

IMU=$(realpath "$IMU")

if [ -n "$BAG" ]; then
    if [ ! -f "$BAG" ]; then
        echo "Calibration bag not found: ${BAG}" >&2
        exit 1
    fi
    BAG=$(realpath "$BAG")
    echo "Using specified extrinsics bag: ${BAG}"
else
    BAG="${OUTPUT_DIR}/camera_imu_extrinsics_$(date +%Y%m%d_%H%M%S).bag"

    # 先进入本工程 ROS 环境。
    source "$ROS_SETUP"
    load_camera_workspace

    # 如果当前没有 ROS master，就临时启动一个。
    if ! rosparam list >/dev/null 2>&1; then
        roscore >"${OUTPUT_DIR}/roscore.log" 2>&1 &
        ROSCORE_PID=$!
        sleep 2
    fi

    # 启动 MAVROS 和 USB 相机驱动，同时发布 IMU 和图像。
    if ! rostopic list 2>/dev/null | grep -qx "$IMU_TOPIC"; then
        roslaunch mavros px4.launch \
            fcu_url:="$FCU_URL" \
            >"${OUTPUT_DIR}/mavros.log" 2>&1 &
        MAVROS_PID=$!
        sleep 3
    fi

    echo "Waiting for live IMU data on ${IMU_TOPIC}..."
    if ! timeout 20 rostopic echo -n 1 "${IMU_TOPIC}/header" >/dev/null 2>&1; then
        echo "No live IMU data received on ${IMU_TOPIC}." >&2
        echo "Check FCU power, /dev/ttyTHS0 and ${OUTPUT_DIR}/mavros.log" >&2
        exit 1
    fi

    roslaunch "$CAMERA_PACKAGE" "$CAMERA_LAUNCH" "${CAMERA_ARGS[@]}" \
        >"${OUTPUT_DIR}/usb_cam.log" 2>&1 &
    CAMERA_PID=$!
    sleep 2

    echo "Waiting for live camera data on ${CAM_TOPIC}..."
    if ! timeout 10 rostopic echo -n 1 "${CAM_TOPIC}/header" >/dev/null 2>&1; then
        echo "No live camera data received on ${CAM_TOPIC}." >&2
        echo "Check the camera connection and ${OUTPUT_DIR}/usb_cam.log" >&2
        exit 1
    fi

    # 录包保持相机原始帧率，预览限制为 5 Hz，避免全分辨率显示阻塞桌面。
    rosrun topic_tools throttle messages "$CAM_TOPIC" 5.0 "$PREVIEW_TOPIC" \
        >"${OUTPUT_DIR}/preview_throttle.log" 2>&1 &
    PREVIEW_THROTTLE_PID=$!
    rosrun rqt_image_view rqt_image_view --clear-config "$PREVIEW_TOPIC" &
    PREVIEW_WINDOW_PID=$!

    echo "Recording to ${BAG}. Press Ctrl-C to finish and calibrate."
    # 增大 rosbag 缓冲区以吸收短时存储抖动，减少全分辨率图像丢帧。
    rosbag record --buffsize=2048 -O "$BAG" "$CAM_TOPIC" "$IMU_TOPIC" &
    RECORD_PID=$!
    trap stop_recording INT
    wait_for_bag "$RECORD_PID"
    RECORD_PID=""
    stop_preview
    trap - INT
fi

python3 "${ROOT_DIR}/scripts/validate_extrinsics_bag.py" "$BAG" "$CAM_TOPIC" "$IMU_TOPIC"

# 在外参专用目录中运行 Kalibr，集中保存输出结果。
echo "Calibrating: ${BAG}"
KALIBR_RATE_ARGS=()
if [[ "$BAG_FREQ" != "0" ]]; then
    KALIBR_RATE_ARGS=(--bag-freq "$BAG_FREQ")
fi
source "$ROS_SETUP"
load_kalibr_workspace
cd "$OUTPUT_DIR"
"$KALIBR" \
    --target "$TARGET" \
    --bag "$BAG" \
    "${KALIBR_RATE_ARGS[@]}" \
    --cam "$CAMCHAIN" \
    --imu "$IMU" \
    --timeoffset-padding "$TIMEOFFSET_PADDING" \
    --dont-show-report

echo "Calibration finished. Results are beside ${BAG}"
