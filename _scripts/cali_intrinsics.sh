#!/usr/bin/env bash
# 采集或复用相机 rosbag，并调用 Kalibr 标定单目相机内参。
# 用法：bash _scripts/cali_intrinsics.sh [CALIBRATION_BAG]

set -e

####################### 参数区 #######################

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "Usage: $0 [CALIBRATION_BAG]"
    exit 0
fi

if [ "$#" -gt 1 ]; then
    echo "Usage: $0 [CALIBRATION_BAG]" >&2
    exit 2
fi

SH_DIR=$(cd "$(dirname "$0")" && pwd)
source "${SH_DIR}/common.sh"

OUTPUT_DIR="${OUTPUT_ROOT}/intrinsics"
TOPIC="$CAM_TOPIC"
KALIBR="${KALIBR_WS}/devel/lib/kalibr/kalibr_calibrate_cameras"
KALIBR_BAG_FREQ="${KALIBR_BAG_FREQ:-0}"
KALIBR_FOCAL_LENGTH_INIT="${KALIBR_FOCAL_LENGTH_INIT:-}"
CAMERA_ARGS=()
[ -z "$CAMERA_PARAM_FILE" ] || CAMERA_ARGS=(camera_params_file:="$CAMERA_PARAM_FILE")

####################### 环境变量区 #######################

# KALIBR_MANUAL_FOCAL_LENGTH_INIT 仅在启动区校验初始焦距后导出。

####################### 通用函数区 #######################

cleanup_processes() {
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
    [ -n "$CAMERA_PID" ] && kill -TERM "$CAMERA_PID" 2>/dev/null || true
    [ -n "$ROSCORE_PID" ] && kill -TERM "$ROSCORE_PID" 2>/dev/null || true
}

stop_recording() {
    # 录够图像后按 Ctrl-C，只停止 rosbag，然后继续跑 Kalibr。
    echo
    echo "Stopping recording..."
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
}

####################### 启动区 #######################

ROSCORE_PID=""
CAMERA_PID=""
RECORD_PID=""
trap cleanup_processes EXIT

mkdir -p "$OUTPUT_DIR"

if [ "$#" -eq 1 ]; then
    BAG_INPUT="$1"
    if [ ! -f "$BAG_INPUT" ]; then
        echo "Calibration bag not found: ${BAG_INPUT}" >&2
        exit 1
    fi
    # Kalibr 会在输出目录中运行，因此保留输入 bag 的绝对路径。
    BAG=$(realpath "$BAG_INPUT")
    echo "Using specified intrinsics bag: ${BAG}"
else
    read -r -p "Use latest existing intrinsics bag? [y/n] " USE_EXISTING_BAG
fi

if [ -n "${BAG:-}" ]; then
    : # A positional bag was validated above; skip recording.
elif [ "${USE_EXISTING_BAG:-}" = "y" ]; then
    # 直接使用时间戳最大的内参录包。
    BAG=$(find "$OUTPUT_DIR" "$CALIB_DIR" -maxdepth 1 -type f \
        -name 'camera_intrinsics_*.bag' -printf '%T@ %p\n' \
        | sort -nr | head -n 1 | cut -d' ' -f2-)
    if [ -z "$BAG" ]; then
        echo "No existing intrinsics bag found in: ${OUTPUT_DIR}" >&2
        exit 1
    fi
else
    BAG="${OUTPUT_DIR}/camera_intrinsics_$(date +%Y%m%d_%H%M%S).bag"

    # 先进入本工程 ROS 环境。
    source "$ROS_SETUP"
    load_camera_workspace

    # 如果当前没有 ROS master，就临时启动一个。
    if ! rosparam list >/dev/null 2>&1; then
        roscore >"${OUTPUT_DIR}/roscore.log" 2>&1 &
        ROSCORE_PID=$!
        sleep 2
    fi

    # 由驱动包的 launch 加载该相机的配置。
    roslaunch "$CAMERA_PACKAGE" "$CAMERA_LAUNCH" "${CAMERA_ARGS[@]}" \
        >"${OUTPUT_DIR}/usb_cam.log" 2>&1 &
    CAMERA_PID=$!
    sleep 2

    # 不允许相机启动失败后继续录制空包。
    if ! kill -0 "$CAMERA_PID" 2>/dev/null || \
       ! timeout 10 rostopic echo -n 1 "${TOPIC}/header" >/dev/null 2>&1; then
        echo "Camera did not publish ${TOPIC}. See ${OUTPUT_DIR}/usb_cam.log" >&2
        [ -n "$CAMERA_PID" ] && kill "$CAMERA_PID" 2>/dev/null || true
        exit 1
    fi
    echo "${CAMERA_PACKAGE} is publishing calibration frames."

    # 清除 rqt_image_view 保存的显示旋转，确保窗口忠实显示 image_raw。
    rosrun rqt_image_view rqt_image_view --clear-config "$TOPIC" &

    # 录制 Kalibr 需要的图像话题。
    echo "Recording ${TOPIC} to ${BAG}"
    echo "Move the AprilGrid through the image. Press Ctrl-C when done."
    rosbag record -O "$BAG" "$TOPIC" &
    RECORD_PID=$!
    trap stop_recording INT
    wait_for_bag "$RECORD_PID"
    RECORD_PID=""
    trap - INT
fi

# 在本次标定类型的专用目录中运行 Kalibr，集中保存 bag、日志和报告。
echo "Running Kalibr, output directory: ${OUTPUT_DIR}"
echo "Using bag: ${BAG}"
KALIBR_RATE_ARGS=()
if [[ "$KALIBR_BAG_FREQ" != "0" ]]; then
    KALIBR_RATE_ARGS=(--bag-freq "$KALIBR_BAG_FREQ")
    echo "Sampling images at ${KALIBR_BAG_FREQ} Hz for target extraction."
fi
echo "Note: Kalibr's extraction progress counts queued images; after N/N it may still be finishing worker tasks."
source "$ROS_SETUP"
load_kalibr_workspace
if [ -n "$KALIBR_FOCAL_LENGTH_INIT" ]; then
    if ! [[ "$KALIBR_FOCAL_LENGTH_INIT" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
       ! awk -v focal="$KALIBR_FOCAL_LENGTH_INIT" 'BEGIN { exit !(focal > 0) }'; then
        echo "KALIBR_FOCAL_LENGTH_INIT must be a positive number." >&2
        exit 2
    fi
    export KALIBR_MANUAL_FOCAL_LENGTH_INIT="$KALIBR_FOCAL_LENGTH_INIT"
    echo "Using focal-length initialization: ${KALIBR_FOCAL_LENGTH_INIT} px"
fi
cd "$OUTPUT_DIR"
"$KALIBR" \
    --target "$TARGET" \
    --bag "$BAG" \
    "${KALIBR_RATE_ARGS[@]}" \
    --models "${CAMERA_MODEL:-pinhole-equi}" \
    --topics "$TOPIC" \
    --dont-show-report

CAMCHAIN_RESULT="${BAG%.bag}-camchain.yaml"
RESULT_TEXT="${BAG%.bag}-results-cam.txt"
if [ ! -s "$CAMCHAIN_RESULT" ] || [ ! -s "$RESULT_TEXT" ]; then
    echo "Kalibr did not produce complete intrinsic-calibration results." >&2
    exit 1
fi
if grep -Eiq '\.?nan' "$CAMCHAIN_RESULT" "$RESULT_TEXT"; then
    echo "Intrinsic calibration failed: Kalibr produced NaN parameters." >&2
    echo "Do not use: ${CAMCHAIN_RESULT}" >&2
    echo "Re-record with varied board distance/tilt and broad image coverage," >&2
    echo "or retry with a different KALIBR_FOCAL_LENGTH_INIT value." >&2
    exit 1
fi
echo "Validated finite intrinsic result: ${CAMCHAIN_RESULT}"
