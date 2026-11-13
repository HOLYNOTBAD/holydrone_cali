#!/bin/bash

set -e

usage() {
    echo "Usage: $0 [--camchain CAMCHAIN_YAML] [--bag-freq HZ] [--pose-knots N] [--bias-knots N] [--timeoffset-padding SEC] [--timeoffset-search-window SEC] [--rolling-shutter-line-delay SEC_PER_ROW] [--imu-use-bag-time] [--reprojection-robust-df N] [--accel-huber-sigma N] [--gyro-huber-sigma N] [--max-angular-speed RAD_S] [CALIBRATION_BAG]" >&2
}

CAMCHAIN=""
BAG=""
BAG_FREQ="${KALIBR_BAG_FREQ:-10}"
POSE_KNOTS="${KALIBR_POSE_KNOTS:-30}"
BIAS_KNOTS="${KALIBR_BIAS_KNOTS:-5}"
TIMEOFFSET_PADDING="${KALIBR_TIMEOFFSET_PADDING:-0.15}"
TIMEOFFSET_SEARCH_WINDOW="${KALIBR_TIMEOFFSET_SEARCH_WINDOW:-0.20}"
REPROJECTION_ROBUST_DF="${KALIBR_REPROJECTION_ROBUST_DF:-0}"
ACCEL_HUBER_SIGMA="${KALIBR_ACCEL_HUBER_SIGMA:-0}"
GYRO_HUBER_SIGMA="${KALIBR_GYRO_HUBER_SIGMA:-0}"
# Keep a conservative angular speed so AprilGrid corners remain sharp across
# all three axes on the IMX577-1 30-degree stream.
MAX_ANGULAR_SPEED="${CALIB_MAX_ANGULAR_SPEED_RAD_S:-0.45}"
ROLLING_SHUTTER_LINE_DELAY="${KALIBR_ROLLING_SHUTTER_LINE_DELAY:-0}"
IMU_USE_BAG_TIME=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        --camchain)
            if [ "$#" -lt 2 ]; then
                usage
                exit 2
            fi
            CAMCHAIN="$2"
            shift 2
            ;;
        --bag-freq)
            if [ "$#" -lt 2 ]; then
                usage
                exit 2
            fi
            BAG_FREQ="$2"
            shift 2
            ;;
        --pose-knots)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            POSE_KNOTS="$2"
            shift 2
            ;;
        --bias-knots)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            BIAS_KNOTS="$2"
            shift 2
            ;;
        --timeoffset-padding)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            TIMEOFFSET_PADDING="$2"
            shift 2
            ;;
        --timeoffset-search-window)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            TIMEOFFSET_SEARCH_WINDOW="$2"
            shift 2
            ;;
        --rolling-shutter-line-delay)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            ROLLING_SHUTTER_LINE_DELAY="$2"
            shift 2
            ;;
        --reprojection-robust-df)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            REPROJECTION_ROBUST_DF="$2"
            shift 2
            ;;
        --imu-use-bag-time)
            IMU_USE_BAG_TIME=true
            shift
            ;;
        --accel-huber-sigma)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            ACCEL_HUBER_SIGMA="$2"
            shift 2
            ;;
        --gyro-huber-sigma)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            GYRO_HUBER_SIGMA="$2"
            shift 2
            ;;
        --max-angular-speed)
            [ "$#" -ge 2 ] || { usage; exit 2; }
            MAX_ANGULAR_SPEED="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        -*)
            usage
            exit 2
            ;;
        *)
            if [ -n "$BAG" ]; then
                usage
                exit 2
            fi
            BAG="$1"
            shift
            ;;
    esac
done

SH_DIR=$(cd "$(dirname "$0")" && pwd)
source "${SH_DIR}/common.sh"

# 公共配置放在 calibration 根目录，外参录包、日志和报告放在专用子目录。

OUTPUT_DIR="${OUTPUT_ROOT}/extrinsics"
INTRINSICS_DIR="${OUTPUT_ROOT}/intrinsics"
# Use the camchain produced for the same camera capture settings.
DEFAULT_CAMCHAIN="${CAMCHAIN_FILE:-}"

IMU="${IMU_CONFIG:-${CALIB_DIR}/imu_mavros.example.yaml}"

PREVIEW_TOPIC="/camera/calibration_preview"

KALIBR="${KALIBR_WS}/devel/lib/kalibr/kalibr_calibrate_imu_camera"
KALIBR_SOURCE="${KALIBR_WS}/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera"
# Use the IMX577-1 30-degree camera parameters that this workflow will calibrate.


RECORD_PID=""
ANGULAR_MONITOR_PID=""
PREVIEW_THROTTLE_PID=""
PREVIEW_WINDOW_PID=""
ROSCORE_PID=""
MAVROS_PID=""
CAMERA_PID=""

cleanup_processes() {
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
    [ -n "$ANGULAR_MONITOR_PID" ] && kill -TERM "$ANGULAR_MONITOR_PID" 2>/dev/null || true
    [ -n "$PREVIEW_THROTTLE_PID" ] && kill -TERM "$PREVIEW_THROTTLE_PID" 2>/dev/null || true
    [ -n "$PREVIEW_WINDOW_PID" ] && kill -TERM "$PREVIEW_WINDOW_PID" 2>/dev/null || true
    [ -n "$CAMERA_PID" ] && kill -TERM "$CAMERA_PID" 2>/dev/null || true
    [ -n "$MAVROS_PID" ] && kill -TERM "$MAVROS_PID" 2>/dev/null || true
    [ -n "$ROSCORE_PID" ] && kill -TERM "$ROSCORE_PID" 2>/dev/null || true
}

trap cleanup_processes EXIT

stop_recording() {
    # 录够图像和 IMU 后按 Ctrl-C，只停止 rosbag，然后继续跑 Kalibr。
    echo
    echo "Stopping recording..."
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
    [ -n "$ANGULAR_MONITOR_PID" ] && kill -TERM "$ANGULAR_MONITOR_PID" 2>/dev/null || true
    [ -n "$PREVIEW_THROTTLE_PID" ] && kill -TERM "$PREVIEW_THROTTLE_PID" 2>/dev/null || true
    [ -n "$PREVIEW_WINDOW_PID" ] && kill -TERM "$PREVIEW_WINDOW_PID" 2>/dev/null || true
}

wait_for_recording_to_finalize() {
    local pid="$1"
    # A trapped Ctrl-C interrupts bash's first wait even though rosbag is still
    # flushing its queue and indexing the .bag.active file. Keep waiting until
    # rosbag exits and has renamed the finalized file to .bag.
    while kill -0 "$pid" 2>/dev/null; do
        wait "$pid" 2>/dev/null || true
    done
    wait "$pid" 2>/dev/null || true
}

mkdir -p "$OUTPUT_DIR" "$INTRINSICS_DIR"

if [ -z "$CAMCHAIN" ]; then
    CAMCHAIN="$DEFAULT_CAMCHAIN"
fi

if [ -z "$CAMCHAIN" ] || [ ! -f "$CAMCHAIN" ]; then
    echo "Camera intrinsics camchain not found: ${CAMCHAIN}" >&2
    echo "Run scripts/cali_intrinsics.sh first, then pass --camchain CAMCHAIN_YAML." >&2
    exit 1
fi
# Kalibr runs after cd'ing into OUTPUT_DIR, so keep input paths absolute.
CAMCHAIN=$(realpath "$CAMCHAIN")

for VALUE_NAME in BAG_FREQ POSE_KNOTS BIAS_KNOTS TIMEOFFSET_PADDING TIMEOFFSET_SEARCH_WINDOW REPROJECTION_ROBUST_DF ACCEL_HUBER_SIGMA GYRO_HUBER_SIGMA MAX_ANGULAR_SPEED ROLLING_SHUTTER_LINE_DELAY; do
    VALUE="${!VALUE_NAME}"
    if ! [[ "$VALUE" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "Invalid numeric value for ${VALUE_NAME}: ${VALUE}" >&2
        exit 2
    fi
done

if [ ! -f "$KALIBR_SOURCE" ] || \
   ! grep -q -- '--pose-knots-per-second' "$KALIBR_SOURCE" || \
   ! grep -q -- '--timeoffset-search-window' "$KALIBR_SOURCE" || \
   ! grep -q -- '--reprojection-robust-df' "$KALIBR_SOURCE"; then
    echo "Kalibr does not contain the project calibration-stability extensions." >&2
    echo "Apply the project patch first:" >&2
    echo "  cd ${KALIBR_WS}/src/kalibr && git apply ${ROOT_DIR}/patches/kalibr-local.patch" >&2
    exit 1
fi

echo "Selected camera intrinsics: ${CAMCHAIN}"
grep -E '^[[:space:]]*(resolution|rostopic):' "$CAMCHAIN" || true
echo "IMU configuration: ${IMU}"
grep -E '^[[:space:]]*(rostopic|update_rate):' "$IMU" || true
echo "Confirm that resolution, ROI, image rotation, topics and IMU rate match the recorded data."

if [ -n "$BAG" ]; then
    if [ ! -f "$BAG" ]; then
        echo "Calibration bag not found: ${BAG}" >&2
        exit 1
    fi
    BAG=$(realpath "$BAG")
    echo "Using specified extrinsics bag: ${BAG}"
else
    read -r -p "Use latest existing extrinsics bag? [y/n] " USE_EXISTING_BAG
fi

if [ -n "${BAG:-}" ]; then
    : # A positional bag was validated above; skip recording.
elif [ "${USE_EXISTING_BAG:-}" = "y" ]; then
    # 直接使用时间戳最大的外参录包。
    BAG=$(find "$OUTPUT_DIR" "$CALIB_DIR" -maxdepth 1 -type f \
        -name 'camera_imu_extrinsics_*.bag' -printf '%T@ %p\n' \
        | sort -nr | head -n 1 | cut -d' ' -f2-)
    if [ -z "$BAG" ]; then
        echo "No existing extrinsics bag found in: ${OUTPUT_DIR}" >&2
        exit 1
    fi
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

    roslaunch camera_driver parallel_mjpeg_camera.launch \
        camera_params_file:="${CAMERA_PARAM_FILE}" \
        >"${OUTPUT_DIR}/usb_cam.log" 2>&1 &
    CAMERA_PID=$!
    sleep 2

    echo "Waiting for live camera data on ${CAM_TOPIC}..."
    if ! timeout 10 rostopic echo -n 1 "${CAM_TOPIC}/header" >/dev/null 2>&1; then
        echo "No live camera data received on ${CAM_TOPIC}." >&2
        echo "Check the camera connection and ${OUTPUT_DIR}/usb_cam.log" >&2
        exit 1
    fi

    # Keep recording at the full camera rate while limiting the GUI preview to
    # 5 Hz so full-resolution image rendering does not stall the desktop.
    rosrun topic_tools throttle messages "$CAM_TOPIC" 5.0 "$PREVIEW_TOPIC" \
        >"${OUTPUT_DIR}/preview_throttle.log" 2>&1 &
    PREVIEW_THROTTLE_PID=$!
    rosrun rqt_image_view rqt_image_view --clear-config "$PREVIEW_TOPIC" &
    PREVIEW_WINDOW_PID=$!

    # 录制 Kalibr camera-IMU 外参标定需要的话题。
    echo "Using camchain: ${CAMCHAIN}"
    echo "Using camera parameters: ${CAMERA_PARAM_FILE}"
    echo "Recording ${CAM_TOPIC} and ${IMU_TOPIC} to ${BAG}"
    echo "Keep the AprilGrid fixed. Move the rigid camera-IMU assembly with"
    echo "smooth translation and separate roll/pitch/yaw excitation for 60-90 seconds."
    echo "Rotate each axis slowly in both directions; do not combine all axes at once."
    echo "Aim for 0.15-0.40 rad/s while rotating and keep every axis well exercised."
    echo "Keep the grid fully visible at varied distances and tilts; avoid blur and impacts."
    echo "Live angular-speed limit: ${MAX_ANGULAR_SPEED} rad/s"
    echo "Remain still for 3-5 seconds at both ends. Press Ctrl-C when done."
    python3 "${SH_DIR}/monitor_imu_angular_speed.py" \
        --topic "$IMU_TOPIC" --limit "$MAX_ANGULAR_SPEED" &
    ANGULAR_MONITOR_PID=$!
    # Full-resolution decoded frames are large. A larger queue absorbs short
    # storage stalls and reduces dropped messages while rosbag writes them.
    rosbag record --buffsize=2048 -O "$BAG" "$CAM_TOPIC" "$IMU_TOPIC" &
    RECORD_PID=$!
    trap stop_recording INT
    wait_for_recording_to_finalize "$RECORD_PID"
    RECORD_PID=""
    [ -n "$ANGULAR_MONITOR_PID" ] && kill -TERM "$ANGULAR_MONITOR_PID" 2>/dev/null || true
    wait "$ANGULAR_MONITOR_PID" 2>/dev/null || true
    ANGULAR_MONITOR_PID=""
    [ -n "$PREVIEW_THROTTLE_PID" ] && kill -TERM "$PREVIEW_THROTTLE_PID" 2>/dev/null || true
    [ -n "$PREVIEW_WINDOW_PID" ] && kill -TERM "$PREVIEW_WINDOW_PID" 2>/dev/null || true
    wait "$PREVIEW_THROTTLE_PID" 2>/dev/null || true
    wait "$PREVIEW_WINDOW_PID" 2>/dev/null || true
    PREVIEW_THROTTLE_PID=""
    PREVIEW_WINDOW_PID=""
    trap - INT
fi

echo "Bag summary (expect about 30 Hz images and 50 Hz IMU):"
rosbag info "$BAG" || {
    echo "rosbag validation failed: ${BAG}" >&2
    exit 1
}
if ! rosbag info --yaml "$BAG" | python3 -c '
import sys
import yaml

camera_topic, imu_topic = sys.argv[1:3]
info = yaml.safe_load(sys.stdin)
counts = {entry["topic"]: entry["messages"] for entry in info.get("topics", [])}
missing = [topic for topic in (camera_topic, imu_topic) if counts.get(topic, 0) <= 0]
if missing:
    print("Calibration bag is missing required data: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
' "$CAM_TOPIC" "$IMU_TOPIC"; then
    echo "Cannot run camera-IMU calibration with: ${BAG}" >&2
    exit 1
fi

# 在外参专用目录中运行 Kalibr，集中保存输出结果。
echo "Running Kalibr, output directory: ${OUTPUT_DIR}"
echo "Using camchain: ${CAMCHAIN}"
echo "Using bag: ${BAG}"
echo "Spline settings: pose=${POSE_KNOTS}/s, bias=${BIAS_KNOTS}/s"
echo "Time-offset search padding: ${TIMEOFFSET_PADDING} s"
echo "Initial time-offset correlation window: +/-${TIMEOFFSET_SEARCH_WINDOW} s"
echo "Robust losses: reprojection BZ=${REPROJECTION_ROBUST_DF}, accel Huber=${ACCEL_HUBER_SIGMA} sigma, gyro Huber=${GYRO_HUBER_SIGMA} sigma"
echo "Rolling-shutter line delay: ${ROLLING_SHUTTER_LINE_DELAY} s/published-row (0 = disabled)"
KALIBR_RATE_ARGS=()
KALIBR_IMU_TIME_ARGS=()
if $IMU_USE_BAG_TIME; then
    KALIBR_IMU_TIME_ARGS=(--imu-use-bag-time)
    echo "IMU timestamp source: rosbag receive time (experimental)."
else
    echo "IMU timestamp source: message header."
fi
if [[ "$BAG_FREQ" != "0" ]]; then
    KALIBR_RATE_ARGS=(--bag-freq "$BAG_FREQ")
    echo "Sampling camera images at ${BAG_FREQ} Hz for target extraction."
else
    echo "Processing all camera images (bag frequency sampling disabled)."
fi
source "$ROS_SETUP"
load_kalibr_workspace
cd "$OUTPUT_DIR"
"$KALIBR" \
    --target "$TARGET" \
    --bag "$BAG" \
    "${KALIBR_RATE_ARGS[@]}" \
    "${KALIBR_IMU_TIME_ARGS[@]}" \
    --cam "$CAMCHAIN" \
    --imu "$IMU" \
    --pose-knots-per-second "$POSE_KNOTS" \
    --bias-knots-per-second "$BIAS_KNOTS" \
    --timeoffset-padding "$TIMEOFFSET_PADDING" \
    --timeoffset-search-window "$TIMEOFFSET_SEARCH_WINDOW" \
    --rolling-shutter-line-delay "$ROLLING_SHUTTER_LINE_DELAY" \
    --reprojection-robust-df "$REPROJECTION_ROBUST_DF" \
    --accel-huber-sigma "$ACCEL_HUBER_SIGMA" \
    --gyro-huber-sigma "$GYRO_HUBER_SIGMA" \
    --dont-show-report

echo "Calibration finished. Validate at least three independent bags."
echo "Targets: mean reprojection error <1.5 px; cross-run rotation <0.3-0.5 deg; translation <5-10 mm."
echo "Use timeshift_cam_imu from the output camchain. The search window limits the initial correlation; padding bounds later optimization."
