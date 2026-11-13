#!/bin/bash

set -e

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "Usage: $0 [STATIONARY_IMU_BAG]"
    exit 0
fi
[ "$#" -le 1 ] || { echo "Usage: $0 [STATIONARY_IMU_BAG]" >&2; exit 2; }

# 获取脚本所在目录和工程根目录，使脚本可以从任意工作目录启动。
SH_DIR=$(cd "$(dirname "$0")" && pwd)
source "${SH_DIR}/common.sh"

# 公共配置保留在 calibration 根目录，采集数据和 Allan 报告放在专用子目录。

OUTPUT_DIR="${OUTPUT_ROOT}/imu"
IMU_CONFIG="${OUTPUT_DIR}/imu_mavros.yaml"
ALLAN_CONFIG="${ALLAN_CONFIG:-${CALIB_DIR}/imu_allan.yaml}"


# allan_variance_ros 使用独立 catkin 工作空间，避免污染主工程依赖。


# 官方建议静置采集至少 3 小时；进度条宽度为 40 个字符。
IMU_CALIBRATION_DURATION="${IMU_CALIBRATION_DURATION:-10800}"
if ! [[ "$IMU_CALIBRATION_DURATION" =~ ^[1-9][0-9]*$ ]]; then
    echo "IMU_CALIBRATION_DURATION must be a positive integer (seconds)." >&2
    exit 2
fi
PROGRESS_BAR_WIDTH=40

RECORD_PID=""
ROSCORE_PID=""
MAVROS_PID=""

cleanup_processes() {
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
    [ -n "$MAVROS_PID" ] && kill -TERM "$MAVROS_PID" 2>/dev/null || true
    [ -n "$ROSCORE_PID" ] && kill -TERM "$ROSCORE_PID" 2>/dev/null || true
}
trap cleanup_processes EXIT

stop_recording() {
    # Ctrl-C 只通知 rosbag 正常结束，以便它完成索引并生成可读取的 bag。
    echo
    echo "Stopping IMU recording..."
    [ -n "$RECORD_PID" ] && kill -INT "$RECORD_PID" 2>/dev/null || true
}

format_duration() {
    # 将秒数格式化为 HH:MM:SS，供进度条显示。
    local total_seconds=$1
    printf '%02d:%02d:%02d' \
        $((total_seconds / 3600)) \
        $(((total_seconds % 3600) / 60)) \
        $((total_seconds % 60))
}

show_recording_progress() {
    # rosbag 存活期间每秒刷新一次进度，到 3 小时后自动停止录制。
    local start_time=$SECONDS
    local elapsed=0
    local remaining=$IMU_CALIBRATION_DURATION
    local percent=0
    local filled=0
    local empty=$PROGRESS_BAR_WIDTH
    local bar=""
    local empty_bar=""

    while kill -0 "$RECORD_PID" 2>/dev/null; do
        # 根据 Bash 内置的 SECONDS 计算已用时间、剩余时间和填充长度。
        elapsed=$((SECONDS - start_time))
        if [ "$elapsed" -ge "$IMU_CALIBRATION_DURATION" ]; then
            elapsed=$IMU_CALIBRATION_DURATION
        fi
        remaining=$((IMU_CALIBRATION_DURATION - elapsed))
        percent=$((elapsed * 100 / IMU_CALIBRATION_DURATION))
        filled=$((elapsed * PROGRESS_BAR_WIDTH / IMU_CALIBRATION_DURATION))
        empty=$((PROGRESS_BAR_WIDTH - filled))
        printf -v bar '%*s' "$filled" ''
        bar=${bar// /#}
        printf -v empty_bar '%*s' "$empty" ''
        empty_bar=${empty_bar// /-}

        printf '\rIMU recording [%s%s] %3d%%  elapsed %s  remaining %s' \
            "$bar" "$empty_bar" "$percent" \
            "$(format_duration "$elapsed")" "$(format_duration "$remaining")"

        if [ "$elapsed" -ge "$IMU_CALIBRATION_DURATION" ]; then
            # 使用 SIGINT 而不是强制结束，确保 rosbag 正常关闭文件。
            echo
            echo "IMU calibration duration reached. Stopping recording..."
            kill -INT "$RECORD_PID" 2>/dev/null || true
            break
        fi
        sleep 1
    done
    echo
}

validate_imu_bag() {
    # 在 Allan 分析前遍历整包，避免用缺数据、错误话题或运动中的数据生成参数。
    local bag_path=$1

    python3 - "$bag_path" "$IMU_TOPIC" "$IMU_CALIBRATION_DURATION" "$ALLAN_CONFIG" <<'PY'
import math
import statistics
import sys

import rosbag
import yaml

bag_path, topic, expected_duration, config_path = sys.argv[1:]
expected_duration = float(expected_duration)
with open(config_path, "r") as stream:
    configured_rate = float(yaml.safe_load(stream)["imu_rate"])

count = 0
zero_stamps = 0
repeated_stamps = 0
backward_stamps = 0
invalid_values = 0
previous_stamp = None
first_stamp = None
last_stamp = None
intervals = []
gyro_norm_squared_sum = 0.0
accel_norm_sum = 0.0
accel_norm_squared_sum = 0.0
message_type = None

try:
    with rosbag.Bag(bag_path) as bag:
        info = bag.get_type_and_topic_info().topics.get(topic)
        if info is None:
            available = ", ".join(sorted(bag.get_type_and_topic_info().topics)) or "<none>"
            raise RuntimeError(f"required topic {topic} not found; available topics: {available}")
        message_type = info.msg_type
        if message_type != "sensor_msgs/Imu":
            raise RuntimeError(f"{topic} has type {message_type}, expected sensor_msgs/Imu")

        for _, msg, _ in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec()
            values = (
                msg.linear_acceleration.x, msg.linear_acceleration.y,
                msg.linear_acceleration.z, msg.angular_velocity.x,
                msg.angular_velocity.y, msg.angular_velocity.z,
            )
            if not all(math.isfinite(value) for value in values):
                invalid_values += 1

            if stamp <= 0.0:
                zero_stamps += 1
            elif previous_stamp is not None:
                delta = stamp - previous_stamp
                if delta == 0.0:
                    repeated_stamps += 1
                elif delta < 0.0:
                    backward_stamps += 1
                else:
                    intervals.append(delta)
            if stamp > 0.0:
                first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
                last_stamp = stamp if last_stamp is None else max(last_stamp, stamp)
                previous_stamp = stamp

            ax, ay, az, gx, gy, gz = values
            accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
            gyro_norm_squared_sum += gx * gx + gy * gy + gz * gz
            accel_norm_sum += accel_norm
            accel_norm_squared_sum += accel_norm * accel_norm
            count += 1
except (rosbag.bag.ROSBagException, OSError) as error:
    raise RuntimeError(f"cannot read rosbag: {error}")

if count == 0:
    raise RuntimeError(f"no messages found on {topic}")
if first_stamp is None or last_stamp is None or not intervals:
    raise RuntimeError("IMU messages do not contain usable header timestamps")

duration = last_stamp - first_stamp
median_interval = statistics.median(intervals)
measured_rate = 1.0 / median_interval
large_gap_limit = 5.0 / measured_rate
large_gaps = sum(interval > large_gap_limit for interval in intervals)
gyro_rms = math.sqrt(gyro_norm_squared_sum / count)
accel_mean = accel_norm_sum / count
accel_variance = max(0.0, accel_norm_squared_sum / count - accel_mean * accel_mean)
accel_std = math.sqrt(accel_variance)

errors = []
warnings = []
if duration < expected_duration * 0.99:
    errors.append(f"duration {duration / 3600.0:.2f} h is shorter than the required 3 h")
if zero_stamps:
    errors.append(f"{zero_stamps} messages have a zero header timestamp")
if invalid_values:
    errors.append(f"{invalid_values} messages contain NaN or Inf")
if repeated_stamps > max(10, count * 0.001):
    errors.append(f"too many repeated timestamps: {repeated_stamps}")
if abs(measured_rate - configured_rate) / configured_rate > 0.10:
    errors.append(
        f"measured rate {measured_rate:.2f} Hz differs from imu_allan.yaml "
        f"({configured_rate:.2f} Hz) by more than 10%")
if large_gaps > max(10, count * 0.001):
    errors.append(f"too many timestamp gaps larger than 5 periods: {large_gaps}")
if not 7.0 <= accel_mean <= 12.5:
    errors.append(f"mean acceleration norm {accel_mean:.3f} m/s^2 is implausible for a stationary IMU")
if accel_std > 0.20:
    errors.append(f"acceleration norm stddev {accel_std:.3f} m/s^2 suggests movement or vibration")
if gyro_rms > 0.05:
    errors.append(f"gyroscope RMS {gyro_rms:.4f} rad/s suggests the IMU was moving")
if backward_stamps:
    warnings.append(
        f"{backward_stamps} timestamps are out of recording order; cookbag.py will reorder them")

print("\nIMU bag validation")
print(f"  file:              {bag_path}")
print(f"  topic/type:        {topic} ({message_type})")
print(f"  messages:          {count}")
print(f"  duration:          {duration / 3600.0:.3f} h")
print(f"  measured rate:     {measured_rate:.3f} Hz")
print(f"  zero/repeated ts:  {zero_stamps}/{repeated_stamps}")
print(f"  backward/large gap:{backward_stamps}/{large_gaps}")
print(f"  accel norm mean:   {accel_mean:.4f} m/s^2")
print(f"  accel norm stddev: {accel_std:.4f} m/s^2")
print(f"  gyro RMS:          {gyro_rms:.6f} rad/s")
for warning in warnings:
    print(f"  WARNING: {warning}")
if errors:
    for error in errors:
        print(f"  ERROR: {error}", file=sys.stderr)
    raise SystemExit("IMU bag validation failed; Allan analysis was not started.")
print("  result:            PASS\n")
PY
}

mkdir -p "$OUTPUT_DIR"

# 可以复用已有静置数据；选择 n 或其他输入时重新采集一份数据。
if [ "$#" -eq 1 ]; then
    [ -f "$1" ] || { echo "IMU bag not found: $1" >&2; exit 1; }
    IMU_BAG=$(realpath "$1")
else
    read -r -p "Use latest existing IMU noise bag? [y/n] " USE_EXISTING_BAG
fi

if [ -n "${IMU_BAG:-}" ]; then
    :
elif [ "${USE_EXISTING_BAG:-}" = "y" ]; then
    # 按文件修改时间选择最新的完整 .bag，不会选择录制中的 .bag.active。
    IMU_BAG=$(find "$OUTPUT_DIR" "$CALIB_DIR" -maxdepth 1 -type f -name 'imu_noise_*.bag' \
        -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)
    if [ -z "$IMU_BAG" ]; then
        echo "No existing IMU noise bag found in: ${OUTPUT_DIR}" >&2
        exit 1
    fi
    echo "Using existing IMU noise bag: ${IMU_BAG}"
else
    # 新录制时加载主工程 ROS 环境，以便使用 MAVROS 和相关消息定义。
    source "$ROS_SETUP"
     # MAVROS is supplied by the ROS installation.

    # 当前没有 ROS master 时临时启动一个。
    if ! rosparam list >/dev/null 2>&1; then
        roscore >"${OUTPUT_DIR}/roscore.log" 2>&1 &
    ROSCORE_PID=$!
        sleep 2
    fi

    # IMU 话题尚未发布时启动 MAVROS；已有 MAVROS 则直接复用。
    if ! rostopic list 2>/dev/null | grep -qx "$IMU_TOPIC"; then
        roslaunch mavros px4.launch \
            fcu_url:="$FCU_URL" \
            >"${OUTPUT_DIR}/mavros.log" 2>&1 &
        MAVROS_PID=$!
        sleep 3
    fi

    IMU_BAG="${OUTPUT_DIR}/imu_noise_$(date +%Y%m%d_%H%M%S).bag"

    echo "Recording stationary ${IMU_TOPIC} to ${IMU_BAG}"
    echo "Keep the IMU completely still for 3 hours. Press Ctrl-C to stop early."
    # 后台录制后由进度函数负责计时；Ctrl-C 会触发 stop_recording。
    rosbag record -O "$IMU_BAG" "$IMU_TOPIC" &
    RECORD_PID=$!
    trap stop_recording INT
    show_recording_progress
    wait_for_bag "$RECORD_PID"
    RECORD_PID=""
    trap - INT

    echo "IMU noise bag saved to: ${IMU_BAG}"
fi

# 无论使用新包还是已有包，后续都进入 Allan 分析阶段。
source "$ROS_SETUP"
 # MAVROS is supplied by the ROS installation.

# 校验失败会立即退出，不会重排 bag，也不会覆盖 imu_mavros.yaml。
validate_imu_bag "$IMU_BAG"

# 提前检查第三方工作空间，缺失时给出对应的安装和构建命令。
if [ ! -f "${ALLAN_WS}/devel/setup.bash" ]; then
    echo "allan_variance_ros workspace not found: ${ALLAN_WS}" >&2
    echo "Install it in the project third-party workspace first:" >&2
    echo "  mkdir -p ${ALLAN_WS}/src" >&2
    echo "  git clone https://github.com/ori-drs/allan_variance_ros.git ${ALLAN_WS}/src/allan_variance_ros" >&2
    echo "  source "$ROS_SETUP"" >&2
    echo "  catkin build --workspace ${ALLAN_WS} allan_variance_ros" >&2
    exit 1
fi
source "${ALLAN_WS}/devel/setup.bash"

# allan_variance_ros 的 C++ 节点需要连接 ROS master，即使它直接读取 bag。
if ! rosparam list >/dev/null 2>&1; then
    roscore >"${OUTPUT_DIR}/roscore.log" 2>&1 &
    ROSCORE_PID=$!
    sleep 2
fi

BAG_BASENAME=$(basename "$IMU_BAG" .bag)
# 每份原始 bag 使用独立输出目录，避免不同批次的 CSV 和图片相互覆盖。
ANALYSIS_DIR="${OUTPUT_DIR}/${BAG_BASENAME}_allan"
COOKED_BAG="${ANALYSIS_DIR}/${BAG_BASENAME}_cooked.bag"
mkdir -p "$ANALYSIS_DIR"

# 官方建议先按消息 header.stamp 重排，减少乱序时间戳对 Allan 计算的影响。
echo "Reordering IMU messages by header timestamp..."
rosrun allan_variance_ros cookbag.py \
    --input "$IMU_BAG" \
    --output "$COOKED_BAG"

# 读取 cooked bag 并生成 ANALYSIS_DIR/allan_variance.csv。
echo "Computing Allan variance..."
rosrun allan_variance_ros allan_variance "$ANALYSIS_DIR" "$ALLAN_CONFIG"

# analysis.py 拟合噪声参数、保存图像，并直接生成 Kalibr 格式的 YAML。
echo "Generating plots and Kalibr IMU parameters..."
# 覆盖前保留上一次参数，分析异常时可以人工恢复。
if [ -f "$IMU_CONFIG" ]; then
    cp -a "$IMU_CONFIG" "${OUTPUT_DIR}/imu_mavros.yaml.bak"
fi
(
    # 图片由上游脚本写到当前目录；Agg 后端允许在无显示器环境运行。
    cd "$ANALYSIS_DIR"
    MPLBACKEND=Agg rosrun allan_variance_ros analysis.py \
        --data "${ANALYSIS_DIR}/allan_variance.csv" \
        --config "$ALLAN_CONFIG" \
        --output "$IMU_CONFIG"
)

echo "Allan analysis directory: ${ANALYSIS_DIR}"
echo "Kalibr IMU configuration updated: ${IMU_CONFIG}"
