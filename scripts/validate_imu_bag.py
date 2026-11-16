#!/usr/bin/env python3
"""在 Allan 分析前校验静置 IMU rosbag 的完整性和静止状态。"""

import argparse
import math
import statistics
import sys

import rosbag
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("topic")
    parser.add_argument("expected_duration", type=float)
    parser.add_argument("config_path")
    args = parser.parse_args()

    with open(args.config_path, "r") as stream:
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

    try:
        with rosbag.Bag(args.bag_path) as bag:
            info = bag.get_type_and_topic_info().topics.get(args.topic)
            if info is None:
                topics = bag.get_type_and_topic_info().topics
                available = ", ".join(sorted(topics)) or "<none>"
                raise RuntimeError(
                    f"required topic {args.topic} not found; available topics: {available}"
                )
            message_type = info.msg_type
            if message_type != "sensor_msgs/Imu":
                raise RuntimeError(
                    f"{args.topic} has type {message_type}, expected sensor_msgs/Imu"
                )

            for _, msg, _ in bag.read_messages(topics=[args.topic]):
                stamp = msg.header.stamp.to_sec()
                values = (
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
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
        raise RuntimeError(f"cannot read rosbag: {error}") from error

    if count == 0:
        raise RuntimeError(f"no messages found on {args.topic}")
    if first_stamp is None or last_stamp is None or not intervals:
        raise RuntimeError("IMU messages do not contain usable header timestamps")

    duration = last_stamp - first_stamp
    measured_rate = 1.0 / statistics.median(intervals)
    large_gap_limit = 5.0 / measured_rate
    large_gaps = sum(interval > large_gap_limit for interval in intervals)
    gyro_rms = math.sqrt(gyro_norm_squared_sum / count)
    accel_mean = accel_norm_sum / count
    accel_variance = max(
        0.0, accel_norm_squared_sum / count - accel_mean * accel_mean
    )
    accel_std = math.sqrt(accel_variance)

    errors = []
    warnings = []
    if duration < args.expected_duration * 0.99:
        errors.append(
            f"duration {duration / 3600.0:.2f} h is shorter than the required "
            f"{args.expected_duration / 3600.0:.2f} h"
        )
    if zero_stamps:
        errors.append(f"{zero_stamps} messages have a zero header timestamp")
    if invalid_values:
        errors.append(f"{invalid_values} messages contain NaN or Inf")
    if repeated_stamps > max(10, count * 0.001):
        errors.append(f"too many repeated timestamps: {repeated_stamps}")
    if abs(measured_rate - configured_rate) / configured_rate > 0.10:
        errors.append(
            f"measured rate {measured_rate:.2f} Hz differs from imu_allan.yaml "
            f"({configured_rate:.2f} Hz) by more than 10%"
        )
    if large_gaps > max(10, count * 0.001):
        errors.append(f"too many timestamp gaps larger than 5 periods: {large_gaps}")
    if not 7.0 <= accel_mean <= 12.5:
        errors.append(
            f"mean acceleration norm {accel_mean:.3f} m/s^2 is implausible for a stationary IMU"
        )
    if accel_std > 0.20:
        errors.append(
            f"acceleration norm stddev {accel_std:.3f} m/s^2 suggests movement or vibration"
        )
    if gyro_rms > 0.05:
        errors.append(f"gyroscope RMS {gyro_rms:.4f} rad/s suggests the IMU was moving")
    if backward_stamps:
        warnings.append(
            f"{backward_stamps} timestamps are out of recording order; cookbag.py will reorder them"
        )

    print("\nIMU bag validation")
    print(f"  file:              {args.bag_path}")
    print(f"  topic/type:        {args.topic} ({message_type})")
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


if __name__ == "__main__":
    main()
