#!/usr/bin/env python3
"""确认外参标定 rosbag 同时包含有效的相机和 IMU 数据。"""

import argparse

import rosbag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("camera_topic")
    parser.add_argument("imu_topic")
    args = parser.parse_args()

    try:
        with rosbag.Bag(args.bag_path) as bag:
            topics = bag.get_type_and_topic_info().topics
            counts = {topic: info.message_count for topic, info in topics.items()}
    except (rosbag.bag.ROSBagException, OSError) as error:
        raise RuntimeError(f"cannot read rosbag: {error}") from error

    missing = [
        topic
        for topic in (args.camera_topic, args.imu_topic)
        if counts.get(topic, 0) <= 0
    ]
    if missing:
        raise SystemExit(
            "Calibration bag is missing required data: " + ", ".join(missing)
        )


if __name__ == "__main__":
    main()
