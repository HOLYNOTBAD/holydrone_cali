#!/usr/bin/env python3
"""Warn when the measured angular-speed norm exceeds the acquisition limit."""
import argparse
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/mavros/imu/data")
    parser.add_argument("--limit", type=float, default=0.45, help="rad/s")
    args = parser.parse_args()
    if not math.isfinite(args.limit) or args.limit <= 0:
        parser.error("--limit must be finite and positive")

    import rospy
    from sensor_msgs.msg import Imu

    rospy.init_node("calibration_angular_speed_monitor", anonymous=True)

    def callback(msg):
        w = msg.angular_velocity
        speed = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
        if not math.isfinite(speed):
            rospy.logwarn_throttle(2, "IMU angular velocity is not finite")
        elif speed > args.limit:
            rospy.logwarn_throttle(
                2, "Angular speed %.3f rad/s exceeds %.3f; move more slowly",
                speed, args.limit)

    rospy.Subscriber(args.topic, Imu, callback, queue_size=20)
    rospy.spin()


if __name__ == "__main__":
    main()
