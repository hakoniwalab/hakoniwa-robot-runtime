#!/usr/bin/env python3
"""Publish the Hakoniwa simulation time as the ROS clock (/clock).

The Robot Runtime stamps its outputs with the simulation time; this node takes
that time from the joint states topic (default /pdu/joint_states) and
publishes it on /clock, so that ROS nodes started with use_sim_time:=true
(controllers, TF, MoveIt) run on the same time as the sensor data. The clock
advances with the joint states (every 20 ms for 50 Hz) and never goes back.
"""
import argparse
import sys

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--state-topic", default="/pdu/joint_states")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = rclpy.create_node("hakoniwa_clock")
    clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    publisher = node.create_publisher(Clock, "/clock", clock_qos)
    last = [-1]

    def on_state(message):
        stamp = message.header.stamp
        now = stamp.sec * 1_000_000_000 + stamp.nanosec
        if now <= last[0]:
            return
        last[0] = now
        clock = Clock()
        clock.clock = stamp
        publisher.publish(clock)

    node.create_subscription(JointState, args.state_topic, on_state, qos_profile_sensor_data)
    node.get_logger().info(f"publishing /clock from {args.state_topic}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
