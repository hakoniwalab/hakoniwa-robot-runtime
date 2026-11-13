#!/usr/bin/env python3
"""Count the LiDAR points MoveIt keeps around the arm (its self filter missed them)
(ros2/docs/evaluation-gazebo.md 3.4).

python3 self_filter_leak.py <out.csv>

For every cloud on /overhead_lidar/filtered_cloud (after MoveIt's self filter):
the points, in base_link, inside the arm's work volume and outside the cube
and the static scenery, with the largest joint speed at that moment. Static
scenery and the cube give no points there, so they are the arm's own points - or
the cube while it is carried. To tell them apart, each of these points is also
given to the nearest part of the robot at the time the cloud is received: the
hand (within HAND_RADIUS of the gripper frame, where a held cube is), or the
segment of the arm between two joint frames (shoulder, upper arm, lower arm,
wrist). Columns leak_<part> count them (-1: no TF yet).
"""
import csv
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

# The arm's reach above the table, away from the walls and posts (x < -0.1).
VOLUME = dict(x=(-0.08, 0.45), y=(-0.40, 0.40), z=(0.06, 0.45))
CUBE_XY_RADIUS = 0.04  # around the pick and place positions (the cube may be held there)
CUBE_PLACES = [(0.28, 0.0), (0.2, -0.2)]
HAND_RADIUS = 0.06    # around gripper_frame_link: the jaws and a held 30 mm cube
# Arm segments between joint frames (origins), and the hand.
SEGMENTS = [("shoulder", "shoulder_link", "upper_arm_link"),
            ("upper_arm", "upper_arm_link", "lower_arm_link"),
            ("lower_arm", "lower_arm_link", "wrist_link"),
            ("wrist", "wrist_link", "gripper_frame_link")]
PARTS = ["hand"] + [name for name, _, _ in SEGMENTS]


def segment_distance(pts, a, b):
    ab = b - a
    t = np.clip(((pts - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def quat_matrix(q):
    return np.array([
        [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
        [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)],
        [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("self_filter_leak")
    tf = Buffer()
    TransformListener(tf, node)
    speed = {"max": 0.0}

    def on_joints(msg):
        if msg.velocity:
            speed["max"] = max(speed["max"], float(np.max(np.abs(msg.velocity[:5]))))

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    out = open(sys.argv[1], "w", newline="")
    writer = csv.writer(out)
    writer.writerow(["wall_s", "stamp_s", "points", "leak", "joint_speed_max"] + [f"leak_{p}" for p in PARTS])

    def origin(frame):
        tr = tf.lookup_transform("base_link", frame, rclpy.time.Time()).transform.translation
        return np.array([tr.x, tr.y, tr.z])

    def by_part(pts):
        counts = dict.fromkeys(PARTS, 0)
        if len(pts) == 0:
            return counts
        try:
            frames = {f: origin(f) for _, a, b in SEGMENTS for f in (a, b)}
        except Exception:
            return dict.fromkeys(PARTS, -1)   # no TF yet: unknown
        hand = np.linalg.norm(pts - frames["gripper_frame_link"], axis=1) < HAND_RADIUS
        counts["hand"] = int(np.sum(hand))
        rest = pts[~hand]
        if len(rest):
            d = np.stack([segment_distance(rest, frames[a], frames[b]) for _, a, b in SEGMENTS])
            nearest = np.argmin(d, axis=0)
            for i, (name, _, _) in enumerate(SEGMENTS):
                counts[name] = int(np.sum(nearest == i))
        return counts

    def on_cloud(cloud):
        try:
            t = tf.lookup_transform("base_link", cloud.header.frame_id, rclpy.time.Time()).transform
        except Exception:
            return
        pts = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        parts = dict.fromkeys(PARTS, 0)
        if pts.size == 0:
            leak = 0
        else:
            pts = pts[np.isfinite(pts).all(axis=1)] @ quat_matrix(t.rotation).T + \
                np.array([t.translation.x, t.translation.y, t.translation.z])
            inside = np.ones(len(pts), bool)
            for i, axis in enumerate("xyz"):
                lo, hi = VOLUME[axis]
                inside &= (pts[:, i] > lo) & (pts[:, i] < hi)
            for cx, cy in CUBE_PLACES:
                inside &= np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) > CUBE_XY_RADIUS
            leak = int(np.sum(inside))
            parts = by_part(pts[inside])
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        writer.writerow([f"{time.time():.3f}", f"{stamp:.3f}", len(pts), leak, f"{speed['max']:.3f}"] +
                        [parts[p] for p in PARTS])
        out.flush()
        speed["max"] = 0.0  # the largest speed since the previous cloud

    node.create_subscription(PointCloud2, "/overhead_lidar/filtered_cloud", on_cloud, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()


if __name__ == "__main__":
    main()
