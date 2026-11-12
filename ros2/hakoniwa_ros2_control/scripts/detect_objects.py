#!/usr/bin/env python3
"""Find objects on the table in the point cloud.

ros2 run hakoniwa_ros2_control detect_objects.py [--register]

Takes the LiDAR cloud as it is, puts it in base_link, keeps the points above
the table inside the workspace, groups
neighbouring points (2 cm grid) and reports each group as a box: centre,
size (from its spread along its main axes) and yaw. The top of the box is the
highest point; its bottom is the table. With --register, the boxes go into
MoveIt's planning scene as object_0, object_1, ...

Move the arm out of the workspace first (for the SO-101 overhead LiDAR: the
shoulder pan at -1.5 rad), so that it neither hides objects nor appears among
them. MoveIt's self-filtered cloud is not used: it also drops the points within
2 cm of every known object, the table included, which cuts off low objects.
"""
import argparse
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def transform_to_base(cloud, buffer, node):
    end = time.time() + 5.0
    while not buffer.can_transform("base_link", cloud.header.frame_id, rclpy.time.Time()) and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    t = buffer.lookup_transform("base_link", cloud.header.frame_id, rclpy.time.Time()).transform
    q = t.rotation
    rotation = np.array([
        [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
        [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)],
        [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)],
    ])
    points = np.array([[p[0], p[1], p[2]] for p in
                       point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)])
    points = points[np.isfinite(points).all(axis=1)]
    return points @ rotation.T + np.array([t.translation.x, t.translation.y, t.translation.z])


def groups(points, cell):
    """Connected groups of points whose grid cells touch (8 neighbours in xy, 3 in z)."""
    cells = {}
    for index, key in enumerate(map(tuple, np.floor(points / cell).astype(int))):
        cells.setdefault(key, []).append(index)
    seen, result = set(), []
    for start in cells:
        if start in seen:
            continue
        stack, members = [start], []
        seen.add(start)
        while stack:
            key = stack.pop()
            members.extend(cells[key])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        other = (key[0] + dx, key[1] + dy, key[2] + dz)
                        if other in cells and other not in seen:
                            seen.add(other)
                            stack.append(other)
        result.append(points[members])
    return result


def box(points):
    """Centre, size and yaw of a box standing on the table (z = 0) around the points."""
    xy = points[:, :2]
    centre_xy = xy.mean(axis=0)
    yaw = 0.0
    if len(points) >= 5:
        eigenvalues, eigenvectors = np.linalg.eigh(np.cov((xy - centre_xy).T))
        main = eigenvectors[:, int(np.argmax(eigenvalues))]
        yaw = math.atan2(main[1], main[0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    local = (xy - centre_xy) @ np.array([[c, -s], [s, c]]).T
    low, high = local.min(axis=0), local.max(axis=0)
    # The middle of the extent, not the mean: the LiDAR sees one side more densely.
    middle = (low + high) / 2.0
    c, s = math.cos(yaw), math.sin(yaw)
    centre_xy = centre_xy + np.array([[c, -s], [s, c]]) @ middle
    top = float(points[:, 2].max())
    size = [float(high[0] - low[0]), float(high[1] - low[1]), top]
    # Fold the yaw of a box into (-45, 45] degrees: a box looks the same every 90 degrees.
    yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return np.array([centre_xy[0], centre_xy[1], top / 2.0]), size, yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cloud-topic", default="/pdu/lidar_scan")
    parser.add_argument("--workspace", nargs=4, type=float, default=[0.05, -0.35, 0.45, 0.3],
                        metavar=("XMIN", "YMIN", "XMAX", "YMAX"))
    parser.add_argument("--base-radius", type=float, default=0.12,
                        help="points this close to the robot base (base_link z axis) are the robot itself [m]")
    parser.add_argument("--min-height", type=float, default=0.005, help="above the table [m]")
    parser.add_argument("--max-height", type=float, default=0.15)
    parser.add_argument("--min-points", type=int, default=8)
    parser.add_argument("--cell", type=float, default=0.02)
    parser.add_argument("--register", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("hakoniwa_detect_objects")
    buffer = Buffer()
    TransformListener(buffer, node)
    clouds = []
    node.create_subscription(PointCloud2, args.cloud_topic, clouds.append, qos_profile_sensor_data)
    end = time.time() + 10.0
    while len(clouds) < 2 and time.time() < end:   # the second one: after this node has the TF
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(clouds) < 2:
        print(f"no cloud on {args.cloud_topic}", file=sys.stderr)
        return 1
    points = transform_to_base(clouds[-1], buffer, node)
    x0, y0, x1, y1 = args.workspace
    keep = (points[:, 0] > x0) & (points[:, 0] < x1) & (points[:, 1] > y0) & (points[:, 1] < y1) & \
        (points[:, 2] > args.min_height) & (points[:, 2] < args.max_height) &         (np.hypot(points[:, 0], points[:, 1]) > args.base_radius)
    found = [g for g in groups(points[keep], args.cell) if len(g) >= args.min_points]
    print(f"cloud {len(points)} points, {int(keep.sum())} above the table in the workspace, {len(found)} objects")
    for index, group in enumerate(sorted(found, key=len, reverse=True)):
        centre, size, yaw = box(group)
        print(f"object_{index}: {len(group)} points, centre {np.round(centre, 3).tolist()}, "
              f"size {np.round(size, 3).tolist()}, yaw {math.degrees(yaw):.1f} deg")
        if args.register:
            subprocess.run(["ros2", "run", "hakoniwa_ros2_control", "add_grasp_object.py", "--id", f"object_{index}",
                            "--position", *[f"{v:.4f}" for v in centre],
                            "--size", *[f"{v:.4f}" for v in size]], check=False)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
