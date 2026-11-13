#!/usr/bin/env python3
"""Pick an object and place it elsewhere, with MoveIt and the trajectory controllers.

ros2 run hakoniwa_ros2_control pick_and_place.py [--config pick_and_place.yaml]

The configuration (config/so101/pick_and_place.yaml, written by
ros2/hakoniwa_ros2_control/tools/so101_grasp_poses.py) gives the object, the
table and the arm joints of the four poses. MoveIt moves the arm between
places, avoiding the obstacle map; the short moves down and up go straight to
arm_controller. The wrist force-torque sensor checks the grasp (the object's
weight) and the release. Every step must succeed, or the run stops there.
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRAVITY = 9.80665
CONTACT_N = 2.0           # an unexpected contact while moving straight
WEIGHT_TOLERANCE_N = 0.1  # how close the force change must be to the object's weight


def duration(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


class PickAndPlace:
    def __init__(self, config, object_mass_kg):
        self.config = config
        self.weight = object_mass_kg * GRAVITY
        self.node = rclpy.create_node("hakoniwa_pick_and_place")
        self.wrenches = []
        self.joints = {}
        self.node.create_subscription(WrenchStamped, "/pdu/wrist_wrench", self.wrenches.append,
                                      qos_profile_sensor_data)
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.joints.update(zip(m.name, m.position)), 10)
        self.arm = ActionClient(self.node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper_topic = self.node.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.clear_octomap = self.node.create_client(Empty, "/clear_octomap")
        self.tf = Buffer()
        TransformListener(self.tf, self.node)
        if not self.arm.wait_for_server(timeout_sec=10.0):
            raise SystemExit("arm_controller is not running")

    # --- small helpers -------------------------------------------------------

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def force(self, samples=25):
        """Mean force [N] of the wrist sensor, in its own axes."""
        del self.wrenches[:]
        while len(self.wrenches) < samples:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return np.mean([[w.wrench.force.x, w.wrench.force.y, w.wrench.force.z] for w in self.wrenches], axis=0)

    def tool(self, *args):
        """Run another script of this package; True when it reports success."""
        result = subprocess.run(["ros2", "run", "hakoniwa_ros2_control", *args], capture_output=True, text=True)
        return "OK" in result.stdout or "SUCCESS" in result.stdout

    def moveit(self, joints):
        # The map keeps points of the arm where the LiDAR (5 Hz) last saw it
        # moving: start from a fresh map.
        if self.clear_octomap.wait_for_service(timeout_sec=5.0):
            future = self.clear_octomap.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self.node, future)
        self.spin(1.5)
        return self.tool("move_to_joints.py", "--group", "arm", "--joints",
                         *[f"{name}={value}" for name, value in zip(ARM, joints)])

    def straight(self, joints, seconds, watch=None):
        """Move the arm straight to joints; with watch, stop on a force change above CONTACT_N."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        goal.trajectory.points = [JointTrajectoryPoint(positions=list(joints), time_from_start=duration(seconds))]
        sent = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        while not result.done():
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if watch is not None and self.wrenches:
                f = self.wrenches[-1].wrench.force
                change = float(np.linalg.norm(np.array([f.x, f.y, f.z]) - watch))
                if change > CONTACT_N:
                    print(f"   contact: the force changed by {change:.2f} N")
                    cancel = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self.node, cancel)
                    return False
        return result.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def gripper(self, position, seconds=1.5):
        # Sent as a topic: closing on an object never reaches the target, which
        # an action goal would report as a failure.
        trajectory = JointTrajectory(joint_names=["gripper"])
        trajectory.points = [JointTrajectoryPoint(positions=[position], time_from_start=duration(seconds))]
        self.gripper_topic.publish(trajectory)
        self.spin(seconds + 1.0)
        return self.joints.get("gripper", float("nan"))

    def transform(self, target, source, timeout=10.0):
        """The latest transform source -> target, waiting until TF has it (None after timeout)."""
        end = time.time() + timeout
        while not self.tf.can_transform(target, source, rclpy.time.Time()):
            if time.time() > end:
                print(f"   no transform {source} -> {target} after {timeout:.0f} s")
                return None
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.tf.lookup_transform(target, source, rclpy.time.Time()).transform

    def points_near(self, xy, radius=0.025):
        """LiDAR points at object height (5-50 mm) near xy, in base_link."""
        clouds = []
        subscription = self.node.create_subscription(PointCloud2, "/pdu/lidar_scan", clouds.append,
                                                     qos_profile_sensor_data)
        end = time.time() + 5.0
        while len(clouds) < 2 and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(subscription)
        if not clouds:
            return None
        cloud = clouds[-1]
        t = self.transform("base_link", cloud.header.frame_id)
        if t is None:
            return None
        q = t.rotation
        rotation = np.array([
            [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
            [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)],
            [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x * q.x + q.y * q.y)],
        ])
        points = np.array([[p[0], p[1], p[2]] for p in
                           point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)])
        points = points[np.isfinite(points).all(axis=1)] @ rotation.T + \
            np.array([t.translation.x, t.translation.y, t.translation.z])
        near = (np.abs(points[:, 0] - xy[0]) < radius) & (np.abs(points[:, 1] - xy[1]) < radius) & \
            (points[:, 2] > 0.005) & (points[:, 2] < 0.05)
        return int(np.sum(near))

    # --- the run -------------------------------------------------------------

    def run(self):
        c = self.config
        obj, table, joints, grip = c["object"], c["table"], c["joints"], c["gripper"]
        position = lambda xyz: [str(v) for v in xyz]

        def step(label, ok, detail=""):
            print(f"{label}: {'OK' if ok else 'FAILED'} {detail}".rstrip(), flush=True)
            if not ok:
                raise SystemExit(1)

        self.spin(1.0)
        print(f"LiDAR points at the pick place {self.points_near(obj['pick'])}, "
              f"at the place place {self.points_near(obj['place'])}")
        step("1. register the table and the object",
             self.tool("add_grasp_object.py", "--id", table["id"], "--position", *position(table["position"]),
                       "--size", *position(table["size"]))
             and self.tool("add_grasp_object.py", "--id", obj["id"], "--position", *position(obj["pick"]),
                           "--size", *position(obj["size"])))
        step("2. open the gripper", self.gripper(grip["open"]) > grip["open"] - 0.1)
        step("3. pre-grasp (MoveIt)", self.moveit(joints["pre_grasp"]))
        self.spin(1.0)
        empty = self.force()
        step("4. down to the object", self.straight(joints["grasp"], 3.0, watch=empty))
        closed = self.gripper(grip["close"])
        step("5. close", closed > grip["closed_on_object_min"], f"(gripper {closed:.3f} rad)")
        step("6. attach the object to the hand",
             self.tool("add_grasp_object.py", "--attach", "--id", obj["id"], "--position", *position(obj["pick"]),
                       "--size", *position(obj["size"])))
        step("7. lift", self.straight(joints["pre_grasp"], 2.0))
        self.spin(1.0)
        change = float(np.linalg.norm(self.force() - empty))
        step("   holding", abs(change - self.weight) < WEIGHT_TOLERANCE_N,
             f"(force change {change:.3f} N, object weight {self.weight:.3f} N)")
        step("8. carry to the pre-place pose (MoveIt)", self.moveit(joints["pre_place"]))
        self.spin(1.0)
        step("9. put down", self.straight(joints["place"], 3.0, watch=self.force()))
        step("10. open", self.gripper(grip["open"]) > grip["open"] - 0.1)
        step("11. detach the object (placed)",
             self.tool("add_grasp_object.py", "--detach", "--id", obj["id"], "--position", *position(obj["place"]),
                       "--size", *position(obj["size"])))
        step("12. up", self.straight(joints["pre_place"], 2.0))
        self.spin(1.0)
        change = float(np.linalg.norm(self.force() - empty))
        step("   released", change < WEIGHT_TOLERANCE_N, f"(force change from before the grasp {change:.3f} N)")
        step("13. home (MoveIt)", self.moveit(joints["home"]))
        self.spin(1.5)
        print(f"LiDAR points at the pick place {self.points_near(obj['pick'])}, "
              f"at the place place {self.points_near(obj['place'])}")


def main():
    default = os.path.join(get_package_share_directory("hakoniwa_ros2_control"), "config", "so101", "pick_and_place.yaml")
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=default)
    parser.add_argument("--object-mass", type=float, default=0.03, help="kg (the grasp check expects its weight)")
    args = parser.parse_args()
    with open(args.config, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    rclpy.init()
    try:
        PickAndPlace(config, args.object_mass).run()
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
