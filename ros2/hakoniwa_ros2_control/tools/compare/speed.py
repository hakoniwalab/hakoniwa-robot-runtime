#!/usr/bin/env python3
"""Simulation-time progress over a fixed wall window while the arm moves (the
9 s trajectory sent again and again), and the CPU of the whole container in it."""
import os, sys, time, rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectoryPoint
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); from measure import cpu_seconds, ARM, POINTS
WINDOW = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
rclpy.init(); node = rclpy.create_node("hakoniwa_compare_speed"); clock = [0.0]
node.create_subscription(Clock, "/clock", lambda m: clock.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9),
                         QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"); client.wait_for_server(30.0)
goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = ARM
goal.trajectory.points = [JointTrajectoryPoint(positions=p, time_from_start=Duration(sec=t)) for p, t in POINTS]
while clock[0] == 0.0: rclpy.spin_once(node, timeout_sec=0.05)
sim0, wall0, cpu0 = clock[0], time.time(), cpu_seconds(); pending = None
while time.time() - wall0 < WINDOW:
    if pending is None or pending.done():
        pending = client.send_goal_async(goal)
    rclpy.spin_once(node, timeout_sec=0.01)
sim1, wall1, cpu1 = clock[0], time.time(), cpu_seconds()
print(f"{sys.argv[1]}: sim {sim1 - sim0:.2f} s in wall {wall1 - wall0:.2f} s -> ratio {(sim1 - sim0) / (wall1 - wall0):.1f}, CPU per sim s {(cpu1 - cpu0) / (sim1 - sim0):.3f}")
