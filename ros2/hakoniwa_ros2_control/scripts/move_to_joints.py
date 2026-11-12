#!/usr/bin/env python3
"""Ask MoveIt (move_group) to plan and execute a joint goal for one group.

ros2 run hakoniwa_ros2_control move_to_joints.py --group arm \
    --joints shoulder_pan=0.5 shoulder_lift=-0.3 elbow_flex=0.4 wrist_flex=0.2 wrist_roll=0.0
"""
import argparse
import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient


def parse_joint(text):
    name, value = text.split("=", 1)
    return name, float(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--group", default="arm")
    parser.add_argument("--joints", nargs="+", type=parse_joint, required=True, metavar="NAME=RAD")
    parser.add_argument("--tolerance", type=float, default=0.01, help="joint goal tolerance [rad]")
    parser.add_argument("--plan-only", action="store_true", help="plan without executing")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("hakoniwa_move_to_joints")
    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=10.0):
        print("move_group (/move_action) is not running", file=sys.stderr)
        return 1

    goal = MoveGroup.Goal()
    goal.request.group_name = args.group
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    constraints = Constraints()
    for name, value in args.joints:
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = value
        joint.tolerance_above = args.tolerance
        joint.tolerance_below = args.tolerance
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    goal.request.goal_constraints.append(constraints)
    goal.planning_options.plan_only = args.plan_only

    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent)
    handle = sent.result()
    if not handle.accepted:
        print("the goal was rejected", file=sys.stderr)
        return 1
    done = handle.get_result_async()
    rclpy.spin_until_future_complete(node, done)
    result = done.result().result
    code = result.error_code.val
    points = len(result.planned_trajectory.joint_trajectory.points)
    print(f"MoveIt error code {code} ({'SUCCESS' if code == MoveItErrorCodes.SUCCESS else 'FAILED'}), "
          f"planned {points} points, planning time {result.planning_time:.3f} s")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if code == MoveItErrorCodes.SUCCESS else 1


if __name__ == "__main__":
    sys.exit(main())
