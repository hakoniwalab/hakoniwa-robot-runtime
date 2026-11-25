#!/usr/bin/env python3
"""Send the existing descent goal and record 27 seconds at its ROS simulation timestamp."""

import argparse
import json
import os
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rosidl_runtime_py.set_message import set_message_fields


NS = 1_000_000_000
STEP_NS = 1_000_000
REQUIRED = ("joint_state_broadcaster", "arm_controller", "wrist_ft_broadcaster")
STATUS = {getattr(GoalStatus, name): name.removeprefix("STATUS_") for name in dir(GoalStatus)
          if name.startswith("STATUS_")}


def stamp_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def wait(node, condition, timeout, reason):
    deadline = time.monotonic() + timeout
    while not condition():
        if time.monotonic() >= deadline:
            raise TimeoutError(reason)
        rclpy.spin_once(node, timeout_sec=0.1)


def controller_states(node, client):
    future = client.call_async(ListControllers.Request())
    wait(node, future.done, 10, "list_controllers did not respond")
    return {item.name: item.state for item in future.result().controller}


def record(node, goal, duration):
    wrench, states = {}, {}
    last_wrench = last_state = 0

    def add_wrench(message):
        nonlocal last_wrench
        force, torque = message.wrench.force, message.wrench.torque
        stamp = stamp_ns(message.header.stamp)
        wrench[stamp] = (force.x, force.y, force.z, torque.x, torque.y, torque.z)
        last_wrench = max(last_wrench, stamp)

    def add_state(message):
        nonlocal last_state
        stamp = stamp_ns(message.header.stamp)
        states[stamp] = (tuple(message.feedback.positions), tuple(message.error.positions))
        last_state = max(last_state, stamp)

    node.create_subscription(WrenchStamped, "/wrist_ft_broadcaster/wrench", add_wrench, 4096)
    node.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state", add_state, 4096)
    client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    list_client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not client.wait_for_server(timeout_sec=60) or not list_client.wait_for_service(timeout_sec=10):
        raise TimeoutError("Gazebo controller action or list_controllers is unavailable")
    wait(node, lambda: wrench and states, 30, "no wrench or trajectory state received")
    initial = controller_states(node, list_client)
    if any(initial.get(name) != "active" for name in REQUIRED):
        raise RuntimeError(f"Controllers are not active: {initial}")

    # Schedule a goal and a sample window on the same simulation-time grid.
    start = (last_wrench // STEP_NS + 2_000) * STEP_NS
    end = start + round(duration * NS)
    goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = divmod(start, NS)
    future = client.send_goal_async(goal)
    wait(node, future.done, 15, "trajectory goal acknowledgement timed out")
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("trajectory goal was rejected")
    result = handle.get_result_async()
    wall_start = time.monotonic()
    wait(node, lambda: last_wrench >= end and last_state >= end and result.done(), 150,
         "27 seconds of simulation samples or trajectory result did not arrive")
    wait_wall_s = time.monotonic() - wall_start

    stamps = np.array(sorted(t for t in wrench if start <= t <= end), dtype=np.int64)
    if len(stamps) < 2:
        raise RuntimeError("fewer than two wrench samples inside the goal interval")
    values = np.array([wrench[t] for t in stamps])
    if not np.isfinite(values).all():
        raise ValueError("nonfinite wrench in the goal interval")
    steps, counts = np.unique(np.diff(stamps), return_counts=True)
    joint_states = [(t, actual, error) for t, (actual, error) in sorted(states.items())
                    if start <= t <= end and len(actual) == len(error) == len(goal.trajectory.joint_names)]
    if not joint_states:
        raise RuntimeError("no valid controller state in the goal interval")
    goal_error = np.asarray(joint_states[-1][1]) - goal.trajectory.points[-1].positions
    errors = np.abs([error for _, _, error in joint_states])
    state_stamps = np.array([t for t, _, _ in joint_states], dtype=np.int64)
    pre_contact = (state_stamps - start) < 18_500_000_000
    response = result.result()
    summary = {
        "controllers_before": initial, "controllers_after": controller_states(node, list_client),
        "action_status": STATUS.get(response.status, str(response.status)),
        "action_error_code": response.result.error_code, "action_error": response.result.error_string,
        "joint_names": list(goal.trajectory.joint_names),
        "max_position_error_rad": float(errors.max()),
        "max_position_error_before_18_5s_rad": float(errors[pre_contact].max()) if pre_contact.any() else None,
        "max_position_error_by_joint_rad": errors.max(axis=0).tolist(),
        "final_position_error_rad": np.abs(goal_error).tolist(),
        "controller_state_samples": len(joint_states), "goal_start_ns": start,
        "first_wrench_offset_ns": int(stamps[0] - start), "last_wrench_offset_ns": int(stamps[-1] - start),
        "wrench_samples": len(stamps), "wrench_span_s": float((stamps[-1] - stamps[0]) / NS),
        "wrench_step_counts_ns": {str(step): int(count) for step, count in zip(steps, counts)},
        "wall_wait_s_after_goal_acceptance": wait_wall_s,
    }
    return stamps / NS, values, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("goal", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) <= 0:
        parser.error("set ROS_DOMAIN_ID to a nonzero domain")
    summary_path = args.output.with_suffix(".json")
    if args.output.exists() or summary_path.exists():
        parser.error("output or its JSON summary already exists")
    goal = FollowJointTrajectory.Goal()
    set_message_fields(goal, json.loads(args.goal.read_text(encoding="utf-8")))
    rclpy.init()
    node = rclpy.create_node("iiwa14_gz_descent")
    try:
        t, w, summary = record(node, goal, 27)
        with args.output.open("xb") as stream:
            np.savez(stream, t=t, w=w)
        with summary_path.open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2)
            stream.write("\n")
        print(json.dumps(summary), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
