#!/usr/bin/env python3
"""One measured trajectory run, the same for Hakoniwa and Gazebo.

Sends the 9 s, three-point trajectory to arm_controller and records, while it
runs, the controller's own state (/arm_controller/controller_state: reference,
feedback, error), the simulation time (/clock) and the CPU time of every
process in the container. Writes a JSON with:
  result         the action result (SUCCEEDED / ...)
  sim_s, wall_s  how long the execution took in simulation and wall time
  ratio          sim_s / wall_s (real-time factor during the execution)
  cpu_s_per_sim_s  CPU seconds of all processes per simulated second
  error_max, error_mean  |reference - feedback| over the 5 arm joints [rad]
  samples        [sim time, feedback positions] for the repeatability check

python3 measure.py --label hakoniwa-rt --out /tmp/cmp/hakoniwa-rt-1.json
"""
import argparse
import json
import os
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
POINTS = [([0.5, 0.0, 0.0, 0.0, 0.0], 3), ([-0.5, 0.3, -0.3, 0.2, 0.5], 6), ([0.0] * 5, 9)]
TICKS = os.sysconf("SC_CLK_TCK")


def cpu_seconds() -> float:
    """utime + stime of every process (the whole container), in seconds."""
    total = 0
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as stream:
                fields = stream.read().rsplit(")", 1)[1].split()
            total += int(fields[11]) + int(fields[12])
        except (OSError, IndexError, ValueError):
            continue
    return total / TICKS


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("hakoniwa_compare_measure")
    states, clock = [], [0.0]
    node.create_subscription(JointTrajectoryControllerState, "/arm_controller/controller_state",
                             lambda m: states.append(m), 100)
    # Best effort: accepts both a reliable and a best-effort /clock publisher.
    clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Clock, "/clock", lambda m: clock.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9), clock_qos)
    client = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=30.0):
        raise SystemExit("arm_controller is not available")
    end = time.time() + 5.0
    while clock[0] == 0.0 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM
    goal.trajectory.points = [JointTrajectoryPoint(positions=p, time_from_start=Duration(sec=t)) for p, t in POINTS]
    del states[:]
    sim0, wall0, cpu0 = clock[0], time.time(), cpu_seconds()
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent)
    result_future = sent.result().get_result_async()
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.01)
    sim1, wall1, cpu1 = clock[0], time.time(), cpu_seconds()
    if states:  # the simulation time of the execution, from the controller's own stamps
        sim0 = states[0].header.stamp.sec + states[0].header.stamp.nanosec * 1e-9
        sim1 = states[-1].header.stamp.sec + states[-1].header.stamp.nanosec * 1e-9
    code = result_future.result().result.error_code
    errors, samples = [], []
    for m in states:
        index = [m.joint_names.index(j) for j in ARM]
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        reference = [m.reference.positions[i] for i in index]
        feedback = [m.feedback.positions[i] for i in index]
        errors.append(max(abs(r - f) for r, f in zip(reference, feedback)))
        samples.append([round(stamp - sim0, 4), [round(v, 6) for v in feedback], [round(v, 6) for v in reference]])
    report = {
        "label": args.label,
        "result": "SUCCEEDED" if code == FollowJointTrajectory.Result.SUCCESSFUL else f"error {code}",
        "sim_s": round(sim1 - sim0, 3),
        "wall_s": round(wall1 - wall0, 3),
        "ratio": round((sim1 - sim0) / (wall1 - wall0), 3),
        "cpu_s_per_sim_s": round((cpu1 - cpu0) / max(sim1 - sim0, 1e-9), 3),
        "error_max": round(max(errors), 4) if errors else None,
        "error_mean": round(sum(errors) / len(errors), 4) if errors else None,
        "state_samples": len(states),
        "samples": samples,
    }
    with open(args.out, "w") as stream:
        json.dump(report, stream)
    print({k: v for k, v in report.items() if k != "samples"})
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
