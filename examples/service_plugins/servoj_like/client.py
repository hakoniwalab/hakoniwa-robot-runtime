"""Check the SO-101 ServoJ, Stop and GetAngle services against JointState."""

import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

from hakoniwa_pdu_rpc import RpcClient, RpcServiceResultCode, TypedRpcServiceError, make_typed_client
from pdu.python.binary_io import PduMetaData
from pdu.python.sensor_msgs.pdu_conv_JointState import pdu_to_py_JointState


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
MOTIONS = [(0, 0.5), (0, -0.5), (0, 0.0), (2, 0.8), (2, 0.0)]
DURATION = 1.5  # seconds of simulation time per motion
TOLERANCE = 0.01  # radians
HOLD_TOLERANCE = 0.005  # radians: braking overshoot right after Stop
RPC_TIMEOUT = 2_000_000  # microseconds
PDU_HEADER = struct.pack(
    "<II", PduMetaData.PDU_META_DATA_MAGICNO, PduMetaData.PDU_META_DATA_VERSION
)


def read_joint_position(hako_cmd, channel, joint):
    packet = subprocess.check_output(
        [hako_cmd, "dump", str(channel)], stderr=subprocess.PIPE, timeout=2
    )
    offset = packet.find(PDU_HEADER)
    if offset < 0:
        return None
    state = pdu_to_py_JointState(bytearray(packet[offset:]))
    return state.position[state.name.index(joint)]


def wait_for_goal(hako_cmd, channel, joint, goal):
    deadline = time.monotonic() + 5
    position = None
    settled_since = None
    while time.monotonic() < deadline:
        sample = read_joint_position(hako_cmd, channel, joint)
        if sample is not None:
            position = sample
            now = time.monotonic()
            if abs(position - goal) <= TOLERANCE:
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= 0.2:
                    return position
            else:
                settled_since = None
        time.sleep(0.05)
    raise TimeoutError(f"{joint} did not settle at {goal:+.2f} rad (last {position})")


def call_servoj(servoj, pose, index, goal, duration):
    request = servoj.create_request()
    request.joint = list(pose)
    request.joint[index] = goal
    request.t = duration
    reply = servoj.call(request, timeout_usec=RPC_TIMEOUT)
    if reply.res != 0:
        raise RuntimeError(f"ServoJ returned res={reply.res}")
    return reply


def wait_for_sim_time(get_angle, target_ns):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        reply = get_angle()
        if reply.sim_time_ns >= target_ns:
            return reply
        time.sleep(0.02)
    raise TimeoutError(f"Simulation clock did not reach {target_ns} ns")


def check_stop_and_get_angle(servoj, stop, angle, hako_cmd, channel):
    index = 0
    joint = JOINTS[index]
    angle_times = []

    def get_angle():
        request = angle.create_request()
        request.unused = 0
        reply = angle.call(request, timeout_usec=RPC_TIMEOUT)
        angle_times.append(reply.sim_time_ns)
        return reply

    pose_a = get_angle()
    call_servoj(servoj, pose_a.joint, index, 1.0, 2.0)
    started = get_angle()
    before_stop = wait_for_sim_time(get_angle, started.sim_time_ns + 500_000_000)
    request = stop.create_request()
    request.mode = 0
    reply = stop.call(request, timeout_usec=RPC_TIMEOUT)
    if reply.res != 0:
        raise RuntimeError(f"S1 Stop returned res={reply.res}")
    first = get_angle()
    elapsed = (before_stop.sim_time_ns - started.sim_time_ns) * 1e-9
    print(f"S1 Stop res={reply.res} after {elapsed:.3f} s of simulation time", flush=True)

    second = wait_for_sim_time(get_angle, first.sim_time_ns + 300_000_000)
    joint_state = read_joint_position(hako_cmd, channel, joint)
    first_angle = first.joint[index]
    held_angle = second.joint[index]
    a = pose_a.joint[index]
    drift = abs(held_angle - first_angle)
    seconds_held = (second.sim_time_ns - first.sim_time_ns) * 1e-9
    held = f"{joint}={held_angle:+.6f} rad (first={first_angle:+.6f}, A={a:+.6f}, goal=+1.0)"
    # The joint is still moving at 0.5 rad/s when Stop is accepted; the position actuator
    # overshoots the held angle a little (about 0.002 rad measured) before it settles.
    if not (a < first_angle < 1.0 and a < held_angle < 1.0 and drift <= HOLD_TOLERANCE):
        raise RuntimeError(
            f"S2 hold check failed: {held}, drift={drift:.6f} rad over {seconds_held:.3f} s, JointState={joint_state}"
        )
    print(f"S2 held {held}, {seconds_held:.3f} s apart, braking drift={drift:.6f} rad", flush=True)

    if not (joint_state is not None and abs(joint_state - held_angle) <= TOLERANCE):
        raise RuntimeError(f"S3 {joint} JointState={joint_state}, GetAngle={held_angle:+.6f} rad")
    print(f"S3 {joint} JointState={joint_state:+.6f} rad, GetAngle={held_angle:+.6f} rad", flush=True)

    call_servoj(servoj, second.joint, index, 0.0, 1.0)
    resumed = get_angle()
    restart_angle = resumed.joint[index]
    from_hold = abs(restart_angle - held_angle)
    if not (from_hold <= TOLERANCE and from_hold < abs(restart_angle - a)):
        raise RuntimeError(f"S4 restart={restart_angle:+.6f} rad, held={held_angle:+.6f}, A={a:+.6f}")
    reached = wait_for_goal(hako_cmd, channel, joint, 0.0)
    print(f"S4 {joint} restarted at {restart_angle:+.6f} rad and reached {reached:+.6f} rad", flush=True)

    request.mode = 1
    try:
        stop.call(request, timeout_usec=RPC_TIMEOUT)
    except TypedRpcServiceError as error:
        if error.result_code != RpcServiceResultCode.INVALID:
            raise RuntimeError(f"S5 expected INVALID for Stop mode=1, got result_code={error.result_code}") from error
    else:
        raise RuntimeError("S5 Stop mode=1 was accepted")
    print("S5 Stop mode=1 returned INVALID", flush=True)

    for earlier, later in zip(angle_times, angle_times[1:]):
        if later <= earlier:
            raise RuntimeError(f"S6 GetAngle time did not increase: {earlier} -> {later} ns")
    print(f"S6 sim_time_ns increased across {len(angle_times)} GetAngle replies", flush=True)


def main(run_dir, pdu_types):
    hako_cmd = Path(os.environ["PREFIX"]) / "bin" / "hako-cmd"
    with RpcClient(
        library_path=os.environ["HAKO_PDU_RPC_LIBRARY"],
        node_id="servoj-python",
        client_name="ServoJClient",
        service_config_path=str(run_dir / "service.json"),
        endpoint_config_path=None,
    ) as client:
        servoj, stop, angle = (
            make_typed_client(
                client, service_name=f"ArmService/{kind}/SO101",
                service_type=f"my_robot_srvs_msgs/{kind}", package="servoj_pdu.my_robot_srvs_msgs",
            )
            for kind in ("ServoJ", "Stop", "GetAngle")
        )
        client.start()
        deadline = time.monotonic() + 20
        while not (run_dir / "start-go").exists():
            if time.monotonic() >= deadline:
                raise TimeoutError("Core did not start")
            time.sleep(0.05)

        logical_channel = next(
            pdu["channel_id"] for pdu in json.loads(pdu_types.read_text())
            if pdu["name"] == "joint_states"
        )
        metadata = json.loads((run_dir / "registered.json").read_text())["master_data"]
        channel = next(
            pdu["real_cid"] for pdu in metadata["pdu_meta_data"]["channel_map"]
            if pdu["robo_name"] == "SO101" and pdu["logical_channel_id"] == logical_channel
        )

        for index, goal in MOTIONS:
            # ServoJ takes a full pose in JOINTS order.
            reply = call_servoj(servoj, [0.0] * len(JOINTS), index, goal, DURATION)
            # The RPC acknowledges immediately. The plugin moves on subsequent ticks.
            time.sleep(DURATION)
            joint = JOINTS[index]
            reached = wait_for_goal(hako_cmd, channel, joint, goal)
            print(f"res={reply.res} {joint} reached={reached:+.6f} rad (goal={goal:+.2f})", flush=True)

        check_stop_and_get_angle(servoj, stop, angle, hako_cmd, channel)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
