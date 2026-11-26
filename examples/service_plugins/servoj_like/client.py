"""Send five ServoJ-like goals and confirm them from the SO-101 JointState PDU."""

import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

from hakoniwa_pdu_rpc import RpcClient, make_typed_client
from pdu.python.binary_io import PduMetaData
from pdu.python.sensor_msgs.pdu_conv_JointState import pdu_to_py_JointState


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
MOTIONS = [(0, 0.5), (0, -0.5), (0, 0.0), (2, 0.8), (2, 0.0)]
DURATION = 1.5  # seconds of simulation time per motion
TOLERANCE = 0.01  # radians
PDU_HEADER = struct.pack(
    "<II", PduMetaData.PDU_META_DATA_MAGICNO, PduMetaData.PDU_META_DATA_VERSION
)


def wait_for_goal(hako_cmd, channel, joint, goal):
    deadline = time.monotonic() + 5
    position = None
    settled_since = None
    while time.monotonic() < deadline:
        packet = subprocess.check_output(
            [hako_cmd, "dump", str(channel)], stderr=subprocess.PIPE, timeout=2
        )
        offset = packet.find(PDU_HEADER)
        if offset >= 0:
            state = pdu_to_py_JointState(bytearray(packet[offset:]))
            position = state.position[state.name.index(joint)]
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


def main(run_dir, pdu_types):
    hako_cmd = Path(os.environ["PREFIX"]) / "bin" / "hako-cmd"
    with RpcClient(
        library_path=os.environ["HAKO_PDU_RPC_LIBRARY"],
        node_id="servoj-python",
        client_name="ServoJClient",
        service_config_path=str(run_dir / "service.json"),
        endpoint_config_path=None,
    ) as client:
        servoj = make_typed_client(
            client, service_name="ArmService/ServoJ/SO101",
            service_type="my_robot_srvs_msgs/ServoJ", package="servoj_pdu.my_robot_srvs_msgs",
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
            request = servoj.create_request()
            request.joint = [0.0] * len(JOINTS)  # ServoJ takes a full pose in JOINTS order.
            request.joint[index] = goal
            request.t = DURATION
            reply = servoj.call(request, timeout_usec=2_000_000)
            if reply.res != 0:
                raise RuntimeError(f"ServoJ returned res={reply.res}")
            # The RPC acknowledges immediately. The plugin moves on subsequent ticks.
            time.sleep(DURATION)
            joint = JOINTS[index]
            reached = wait_for_goal(hako_cmd, channel, joint, goal)
            print(f"res={reply.res} {joint} reached={reached:+.6f} rad (goal={goal:+.2f})", flush=True)


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
