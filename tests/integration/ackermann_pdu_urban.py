"""Verify deterministic PDU-only startup and state bytes with the Urban car plant."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET

from person_step_urban import MANIFEST, wait_for


DELTA_USEC = 1_000
STEPS = 80
CAR = "Car-1"
STATE_ROBOT = "UrbanFleet"


def prepare(urban, directory):
    sys.path.insert(0, str(urban / "tools"))
    import multi_car
    from urban_simulation import publish_every_step, unnotify

    vehicle, = multi_car.resolve_config(urban / "recipes/experiments/urban-car-one.yaml")["vehicles"]
    model = directory / "urban-car.xml"
    multi_car.materialize_vehicle_fleet_model(model, [vehicle])
    tree = ET.parse(model)
    tree.getroot().find("option").set("timestep", "0.001")
    world = tree.getroot().find("worldbody")
    for body in world.findall("body"):
        x, y, z = map(float, body.get("pos").split())
        body.set("pos", f"{x} {y} {z - multi_car.PARKING_HEIGHT_M:.6f}")
    ET.SubElement(world, "geom", name="test-ground", type="plane", size="20 20 0.1")
    tree.write(model)
    multi_car.materialize_runtime(model, directory, [vehicle])
    unnotify(directory / "urban-car-comm.json", CAR, "ackermann_cmd")
    publish_every_step(directory / "urban-fleet-multi-dof-state.json", DELTA_USEC / 1_000)
    assert "services" not in json.loads((directory / MANIFEST).read_text())


def client(config, output):
    import hakopy
    from hakoniwa_pdu.pdu_msgs.ackermann_msgs.pdu_conv_AckermannDrive import py_to_pdu_AckermannDrive
    from hakoniwa_pdu.pdu_msgs.ackermann_msgs.pdu_pytype_AckermannDrive import AckermannDrive
    from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_MultiDOFJointState import pdu_to_py_MultiDOFJointState

    definition = json.loads(config.read_text())
    paths = {item["id"]: Path(item["path"]) for item in definition["paths"]}
    robots = {item["name"]: item["pdutypes_id"] for item in definition["robots"]}

    def channel(robot, name):
        return next(item for item in json.loads(paths[robots[robot]].read_text()) if item["name"] == name)

    drive_channel = channel(CAR, "ackermann_cmd")["channel_id"]
    state = channel(STATE_ROBOT, "vehicle_states")
    errors = []

    def write_drive(step):
        command = AckermannDrive()
        command.speed = 1.0 + step / 100.0
        command.steering_angle = 0.1
        wire = bytearray(py_to_pdu_AckermannDrive(command))
        assert hakopy.pdu_write(CAR, drive_channel, wire, len(wire))

    def initialize(_context):
        hakopy.register_data_recv_event(STATE_ROBOT, state["channel_id"], None)
        write_drive(0)
        return 0

    def run(_context):
        try:
            with output.open("wb") as trace:
                for step in range(STEPS):
                    assert hakopy.usleep(DELTA_USEC)
                    deadline = time.monotonic() + 8.0
                    while not hakopy.check_data_recv_event(STATE_ROBOT, state["channel_id"]):
                        if time.monotonic() >= deadline:
                            raise RuntimeError(f"no vehicle state for PDU step {step}")
                    raw = hakopy.pdu_read(STATE_ROBOT, state["channel_id"], state["pdu_size"])
                    result = pdu_to_py_MultiDOFJointState(raw)
                    stamp = result.header.stamp
                    assert stamp.sec * 1_000_000_000 + stamp.nanosec == (step + 1) * DELTA_USEC * 1_000
                    assert result.joint_names == [CAR], result.joint_names
                    size = struct.unpack_from("<I", raw, 16)[0]
                    assert 24 < size <= len(raw)
                    trace.write(raw[:size])
                    write_drive(step + 1)
                velocity = result.twist[0].linear
                assert math.hypot(velocity.x, velocity.y) > 0.01
            print(f"DONE {STEPS} PDU steps", flush=True)
            while hakopy.usleep(DELTA_USEC):
                pass
        except Exception as error:
            errors.append(error)
            traceback.print_exc()
            return 1
        return 0

    assert hakopy.asset_register("UrbanPduClient", str(config), {
        "on_initialize": initialize, "on_simulation_step": None,
        "on_manual_timing_control": run, "on_reset": lambda _: 0,
    }, DELTA_USEC, hakopy.HAKO_ASSET_MODEL_PLANT)
    print("CLIENT registered", flush=True)
    hakopy.start()
    return bool(errors)


def record(build, foundation, car, binary, run):
    run.mkdir()
    mmap = run / "mmap"
    mmap.mkdir()
    core_config = run / "core.json"
    core_config.write_text(json.dumps({"shm_type": "mmap", "core_mmap_path": str(mmap)}))
    env = dict(os.environ, HAKO_CONFIG_PATH=str(core_config))
    for name in ("DYLD_LIBRARY_PATH", "LD_LIBRARY_PATH"):
        env[name] = os.pathsep.join(filter(None, (str(foundation / "lib"), env.get(name))))
    processes = []
    started = False
    hako_cmd = foundation / "bin/hako-cmd"

    def launch(name, command, marker):
        log = run / f"{name}.log"
        with log.open("wb") as stream:
            process = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT)
        processes.append(process)
        wait_for(log, marker, process)
        return process, log

    def core(command):
        with (run / f"core-{command}.log").open("wb") as stream:
            subprocess.run([hako_cmd, command], env=env, stdout=stream,
                           stderr=subprocess.STDOUT, check=True, timeout=10)

    try:
        launch("conductor", [build / "hako-async-rpc-integration-conductor"], "CONDUCTOR started")
        _, server_log = launch("server", [
            binary, "--manifest", car / MANIFEST, "--no-viewer", "--external-conductor",
            "--realtime-sync-cycle-msec", "0", "--lockstep-input", f"{CAR}/ackermann_cmd",
        ], "Starting Hakoniwa Runner:")
        trace = run / "vehicle-states.bin"
        worker, log = launch("client", [
            sys.executable, __file__, "--client", car / "urban-car-pdudef.json", trace,
        ], "CLIENT registered")
        core("start")
        started = True
        wait_for(log, f"DONE {STEPS} PDU steps", worker)
        assert "HakoServiceServer::initialize" not in server_log.read_text(errors="replace")
        return trace.read_bytes()
    finally:
        try:
            if started:
                core("stop")
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    parser.add_argument("foundation", type=Path)
    parser.add_argument("urban", type=Path)
    parser.add_argument("--baseline", type=Path, help="previous Urban car plant binary to compare")
    args = parser.parse_args()
    work = Path(tempfile.mkdtemp(prefix="hako-ackermann-pdu-urban."))
    print(f"Integration logs: {work}", flush=True)
    os.environ.update(
        HAKONIWA_WORKSPACE_ACTIVE="1",
        HAKONIWA_WORKSPACE_ROOT=os.environ.get("HAKONIWA_WORKSPACE_ROOT")
        or str(args.urban.parent / "hakoniwa-business-pack"),
        HAKONIWA_WORK_DIR=os.environ.get("HAKONIWA_WORK_DIR") or str(args.foundation.parent.parent),
        HAKONIWA_FOUNDATION_PREFIX=str(args.foundation),
        PYTHONDONTWRITEBYTECODE="1",
    )
    car = work / "car"
    car.mkdir()
    prepare(args.urban, car)
    current = args.build / "bin/urban-car-hakoniwa-asset"
    left = record(args.build, args.foundation, car, args.baseline or current, work / "first")
    right = record(args.build, args.foundation, car, current, work / "second")
    assert left and left == right, "PDU state packets differ"
    print(f"{STEPS} PDU steps, {len(right)} bytes, SHA-256 {hashlib.sha256(right).hexdigest()}")


if __name__ == "__main__":
    if sys.argv[1] == "--client":
        raise SystemExit(client(*map(Path, sys.argv[2:])))
    main()

