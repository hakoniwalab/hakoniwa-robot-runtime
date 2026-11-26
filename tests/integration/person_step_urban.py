"""Exercise one Urban car and one person on each PersonStep route."""

import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import traceback
from unittest.mock import patch
import xml.etree.ElementTree as ET


DELTA_USEC = 1_000
STEPS = 80
CAR = "Car-1"
PERSON = "Walker"
ASSET = "PersonStepUrbanClient"
RESPONSE_BODY_OFFSET = 272


def prepare(urban, work, mujoco_library):
    sys.path.insert(0, str(urban / "tools"))
    import multi_car
    import urban_assets
    import urban_composition
    import urban_people

    car = work / "config/car"
    car.mkdir(parents=True)
    vehicles = multi_car.resolve_config(urban / "recipes/experiments/urban-car-one.yaml")["vehicles"]
    vehicle, = vehicles
    vehicle.update(name=CAR, prefix="car_1_")
    model = car / "urban-car.xml"
    multi_car.materialize_vehicle_fleet_model(model, vehicles)
    tree = ET.parse(model)
    tree.getroot().find("option").set("timestep", "0.00025")
    world = tree.getroot().find("worldbody")
    for body in world.findall("body"):
        x, y, z = map(float, body.get("pos").split())
        body.set("pos", f"{x} {y} {z - multi_car.PARKING_HEIGHT_M:.6f}")
    ET.SubElement(world, "geom", name="test-ground", type="plane", size="20 20 0.1")
    # The walker starts close enough to record a repeatable contact edge.
    ET.SubElement(world, "geom", name="test-wall", type="box",
                  pos="0.35 4 0.5", size="0.2 0.4 0.5")
    tree.write(model)
    multi_car.materialize_runtime(car / "urban-car.mjb", car, vehicles)
    multi_car.write_json(car / multi_car.PLANTS_FILE, {
        "plants": [{"root": str(car), "model_xml": str(model)}],
    })
    (work / "validation").mkdir()
    person = urban_composition.Vehicle(
        PERSON, urban_assets.load_manifest(urban / "assets/hakoniwa-person-staff.asset.yaml"),
        "external", {}, {"north_m": 0.0, "east_m": -4.0, "yaw_deg": 90.0},
    )
    composition = urban_composition.Composition(
        "person-step-test", work, None, (person,), (), {"enabled": False},
    )
    # The production lookup assumes a source-tree build directory named build.
    with patch.object(multi_car, "mujoco_library", return_value=mujoco_library):
        assert urban_people.apply_rpc(work, composition, {}, "conductor") == [PERSON]
    return car


def client(pdu_config, service_config, output, hako_cmd, check_gate):
    import hakopy
    from hakoniwa_pdu.pdu_msgs.hako_srv_msgs.pdu_conv_AckermannStepRequestPacket import (
        pdu_to_py_AckermannStepRequestPacket, py_to_pdu_AckermannStepRequestPacket,
    )
    from hakoniwa_pdu.pdu_msgs.hako_srv_msgs.pdu_conv_AckermannStepResponsePacket import (
        pdu_to_py_AckermannStepResponsePacket,
    )
    from hakoniwa_pdu.pdu_msgs.hako_srv_msgs.pdu_conv_PersonStepRequestPacket import (
        pdu_to_py_PersonStepRequestPacket, py_to_pdu_PersonStepRequestPacket,
    )
    from hakoniwa_pdu.pdu_msgs.hako_srv_msgs.pdu_conv_PersonStepResponsePacket import (
        pdu_to_py_PersonStepResponsePacket,
    )

    def send(handle, step, decode, encode, command):
        raw = hakopy.asset_service_client_get_request_buffer(
            handle, hakopy.HAKO_SERVICE_CLIENT_API_OPCODE_REQUEST, -1)
        assert raw is not None
        packet = decode(raw)
        packet.body.step = step
        command(packet.body)
        wire = encode(packet)
        if decode is pdu_to_py_PersonStepRequestPacket and step == 0:
            print(f"Python PersonStep request length={len(wire)}", flush=True)
        assert hakopy.asset_service_client_call_request(handle, wire, 5_000)

    def receive(handle, decode, step, trace):
        event = hakopy.asset_service_client_poll(handle)
        if event == hakopy.HAKO_SERVICE_CLIENT_API_EVENT_NONE:
            return False
        assert event == hakopy.HAKO_SERVICE_CLIENT_API_EVENT_RESPONSE_IN, (step, event)
        raw = hakopy.asset_service_client_get_response(handle, -1)
        response = decode(raw)
        assert response.header.result_code == 0
        assert response.body.result_code == 0 and response.body.step == step
        assert response.body.sim_time_usec == (step + 1) * DELTA_USEC
        if decode is pdu_to_py_PersonStepResponsePacket:
            assert len(response.body.joints.name) == 6
            assert response.body.joints.header.frame_id == "city_map"
        base_off, _, total_size = struct.unpack_from("<III", raw, 8)
        # Core's service header has request correlation fields. Record the
        # complete body, including its dynamic heap, up to metadata.total_size.
        trace.write(raw[base_off + RESPONSE_BODY_OFFSET:total_size])
        return True

    def master_time():
        meta = json.loads(subprocess.check_output([hako_cmd, "jmeta"], timeout=5))
        return meta["master_data"]["time_usec"]["current"]

    def run(_context):
        try:
            car = hakopy.asset_service_client_create(ASSET, f"CarService/AckermannStep/{CAR}", "car")
            person = hakopy.asset_service_client_create(ASSET, f"PersonService/PersonStep/{PERSON}", "person")
            assert car is not None and person is not None
            with (output / "car.bin").open("wb") as car_trace, (output / "person.bin").open("wb") as person_trace:
                for step in range(STEPS):
                    def drive(body):
                        body.drive.speed = 2.0 if step < 40 else 1.0
                        body.drive.steering_angle = 0.1

                    def animate(body):
                        body.cmd_vel.linear.x = 1.0 if step < 20 else 0.0
                        body.animation = ("walk", "wave", "sit", "sit_talk")[step // 20]

                    send(car, step, pdu_to_py_AckermannStepRequestPacket,
                         py_to_pdu_AckermannStepRequestPacket, drive)
                    if check_gate and step == 1:
                        # Let the previous time notification settle, then check
                        # both Core time and the lack of a post-physics reply.
                        time.sleep(0.12)
                        before = master_time()
                        time.sleep(0.35)
                        after = master_time()
                        assert before == after, (before, after)
                        assert (hakopy.asset_service_client_poll(car)
                                == hakopy.HAKO_SERVICE_CLIENT_API_EVENT_NONE)
                        print(f"Core held at {before} us with only Car-1 at step 1", flush=True)
                    send(person, step, pdu_to_py_PersonStepRequestPacket,
                         py_to_pdu_PersonStepRequestPacket, animate)
                    pending = [
                        (car, pdu_to_py_AckermannStepResponsePacket, car_trace),
                        (person, pdu_to_py_PersonStepResponsePacket, person_trace),
                    ]
                    deadline = time.monotonic() + 6
                    while pending:
                        pending = [
                            item for item in pending if not receive(item[0], item[1], step, item[2])
                        ]
                        if pending:
                            assert time.monotonic() < deadline, step
                            time.sleep(0.001)
                    assert hakopy.usleep(DELTA_USEC)
            print(f"DONE {STEPS} mixed steps", flush=True)
            while hakopy.usleep(DELTA_USEC):
                pass
            return 0
        except Exception as exc:
            errors.append(exc)
            traceback.print_exc()
            return -1

    errors = []
    assert hakopy.asset_register(ASSET, str(pdu_config), {
        "on_initialize": lambda _: 0, "on_simulation_step": None,
        "on_manual_timing_control": run, "on_reset": lambda _: 0,
    }, DELTA_USEC, hakopy.HAKO_ASSET_MODEL_PLANT)
    assert hakopy.service_initialize(str(service_config))
    print("CLIENT registered", flush=True)
    hakopy.start()
    return bool(errors)


def wait_for(log, marker, process, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        contents = log.read_text(errors="replace")
        if marker in contents:
            return
        if process.poll() is not None:
            break
        time.sleep(0.05)
    raise RuntimeError(f"{log} missing {marker!r}\n{contents}")


def run_route(build, foundation, car, service_config, run_dir, service, check_gate):
    run_dir.mkdir()
    mmap = run_dir / "mmap"
    mmap.mkdir()
    core_config = run_dir / "core.json"
    core_config.write_text(json.dumps({
        "shm_type": "mmap", "core_mmap_path": str(mmap), "asset_timeout_usec": 600_000_000,
    }))
    env = dict(os.environ, HAKO_CONFIG_PATH=str(core_config))
    for name in ("DYLD_LIBRARY_PATH", "LD_LIBRARY_PATH"):
        env[name] = os.pathsep.join(filter(None, (str(foundation / "lib"), env.get(name))))
    processes = []

    def launch(name, command, ready):
        log = run_dir / f"{name}.log"
        with log.open("wb") as out:
            process = subprocess.Popen(command, env=env, stdout=out, stderr=subprocess.STDOUT)
        processes.append(process)
        wait_for(log, ready, process)
        return process, log

    def core(command):
        with (run_dir / f"core-{command}.log").open("wb") as out:
            subprocess.run([hako_cmd, command], env=env, stdout=out,
                           stderr=subprocess.STDOUT, timeout=10, check=True)

    hako_cmd = foundation / "bin/hako-cmd"
    started = False
    try:
        launch("conductor", [build / "hako-async-rpc-integration-conductor"], "CONDUCTOR started")
        args = [
            build / "bin/urban-car-hakoniwa-asset", "--manifest", car / "urban-car-asset-manifest.json",
            "--no-viewer", "--external-conductor", "--realtime-sync-cycle-msec", "0",
            "--physics-substeps", "4", "--ackermann-rpc-input", CAR, "--person-rpc-input", PERSON,
            "--rpc-service-config", service_config, "--person-contact-log", run_dir / "contacts.jsonl",
        ]
        if service:
            args += ["--ackermann-rpc-use-service", "--person-rpc-use-service"]
        server, _ = launch("server", args, "Starting Hakoniwa Runner:")
        client_process, client_log = launch("client", [
            sys.executable, __file__, "--client", car / "urban-car-pdudef.json",
            service_config, run_dir, hako_cmd, str(int(check_gate)),
        ], "CLIENT registered")
        core("start")
        started = True
        wait_for(client_log, f"DONE {STEPS} mixed steps", client_process)
        core("stop")
        started = False
        assert server.wait(timeout=8) == 0
        # hakopy.start() stays registered for Reset after acknowledging Stop.
        for line in client_log.read_text(errors="replace").splitlines():
            if line.startswith(("Core held", "Python PersonStep")):
                print(f"{run_dir.name}: {line}", flush=True)
    finally:
        if started:
            try:
                core("stop")
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
                pass
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def main(build, foundation, urban, mujoco_library):
    work = Path(tempfile.mkdtemp(prefix="hako-person-step-urban."))
    print(f"Integration logs: {work}", flush=True)
    os.environ.update(
        HAKONIWA_WORKSPACE_ACTIVE="1",
        HAKONIWA_WORKSPACE_ROOT=str(urban.parent / "hakoniwa-business-pack"),
        HAKONIWA_WORK_DIR=str(foundation.parent.parent),
        PYTHONDONTWRITEBYTECODE="1",
    )
    car = prepare(urban, work, mujoco_library)
    service = work / "service.json"
    subprocess.run([
        sys.executable, Path(__file__).resolve().parents[2] / "tools/gen_rpc_service_config.py",
        "--pdu-config", car / "urban-car-pdudef.json", "--output", service,
        "--car", CAR, "--person", PERSON,
    ], check=True)
    baseline = {}
    for label in ("legacy-1", "legacy-2", "service-1", "service-2"):
        directory = work / label
        run_route(build, foundation, car, service, directory, label.startswith("service"),
                  check_gate=label == "service-1")
        for filename in ("person.bin", "car.bin", "contacts.jsonl"):
            data = (directory / filename).read_bytes()
            assert data, (label, filename)
            assert data == baseline.setdefault(filename, data), (label, filename)
            print(label, filename, len(data), hashlib.sha256(data).hexdigest(), flush=True)


if __name__ == "__main__":
    if sys.argv[1] == "--client":
        pdu_config, service_config, output, hako_cmd = map(Path, sys.argv[2:6])
        raise SystemExit(client(pdu_config, service_config, output, hako_cmd, sys.argv[6] == "1"))
    main(*map(Path, sys.argv[1:]))
