"""Exercise manifest services with two Urban cars and one person."""

from contextlib import ExitStack
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import traceback
from unittest.mock import patch
import xml.etree.ElementTree as ET

from check_step_goldens import verify


DELTA_USEC = 1_000
STEPS = 80
CARS = ("Car-1", "Car-2")
PERSON = "Walker"
ASSET = "PersonStepUrbanClient"
RESPONSE_BODY_OFFSET = 272
MANIFEST = "urban-car-asset-manifest.json"
SERVICE_MANIFEST = "urban-car-service-manifest.json"
ASYNC_MANIFEST = "urban-car-async-manifest.json"


def prepare(urban, work, mujoco_library):
    sys.path.insert(0, str(urban / "tools"))
    import multi_car
    import urban_assets
    import urban_composition
    import urban_people

    car = work / "config/car"
    car.mkdir(parents=True)
    template, = multi_car.resolve_config(urban / "recipes/experiments/urban-car-one.yaml")["vehicles"]
    vehicles = []
    for index, car_name in enumerate(CARS, start=1):
        vehicle = deepcopy(template)
        vehicle.update(name=car_name, prefix=f"car_{index}_")
        vehicles.append(vehicle)
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
    for filename, lockstep in ((SERVICE_MANIFEST, True), (ASYNC_MANIFEST, False)):
        shutil.copyfile(car / MANIFEST, car / filename)
        multi_car.configure_step_services(
            car / filename, vehicles=CARS, people=[PERSON], lockstep=lockstep)
    return car


def client(pdu_config, service_config, output, hako_cmd, mode):
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
        if mode == "async":
            assert response.body.sim_time_usec > 10 * DELTA_USEC
        else:
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
            handles = {
                name: hakopy.asset_service_client_create(ASSET, f"CarService/AckermannStep/{name}", name)
                for name in CARS
            }
            handles[PERSON] = hakopy.asset_service_client_create(
                ASSET, f"PersonService/PersonStep/{PERSON}", PERSON)
            assert all(handle is not None for handle in handles.values())
            if mode == "async":
                for _ in range(10):
                    assert hakopy.usleep(DELTA_USEC)
                elapsed = master_time()
                assert elapsed >= 10 * DELTA_USEC, elapsed
                print(f"Core advanced to {elapsed} us before any RPC request", flush=True)

            response_decoders = {
                **dict.fromkeys(CARS, pdu_to_py_AckermannStepResponsePacket),
                PERSON: pdu_to_py_PersonStepResponsePacket,
            }
            steps = 1 if mode == "async" else STEPS
            with ExitStack() as files:
                traces = {
                    name: files.enter_context((output / f"{name}.bin").open("wb"))
                    for name in handles
                }
                for step in range(steps):
                    order = list(handles)
                    check_missing = mode == "check-gate" and 1 <= step <= len(order)
                    if check_missing:
                        # Withhold a different service on each tick so every
                        # gate member is necessary, independent of send order.
                        order.append(order.pop(step - 1))

                    for index, name in enumerate(order):
                        if name == PERSON:
                            def animate(body):
                                body.cmd_vel.linear.x = 1.0 if step < 20 else 0.0
                                body.animation = ("walk", "wave", "sit", "sit_talk")[step // 20]

                            send(handles[name], step, pdu_to_py_PersonStepRequestPacket,
                                 py_to_pdu_PersonStepRequestPacket, animate)
                        else:
                            def drive(body):
                                body.drive.speed = (2.0 if step < 40 else 1.0) + CARS.index(name)
                                body.drive.steering_angle = 0.1 if name == CARS[0] else -0.1

                            send(handles[name], step, pdu_to_py_AckermannStepRequestPacket,
                                 py_to_pdu_AckermannStepRequestPacket, drive)
                        if check_missing and index == len(order) - 2:
                            time.sleep(0.12)
                            before = master_time()
                            time.sleep(0.25)
                            after = master_time()
                            assert before == after == step * DELTA_USEC, (before, after)
                            for waiting in order[:-1]:
                                assert (hakopy.asset_service_client_poll(handles[waiting])
                                        == hakopy.HAKO_SERVICE_CLIENT_API_EVENT_NONE)
                            print(f"Core held at {before} us waiting for {order[-1]}", flush=True)
                    pending = list(handles)
                    deadline = time.monotonic() + 6
                    while pending:
                        pending = [
                            name for name in pending
                            if not receive(handles[name], response_decoders[name], step, traces[name])
                        ]
                        if pending:
                            assert time.monotonic() < deadline, step
                            if mode == "async":
                                assert hakopy.usleep(DELTA_USEC)
                            else:
                                time.sleep(0.001)
                    assert hakopy.usleep(DELTA_USEC)
            print(f"DONE {steps} mixed steps", flush=True)
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


def wait_for(log, marker, process, timeout=20, *, allow_clean_exit=False):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        contents = log.read_text(errors="replace")
        if marker in contents:
            return
        status = process.poll()
        if allow_clean_exit and status == 0:
            return
        if status is not None:
            break
        time.sleep(0.05)
    raise RuntimeError(f"{log} missing {marker!r}\n{contents}")


def run_route(build, foundation, car, service_config, run_dir, route, check_gate=False):
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
        manifest = {"service": SERVICE_MANIFEST, "async": ASYNC_MANIFEST}[route]
        args = [
            build / "bin/urban-car-hakoniwa-asset", "--manifest", car / manifest,
            "--no-viewer", "--external-conductor", "--realtime-sync-cycle-msec", "0",
            "--physics-substeps", "4", "--rpc-service-config", service_config,
            "--person-contact-log", run_dir / "contacts.jsonl",
        ]
        server, server_log = launch("server", args, "Starting Hakoniwa Runner:")
        client_process, client_log = launch("client", [
            sys.executable, __file__, "--client", car / "urban-car-pdudef.json",
            service_config, run_dir, hako_cmd, "check-gate" if check_gate else route,
        ], "CLIENT registered")
        core("start")
        started = True
        steps = 1 if route == "async" else STEPS
        wait_for(client_log, f"DONE {steps} mixed steps", client_process)
        core("stop")
        started = False
        if route == "async":
            # With continuous stepping Core can acknowledge Stop inside
            # sleep_no_wait() and keep the asset registered for Reset.
            wait_for(server_log, "WAIT RESET", server, timeout=8, allow_clean_exit=True)
        else:
            assert server.wait(timeout=8) == 0
        # hakopy.start() stays registered for Reset after acknowledging Stop.
        for line in client_log.read_text(errors="replace").splitlines():
            if line.startswith(("Core held", "Core advanced", "Python PersonStep")):
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


def check_manifest_contract(build, foundation, car, service_config, work):
    from jsonschema import Draft202012Validator
    schema_path = Path(__file__).resolve().parents[2] / "schemas/runtime/asset-manifest.schema.json"
    schema = json.loads(schema_path.read_text())
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema)
    manifest = json.loads((car / SERVICE_MANIFEST).read_text())
    validator.validate(manifest)
    base = json.loads((car / MANIFEST).read_text())
    validator.validate(base)
    validator.validate(json.loads((car / ASYNC_MANIFEST).read_text()))
    assert {key: value for key, value in manifest.items() if key != "services"} == base

    def rejected(label, replacement, reason):
        path = car / f"invalid-{label}.json"
        path.write_text(json.dumps(replacement))
        result = subprocess.run([
            build / "bin/urban-car-hakoniwa-asset", "--manifest", path,
            "--no-viewer", "--external-conductor", "--rpc-service-config", service_config,
        ], capture_output=True, text=True, timeout=8)
        assert result.returncode != 0 and reason in result.stderr, (label, result.stderr)
        print(f"Rejected {label}: {reason}", flush=True)

    invalid = deepcopy(manifest)
    invalid["services"][0]["lockstep"] = "true"
    assert not validator.is_valid(invalid)
    rejected("lockstep-string", invalid, "lockstep must be a boolean")
    print("Runtime asset schema accepted base, lockstep and async manifests and rejected string lockstep")

    invalid = deepcopy(manifest)
    invalid["services"][0]["type"] = "unknown"
    rejected("unknown-type", invalid, "unknown service type")
    invalid = deepcopy(manifest)
    invalid["services"][0].update(name="CarService/AckermannStep/Missing", pdu_robot="Missing")
    rejected("missing-vehicle", invalid, "Ackermann RPC vehicle has no controller: Missing")
    invalid = deepcopy(manifest)
    invalid["services"][2].update(name="PersonService/PersonStep/Missing", pdu_robot="Missing")
    rejected("missing-person", invalid, "person Missing is missing MuJoCo object")
    invalid = deepcopy(manifest)
    output = next(component for component in invalid["components"]
                  if component["type"] == "multi_dof_joint_state")
    state = json.loads(Path(output["config"]).read_text())
    for section in ("spec", "mjcf_binding"):
        state[section]["bodies"] = [body for body in state[section]["bodies"]
                                    if body["name"] != CARS[0]]
    state_path = car / "missing-car-state.json"
    state_path.write_text(json.dumps(state))
    output["config"] = str(state_path)
    rejected("missing-multidof", invalid, "Ackermann RPC vehicle has no MultiDOF state binding: Car-1")
    for flag in ("--ackermann-rpc-use-service", "--person-rpc-use-service"):
        result = subprocess.run([build / "bin/urban-car-hakoniwa-asset", flag],
                                capture_output=True, text=True, timeout=8)
        assert result.returncode != 0 and f"Unknown or incomplete argument: {flag}" in result.stderr

    async_run = work / "async"
    run_route(build, foundation, car, service_config, async_run, "async")
    first_contact = json.loads((async_run / "contacts.jsonl").read_text().splitlines()[0])
    assert first_contact["time_sec"] < 0.01


def main(build, foundation, urban, mujoco_library, *, contract):
    work = Path(tempfile.mkdtemp(prefix="hako-person-step-urban."))
    print(f"Integration logs: {work}", flush=True)
    os.environ.update(
        HAKONIWA_WORKSPACE_ACTIVE="1",
        HAKONIWA_WORKSPACE_ROOT=os.environ.get("HAKONIWA_WORKSPACE_ROOT")
        or str(urban.parent / "hakoniwa-business-pack"),
        HAKONIWA_WORK_DIR=os.environ.get("HAKONIWA_WORK_DIR") or str(foundation.parent.parent),
        HAKONIWA_FOUNDATION_PREFIX=str(foundation),
        PYTHONDONTWRITEBYTECODE="1",
    )
    car = prepare(urban, work, mujoco_library)
    service = work / "service.json"
    subprocess.run([
        sys.executable, Path(__file__).resolve().parents[2] / "tools/gen_rpc_service_config.py",
        "--pdu-config", car / "urban-car-pdudef.json", "--output", service,
        "--car", CARS[0], "--car", CARS[1], "--person", PERSON,
    ], check=True)
    if contract:
        check_manifest_contract(build, foundation, car, service, work)
        return
    runs = []
    for label in ("service-1", "service-2"):
        directory = work / label
        run_route(build, foundation, car, service, directory, label.split("-")[0],
                  check_gate=label == "service-1")
        runs.append(directory)
        for filename in (*(f"{name}.bin" for name in (*CARS, PERSON)), "contacts.jsonl"):
            data = (directory / filename).read_bytes()
            assert data, (label, filename)
            print(label, filename, len(data), hashlib.sha256(data).hexdigest(), flush=True)
    verify("urban-mixed", *runs, repeat=True)


if __name__ == "__main__":
    if sys.argv[1] == "--client":
        pdu_config, service_config, output, hako_cmd = map(Path, sys.argv[2:6])
        raise SystemExit(client(pdu_config, service_config, output, hako_cmd, sys.argv[6]))
    main(*map(Path, sys.argv[1:5]), contract="--contract" in sys.argv[5:])
