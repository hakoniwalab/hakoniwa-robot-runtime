"""Run the SO-101 ServoJ-like demo. Pass --no-viewer for a headless run."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def prepare(run_dir, recipe):
    run_dir.mkdir()
    (run_dir / "mmap").mkdir()
    core_config = run_dir / "core.json"
    write_json(core_config, {
        "shm_type": "mmap", "core_mmap_path": str(run_dir / "mmap"),
        "asset_timeout_usec": 600_000_000,
    })
    os.environ["HAKO_CONFIG_PATH"] = str(core_config)

    manifest = json.loads((recipe / "asset-manifest.json").read_text())
    manifest["model"] = str(Path(os.environ["SO101_MODEL"]) / "so101.xml")
    for key in ("pdu_def", "endpoint", "runtime_config"):
        manifest[key] = str(recipe / manifest[key])
    for component in manifest["components"]:
        component["config"] = str(recipe / component["config"])
    service_name = "ArmService/ServoJ/SO101"
    manifest["services"] = [{
        "name": service_name, "type": "plugin", "pdu_robot": "SO101",
        "plugin": {"library": os.environ["PLUGIN"]}, "lockstep": False, "priority": 16,
    }]
    write_json(run_dir / "asset-manifest.json", manifest)
    write_json(run_dir / "service.json", {
        "pduMetaDataSize": 24,
        "services": [{
            "name": service_name, "type": "my_robot_srvs_msgs/ServoJ", "maxClients": 1,
            "pduSize": {
                "server": {"baseSize": 336, "heapSize": 0},
                "client": {"baseSize": 280, "heapSize": 0},
            },
            "clients": [{"name": "ServoJClient", "client_endpoint": {
                "nodeId": "servoj-python", "protocol": "shm", "assetName": "ServoJClientAsset",
                "deltaTimeUsec": 2000, "coreConfig": str(core_config),
            }}],
        }],
    })


def wait_until(ready, children, description):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and all(p.poll() is None for p in children):
        if ready():
            return
        time.sleep(0.1)
    raise RuntimeError(f"Timed out or a process exited while waiting for {description}")


def run(run_dir, no_viewer):
    recipe = Path(os.environ["ROBOT_ARM"]).resolve() / "recipes" / "so101"
    prepare(run_dir, recipe)
    hako_cmd = Path(os.environ["PREFIX"]) / "bin" / "hako-cmd"
    command = [
        os.environ["ARM_ASSET"], "--manifest", str(run_dir / "asset-manifest.json"),
        "--rpc-service-config", str(run_dir / "service.json"), "--realtime-sync-cycle-msec", "2",
    ]
    if no_viewer:
        command.append("--no-viewer")

    def client_registered():
        result = subprocess.run([hako_cmd, "jmeta"], capture_output=True, timeout=5)
        if result.returncode == 0 and b'"ServoJClientAsset"' in result.stdout:
            (run_dir / "registered.json").write_bytes(result.stdout)
            return True
        return False

    children = []
    started = False
    arm_log = run_dir / "arm.log"
    ready_message = (b"Starting Hakoniwa Runner:" if no_viewer
                     else b"MuJoCo Viewer scene capacity:")
    try:
        with arm_log.open("w") as log:
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        wait_until(lambda: ready_message in arm_log.read_bytes(),
                   children, f"the Arm asset (see {arm_log})")
        children.append(subprocess.Popen([
            sys.executable, str(Path(__file__).with_name("client.py")), str(run_dir),
            str(recipe / "config" / "pdu" / "so101-pdutypes.json"),
        ]))
        wait_until(client_registered, children, "the ServoJ client to register")
        subprocess.run([hako_cmd, "start"], check=True, timeout=5)
        started = True
        (run_dir / "start-go").touch()
        if children[-1].wait(timeout=45) != 0:
            raise RuntimeError("The motion client failed")
    finally:
        try:
            if started:
                with (run_dir / "stop.log").open("w") as log:
                    subprocess.run([hako_cmd, "stop"], stdout=log, stderr=subprocess.STDOUT, timeout=5)
        finally:
            for process in reversed(children):
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="new directory for the Core mmap and logs")
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()
    run(args.run_dir.resolve(), args.no_viewer)
