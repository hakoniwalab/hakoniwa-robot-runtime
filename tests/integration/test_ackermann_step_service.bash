#!/bin/bash
set -euo pipefail

BUILD_DIR=$1
FOUNDATION=$2
SOURCE_DIR=$3
URBAN_DIR=${4:-}
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hako-ackermann-step-service.XXXXXX")
export DYLD_LIBRARY_PATH="${FOUNDATION}/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${FOUNDATION}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
PIDS=()
cleanup_processes() {
    local pid
    for pid in "${PIDS[@]:-}"; do
        [ -z "$pid" ] || kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    PIDS=()
}
trap 'cleanup_processes; echo "Integration logs: $WORK_DIR"' EXIT

wait_log() {
    local file=$1 marker=$2 pid=$3
    for ((attempt = 0; attempt < 150; ++attempt)); do
        # Core's background thread can write its own log mid-line.
        grep -qF -- "$marker" "$file" && return 0
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    echo "Missing '$marker' in $file" >&2
    cat "$file" >&2
    return 1
}

wait_exit() {
    local pid=$1
    for ((attempt = 0; attempt < 100; ++attempt)); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.1
    done
    echo "Process $pid did not exit" >&2
    return 1
}

SERVICE_CONFIG="$WORK_DIR/service.json"

begin_run() {
    local run=$1 conductor_pid
    mkdir -p "$run/mmap"
    export HAKO_CONFIG_PATH="$run/core.json"
    cat >"$HAKO_CONFIG_PATH" <<JSON
{"shm_type":"mmap","core_mmap_path":"$run/mmap","asset_timeout_usec":600000000}
JSON
    "$BUILD_DIR/hako-async-rpc-integration-conductor" >"$run/conductor.log" 2>&1 &
    conductor_pid=$!
    PIDS+=("$conductor_pid")
    wait_log "$run/conductor.log" "CONDUCTOR started" "$conductor_pid"
}

end_run() {
    local run=$1 server_pid=$2 client_pid=$3
    "$FOUNDATION/bin/hako-cmd" stop >"$run/stop.log" 2>&1
    wait_exit "$client_pid"
    wait_exit "$server_pid"
    wait "$client_pid"
    wait "$server_pid"
    cleanup_processes
}

run_route() {
    local route=$1 executable=$2
    shift 2
    local run="$WORK_DIR/$route" server_pid client_pid
    begin_run "$run"
    "$BUILD_DIR/$executable" "$SOURCE_DIR/pdu_config.json" "$SERVICE_CONFIG" "$@" \
        >"$run/server.log" 2>&1 &
    server_pid=$!
    PIDS+=("$server_pid")
    wait_log "$run/server.log" "SERVER registered" "$server_pid"
    "$BUILD_DIR/hako-ackermann-rpc-integration-client" "$SERVICE_CONFIG" \
        AckermannClientA CarService/AckermannStep/A A "0:5.0,1:6.0,2:7.0" \
        >"$run/client.log" 2>&1 &
    client_pid=$!
    PIDS+=("$client_pid")
    wait_log "$run/client.log" "CLIENT registered channel=A" "$client_pid"
    "$FOUNDATION/bin/hako-cmd" start >"$run/start.log" 2>&1
    wait_log "$run/client.log" "DONE channel=A count=3" "$client_pid"
    wait_log "$run/server.log" "STEP count=3" "$server_pid"
    end_run "$run" "$server_pid" "$client_pid"
    grep '^RESPONSE ' "$run/client.log" >"$run/responses.txt"
}

prepare_urban() {
    mkdir -p "$WORK_DIR/car"
    HAKONIWA_WORKSPACE_ACTIVE=1 \
    HAKONIWA_WORKSPACE_ROOT="$URBAN_DIR/../hakoniwa-business-pack" \
    HAKONIWA_WORK_DIR="$URBAN_DIR/../hakoniwa-business-pack/work" \
    PYTHONPATH="$URBAN_DIR/tools" PYTHONDONTWRITEBYTECODE=1 \
        "$FOUNDATION/python/bin/python3" - "$URBAN_DIR" "$WORK_DIR/car" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import multi_car

urban, work = map(Path, sys.argv[1:])
recipe = urban / "recipes/experiments/urban-car-one.yaml"
vehicles = multi_car.resolve_config(recipe)["vehicles"]
assert len(vehicles) == 1 and vehicles[0]["name"] == "Car-1"
model = work / "urban-car.xml"
multi_car.materialize_vehicle_fleet_model(model, vehicles)
tree = ET.parse(model)
root = tree.getroot()
root.find("option").set("timestep", "0.00025")
world = root.find("worldbody")
body = world.find("body")
x, y, z = map(float, body.get("pos").split())
# The fleet generator parks cars above the City until spawn poses are applied.
body.set("pos", f"{x} {y} {z - multi_car.PARKING_HEIGHT_M:.6f}")
ET.SubElement(world, "geom", name="test-ground", type="plane", size="20 20 0.1")
tree.write(model)
multi_car.materialize_runtime(model, work, vehicles)
PY
    "$FOUNDATION/python/bin/python3" "$SOURCE_DIR/../../tools/gen_rpc_service_config.py" \
        --pdu-config "$WORK_DIR/car/urban-car-pdudef.json" --output "$SERVICE_CONFIG" --car Car-1
}

run_urban() {
    local run="$WORK_DIR/$1" server_pid client_pid
    shift
    begin_run "$run"
    "$BUILD_DIR/bin/urban-car-hakoniwa-asset" \
        --manifest "$WORK_DIR/car/urban-car-asset-manifest.json" \
        --asset-name AckermannUrban --no-viewer --external-conductor \
        --realtime-sync-cycle-msec 0 --physics-substeps 4 \
        --ackermann-rpc-input Car-1 --rpc-service-config "$SERVICE_CONFIG" "$@" \
        >"$run/server.log" 2>&1 &
    server_pid=$!
    PIDS+=("$server_pid")
    wait_log "$run/server.log" "[INFO] Starting Hakoniwa Runner: asset=AckermannUrban" "$server_pid"
    "$BUILD_DIR/hako-ackermann-step-urban-client" "$SERVICE_CONFIG" "$run/state.bin" \
        >"$run/client.log" 2>&1 &
    client_pid=$!
    PIDS+=("$client_pid")
    wait_log "$run/client.log" "CLIENT registered channel=Car-1" "$client_pid"
    "$FOUNDATION/bin/hako-cmd" start >"$run/start.log" 2>&1
    wait_log "$run/client.log" "DONE channel=Car-1 count=80" "$client_pid"
    end_run "$run" "$server_pid" "$client_pid"
    [[ $(wc -c <"$run/state.bin") -eq $((80 * 14 * 8)) ]]
}

if [[ -z "$URBAN_DIR" ]]; then
    "$FOUNDATION/python/bin/python3" "$SOURCE_DIR/../../tools/gen_rpc_service_config.py" \
        --pdu-config "$SOURCE_DIR/pdu_config.json" --output "$SERVICE_CONFIG" --car A
    run_route legacy hako-ackermann-rpc-integration-server A
    run_route service hako-ackermann-step-service-server
    diff -u "$WORK_DIR/legacy/responses.txt" "$WORK_DIR/service/responses.txt"
    cat "$WORK_DIR/service/responses.txt"
else
    prepare_urban
    run_urban legacy-1
    run_urban legacy-2
    run_urban service-1 --ackermann-rpc-use-service
    run_urban service-2 --ackermann-rpc-use-service
    cmp "$WORK_DIR/legacy-1/state.bin" "$WORK_DIR/legacy-2/state.bin"
    cmp "$WORK_DIR/service-1/state.bin" "$WORK_DIR/service-2/state.bin"
    cmp "$WORK_DIR/legacy-1/state.bin" "$WORK_DIR/service-1/state.bin"
    echo "Urban AckermannStep sim_time_usec, pose and twist matched byte for byte over 80 steps in all four runs"
fi
