#!/bin/bash
set -euo pipefail

BUILD_DIR=$1
FOUNDATION=$2
SOURCE_DIR=$3
URBAN_DIR=${4:-}
MODE=${5:-single}
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
    local run=$1 pid
    shift
    "$FOUNDATION/bin/hako-cmd" stop >"$run/stop.log" 2>&1
    for pid in "$@"; do
        wait_exit "$pid"
        wait "$pid"
    done
    cleanup_processes
}

start_fixture() {
    local run=$1 executable=$2
    shift 2
    begin_run "$run"
    "$BUILD_DIR/$executable" "$SOURCE_DIR/pdu_config.json" "$SERVICE_CONFIG" "$@" \
        >"$run/server.log" 2>&1 &
    SERVER_PID=$!
    PIDS+=("$SERVER_PID")
    wait_log "$run/server.log" "SERVER registered" "$SERVER_PID"
}

start_fixture_client() {
    local run=$1 channel=$2 plan=$3
    "$BUILD_DIR/hako-ackermann-rpc-integration-client" "$SERVICE_CONFIG" \
        "AckermannClient$channel" "CarService/AckermannStep/$channel" "$channel" "$plan" \
        >"$run/client-$channel.log" 2>&1 &
    CLIENT_PID=$!
    PIDS+=("$CLIENT_PID")
    wait_log "$run/client-$channel.log" "CLIENT registered channel=$channel" "$CLIENT_PID"
}

run_route() {
    local run="$WORK_DIR/$1" executable=$2
    shift 2
    start_fixture "$run" "$executable" "$@"
    start_fixture_client "$run" A "0:5.0,1:6.0,2:7.0"
    local client_a=$CLIENT_PID
    local client_b=
    if [[ "$MODE" == two ]]; then
        start_fixture_client "$run" B "0:0.0,1:1.0,2:2.0"
        client_b=$CLIENT_PID
    fi
    "$FOUNDATION/bin/hako-cmd" start >"$run/start.log" 2>&1
    wait_log "$run/client-A.log" "DONE channel=A count=3" "$client_a"
    wait_log "$run/server.log" "STEP count=3" "$SERVER_PID"
    if [[ -n "$client_b" ]]; then
        wait_log "$run/client-B.log" "DONE channel=B count=3" "$client_b"
        end_run "$run" "$SERVER_PID" "$client_a" "$client_b"
        grep '^RESPONSE ' "$run/client-B.log" >"$run/responses-B.txt"
    else
        end_run "$run" "$SERVER_PID" "$client_a"
    fi
    grep '^RESPONSE ' "$run/client-A.log" >"$run/responses-A.txt"
}

master_time() {
    "$FOUNDATION/bin/hako-cmd" jmeta | "$FOUNDATION/python/bin/python3" -c \
        'import json,sys; print(json.load(sys.stdin)["master_data"]["time_usec"]["current"])'
}

run_timeout() {
    local run="$WORK_DIR/timeout"
    start_fixture "$run" hako-ackermann-step-service-server A B
    start_fixture_client "$run" A "0:5.0,1:6.0,2:7.0"
    local client_a=$CLIENT_PID
    start_fixture_client "$run" B "0:0.0"
    local client_b=$CLIENT_PID
    "$FOUNDATION/bin/hako-cmd" start >"$run/start.log" 2>&1
    wait_log "$run/client-A.log" "RESPONSE channel=A k=0 speed=5" "$client_a"
    wait_log "$run/client-B.log" "DONE channel=B count=1" "$client_b"
    wait_log "$run/server.log" "STEP count=1" "$SERVER_PID"
    # Allow the first tick's Core timing notification to settle.
    sleep 0.2
    local before after
    before=$(master_time)
    sleep 0.4
    after=$(master_time)
    [[ "$before" == "$after" ]]
    echo "Core held at $before us while waiting for B"
    wait_exit "$SERVER_PID"
    local result=0
    wait "$SERVER_PID" || result=$?
    [[ "$result" -eq 1 ]]
    grep -Fx 'SERVER ERROR: lockstep timeout: missing CarService/AckermannStep/B' "$run/server.log"
    [[ $(grep -c '^STEP count=' "$run/server.log") -eq 1 ]]
    [[ $(grep -c '^RESPONSE ' "$run/client-A.log") -eq 1 ]]
    echo "Exactly one physics step completed before the timeout"
}

prepare_urban() {
    local vehicle
    local workspace_root="${HAKONIWA_WORKSPACE_ROOT:-$URBAN_DIR/../hakoniwa-business-pack}"
    local service_args=()
    for vehicle in "${VEHICLES[@]}"; do
        service_args+=(--car "$vehicle")
    done
    mkdir -p "$WORK_DIR/car"
    HAKONIWA_WORKSPACE_ACTIVE=1 \
    HAKONIWA_WORKSPACE_ROOT="$workspace_root" \
    HAKONIWA_WORK_DIR="${HAKONIWA_WORK_DIR:-$workspace_root/work}" \
    HAKONIWA_FOUNDATION_PREFIX="$FOUNDATION" \
    PYTHONPATH="$URBAN_DIR/tools" PYTHONDONTWRITEBYTECODE=1 \
        "$FOUNDATION/python/bin/python3" - "$URBAN_DIR" "$WORK_DIR/car" "${VEHICLES[@]}" <<'PY'
from copy import deepcopy
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET
import multi_car

urban, work = map(Path, sys.argv[1:3])
recipe = urban / "recipes/experiments/urban-car-one.yaml"
template, = multi_car.resolve_config(recipe)["vehicles"]
vehicles = []
for index, name in enumerate(sys.argv[3:], start=1):
    vehicle = deepcopy(template)
    vehicle.update(name=name, prefix=f"car_{index}_")
    vehicles.append(vehicle)
model = work / "urban-car.xml"
multi_car.materialize_vehicle_fleet_model(model, vehicles)
tree = ET.parse(model)
root = tree.getroot()
root.find("option").set("timestep", "0.00025")
world = root.find("worldbody")
# The fleet generator parks cars above the City until spawn poses are applied.
# Keep its separated x positions and bring both cars down to the plane.
for body in world.findall("body"):
    x, y, z = map(float, body.get("pos").split())
    body.set("pos", f"{x} {y} {z - multi_car.PARKING_HEIGHT_M:.6f}")
ET.SubElement(world, "geom", name="test-ground", type="plane", size="20 20 0.1")
tree.write(model)
multi_car.materialize_runtime(model, work, vehicles)
manifest = work / "urban-car-asset-manifest.json"
configured = work / "urban-car-service-manifest.json"
shutil.copyfile(manifest, configured)
multi_car.configure_step_services(configured, vehicles=[vehicle["name"] for vehicle in vehicles],
                                  lockstep=True)
PY
    "$FOUNDATION/python/bin/python3" "$SOURCE_DIR/../../tools/gen_rpc_service_config.py" \
        --pdu-config "$WORK_DIR/car/urban-car-pdudef.json" --output "$SERVICE_CONFIG" "${service_args[@]}"
}

start_urban() {
    local run=$1
    local manifest="$WORK_DIR/car/urban-car-service-manifest.json"
    begin_run "$run"
    "$BUILD_DIR/bin/urban-car-hakoniwa-asset" \
        --manifest "$manifest" \
        --asset-name AckermannUrban --no-viewer --external-conductor \
        --realtime-sync-cycle-msec 0 --physics-substeps 4 \
        --rpc-service-config "$SERVICE_CONFIG" \
        >"$run/server.log" 2>&1 &
    SERVER_PID=$!
    PIDS+=("$SERVER_PID")
    wait_log "$run/server.log" "[INFO] Starting Hakoniwa Runner: asset=AckermannUrban" "$SERVER_PID"
}

run_urban() {
    local run="$WORK_DIR/$1" vehicle index
    local client_pids=()
    start_urban "$run"
    for vehicle in "${VEHICLES[@]}"; do
        "$BUILD_DIR/hako-ackermann-step-urban-client" "$SERVICE_CONFIG" "$run/state-$vehicle.bin" "$vehicle" \
            >"$run/client-$vehicle.log" 2>&1 &
        client_pids+=("$!")
        PIDS+=("$!")
        wait_log "$run/client-$vehicle.log" "CLIENT registered channel=$vehicle" "$!"
    done
    "$FOUNDATION/bin/hako-cmd" start >"$run/start.log" 2>&1
    for index in "${!VEHICLES[@]}"; do
        vehicle=${VEHICLES[index]}
        wait_log "$run/client-$vehicle.log" "DONE channel=$vehicle count=80" "${client_pids[index]}"
    done
    end_run "$run" "$SERVER_PID" "${client_pids[@]}"
    for vehicle in "${VEHICLES[@]}"; do
        [[ $(wc -c <"$run/state-$vehicle.bin") -eq $((80 * 14 * 8)) ]]
    done
}

run_urban_startup() {
    local run="$WORK_DIR/startup" client_pid
    start_urban "$run"
    "$BUILD_DIR/hako-ackermann-step-startup-client" \
        "$WORK_DIR/car/urban-car-pdudef.json" "$SERVICE_CONFIG" >"$run/client.log" 2>&1 &
    client_pid=$!
    PIDS+=("$client_pid")
    wait_log "$run/client.log" "STARTUP CLIENT registered" "$client_pid"
    "$FOUNDATION/bin/hako-cmd" start >"$run/start.log" 2>&1
    wait_log "$run/client.log" "STARTUP CLIENT complete batches=3" "$client_pid"
    grep -Fx "STARTUP CLIENT received both replies k=0 world_time_usec=0" "$run/client.log"
    end_run "$run" "$SERVER_PID" "$client_pid"
}

if [[ -z "$URBAN_DIR" ]]; then
    config_args=(--car A)
    if [[ "$MODE" != single ]]; then config_args+=(--car B); fi
    "$FOUNDATION/python/bin/python3" "$SOURCE_DIR/../../tools/gen_rpc_service_config.py" \
        --pdu-config "$SOURCE_DIR/pdu_config.json" --output "$SERVICE_CONFIG" "${config_args[@]}"
    case "$MODE" in
        single)
            run_route service hako-ackermann-step-service-server
            ;;
        two)
            run_route service hako-ackermann-step-service-server A B
            cat "$WORK_DIR/service/responses-B.txt"
            ;;
        timeout) run_timeout; exit 0 ;;
        *) echo "Unknown mode: $MODE" >&2; exit 2 ;;
    esac
    "$FOUNDATION/python/bin/python3" "$SOURCE_DIR/check_step_goldens.py" "ackermann-$MODE" \
        "$WORK_DIR/service"
    cat "$WORK_DIR/service/responses-A.txt"
else
    case "$MODE" in
        single) VEHICLES=(Car-1) ;;
        two|startup) VEHICLES=(Car-1 Car-2) ;;
        *) echo "Unknown Urban mode: $MODE" >&2; exit 2 ;;
    esac
    prepare_urban
    if [[ "$MODE" == startup ]]; then
        run_urban_startup
        exit 0
    fi
    run_urban service-1
    run_urban service-2
    "$FOUNDATION/python/bin/python3" "$SOURCE_DIR/check_step_goldens.py" "urban-$MODE" \
        "$WORK_DIR/service-1" "$WORK_DIR/service-2" --repeat
    if [[ "$MODE" == two ]] && cmp -s "$WORK_DIR/service-1/state-Car-1.bin" "$WORK_DIR/service-1/state-Car-2.bin"; then
        echo "Both vehicles produced the same trace despite distinct initial poses and commands" >&2
        exit 1
    fi
fi
