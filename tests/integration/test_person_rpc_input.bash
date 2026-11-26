#!/bin/bash
set -uo pipefail

BUILD_DIR=$1
FOUNDATION=$2
SOURCE_DIR=$3
MODE=${4:-legacy}
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hako-person-rpc-integration.XXXXXX")
export DYLD_LIBRARY_PATH="${FOUNDATION}/lib${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${FOUNDATION}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
PIDS=()
cleanup() { for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done; wait 2>/dev/null || true; echo "Integration logs: ${WORK_DIR}"; }
trap cleanup EXIT
wait_log() { local file=$1 pattern=$2 n=0; while [ "$n" -lt 150 ]; do grep -q -- "$pattern" "$file" 2>/dev/null && return 0; sleep 0.1; n=$((n+1)); done; return 1; }
run_limited() { local n=0; "$@" & local pid=$!; while [ "$n" -lt 80 ]; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid"; return $?; fi; sleep 0.1; n=$((n+1)); done; kill "$pid" 2>/dev/null || true; return 124; }

SERVICE_CONFIG="${WORK_DIR}/person-service.json"
"${FOUNDATION}/python/bin/python3" \
  "${SOURCE_DIR}/../../tools/gen_rpc_service_config.py" \
  --pdu-config "${SOURCE_DIR}/pdu_config.json" \
  --output "${SERVICE_CONFIG}" \
  --person A --person B >"${WORK_DIR}/generator.log" || exit 1
run_route() {
    local dir="$WORK_DIR/$1"
    shift
    mkdir -p "$dir/mmap"
    export HAKO_CONFIG_PATH="$dir/cpp_core_config.json"
    cat > "$HAKO_CONFIG_PATH" <<JSON
{"shm_type":"mmap","core_mmap_path":"$dir/mmap","asset_timeout_usec":600000000}
JSON
    "${BUILD_DIR}/hako-async-rpc-integration-conductor" >"$dir/conductor.log" 2>&1 & PIDS+=("$!")
    wait_log "$dir/conductor.log" "CONDUCTOR started" || exit 1
    "${BUILD_DIR}/hako-person-rpc-integration-server" \
        "${SOURCE_DIR}/pdu_config.json" "$SERVICE_CONFIG" "$@" \
        >"$dir/server.log" 2>&1 & PIDS+=("$!")
    wait_log "$dir/server.log" "SERVER registered" || exit 1
    local channel
    for channel in A B; do
        "${BUILD_DIR}/hako-person-rpc-integration-client" \
            "$SERVICE_CONFIG" "PersonClient$channel" "$channel" unused \
            >"$dir/client-$channel.log" 2>&1 & PIDS+=("$!")
        wait_log "$dir/client-$channel.log" "CLIENT registered" || exit 1
    done
    run_limited "${FOUNDATION}/bin/hako-cmd" start >"$dir/start.log" 2>&1 || exit 1
    for channel in A B; do
        wait_log "$dir/client-$channel.log" "DONE channel=$channel count=3" || exit 1
        grep '^RESPONSE ' "$dir/client-$channel.log" >"$dir/responses-$channel.txt" || exit 1
    done
    wait_log "$dir/server.log" "STEP count=3" || exit 1
    run_limited "${FOUNDATION}/bin/hako-cmd" stop >"$dir/stop.log" 2>&1 || exit 1
    local pid
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    PIDS=()
}
run_route legacy
if [[ "$MODE" == compare ]]; then
    run_route service --service
    for channel in A B; do
        diff -u "$WORK_DIR/legacy/responses-$channel.txt" "$WORK_DIR/service/responses-$channel.txt" || exit 1
    done
    echo "Both PersonStep clients received identical response lines on both routes"
fi
echo "PASS person-rpc-input"
