#!/bin/bash
set -euo pipefail

BUILD_DIR=${1:?build directory required}
FOUNDATION=${2:?foundation prefix required}
SOURCE_DIR=${3:?integration source directory required}
MODE=${4:-zero}
STEPS=1
if [ "$MODE" != zero ]; then STEPS=5; fi
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hako-async-service-reply.XXXXXX")
export DYLD_LIBRARY_PATH="${FOUNDATION}/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${FOUNDATION}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        [ -z "$pid" ] || kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "Integration logs: $WORK_DIR"
}
trap cleanup EXIT

wait_log() {
    local file=$1 marker=$2 pid=$3
    for ((attempt = 0; attempt < 150; ++attempt)); do
        grep -qF -- "$marker" "$file" && return 0
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    echo "Missing '$marker' in $file" >&2
    cat "$file" >&2
    return 1
}

mkdir -p "$WORK_DIR/mmap"
export HAKO_CONFIG_PATH="$WORK_DIR/cpp_core_config.json"
cat > "$HAKO_CONFIG_PATH" <<JSON
{"shm_type":"mmap","core_mmap_path":"$WORK_DIR/mmap","asset_timeout_usec":600000000}
JSON

"$BUILD_DIR/hako-async-rpc-integration-conductor" >"$WORK_DIR/conductor.log" 2>&1 &
PIDS+=("$!")
wait_log "$WORK_DIR/conductor.log" "CONDUCTOR started" "$!"
"$BUILD_DIR/hako-async-service-reply-server" "$SOURCE_DIR/pdu_config.json" \
    "$SOURCE_DIR/service.json" "$STEPS" >"$WORK_DIR/server.log" 2>&1 &
SERVER_PID=$!
PIDS+=("$SERVER_PID")
wait_log "$WORK_DIR/server.log" "SERVER registered" "$SERVER_PID"
"$BUILD_DIR/hako-async-service-reply-client" "$SOURCE_DIR/service.json" "$MODE" \
    >"$WORK_DIR/client.log" 2>&1 &
CLIENT_PID=$!
PIDS+=("$CLIENT_PID")
wait_log "$WORK_DIR/client.log" "CLIENT registered" "$CLIENT_PID"

"$FOUNDATION/bin/hako-cmd" start >"$WORK_DIR/start.log" 2>&1
if ! wait_log "$WORK_DIR/client.log" "CLIENT done" "$CLIENT_PID"; then
    tail -30 "$WORK_DIR/server.log" >&2
    exit 1
fi
"$FOUNDATION/bin/hako-cmd" stop >"$WORK_DIR/stop.log" 2>&1
wait "$CLIENT_PID"
if grep -qF "SERVER ERROR" "$WORK_DIR/server.log"; then
    cat "$WORK_DIR/server.log" >&2
    exit 1
fi
cat "$WORK_DIR/client.log"
grep -F "SERVER response" "$WORK_DIR/server.log"
