#!/bin/bash
set -euo pipefail

BUILD_DIR=${1:?build directory required}
FOUNDATION=${2:?foundation prefix required}
SOURCE_DIR=${3:?integration source directory required}
MODE=${4:?deterministic or timeout required}
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hako-async-service-lockstep.XXXXXX")
HAKO_CMD="$FOUNDATION/bin/hako-cmd"
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

cleanup() {
    cleanup_processes
    echo "Integration logs: $WORK_DIR"
}
trap cleanup EXIT

wait_log() {
    local file=$1 marker=$2 pid=$3
    for ((attempt = 0; attempt < 150; ++attempt)); do
        # Core's background thread can append a log message before the newline.
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

master_time() {
    "$HAKO_CMD" jmeta | "$FOUNDATION/python/bin/python3" -c \
        'import json,sys; print(json.load(sys.stdin)["master_data"]["time_usec"]["current"])'
}

launch_client() {
    local channel=$1 mode=$2
    "$BUILD_DIR/hako-async-service-lockstep-client" "$SOURCE_DIR/service.json" \
        "$channel" "$mode" "$CASE_DIR/client-$channel.bin" "$CASE_DIR/release-b" \
        >"$CASE_DIR/client-$channel.log" 2>&1 &
    CLIENT_PID=$!
    PIDS+=("$CLIENT_PID")
    wait_log "$CASE_DIR/client-$channel.log" "CLIENT registered" "$CLIENT_PID"
}

begin_case() {
    CASE_DIR="$WORK_DIR/$1"
    mkdir -p "$CASE_DIR/mmap"
    export HAKO_CONFIG_PATH="$CASE_DIR/cpp_core_config.json"
    cat >"$HAKO_CONFIG_PATH" <<JSON
{"shm_type":"mmap","core_mmap_path":"$CASE_DIR/mmap","asset_timeout_usec":600000000}
JSON
    "$BUILD_DIR/hako-async-rpc-integration-conductor" >"$CASE_DIR/conductor.log" 2>&1 &
    PIDS+=("$!")
    wait_log "$CASE_DIR/conductor.log" "CONDUCTOR started" "$!"
    "$BUILD_DIR/hako-async-service-lockstep-server" "$SOURCE_DIR/pdu_config.json" \
        "$SOURCE_DIR/service.json" >"$CASE_DIR/server.log" 2>&1 &
    SERVER_PID=$!
    PIDS+=("$SERVER_PID")
    wait_log "$CASE_DIR/server.log" "SERVER registered" "$SERVER_PID"
    launch_client A complete
    CLIENT_A_PID=$CLIENT_PID
    launch_client B "$2"
    CLIENT_B_PID=$CLIENT_PID
    "$HAKO_CMD" start >"$CASE_DIR/start.log" 2>&1
}

freeze_check() {
    wait_log "$CASE_DIR/client-A.log" "SENT step=6" "$CLIENT_A_PID"
    wait_log "$CASE_DIR/server.log" "SERVER step=5" "$SERVER_PID"
    local before after
    before=$(master_time)
    sleep 0.4
    after=$(master_time)
    if [ "$before" != "$after" ] || grep -qFx "SERVER step=6" "$CASE_DIR/server.log"; then
        echo "Core progressed with only A's request for step 6 ($before -> $after)" >&2
        return 1
    fi
    echo "Core held at ${before} us while waiting for B"
}

run_complete() {
    begin_case "$1" hold
    freeze_check
    touch "$CASE_DIR/release-b"
    wait_log "$CASE_DIR/client-A.log" "CLIENT complete steps=20" "$CLIENT_A_PID"
    wait_log "$CASE_DIR/client-B.log" "CLIENT complete steps=20" "$CLIENT_B_PID"
    "$HAKO_CMD" stop >"$CASE_DIR/stop.log" 2>&1
    wait_exit "$SERVER_PID"
    wait "$SERVER_PID"
    wait "$CLIENT_A_PID"
    wait "$CLIENT_B_PID"
    grep -Fx "SERVER exit=0 steps=20" "$CASE_DIR/server.log"
    cleanup_processes
}

run_timeout() {
    begin_case timeout stop
    wait_log "$CASE_DIR/client-B.log" "CLIENT complete steps=5" "$CLIENT_B_PID"
    freeze_check
    wait_exit "$SERVER_PID"
    if wait "$SERVER_PID"; then
        echo "Server returned success with a missing request" >&2
        return 1
    fi
    grep -Eix 'SERVER ERROR:.*(timeout|timed out).*Service/AsyncB.*|SERVER ERROR:.*Service/AsyncB.*(timeout|timed out).*' \
        "$CASE_DIR/server.log"
    grep -Fx "SERVER exit=1 steps=5" "$CASE_DIR/server.log"
}

case "$MODE" in
    deterministic)
        run_complete first
        run_complete second
        cmp "$WORK_DIR/first/client-A.bin" "$WORK_DIR/second/client-A.bin"
        cmp "$WORK_DIR/first/client-B.bin" "$WORK_DIR/second/client-B.bin"
        echo "Both clients received identical response bytes in both 20-step runs"
        ;;
    timeout) run_timeout ;;
    *) echo "Unknown case $MODE" >&2; exit 2 ;;
esac
