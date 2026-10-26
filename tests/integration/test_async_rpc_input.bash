#!/bin/bash
set -uo pipefail

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <build-dir> <foundation-prefix> <integration-source-dir>" >&2
    exit 2
fi

BUILD_DIR=$1
FOUNDATION=$2
SOURCE_DIR=$3
SERVER="${BUILD_DIR}/hako-async-rpc-integration-server"
CLIENT="${BUILD_DIR}/hako-async-rpc-integration-client"
CONDUCTOR="${BUILD_DIR}/hako-async-rpc-integration-conductor"
HAKO_CMD="${FOUNDATION}/bin/hako-cmd"
PYTHON="${FOUNDATION}/python/bin/python3"
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hako-async-rpc-integration.XXXXXX")

export DYLD_LIBRARY_PATH="${FOUNDATION}/lib${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${FOUNDATION}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

PIDS=()
cleanup_processes() {
    local pid
    for pid in "${PIDS[@]:-}"; do
        [ -n "${pid}" ] && kill "${pid}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    PIDS=()
}
cleanup_all() {
    cleanup_processes
    echo "Integration logs: ${WORK_DIR}"
}
trap cleanup_all EXIT

fail() {
    echo "FAIL: $*" >&2
    return 1
}

wait_log() {
    local file=$1 pattern=$2 timeout=$3 elapsed=0
    while [ "${elapsed}" -lt "$((timeout * 10))" ]; do
        grep -q -- "${pattern}" "${file}" 2>/dev/null && return 0
        sleep 0.1
        elapsed=$((elapsed + 1))
    done
    echo "Timed out waiting for '${pattern}' in ${file}" >&2
    return 1
}

wait_exit() {
    local pid=$1 timeout=$2 elapsed=0
    while [ "${elapsed}" -lt "$((timeout * 10))" ]; do
        ! kill -0 "${pid}" 2>/dev/null && return 0
        sleep 0.1
        elapsed=$((elapsed + 1))
    done
    return 1
}

run_limited() {
    local timeout=$1
    shift
    "$@" &
    local pid=$! elapsed=0
    while [ "${elapsed}" -lt "$((timeout * 10))" ]; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            wait "${pid}"
            return $?
        fi
        sleep 0.1
        elapsed=$((elapsed + 1))
    done
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
    return 124
}

master_time() {
    "${HAKO_CMD}" jmeta | "${PYTHON}" -c \
        'import json,sys; print(json.load(sys.stdin)["master_data"]["time_usec"]["current"])'
}

begin_case() {
    local name=$1
    CASE_DIR="${WORK_DIR}/${name}"
    mkdir -p "${CASE_DIR}/mmap"
    cat > "${CASE_DIR}/cpp_core_config.json" <<JSON
{
  "shm_type": "mmap",
  "core_mmap_path": "${CASE_DIR}/mmap",
  "asset_timeout_usec": 600000000
}
JSON
    export HAKO_CONFIG_PATH="${CASE_DIR}/cpp_core_config.json"
    "${CONDUCTOR}" > "${CASE_DIR}/conductor.log" 2>&1 &
    CONDUCTOR_PID=$!
    PIDS+=("${CONDUCTOR_PID}")
    wait_log "${CASE_DIR}/conductor.log" "CONDUCTOR started" 5
    "${SERVER}" "${SOURCE_DIR}/pdu_config.json" "${SOURCE_DIR}/service.json" \
        "${CASE_DIR}/steps.jsonl" > "${CASE_DIR}/server.log" 2>&1 &
    SERVER_PID=$!
    PIDS+=("${SERVER_PID}")
}

launch_client() {
    local channel=$1 plan=$2
    shift 2
    local asset="AsyncRpcClient${channel}"
    local service="Service/Async${channel}"
    "${CLIENT}" "${SOURCE_DIR}/service.json" "${asset}" "${service}" \
        "${channel}" "${plan}" "$@" > "${CASE_DIR}/client-${channel}.log" 2>&1 &
    local pid=$!
    PIDS+=("${pid}")
    if [ "${channel}" = A ]; then CLIENT_A_PID=${pid}; else CLIENT_B_PID=${pid}; fi
}

run_happy_partial_stop() {
    begin_case happy-partial-stop || return 1
    launch_client A "0:101,1:102,2:103,3:104,4:105"
    launch_client B "0:201,1:202,2:203,3:204" 3 "${CASE_DIR}/release-b-step3"
    wait_log "${CASE_DIR}/client-A.log" "CLIENT registered channel=A" 5 || return 1
    wait_log "${CASE_DIR}/client-B.log" "CLIENT registered channel=B" 5 || return 1
    run_limited 5 "${HAKO_CMD}" start > "${CASE_DIR}/start.log" 2>&1 || return 1

    wait_log "${CASE_DIR}/client-A.log" "RESPONSE channel=A k=2 value=103" 10 || return 1
    wait_log "${CASE_DIR}/client-B.log" "RESPONSE channel=B k=2 value=203" 10 || return 1
    wait_log "${CASE_DIR}/server.log" "STEP count=3 completed_k=2" 10 || return 1
    [ "$(wc -l < "${CASE_DIR}/steps.jsonl" | tr -d ' ')" = 3 ] || return 1
    [ "$(grep -c '\"A\":\"fnv1a64:' "${CASE_DIR}/steps.jsonl")" = 3 ] || return 1
    [ "$(grep -c '\"B\":\"fnv1a64:' "${CASE_DIR}/steps.jsonl")" = 3 ] || return 1

    wait_log "${CASE_DIR}/client-A.log" "SENT channel=A k=3 value=104" 5 || return 1
    sleep 0.3
    grep -q "STEP count=4" "${CASE_DIR}/server.log" && return 1
    local before after
    before=$(master_time) || return 1
    sleep 0.5
    after=$(master_time) || return 1
    [ "${before}" = "${after}" ] || {
        echo "Core time moved while B/k=3 was missing: ${before} -> ${after}" >&2
        return 1
    }
    echo "PARTIAL stable_time_usec=${before}" >> "${CASE_DIR}/assertions.log"

    touch "${CASE_DIR}/release-b-step3"
    wait_log "${CASE_DIR}/client-A.log" "RESPONSE channel=A k=3 value=104" 10 || return 1
    wait_log "${CASE_DIR}/client-B.log" "RESPONSE channel=B k=3 value=204" 10 || return 1
    wait_log "${CASE_DIR}/server.log" "STEP count=4 completed_k=3" 10 || return 1
    wait_log "${CASE_DIR}/client-A.log" "SENT channel=A k=4 value=105" 5 || return 1

    run_limited 8 "${HAKO_CMD}" stop > "${CASE_DIR}/stop.log" 2>&1 || return 1
    wait_exit "${SERVER_PID}" 5 || return 1
    wait "${SERVER_PID}"
    local server_rc=$?
    [ "${server_rc}" = 0 ] || return 1
    ! grep -q "async RPC timeout" "${CASE_DIR}/server.log" || return 1
    wait_log "${CASE_DIR}/server.log" "SERVER exit=0 steps=4" 1 || return 1
    cleanup_processes
    echo "PASS happy-partial-stop"
}

run_skipped_step() {
    begin_case skipped-step || return 1
    launch_client A "0:301,1:302,2:303,3:304,4:305"
    launch_client B "0:401,1:402,2:403,3:404,5:405"
    wait_log "${CASE_DIR}/client-A.log" "CLIENT registered channel=A" 5 || return 1
    wait_log "${CASE_DIR}/client-B.log" "CLIENT registered channel=B" 5 || return 1
    run_limited 5 "${HAKO_CMD}" start > "${CASE_DIR}/start.log" 2>&1 || return 1
    wait_log "${CASE_DIR}/client-B.log" "SENT channel=B k=5 value=405" 10 || return 1
    wait_exit "${SERVER_PID}" 10 || return 1
    wait "${SERVER_PID}"
    local server_rc=$?
    [ "${server_rc}" -ne 0 ] || return 1
    grep -q "channel B expected k=4 but received k=5" "${CASE_DIR}/server.log" || return 1
    cleanup_processes
    echo "PASS skipped-step"
}

failed=0
run_happy_partial_stop || { echo "FAIL happy-partial-stop logs=${CASE_DIR}" >&2; failed=1; cleanup_processes; }
run_skipped_step || { echo "FAIL skipped-step logs=${CASE_DIR}" >&2; failed=1; cleanup_processes; }
exit "${failed}"
