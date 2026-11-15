#!/bin/bash
# Stage A of docs/design-async-rpc.md: a cancel through Core reaches the
# service as an event, the service answers CANCELED, the client sees
# REQUEST_CANCEL_DONE, and the next request is served normally.
set -uo pipefail

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <build-dir> <foundation-prefix> <integration-source-dir>" >&2
    exit 2
fi
BUILD_DIR=$1
FOUNDATION=$2
SOURCE_DIR=$3
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hako-async-rpc-cancel.XXXXXX")
export DYLD_LIBRARY_PATH="${FOUNDATION}/lib${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${FOUNDATION}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
PIDS=()
cleanup() { for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done; wait 2>/dev/null || true; echo "Integration logs: ${WORK_DIR}"; }
trap cleanup EXIT
fail() { echo "FAIL: $*" >&2; exit 1; }
wait_log() { local file=$1 pattern=$2 n=0; while [ "$n" -lt 150 ]; do grep -q -- "$pattern" "$file" 2>/dev/null && return 0; sleep 0.1; n=$((n+1)); done; echo "Timed out waiting for '$pattern' in $file" >&2; return 1; }
run_limited() { local n=0; "$@" & local pid=$!; while [ "$n" -lt 80 ]; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid"; return $?; fi; sleep 0.1; n=$((n+1)); done; kill "$pid" 2>/dev/null || true; return 124; }

mkdir -p "${WORK_DIR}/mmap"
cat > "${WORK_DIR}/cpp_core_config.json" <<JSON
{"shm_type":"mmap","core_mmap_path":"${WORK_DIR}/mmap","asset_timeout_usec":600000000}
JSON
export HAKO_CONFIG_PATH="${WORK_DIR}/cpp_core_config.json"
"${BUILD_DIR}/hako-async-rpc-integration-conductor" >"${WORK_DIR}/conductor.log" 2>&1 & PIDS+=("$!")
wait_log "${WORK_DIR}/conductor.log" "CONDUCTOR started" || fail "conductor"
"${BUILD_DIR}/hako-async-rpc-cancel-server" \
  "${SOURCE_DIR}/pdu_config.json" "${SOURCE_DIR}/service.json" \
  >"${WORK_DIR}/server.log" 2>&1 & SERVER_PID=$!; PIDS+=("${SERVER_PID}")
wait_log "${WORK_DIR}/server.log" "SERVER registered" || fail "server registration"
"${BUILD_DIR}/hako-async-rpc-cancel-client" "${SOURCE_DIR}/service.json" \
  >"${WORK_DIR}/client.log" 2>&1 & CLIENT_PID=$!; PIDS+=("${CLIENT_PID}")
wait_log "${WORK_DIR}/client.log" "CLIENT registered" || fail "client registration"
run_limited "${FOUNDATION}/bin/hako-cmd" start >"${WORK_DIR}/start.log" 2>&1 || fail "start"

wait_log "${WORK_DIR}/server.log" "SERVER request a=1 b=7" || fail "held request"
wait_log "${WORK_DIR}/client.log" "CLIENT cancel sent" || fail "cancel sent"
wait_log "${WORK_DIR}/server.log" "SERVER cancel b=7" || fail "cancel event"
wait_log "${WORK_DIR}/server.log" "SERVER replied code=canceled sum=-7" || fail "canceled reply"
wait_log "${WORK_DIR}/client.log" "CLIENT cancel done" || fail "cancel done"
wait_log "${WORK_DIR}/server.log" "SERVER replied code=ok sum=9" || fail "next request"
wait_log "${WORK_DIR}/client.log" "CLIENT done" || fail "next response"
! grep -q "ERROR" "${WORK_DIR}/server.log" || fail "server error"

run_limited "${FOUNDATION}/bin/hako-cmd" stop >"${WORK_DIR}/stop.log" 2>&1 || fail "stop"
# After stop the server's driver waits for a reset and the client may exit;
# neither is part of this test, so the processes are killed by cleanup.
echo "PASS async-rpc-cancel"
