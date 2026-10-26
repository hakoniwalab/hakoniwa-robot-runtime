#!/bin/bash
set -uo pipefail

BUILD_DIR=$1
FOUNDATION=$2
SOURCE_DIR=$3
WORK_DIR=$(mktemp -d "${TMPDIR:-/tmp}/hako-ackermann-rpc-integration.XXXXXX")
export DYLD_LIBRARY_PATH="${FOUNDATION}/lib${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${FOUNDATION}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
PIDS=()
cleanup() { for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done; wait 2>/dev/null || true; echo "Integration logs: ${WORK_DIR}"; }
trap cleanup EXIT
wait_log() { local file=$1 pattern=$2 n=0; while [ "$n" -lt 150 ]; do grep -q -- "$pattern" "$file" 2>/dev/null && return 0; sleep 0.1; n=$((n+1)); done; return 1; }
run_limited() { local n=0; "$@" & local pid=$!; while [ "$n" -lt 80 ]; do if ! kill -0 "$pid" 2>/dev/null; then wait "$pid"; return $?; fi; sleep 0.1; n=$((n+1)); done; kill "$pid" 2>/dev/null || true; return 124; }

mkdir -p "${WORK_DIR}/mmap"
SERVICE_CONFIG="${WORK_DIR}/ackermann-service.json"
"${FOUNDATION}/python/bin/python3" \
  "${SOURCE_DIR}/../../tools/gen_rpc_service_config.py" \
  --pdu-config "${SOURCE_DIR}/pdu_config.json" \
  --output "${SERVICE_CONFIG}" \
  --car A --car B >"${WORK_DIR}/generator.log" || exit 1
cat > "${WORK_DIR}/cpp_core_config.json" <<JSON
{"shm_type":"mmap","core_mmap_path":"${WORK_DIR}/mmap","asset_timeout_usec":600000000}
JSON
export HAKO_CONFIG_PATH="${WORK_DIR}/cpp_core_config.json"
"${BUILD_DIR}/hako-async-rpc-integration-conductor" >"${WORK_DIR}/conductor.log" 2>&1 & PIDS+=("$!")
wait_log "${WORK_DIR}/conductor.log" "CONDUCTOR started" || exit 1
"${BUILD_DIR}/hako-ackermann-rpc-integration-server" \
  "${SOURCE_DIR}/pdu_config.json" "${SERVICE_CONFIG}" \
  >"${WORK_DIR}/server.log" 2>&1 & PIDS+=("$!")
wait_log "${WORK_DIR}/server.log" "SERVER registered" || exit 1
"${BUILD_DIR}/hako-ackermann-rpc-integration-client" \
  "${SERVICE_CONFIG}" AckermannClientA CarService/AckermannStep/A A \
  "0:5.0,1:6.0,2:7.0" >"${WORK_DIR}/client-A.log" 2>&1 & PIDS+=("$!")
"${BUILD_DIR}/hako-ackermann-rpc-integration-client" \
  "${SERVICE_CONFIG}" AckermannClientB CarService/AckermannStep/B B \
  "0:0.0,1:1.0,2:2.0" >"${WORK_DIR}/client-B.log" 2>&1 & PIDS+=("$!")
wait_log "${WORK_DIR}/client-A.log" "CLIENT registered" || exit 1
wait_log "${WORK_DIR}/client-B.log" "CLIENT registered" || exit 1
run_limited "${FOUNDATION}/bin/hako-cmd" start >"${WORK_DIR}/start.log" 2>&1 || exit 1
wait_log "${WORK_DIR}/client-A.log" "DONE channel=A count=3" || exit 1
wait_log "${WORK_DIR}/client-B.log" "DONE channel=B count=3" || exit 1
wait_log "${WORK_DIR}/server.log" "STEP count=3" || exit 1
grep -q "RESPONSE channel=A k=2 speed=7" "${WORK_DIR}/client-A.log" || exit 1
grep -q "RESPONSE channel=B k=2 speed=2" "${WORK_DIR}/client-B.log" || exit 1
run_limited "${FOUNDATION}/bin/hako-cmd" stop >"${WORK_DIR}/stop.log" 2>&1 || exit 1
echo "PASS ackermann-rpc-input"
