#!/bin/bash
# One run of the test driver (README.md), inside the container:
#   run.sh ros|pdu <size bytes> <rate Hz> <count> [poll_us (pdu) | qos (ros)]
# Results: /tmp/transport-bench/<transport>-<size>-<rate>-{send,recv}.json
set -u
TRANSPORT=$1 SIZE=$2 RATE=$3 COUNT=$4 EXTRA=${5:-}
F=/workspace/work-docker-jazzy/foundation
OUT=/tmp/transport-bench
mkdir -p $OUT
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/ros2-work-jazzy-asset/install/setup.bash; set -u
BENCH="$(ros2 pkg prefix hakoniwa_transport_bench)/lib/hakoniwa_transport_bench/transport_bench"
# One binary for both transports: it links the Hakoniwa core either way.
export HAKO_CONFIG_PATH=$F/config/cpp_core_config.json LD_LIBRARY_PATH=$F/install/lib:${LD_LIBRARY_PATH:-}
TAG=$TRANSPORT-$SIZE-$RATE
rm -f $OUT/$TAG-send.json $OUT/$TAG-recv.json $OUT/$TAG-trace.txt   # never report an earlier run
COMMON="--transport $TRANSPORT --size $SIZE --rate $RATE --count $COUNT"

if [ "$TRANSPORT" = ros ]; then
  QOS=${EXTRA:-reliable}
  $BENCH $COMMON --role recv --qos $QOS --out $OUT/$TAG-recv.json --trace $OUT/$TAG-trace.txt > $OUT/$TAG-recv.log 2>&1 &
  RECV=$!
  $BENCH $COMMON --role send --qos $QOS --out $OUT/$TAG-send.json > $OUT/$TAG-send.log 2>&1
  wait $RECV
else
  PDU_SIZE=$((SIZE + 4096))   # the PDU header, the cloud's fixed part and its fields, with room
  cat > $OUT/pdudef.json <<JSON
{"robots": [{"name": "Bench", "rpc_pdu_readers": [], "rpc_pdu_writers": [],
  "shm_pdu_readers": [{"type": "sensor_msgs/PointCloud2", "org_name": "cloud", "name": "Bench_cloud", "channel_id": 0, "pdu_size": $PDU_SIZE, "write_cycle": 1, "method_type": "SHM"}],
  "shm_pdu_writers": [{"type": "sensor_msgs/PointCloud2", "org_name": "cloud", "name": "Bench_cloud", "channel_id": 0, "pdu_size": $PDU_SIZE, "write_cycle": 1, "method_type": "SHM"}]}]}
JSON
  # The conductor only (no asset advances time); on TERM it stops as Hakoniwa
  # expects (a killed conductor leaves the master lock to a dead owner).
  $F/install/python/bin/python -c "
import signal, sys, time, hakopy
hakopy.conductor_start(1000, 1000)
signal.signal(signal.SIGTERM, lambda *_: (hakopy.conductor_stop(), sys.exit(0)))
time.sleep(3600)" > $OUT/conductor.log 2>&1 &
  CONDUCTOR=$!
  sleep 1
  PDU="--pdu-config $OUT/pdudef.json --pdu-size $PDU_SIZE --poll-us ${EXTRA:-0}"
  $BENCH $COMMON $PDU --role recv --out $OUT/$TAG-recv.json --trace $OUT/$TAG-trace.txt > $OUT/$TAG-recv.log 2>&1 &
  RECV=$!
  $BENCH $COMMON $PDU --role send --out $OUT/$TAG-send.json > $OUT/$TAG-send.log 2>&1 &
  SEND=$!
  sleep 2
  $F/install/bin/hako-cmd start > /dev/null 2>&1
  # The ends report from inside the simulation; stop it once both have written.
  while [ ! -s $OUT/$TAG-recv.json ] || [ ! -s $OUT/$TAG-send.json ]; do
    kill -0 $RECV 2>/dev/null || kill -0 $SEND 2>/dev/null || break
    sleep 0.5
  done
  $F/install/bin/hako-cmd stop > /dev/null 2>&1; sleep 1
  $F/install/bin/hako-cmd reset > /dev/null 2>&1
  # The ends return from the simulation by themselves; then the conductor.
  for i in $(seq 1 20); do kill -0 $RECV 2>/dev/null || kill -0 $SEND 2>/dev/null || break; sleep 0.5; done
  kill -0 $RECV 2>/dev/null && echo "WARNING: the receiver did not end" >&2
  kill -0 $SEND 2>/dev/null && echo "WARNING: the sender did not end" >&2
  kill -TERM $CONDUCTOR 2>/dev/null; wait $CONDUCTOR 2>/dev/null
fi
cat $OUT/$TAG-send.json $OUT/$TAG-recv.json 2>/dev/null
