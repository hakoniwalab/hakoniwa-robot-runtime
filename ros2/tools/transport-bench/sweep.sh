#!/bin/bash
# The sizes x rates sweep (README.md), inside the container: every case 3 times,
# ROS 2 (default RMW, reliable) and the PDU polled every 100 us and without pause;
# CASES picks them (default "ros pdu-100 pdu-0"; gz: Gazebo's transport).
#   [TYPE=fixed] [CASES="..."] sweep.sh [results dir (default /tmp/transport-bench-sweep[-fixed])]
# Stops at the first case without a result.
HERE=$(cd "$(dirname "$0")" && pwd)
TYPE=${TYPE:-cloud}; export TYPE
SUFFIX=; [ "$TYPE" = fixed ] && SUFFIX=-fixed
R=${1:-/tmp/transport-bench-sweep$SUFFIX}
mkdir -p $R
for SIZE in 1024 65536 327680 1048576 4194304; do
  for RATE in 10 100 1000; do
    COUNT=$((RATE * 5)); [ $COUNT -lt 100 ] && COUNT=100
    for K in 1 2 3; do
      for CASE in ${CASES:-ros pdu-100 pdu-0}; do
        case $CASE in
          ros|gz) ARGS="$CASE $SIZE $RATE $COUNT" ;;
          pdu-*) ARGS="pdu $SIZE $RATE $COUNT ${CASE#pdu-}" ;;
        esac
        T=${CASE%%-*}$SUFFIX-$SIZE-$RATE
        timeout 120 $HERE/run.sh $ARGS > /dev/null 2>> $R/warnings.txt
        if [ ! -s /tmp/transport-bench/$T-recv.json ]; then echo "FAILED $CASE $SIZE $RATE $K"; exit 1; fi
        cp /tmp/transport-bench/$T-recv.json $R/$CASE-$SIZE-$RATE-$K-recv.json
        cp /tmp/transport-bench/$T-send.json $R/$CASE-$SIZE-$RATE-$K-send.json
      done
    done
    echo "done $SIZE $RATE"
  done
done
