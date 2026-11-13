#!/bin/bash
# The controller-plugin form in the Gazebo comparison (docs/evaluation-gazebo.md 3.2, 3.3),
# inside the container:
#   run-hakoniwa-plugin.sh rt|max <runs> [speed]
# Starts only the Robot Runtime (SO-101, no sensors) with ros2_control inside it (the
# controller plugin; launcher-hakoniwa-plugin-<mode>.json), then measures <runs>
# trajectories with the comparison's measure.py (results in
# /tmp/compare/hakoniwa-plugin-<mode>-<n>.json) or, with "speed", the speed over 5 s
# (speed.py). rt: the conductor sleeps while it cannot advance
# (HAKO_CONDUCTOR_IDLE_SLEEP_USEC 200, as the pick launcher); max: no sleep (it
# would add a wait to each step and hold the speed back).
# measure.py and speed.py are used unchanged from the comparison checkout.
HERE=$(cd "$(dirname "$0")" && pwd)
TOOLS=/workspace/hakoniwa-robot-runtime/ros2/hakoniwa_ros2_control/tools/compare
MODE=$1
RUNS=$2
LABEL=hakoniwa-plugin-$MODE
F=/workspace/work-docker-jazzy/foundation
export HAKO_CONFIG_PATH=$F/config/cpp_core_config.json
export LD_LIBRARY_PATH=$F/install/lib
mkdir -p /tmp/compare
$F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher $HERE/launcher-hakoniwa-plugin-$MODE.json \
  --background /tmp/compare/$LABEL-session.json > /tmp/compare/$LABEL-launcher.out 2>&1
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/ros2-work-jazzy-asset/install/setup.bash
for i in $(seq 1 60); do
  if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active'; then
    break
  fi
  sleep 1
done
sleep 3
for k in $(seq 1 $RUNS); do
  if [ "$3" = speed ]; then
    python3 $TOOLS/speed.py $LABEL 5
  else
    python3 $TOOLS/measure.py --label $LABEL --out /tmp/compare/$LABEL-$k.json
    sleep 2
  fi
done
$F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher_ctl terminate /tmp/compare/$LABEL-session.json > /dev/null 2>&1
sleep 3
