#!/bin/bash
# Hakoniwa side of the comparison (Docker only, ros2/docs/evaluation-gazebo.md):
#   run-hakoniwa.sh rt|max <runs> [speed]
# Starts the Robot Runtime (SO-101, no sensors) and ros2_control (HakoniwaAssetSystem)
# with the Hakoniwa launcher, then measures <runs> trajectories (measure.py, results in
# /tmp/compare/hakoniwa-<mode>-<n>.json) or, with "speed", the speed over 5 s (speed.py).
HERE=$(cd "$(dirname "$0")" && pwd)
F=/workspace/work-docker-jazzy/foundation
export HAKO_CONFIG_PATH=$F/config/cpp_core_config.json LD_LIBRARY_PATH=$F/install/lib
mkdir -p /tmp/compare
$F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher $HERE/launcher-hakoniwa-$1.json \
  --background /tmp/compare/hakoniwa-$1-session.json > /tmp/compare/hakoniwa-$1-launcher.out 2>&1
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/ros2-work-jazzy-asset/install/setup.bash
for i in $(seq 1 60); do timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active' && break; sleep 1; done
sleep 3
for k in $(seq 1 $2); do
  if [ "$3" = speed ]; then python3 $HERE/speed.py hakoniwa-$1 5
  else python3 $HERE/measure.py --label hakoniwa-$1 --out /tmp/compare/hakoniwa-$1-$k.json; sleep 2; fi
done
$F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher_ctl terminate /tmp/compare/hakoniwa-$1-session.json > /dev/null 2>&1
sleep 3
