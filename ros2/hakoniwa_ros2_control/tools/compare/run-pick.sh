#!/bin/bash
# Pick and place with sensors and MoveIt (ros2/docs/evaluation-gazebo.md 3.4), inside the container:
#   run-pick.sh hakoniwa|hakoniwa-plugin|gazebo <runs> [instant]
# hakoniwa: ros2_control as a separate Hakoniwa asset (HakoniwaAssetSystem), with the
# launcher of the comparison (this repository).
# hakoniwa-plugin: ros2_control inside the Robot Runtime (the controller plugin),
# MoveIt in a separate process (config/so101/hakoniwa-launcher-docker-plugin.json).
# Each run starts the simulator, ros2_control and MoveIt, runs pick_and_place.py once
# while self_filter_leak.py counts the arm's points left in MoveIt's map, and stops.
# Hakoniwa sensors: SENSOR_SET=hakoniwa (default, Hakoniwa's sensor models, as the
# comparison measured; needs the hakoniwa implementation built in) or reference (the
# true values). "instant": the LiDAR as an instant scan. Copies of the configuration
# go under /tmp/compare-pick/sensors-<set>-<mode> (the recipe's files are unchanged).
# Results in /tmp/compare-pick (or $COMPARE_PICK_DIR)/<label>-<n>.{out,leak.csv,ps0,ps1}; pick_summary.py sums them.
HERE=$(cd "$(dirname "$0")" && pwd)
SIM=$1 RUNS=$2 MODE=${3:-rotating}
F=/workspace/work-docker-jazzy/foundation
RECIPE=/workspace/work-docker-jazzy/recipes/so101-joint-trajectory-control
O=${COMPARE_PICK_DIR:-/tmp/compare-pick}; mkdir -p $O $RECIPE/logs
SET=${SENSOR_SET:-hakoniwa}
LABEL=$SIM
[ "$SIM" = hakoniwa ] && LABEL=hakoniwa-$MODE
[ "$SIM" = hakoniwa-plugin ] && LABEL=hakoniwa-plugin-$MODE
[ "$SIM" != gazebo ] && [ "$SET" != hakoniwa ] && LABEL=$LABEL-$SET
export HAKO_CONFIG_PATH=$F/config/cpp_core_config.json LD_LIBRARY_PATH=$F/install/lib
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/ros2-work-jazzy-asset/install/setup.bash; set -u

# The separate-process form as measured before (its launcher sets
# HAKO_CONDUCTOR_IDLE_SLEEP_USEC and uses the recipe's build).
LAUNCHER=/workspace/hakoniwa-robot-runtime/ros2/hakoniwa_ros2_control/config/so101/hakoniwa-launcher-docker-asset.json
ROS_LOG=ros2-control
if [ "$SIM" = hakoniwa-plugin ]; then
  LAUNCHER=$HERE/../../config/so101/hakoniwa-launcher-docker-plugin.json
  ROS_LOG=moveit
fi
if [ "$SIM" != gazebo ]; then
  # The Hakoniwa sensors in copies of the configuration (the recipe's files are
  # unchanged): the sensor set of the comparison, SENSOR_SET (default hakoniwa:
  # so101-sensors-hakoniwa.json, Hakoniwa's sensor models with the rotating LiDAR,
  # as measured; reference: so101-sensors.json, the true values of an instant scan),
  # and with "instant" that set's LiDAR as an instant scan.
  C=/workspace/hakoniwa-robot-arm/recipes/so101/config/sensors
  SETFILE=so101-sensors-$SET.json; [ "$SET" = reference ] && SETFILE=so101-sensors.json
  MANIFEST=$RECIPE/config/asset-manifest-sensors.json
  [ "$SIM" = hakoniwa-plugin ] && MANIFEST=$(readlink -f $HERE/../../config/so101/plugin/asset-manifest-sensors-plugin.json)
  I=$O/sensors-$SET-$MODE; rm -rf $I; mkdir -p $I/sensors; cp -r $C/lidar $C/force_torque $C/$SETFILE $I/sensors/
  python3 - "$I" "$SETFILE" "$MODE" "$MANIFEST" "$(readlink -f $LAUNCHER)" <<'PY'
import json, sys
i, setfile, mode, manifest, launcher = sys.argv[1:]
sensors = json.load(open(f"{i}/sensors/{setfile}"))
if mode == "instant":
    for s in sensors["sensors"]:
        if s["type"] == "lidar":
            path = f"{i}/sensors/{s['definition']}"
            j = json.load(open(path)); j["scan_mode"] = "instant"; json.dump(j, open(path, "w"), indent=2)
m = json.load(open(manifest))
n = 0
for c in m["components"]:
    if c["id"] == "sensors":
        c["config"] = f"{i}/sensors/{setfile}"; n += 1
assert n == 1
name = manifest.rsplit("/", 1)[1]
json.dump(m, open(f"{i}/{name}", "w"), indent=2)
text = open(launcher).read()
assert manifest in text, (manifest, launcher)
open(f"{i}/launcher.json", "w").write(text.replace(manifest, f"{i}/{name}"))
PY
  LAUNCHER=$I/launcher.json
fi

start() {
  if [ "$SIM" != gazebo ]; then
    $F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher $LAUNCHER \
      --background $O/session.json > $O/$LABEL-launcher.out 2>&1
  else
    setsid ros2 launch hakoniwa_ros2_control so101_gz.launch.py \
      so101_urdf:=/workspace/work-docker-jazzy/model-forge/so101/source/Simulation/SO101/so101_new_calib.urdf \
      collision_hulls:=/workspace/work-docker-jazzy/model-forge/so101/gazebo-hull sensors:=true moveit:=true \
      > $O/$LABEL-launch.out 2>&1 &
    GZ_LAUNCH=$!
  fi
  for i in $(seq 1 120); do timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active' && break; sleep 1; done
  for i in $(seq 1 60); do timeout 5 ros2 node list 2>/dev/null | grep -q move_group && break; sleep 1; done
  sleep 5
}
stop() {
  if [ "$SIM" != gazebo ]; then
    cp $RECIPE/logs/$ROS_LOG.out $O/$LABEL-$1.ros.out
    $F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher_ctl terminate $O/session.json > /dev/null 2>&1
  else
    cp $O/$LABEL-launch.out $O/$LABEL-$1.ros.out
    kill -INT -$GZ_LAUNCH 2>/dev/null; sleep 5; kill -KILL -$GZ_LAUNCH 2>/dev/null
  fi
  sleep 5
}

for k in $(seq 1 $RUNS); do
  start
  ps -eo pid=,times=,args= > $O/$LABEL-$k.ps0
  python3 $HERE/self_filter_leak.py $O/$LABEL-$k.leak.csv > /dev/null 2>&1 &
  LEAK=$!
  U0=$(awk '/usage_usec/{print $2}' /sys/fs/cgroup/cpu.stat); T0=$(date +%s%N)
  timeout 300 ros2 run hakoniwa_ros2_control pick_and_place.py > $O/$LABEL-$k.out 2>&1; RC=$?
  T1=$(date +%s%N); U1=$(awk '/usage_usec/{print $2}' /sys/fs/cgroup/cpu.stat)
  ps -eo pid=,times=,args= > $O/$LABEL-$k.ps1
  kill -INT $LEAK; wait $LEAK 2>/dev/null
  echo "rc=$RC wall_ms=$(( (T1 - T0) / 1000000 )) cpu_ms=$(( (U1 - U0) / 1000 ))" >> $O/$LABEL-$k.out
  stop $k
  echo "$LABEL $k: $(tail -1 $O/$LABEL-$k.out) $(grep FAILED $O/$LABEL-$k.out)"
done
