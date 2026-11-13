#!/bin/bash
# Pick and place with sensors and MoveIt (ros2/docs/evaluation-gazebo.md 3.4), inside the container:
#   run-pick.sh hakoniwa|hakoniwa-plugin|gazebo <runs> [instant]
# hakoniwa: ros2_control as a separate Hakoniwa asset (HakoniwaAssetSystem), with the
# launcher of the comparison (this repository).
# hakoniwa-plugin: ros2_control inside the Robot Runtime (the controller plugin),
# MoveIt in a separate process (config/so101/hakoniwa-launcher-docker-plugin.json).
# Each run starts the simulator, ros2_control and MoveIt, runs pick_and_place.py once
# while self_filter_leak.py counts the arm's points left in MoveIt's map, and stops.
# "instant" (hakoniwa, hakoniwa-plugin): the overhead LiDAR as an instant scan, in copies of the
# configuration under /tmp/compare-pick/instant (the recipe's files are unchanged).
# Results in /tmp/compare-pick (or $COMPARE_PICK_DIR)/<label>-<n>.{out,leak.csv,ps0,ps1}; pick_summary.py sums them.
HERE=$(cd "$(dirname "$0")" && pwd)
SIM=$1 RUNS=$2 MODE=${3:-rotating}
F=/workspace/work-docker-jazzy/foundation
RECIPE=/workspace/work-docker-jazzy/recipes/so101-joint-trajectory-control
O=${COMPARE_PICK_DIR:-/tmp/compare-pick}; mkdir -p $O $RECIPE/logs
LABEL=$SIM
[ "$SIM" = hakoniwa ] && LABEL=hakoniwa-$MODE
[ "$SIM" = hakoniwa-plugin ] && LABEL=hakoniwa-plugin-$MODE
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
if [ "$SIM" = hakoniwa-plugin ] && [ "$MODE" = instant ]; then
  # As the hakoniwa mode: the overhead LiDAR as an instant scan, in copies of the
  # sensor configuration; the plugin manifest and the launcher point at them.
  C=/workspace/hakoniwa-robot-arm/recipes/so101/config/sensors
  PM=$HERE/../../config/so101/plugin/asset-manifest-sensors-plugin.json
  I=$O/instant; rm -rf $I; mkdir -p $I/sensors; cp -r $C/lidar $C/force_torque $C/so101-sensors.json $I/sensors/
  python3 - "$I" "$(readlink -f $PM)" "$(readlink -f $LAUNCHER)" <<'PY'
import json, sys
i, manifest, launcher = sys.argv[1:]
lidar = f"{i}/sensors/lidar/overhead-lidar3d.json"
j = json.load(open(lidar)); j["scan_mode"] = "instant"; json.dump(j, open(lidar, "w"), indent=2)
m = json.load(open(manifest))
n = 0
for c in m["components"]:
    if c["id"] == "sensors":
        c["config"] = f"{i}/sensors/so101-sensors.json"; n += 1
assert n == 1
json.dump(m, open(f"{i}/asset-manifest-sensors-plugin.json", "w"), indent=2)
text = open(launcher).read()
assert manifest in text, (manifest, launcher)
open(f"{i}/launcher.json", "w").write(text.replace(manifest, f"{i}/asset-manifest-sensors-plugin.json"))
PY
  LAUNCHER=$I/launcher.json
fi
if [ "$SIM" = hakoniwa ] && [ "$MODE" = instant ]; then
  C=/workspace/hakoniwa-robot-arm/recipes/so101/config/sensors
  I=$O/instant; rm -rf $I; mkdir -p $I/sensors; cp -r $C/lidar $C/force_torque $C/so101-sensors.json $I/sensors/
  python3 - "$I" "$RECIPE/config/asset-manifest-sensors.json" "$LAUNCHER" <<'PY'
import json, sys
i, manifest, launcher = sys.argv[1:]
lidar = f"{i}/sensors/lidar/overhead-lidar3d.json"
j = json.load(open(lidar)); j["scan_mode"] = "instant"; json.dump(j, open(lidar, "w"), indent=2)
m = json.load(open(manifest))
for c in m["components"]:
    if c["id"] == "sensors":
        c["config"] = f"{i}/sensors/so101-sensors.json"
json.dump(m, open(f"{i}/asset-manifest-sensors.json", "w"), indent=2)
text = open(launcher).read()
assert manifest in text
open(f"{i}/launcher.json", "w").write(text.replace(manifest, f"{i}/asset-manifest-sensors.json"))
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
