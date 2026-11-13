#!/bin/bash
# Gazebo side of the comparison (Docker only, docs/evaluation-gazebo.md):
#   run-gazebo.sh rt|max|urdf-max <runs> [speed]
# gz sim -s with gz_ros2_control (so101_gz.launch.py); rt/max with the collision hulls
# (/workspace/work-docker-jazzy/model-forge/so101/gazebo-hull, tools/so101_collision_hulls.py),
# urdf-max with the URDF's own collision meshes. Results as run-hakoniwa.sh, label gazebo-<mode>.
HERE=$(cd "$(dirname "$0")" && pwd)
RTF=1.0; case "$1" in *max) RTF=0;; esac
HULLS=collision_hulls:=/workspace/work-docker-jazzy/model-forge/so101/gazebo-hull; [ "$1" = urdf-max ] && HULLS=
mkdir -p /tmp/compare
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/ros2-work-jazzy-asset/install/setup.bash
ros2 launch hakoniwa_ros2_control so101_gz.launch.py \
  so101_urdf:=/workspace/work-docker-jazzy/model-forge/so101/source/Simulation/SO101/so101_new_calib.urdf \
  real_time_factor:=$RTF $HULLS > /tmp/compare/gazebo-$1-launch.log 2>&1 &
for i in $(seq 1 60); do timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active' && break; sleep 1; done
sleep 3
for k in $(seq 1 $2); do
  if [ "$3" = speed ]; then python3 $HERE/speed.py gazebo-$1 5
  else python3 $HERE/measure.py --label gazebo-$1 --out /tmp/compare/gazebo-$1-$k.json; sleep 2; fi
done
pkill -f "ros2 launch hakoniwa_ros2_control so101_gz"; pkill -f "gz sim"; pkill -f parameter_bridge; pkill -f robot_state_publisher
sleep 3
