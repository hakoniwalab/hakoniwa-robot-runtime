#!/bin/bash
# Physics only (no ros2_control), as fast as possible, three 5 s wall windows each
# (ros2/docs/evaluation-gazebo.md): the simulation seconds per wall second and the CPU
# seconds (all processes) per simulation second.
#  Hakoniwa: the Robot Runtime alone (SO-101 environment, real-time sync 0); progress
#            from the world time (hako-cmd pmeta).
#  Gazebo:   the SO-101 alone, collision hulls, base_link collision left out, no ground
#            (without control the arm would lie on it); progress from Gazebo's own
#            real-time factor (/stats).
HERE=$(cd "$(dirname "$0")" && pwd)
F=/workspace/work-docker-jazzy/foundation
export HAKO_CONFIG_PATH=$F/config/cpp_core_config.json LD_LIBRARY_PATH=$F/install/lib
mkdir -p /tmp/compare
cpu() { python3 -c "
import os
t = 0
for p in os.listdir('/proc'):
    if p.isdigit():
        try:
            f = open('/proc/%s/stat' % p).read().rsplit(')', 1)[1].split(); t += int(f[11]) + int(f[12])
        except Exception:
            pass
print(t / os.sysconf('SC_CLK_TCK'))"; }

echo "== Hakoniwa (Robot Runtime only)"
$F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher $HERE/launcher-hakoniwa-physics.json \
  --background /tmp/compare/physics-session.json > /dev/null 2>&1
sleep 6
for k in 1 2 3; do
  t0=$($F/install/bin/hako-cmd pmeta 2>/dev/null | awk '/time_usec/{print $3}'); c0=$(cpu); w0=$(date +%s.%N); sleep 5
  t1=$($F/install/bin/hako-cmd pmeta 2>/dev/null | awk '/time_usec/{print $3}'); c1=$(cpu); w1=$(date +%s.%N)
  python3 -c "s = ($t1 - $t0) / 1e6; print('ratio %.1f, CPU per sim s %.4f' % (s / ($w1 - $w0), ($c1 - $c0) / s))"
done
$F/install/python/bin/python -m hakoniwa_pdu.apps.launcher.hako_launcher_ctl terminate /tmp/compare/physics-session.json > /dev/null 2>&1
sleep 3

echo "== Gazebo (SO-101 alone)"
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/ros2-work-jazzy-asset/install/setup.bash
SHARE=$(ros2 pkg prefix hakoniwa_ros2_control)/share/hakoniwa_ros2_control
python3 -c "
import importlib.util, re
spec = importlib.util.spec_from_file_location('gz', '$SHARE/launch/so101_gz.launch.py'); gz = importlib.util.module_from_spec(spec); spec.loader.exec_module(gz)
path = gz.resolved_urdf('/workspace/work-docker-jazzy/model-forge/so101/source/Simulation/SO101/so101_new_calib.urdf',
                        '/workspace/work-docker-jazzy/model-forge/so101/gazebo-hull')
t = open(path).read().replace('<robot name=\"so101_new_calib\">', '<robot name=\"so101_new_calib\"><link name=\"world\"/><joint name=\"w\" type=\"fixed\"><parent link=\"world\"/><child link=\"base_link\"/></joint>', 1)
open('/tmp/compare/so101_gz_physics.urdf', 'w').write(t)
w = open('$SHARE/config/so101/gazebo/so101_world.sdf.in').read().replace('@REAL_TIME_FACTOR@', '0')
open('/tmp/compare/world_physics.sdf', 'w').write(re.sub(r'<model name=\"ground\">.*?</model>', '', w, flags=re.S))
"
gz sim -s -r -v 0 /tmp/compare/world_physics.sdf > /tmp/compare/physics-gz.log 2>&1 &
sleep 3
gz service -s /world/so101/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'sdf_filename: "/tmp/compare/so101_gz_physics.urdf" name: "so101"' > /dev/null
sleep 3
for k in 1 2 3; do
  c0=$(cpu); r=$(timeout 5 gz topic -e -t /stats | grep -E '^real_time_factor' | awk '{s += $2; n++} END {print s / n}'); c1=$(cpu)
  python3 -c "r = $r; print('ratio %.1f, CPU per sim s %.4f' % (r, ($c1 - $c0) / (r * 5)))"
done
pkill -f "gz sim"
