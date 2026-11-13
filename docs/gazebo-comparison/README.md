# Hakoniwa vs Gazebo: SO-101 with ros2_control and MoveIt

*Draft. Numbers come from one clean measurement run (2026-11-14) and are filled in when it ends.*

We ran the same robot, the same ROS 2 controllers, the same trajectories and the same pick-and-place script on Hakoniwa and on Gazebo, in the same container on the same day, and measured transport, physics, control and a full MoveIt task.

## Summary

| | Hakoniwa | Gazebo |
|---|---|---|
| Transport, 4 MB point cloud at 100 Hz (median latency) | TBD | TBD |
| Physics only (real-time factor) | TBD | TBD |
| Control in real time (max tracking error) | TBD | TBD |
| As fast as possible (real-time factor / control updates per simulated second / max tracking error) | TBD | TBD |
| Pick and place with a 3D LiDAR and MoveIt (success, CPU) | TBD | TBD |

## 1. What is compared

### Hakoniwa: ros2_control inside the simulator

Hakoniwa runs the physics (MuJoCo) in its Robot Runtime, a Hakoniwa asset. ros2_control runs **inside the same process** as a plugin: the Robot Runtime calls `ControllerManager::read/update/write` on its simulation thread, in simulation time, every control period (2 ms physics steps, 100 Hz control). Only what other programs need goes out on ROS 2: `/clock`, the joint states, the sensors (`/pdu/lidar_scan`, `/pdu/wrist_wrench`) and the controllers' actions for MoveIt. This is the same structure as Hakoniwa's drone simulator, where the flight controller runs in the physics process (Control Link).

The plugin boundary is C only and carries no ROS types; ROS lives in the plugin. Design: [design-controller-plugin.md](../design-controller-plugin.md).

### Gazebo: gz_ros2_control

Gazebo Harmonic runs as a server (`gz sim -s`), with `gz_ros2_control`: the controller manager runs inside Gazebo, in simulation time. Sensors reach ROS 2 through `ros_gz_bridge`.

### Kept the same

| Item | Both |
|---|---|
| Robot | SO-101, the same URDF |
| Controllers | `joint_trajectory_controller` (arm, gripper), `joint_state_broadcaster`, controller manager at 100 Hz (`ros2_controllers.yaml`) |
| Physics step | 2 ms |
| Trajectory (control tests) | 3 points in 9 s |
| Work cell (pick and place) | floor, front wall, two posts, a 30 mm / 30 g cube, an overhead 3D LiDAR (360 x 32 rays, 2 Hz, 0.05-2 m), a wrist force-torque sensor (50 Hz) |
| Grasp contact | only the jaw pads (40 x 12 x 3 mm, friction 2.0) touch objects; friction pairs matched (MuJoCo uses the larger of two surfaces, Gazebo the smaller) |
| MoveIt | the same move_group configuration, obstacle map from the LiDAR |
| Script | the same `pick_and_place.py` (13 steps) |
| Machine | one container, one run at a time, nothing else running |

### Not the same (by design of each simulator)

- Position control: MuJoCo uses a force-limited position actuator; `gz_ros2_control` turns a position command into a joint velocity. Tracking errors include this difference.
- Collision shapes in Gazebo were set to what MuJoCo uses (convex hulls of the URDF meshes, no collision for the base fixed to the world); with the URDF meshes as they are, Gazebo did not reach real time on this machine (section 3).
- The Hakoniwa LiDAR rotates (one scan over 0.5 s, like a real spinning LiDAR); Gazebo's `gpu_lidar` captures the whole scan at once. Hakoniwa can also scan at once (`scan_mode: instant`); both are measured.

## 2. Environment and versions

TBD: the version table from the measurement run (commits of hakoniwa-robot-runtime, hakoniwa-robot-arm, hakoniwa-pdu-registry, the transport bench and the comparison tools; Gazebo, ROS 2 and container versions).

- macOS (Apple Silicon), Docker Desktop: 6 cores, 8 GB, Linux arm64, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. No GPU.

## 3. Method

| Metric | How |
|---|---|
| Real-time factor | simulated time advanced over 5 s of wall time (`/clock`) |
| CPU | CPU time of every process in the container (`/proc`), per simulated second (control tests) or in cores over the task (pick and place) |
| Control updates per simulated second | timestamps of `/arm_controller/controller_state` |
| Tracking error | `joint_trajectory_controller`'s own error (desired - actual), the 5 arm joints, max and mean over the trajectory |
| Repeatability | the same trajectory 3 times, aligned at its start; the largest joint difference between any two runs |
| Transport latency | one program, only the transport swapped (ROS 2, ROS 2 zero copy, gz-transport, Hakoniwa shared memory); the sender's and receiver's message types, CLOCK_MONOTONIC |
| Pick and place | 13 steps all OK; the LiDAR points left around the arm after MoveIt's self filter, per scan |

Every case is run 3 times.

## 4. Results

TBD after the run: 4.1 Transport, 4.2 Physics only, 4.3 Control in real time, 4.4 As fast as possible, 4.5 Pick and place.

## 5. Discussion

TBD after the run. Points to cover:
- Control in simulation time inside the simulator: what it changes when running faster than real time.
- The rotating LiDAR's motion distortion and MoveIt's self filter.
- Where Gazebo is still ahead.

## 6. Reproduce

TBD after the run: the exact build and run commands (the run uses fixed commits and fresh Release builds).

## 7. Limitations

- One machine (Docker on macOS, 6 cores, no GPU) and one robot (SO-101).
- No sensor noise in these runs.
- Transport latencies are within one machine; they are not the speed of the whole simulation.
