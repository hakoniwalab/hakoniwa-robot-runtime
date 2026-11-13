# Hakoniwa vs Gazebo: SO-101 with ros2_control and MoveIt

We ran the same robot, the same ROS 2 controllers, the same trajectories and the same pick-and-place script on Hakoniwa and on Gazebo, in the same container on the same day, and measured transport, physics, control and a full MoveIt task.

The main result is about time, more than speed. Hakoniwa runs ros2_control inside the simulator and steps it in simulation time, so how fast the simulation runs no longer changes what the controllers do: at 75 times real time, the controllers ran every 10 ms of simulated time without missing one, and tracked the trajectory exactly as in real time. Speed then comes on top of that.

## Summary

| | Hakoniwa | Gazebo |
|---|---|---|
| Transport: 4 MB point cloud at 100 Hz (median latency) | 0.61 ms | 4.05 ms (ROS 2: 4.71 ms) |
| Physics only (real-time factor) | 119-131 | 22-26 |
| Control in real time: max tracking error | 0.0019 rad | 0.031 rad |
| Control in real time: CPU per simulated second | 0.54-0.58 | 1.22-1.40 |
| As fast as possible: real-time factor | 73-77 | 11-14 |
| As fast as possible: max tracking error | 0.0019 rad (same as in real time) | 0.031 rad |
| As fast as possible: difference between runs | 0.0001 rad | 0.0048 rad |
| Pick and place with a 3D LiDAR and MoveIt: success / CPU | 3/3 (instant LiDAR), 2/3 (rotating LiDAR) / 0.55-0.60 cores | 3/3 / 1.18 cores |

In short:

- **Control in simulation time**: the controllers run every control period of simulated time however fast the simulation runs (checked step by step in the Robot Runtime's own log), and the tracking error is the same in real time and at 75 times real time (section 5.1).
- **Repeatability**: runs of the same trajectory differ by up to 0.0001 rad, and the difference does not grow along the trajectory: it is the difference in the arm's pose at the moment the trajectory started, which depends on when the goal arrived from outside. They are not bit-for-bit identical (section 5.2). Gazebo's runs differ by 0.002-0.005 rad.
- **Speed**: the physics alone runs about 5 times faster than Gazebo's, and the whole control loop with ros2_control about 6 times faster.
- **Transport**: Hakoniwa's shared memory has the lowest median latency at every size at 100 Hz. Under overload (4 MB at 1000 Hz) nothing is corrupted and no backlog builds up; ROS 2 zero copy has a lower median there, but in our configuration some messages were overwritten while being read (section 4.1).
- **CPU**: in real time, Hakoniwa uses less than half of Gazebo's CPU, for the control test and for the full task.
- **The full task**: both pass pick and place. Hakoniwa's rotating LiDAR, like a real one, can make MoveIt mistake its own moving arm for an obstacle (1 run in 6 over two sets of runs); a LiDAR that scans at once, as Gazebo's, does not (section 5.3).

## 1. What is compared

### Hakoniwa: ros2_control inside the simulator

Hakoniwa runs the physics (MuJoCo) in its Robot Runtime, a Hakoniwa asset. ros2_control runs **inside the same process** as a plugin: the Robot Runtime calls `ControllerManager::read/update/write` on its simulation thread, in simulation time, every control period (2 ms physics steps, 100 Hz control). Only what other programs need goes out on ROS 2: `/clock`, the joint states, the sensors (`/pdu/lidar_scan`, `/pdu/wrist_wrench`) and the controllers' actions for MoveIt. This is the same structure as Hakoniwa's drone simulator, where the flight controller runs in the physics process.

The plugin boundary is C only and carries no ROS types; ROS lives in the plugin. Design: [design-controller-plugin.md](../../../docs/design-controller-plugin.md).

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
- Collision shapes in Gazebo were set to what MuJoCo uses (convex hulls of the URDF meshes, no collision for the base fixed to the world); with the URDF meshes as they are, Gazebo did not reach real time on this machine.
- The Hakoniwa LiDAR rotates (one scan over 0.5 s, like a real spinning LiDAR); Gazebo's `gpu_lidar` captures the whole scan at once. Hakoniwa can also scan at once (`scan_mode: instant`); both are measured.

## 2. Environment and versions

| | |
|---|---|
| Host | macOS (Apple Silicon), Docker Desktop: 6 CPUs, 8 GB; no GPU |
| Container | Ubuntu 24.04.5, Linux 5.15 aarch64, GCC 13.3.0 |
| ROS 2 | Jazzy; ros2_control 4.48.1, gz_ros2_control 1.2.20, MoveIt (move_group) 2.12.4, rmw_fastrtps_cpp 8.4.4 |
| Gazebo | Harmonic, gz sim 8.15.0 |
| Hakoniwa core | hakoniwa-core-pro 1.4.0 (722ce02), hakoniwa-pdu-endpoint 1.0.0 (3b7b25a), hakoniwa-pdu-python 1.7.0 (5a85ca8) |
| hakoniwa-robot-runtime | Robot Runtime and controller plugin 3826abf; Gazebo side and comparison tools 9556611 |
| hakoniwa-robot-arm | 3e15d54 |
| hakoniwa-pdu-registry | `main` a8ad04c (PDU types and converters) |
| Transport bench | hakoniwa-robot-runtime 3cc055d (`ros2/tools/transport-bench`) |

Every Hakoniwa program was built fresh (Release) from these commits for this run. All of them are on the main branches of these repositories.

## 3. Method

| Metric | How |
|---|---|
| Real-time factor | simulated time advanced over 5 s of wall time (`/clock`) |
| CPU | CPU time of every process in the container (`/proc`), per simulated second (control tests) or in cores over the task (pick and place) |
| Control updates per simulated second | timestamps of `/arm_controller/controller_state` |
| Tracking error | `joint_trajectory_controller`'s own error (desired - actual), the 5 arm joints, max over the trajectory |
| Difference between runs | the same trajectory run several times; runs aligned on the control tick (10 ms) where the controller's desired positions match, then the largest joint difference at the same tick, from the `controller_state` messages received (section 5.2); for Hakoniwa also from the Robot Runtime's log of every control step |
| Transport latency | one program, only the transport swapped; the sender's and receiver's message types; CLOCK_MONOTONIC from the send call to the receive callback |
| Pick and place | 13 steps all OK; the LiDAR points left around the arm after MoveIt's self filter, per scan |

Every case is run 3 times.

## 4. Results

### 4.1 Transport

One program sends a message, another receives it; only the transport is swapped:

- **ROS 2**: rclcpp, the default RMW (Fast DDS), reliable.
- **ROS 2 zero copy**: Fast DDS data sharing with loaned messages (fixed-size types only).
- **gz-transport**: Gazebo's transport, default settings.
- **Hakoniwa**: Hakoniwa's shared memory (PDU); the receiver checks for new data every 100 µs.

Sizes 1 KB to 4 MB, at 10, 100 and 1000 Hz, 3 runs each.

**Point cloud (`sensor_msgs/PointCloud2`, converted to and from the PDU on the Hakoniwa side), median latency at 100 Hz**

| Size | ROS 2 | gz-transport | Hakoniwa |
|---|---|---|---|
| 1 KB | 0.41 ms | 0.41 ms | 0.15 ms |
| 64 KB | 0.43 ms | 0.56 ms | 0.14 ms |
| 320 KB | 0.66 ms | 1.02 ms | 0.17 ms |
| 1 MB | 1.59 ms | 2.37 ms | 0.21 ms |
| 4 MB | 4.71 ms | 4.05 ms | 0.61 ms |

**Fixed-size message (no conversion; ROS 2 with zero copy), median latency at 100 Hz**

| Size | ROS 2 zero copy | gz-transport | Hakoniwa |
|---|---|---|---|
| 1 MB | 0.34 ms | 2.45 ms | 0.15 ms |
| 4 MB | 0.39 ms | 4.10 ms | 0.29 ms |

**Overload: 4 MB at 1000 Hz**

| | Received / sent | Corrupted | Median latency |
|---|---|---|---|
| ROS 2 (point cloud) | 3800-3851 / 5000 | 0 | 8.1 ms |
| ROS 2 zero copy (fixed) | 4972-4985 / 5000 | **58** | 0.13 ms |
| gz-transport (point cloud) | 4657-4790 / 5000 | 0 | **461 ms** (a backlog builds up) |
| Hakoniwa (point cloud) | 4990-4991 / 5000 | 0 | 0.52 ms |
| Hakoniwa (fixed) | 4972-4994 / 5000 | 0 | 0.27 ms |

Under overload the three fail differently. ROS 2 drops messages, and with zero copy some messages arrive corrupted. gz-transport delivers almost everything, but a backlog builds up to about half a second. Hakoniwa keeps only the latest value: a message not yet read when the next one arrives is skipped, and nothing is corrupted or late.

The corrupted zero-copy messages were examined afterwards (3 more runs). Each carried the sequence number of one message in its head, but its body (from its first byte) belonged to the message 11 later: the shared-memory sample was reused by the sender while the receiver was still reading it. The publisher kept the default history depth of 10 (KEEP_LAST), so the sender cycles through 11 samples. With a depth of 100, no message was corrupted in 3 runs, and the median latency rose from 0.14 ms to 0.61 ms (messages wait in the history). Fast DDS offers a check that a loaned sample is still valid after reading it (`is_sample_valid`); the ROS 2 layer we used (rmw_fastrtps, Jazzy) does not call it when it hands a loaned message to the subscription. So this is a property of this configuration (zero copy with a short KEEP_LAST history under overload), not a defect of zero copy as such.

All sizes and rates, with p99, max and CPU: [transport-tables.md](transport-tables.md).

### 4.2 Physics only

The SO-101 alone, no controllers, no ROS, as fast as possible (3 invocations x 3 measurements):

| | Real-time factor | CPU per simulated second |
|---|---|---|
| Hakoniwa (Robot Runtime) | 119-131 | 0.015-0.017 |
| Gazebo (`gz sim -s`) | 22-26 | 0.057-0.069 |

### 4.3 Control in real time

The trajectory with ros2_control, the simulators paced to real time:

| | Max tracking error | CPU per simulated second | Control updates per simulated second | Difference between runs |
|---|---|---|---|---|
| Hakoniwa | 0.0019 rad | 0.54-0.58 | 100 (every 10 ms) | 0.0000 rad |
| Gazebo | 0.031 rad | 1.22-1.40 | 100 (10-14 ms apart) | 0.0017 rad |

Hakoniwa paces itself to real time by sleeping while it cannot advance, so its CPU here is mostly ROS 2 and the controllers.

### 4.4 As fast as possible

| | Real-time factor (trajectory / 5 s) | Max tracking error | CPU per simulated second | Difference between runs |
|---|---|---|---|---|
| Hakoniwa | 73-77 / 76-77 | 0.0019 rad | 0.056-0.059 | 0.0001 rad |
| Gazebo | 11-12 / 12-14 | 0.031 rad | 0.35-0.42 | 0.0048 rad |

As fast as possible, Hakoniwa's controllers still ran every 10 ms of simulated time (the Robot Runtime's own record of every control step); the measuring program, a separate ROS 2 subscriber, received 86-92 of the 100 `controller_state` messages per simulated second. The tracking error is the same as in real time: running faster does not change the control.

### 4.5 Pick and place

| | Hakoniwa (LiDAR rotating) | Hakoniwa (LiDAR instant) | Gazebo |
|---|---|---|---|
| 13 steps all OK | 2 / 3 | 3 / 3 | 3 / 3 |
| CPU, whole container (cores) | 0.60 | 0.55 | 1.18 |
| Simulator process (cores) | 0.25 (Robot Runtime with ros2_control) | 0.22 | 0.62 (`gz sim`) + 0.09 (bridge) |
| move_group (cores) | 0.19 | 0.18 | 0.30 |
| Points left around the arm while it moves (mean / max per scan) | 19.8 / 235 | 0.3 / 5 | 0 / 0 |
| Points left around the arm while it is still (mean / max per scan) | 1.1 / 3 | 1.0 / 3 | 0 / 0 |

The failed run stopped at step 8 (carrying the cube to the place pose): MoveIt found a contact between the obstacle map and its own upper arm, and aborted the motion (section 5.3). A second set of runs (3 rotating, 3 instant), made to sort the left points by robot part, passed 3 / 3 with both LiDAR modes.

## 5. Discussion

### 5.1 Control in simulation time, faster than real time

Running ros2_control inside the simulator showed a problem that only appears when the simulation runs much faster than real time. In ROS 2 Jazzy, once the controller manager's node clock has started, ros2_control gives the controllers that clock's time, not the time passed to `update()`. With `use_sim_time`, the node clock follows `/clock` through a subscription. At about 70 times real time, `/clock` messages fall behind: the controllers saw the same time for several control periods, their trajectory stood still, and then jumped. The tracking error reached 0.05-0.09 rad in some runs and was different in every run.

The plugin now sets the node clocks of the controller manager and of every controller to the step's simulation time itself, before each `read/update/write`; they no longer follow `/clock` (which is still published for MoveIt and the other programs). With this, the tracking error was 0.0019 rad in every run: 10 runs in real time and 10 as fast as possible, then 3 + 3 in the clean run (section 4.3, 4.4). Before the fix, the same measurement as fast as possible gave 0.006-0.047 rad.

The Robot Runtime's own log of every control step (6 more runs, 3 in real time and 3 as fast as possible) shows the controllers ran every 10 ms of simulated time, without a missed or repeated step, at 75 times real time as in real time. The control period is set by simulation time, not by how fast the simulation runs.

This is the point of running control inside the simulator, in simulation time: speeding the simulation up does not change what the controllers do.

### 5.2 Repeatability: what is and is not the same

Two things are kept apart here: the control period (does the controller run at every period of simulated time?) and the result (does the arm make the same motion in every run?).

- **Control period**: deterministic. In the Robot Runtime's log of every control step, the controllers ran every 10 ms of simulated time in every run, real time or 75 times faster.
- **Result**: close, but not bit-for-bit. In the same log, the joint positions of three runs differ by up to 0.0001 rad (as fast as possible) and 0.00009 rad (real time). At every step of the trajectory the difference is no larger than the difference in the arm's pose at the step the trajectory started: the control adds none. The start pose differs because the goal comes from a program outside the simulator (here the measuring program; in the pick task, MoveIt), at a wall-clock moment, so the trajectory starts at a different simulated time in each run (4.7-5.7 s in real time, 340-570 s as fast as possible) while the arm, holding its pose, is still settling by tiny amounts. Starting every run's trajectory at the same simulated step would remove this, and would show whether the runs are then bit-for-bit identical; we have not done that yet.

The numbers in sections 4.3 and 4.4 (Hakoniwa 0.0000 and 0.0001 rad, Gazebo 0.0017 and 0.0048 rad) come from the same method for both simulators: the `controller_state` messages a separate ROS 2 program received. When the simulation runs 70 times faster than real time, that program misses 8-14 % of these messages, so the runs cannot be aligned on the first sample that moved (that misread the start by tens of milliseconds and alone showed as a 0.07 rad "difference"). They are aligned instead on the control tick where the controller's desired positions match, and compared only at ticks both runs received. For Hakoniwa, the full log above confirms the bound for every step.

### 5.3 The rotating LiDAR and MoveIt's self filter

Hakoniwa's rotating LiDAR takes 0.5 s for one scan, and the cloud is stamped with the time the scan started. MoveIt removes the robot from the cloud (its self filter) using the arm's position at that one time. The points the arm left while it moved during the scan are not removed and stay in the obstacle map. When they lie on the path the arm is about to take, MoveIt stops the motion: the obstacle is the arm itself.

While the arm moves, about 20 such points per scan are left on average, up to 235; with the scan taken at once, 0.3 (up to 5).

Are these points the arm, or the cube it carries? The counter leaves out the cube where it rests (at the pick and place positions), but not while it is carried. In the second set of runs, each left point was also given to the nearest part of the robot when the cloud arrived: the hand (within 6 cm of the gripper frame, where a held 30 mm cube is) or an arm segment between two joints. While the arm moved, over 3 rotating-LiDAR runs:

| Nearest part | Points left (sum over 3 runs) |
|---|---|
| Wrist | 852 |
| Lower arm | 629 |
| Hand (with the cube, if held) | 213 |
| Upper arm | 45 |

At most 12 % of the points can be the cube; the rest are the arm itself. With the scan taken at once, 24 points in total (wrist 14, hand 10). The failed run also names the part: MoveIt's own log reports the contact with the upper arm, far from the hand. This is what a real spinning LiDAR does (motion distortion), and the failure appears in simulation before it appears on a robot. Gazebo's `gpu_lidar` scans at once, so this does not appear there. Rotating LiDARs are also simulated elsewhere (for example CARLA and Isaac Sim); we do not claim it is unique to Hakoniwa.

Each point of the Hakoniwa cloud already carries its own measurement time (`time`), as real LiDAR drivers provide. The proper fix is on the receiving side: filter or deskew each point with the arm's position at that point's time. MoveIt's standard self filter does not use per-point time.

### 5.4 Where Gazebo is still ahead

- **Ecosystem**: Gazebo comes with many sensors, rendering, worlds and plugins out of the box, and is the reference simulator of the ROS community. Here we compared one robot and one task.
- **Setup**: `gz_ros2_control` is a standard package; Hakoniwa's controller plugin is new.
- **Tracking error**: Hakoniwa's smaller error partly comes from the position control model (MuJoCo's position actuator against a velocity-driven joint in `gz_ros2_control`, section 1), not only from the simulator.
- **The rotating LiDAR** is closer to a real sensor, but it also made 1 of 6 Hakoniwa runs fail, as a real one can (section 5.3); a user who wants the ideal sensor sets `scan_mode: instant`.

## 6. Reproduce

All runs were made in one container (ROS 2 Jazzy, Gazebo Harmonic, the Hakoniwa core installed), from the commits in section 2, built fresh in Release. The scripts below take their paths from that container's layout (the Docker layout of [ros2/hakoniwa_ros2_control](../../hakoniwa_ros2_control/README.md), section "Docker だけで動かす（コントローラプラグイン）", which builds the Robot Runtime and the controller plugin); they are written to show what is run and in which order.

| Step | Script | What it does |
|---|---|---|
| Transport | `ros2/tools/transport-bench/sweep.sh` ([README](../../tools/transport-bench/README.md)), with `TYPE=fixed` for the fixed-size messages and `CASES="gz"` for gz-transport; `summarize.py` | every size and rate, 3 runs, one transport at a time |
| Physics only | `ros2/hakoniwa_ros2_control/tools/compare/run-physics.sh` | each simulator alone, 3 x 5 s of wall time |
| Control, Hakoniwa | `ros2/hakoniwa_ros2_control/tools/compare/run-hakoniwa-plugin.sh rt 3`, `... max 3`, `... max 3 speed` | the Robot Runtime with ros2_control inside it, then the trajectory measured by `measure.py` |
| Control, Gazebo | `ros2/hakoniwa_ros2_control/tools/compare/run-gazebo.sh rt 3`, `... max 3`, `... max 3 speed` | `gz sim -s` with `gz_ros2_control`, same measurement |
| Difference between runs | `ros2/hakoniwa_ros2_control/tools/compare/repeat_ticks.py <results dir>` | runs aligned on the control tick (section 5.2) |
| Pick and place | `ros2/hakoniwa_ros2_control/tools/compare/run-pick.sh hakoniwa-plugin 3`, `... hakoniwa-plugin 3 instant`, `... gazebo 3`; `pick_summary.py` | simulator, ros2_control and MoveIt started, `pick_and_place.py` run once, points left counted by `self_filter_leak.py`. The Hakoniwa sensors are Hakoniwa's sensor models, as measured (`SENSOR_SET=hakoniwa`, the default; it needs the `hakoniwa` sensor implementation built in). `SENSOR_SET=reference` runs the same task with the reference implementation (true values, instant LiDAR), which needs no library |

Each case ran alone, one after another, with nothing else running in the container.

## 7. Limitations

- One machine (Docker on macOS, 6 cores, no GPU) and one robot (SO-101).
- No sensor noise in these runs.
- Transport latencies are within one machine; they are not the speed of the whole simulation.
- Tracking error compares two different position-control models (section 1).
- Three runs per case (ten for the clock fix in section 5.1); the pick-and-place failure rate of the rotating LiDAR needs more runs to be stated as a rate.
