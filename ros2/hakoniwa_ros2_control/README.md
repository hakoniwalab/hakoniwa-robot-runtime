# hakoniwa_ros2_control

ros2_control のハードウェア `HakoniwaSystem` と、SO-101 の設定です。ROS 2 の標準のコントローラ（joint_trajectory_controller など）と MoveIt 2 で、箱庭のロボットを動かします。設計は [docs/ros2-control.md](../../docs/ros2-control.md) にあります。

- `HakoniwaSystem` は ROS 2 のトピックだけを読み書きします。箱庭の PDU との変換は、箱庭 ROS 2 Bridge（hakoniwa_pdu_ros）が行います。
  - `read()`：`/pdu/joint_states` の最新の値を、関節ごとの State Interface（position、velocity）に入れます。
  - `write()`：関節ごとの Command Interface の値を、`/actuator/<joint>/command`（`std_msgs/Float64`）に出します。
- 関節の状態が届く前は、状態も指令も値なし（NaN）です。値なしの指令は出さないので、起動の直後にロボットが勝手に動くことはありません。

| ファイル | 中身 |
|---|---|
| `src/hakoniwa_system.cpp` | `HakoniwaSystem`（`hardware_interface::SystemInterface`） |
| `config/so101/so101.urdf.xacro` | SO-101 の URDF をそのまま取り込み、`<ros2_control>` を足したもの |
| `config/so101/ros2_controllers.yaml` | controller_manager（100 Hz）、joint_state_broadcaster、`arm_controller`（腕の 5 関節）、`gripper_controller`（グリッパー）。どちらも joint_trajectory_controller |
| `config/so101/moveit/` | MoveIt の設定：SRDF（`arm`・`gripper` のグループ）、逆運動学（KDL）、関節の速度・加速度の上限、実行先のコントローラ |
| `launch/so101.launch.py` | robot_state_publisher、controller_manager、3 つのコントローラの起動。`moveit:=true` で move_group も起動する |
| `scripts/move_to_joints.py` | MoveIt（move_group）に関節の目標を渡し、計画と実行を頼む |

## SO-101 を動かす（macOS のホスト ＋ Docker の ROS 2）

ホストで Robot Runtime と TCP Bridge を動かし、Docker の中の ROS 2 から TCP でつなぎます。

```text
ホスト  : Robot Runtime（SO-101）＋ TCP Bridge（0.0.0.0:54001）
Docker  : 箱庭 ROS 2 Bridge ＋ ros2_control（HakoniwaSystem、arm_controller、gripper_controller）＋ MoveIt 2（move_group）
```

前提：

- `hakoniwa-business-pack`、`hakoniwa-robot-arm`、`hakoniwa-robot-runtime`、`hakoniwa-pdu-ros` が同じディレクトリに並んでいること。
- そのディレクトリの `work-host` に、SO-101 の Foundation と Recipe が準備され、SO-101 の forge が済んでいること（hakoniwa-robot-arm の [host-docker のセットアップ](https://github.com/hakoniwalab/hakoniwa-robot-arm/blob/main/docs/procedures/host-docker/setup.md) で、Recipe を SO-101 にしたもの）。
- Docker Desktop が動いていること。

### 1. ホスト：Robot Runtime と TCP Bridge を起動する

通常のホストの端末で、`hakoniwa-robot-arm` のリポジトリのルートから実行します。Docker の中からホストに接続するので、接続先を `host.docker.internal` にします。

```bash
export HAKONIWA_ROS2_TCP_HOST=host.docker.internal
source profiles/tool-env/enter-host-hako.bash
```

開いた `host-hako` の端末で実行します。

```bash
cd ../hakoniwa-robot-arm
python tools/recipe/so101.py configure --ros2-control --environment --realtime-sync-cycle-msec 2
python tools/recipe/so101.py build
python tools/recipe/so101.py doctor
python tools/recipe/so101.py start
python tools/recipe/so101.py status
```

`configure` の表示が `ROS 2 TCP       : enabled (ros2_control)` と `Realtime pacing : 2 ms simulation-time cycle` で、`status` が `"state": "RUNNING"` なら OK です。MuJoCo のビューアがホストに開きます。

- `--ros2-control` は、ROS 2 TCP の構成から Robot Runtime の軌道の受け口（JointTrajectory）を外します。軌道は ROS 2 側の joint_trajectory_controller が補間します。
- `--realtime-sync-cycle-msec 2` は、実時間合わせの周期を Robot Runtime の刻み（2 ms）に合わせます。TCP Bridge の刻みも 2 ms です。これより長いと、指令の一部が捨てられます（設計書の 4.1）。

### 2. Docker：ROS 2 のイメージを作り、コンテナに入る

通常のホストの端末（`host-hako` ではない）で、`hakoniwa-robot-arm` のリポジトリのルートから実行します。

```bash
docker build -t hakoniwa-ros2-control:jazzy -f ../hakoniwa-robot-runtime/ros2/docker/Dockerfile.jazzy ../hakoniwa-robot-runtime/ros2/docker
IMAGE_NAME=hakoniwa-ros2-control CONTAINER_NAME=hakoniwa-ros2-control-jazzy HAKONIWA_DOCKER_GUI=off HAKONIWA_ROS2_TCP_CONFIG=../work-host/recipes/so101-joint-trajectory-control/config/ros2-tcp bash docker/run.bash jazzy
```

コンテナの端末（以下「端末 A」）が開けば OK です。この端末を閉じるとコンテナは消えます。

### 3. 端末 A：箱庭 ROS 2 Bridge と hakoniwa_ros2_control をビルドする

```bash
cd ../hakoniwa-robot-arm
/usr/bin/python3 tools/recipe/ros2_workspace.py build
source "$HAKONIWA_ROS2_WS/activate.bash"
/usr/bin/python3 tools/recipe/ros2_workspace.py doctor
colcon --log-base "${HAKONIWA_ROS2_WS}-control/log" build \
  --base-paths /workspace/hakoniwa-robot-runtime/ros2/hakoniwa_ros2_control \
  --build-base "${HAKONIWA_ROS2_WS}-control/build" \
  --install-base "${HAKONIWA_ROS2_WS}-control/install" \
  --packages-select hakoniwa_ros2_control
```

doctor に `hakoniwa_pdu_ros bridge` が表示され、colcon の summary に `1 package finished` が出れば OK です。

### 4. 端末 A：箱庭 ROS 2 Bridge を起動する

```bash
ros2 run hakoniwa_pdu_ros bridge --config "$HAKONIWA_ROS_BINDING"
```

終了せずに動き続ければ OK です。ホストの TCP Bridge より先に起動したときは、つながるまで `Connection refused` が何度か出ます。この端末は開いたままにします。

### 5. 端末 B：ros2_control を起動する

通常のホストの端末で、`hakoniwa-robot-arm` のリポジトリのルートから、コンテナに 2 つ目の端末（端末 B）を開きます。

```bash
CONTAINER_NAME=hakoniwa-ros2-control-jazzy bash docker/attach.bash jazzy
```

端末 B で実行します。

```bash
source "$HAKONIWA_ROS2_WS/activate.bash"
source "${HAKONIWA_ROS2_WS}-control/install/setup.bash"
ros2 launch hakoniwa_ros2_control so101.launch.py so101_urdf:=/workspace/work-host/model-forge/so101/source/Simulation/SO101/so101_new_calib.urdf moveit:=true
```

`Configured and activated` が `joint_state_broadcaster`・`arm_controller`・`gripper_controller` の 3 つで出て、move_group の `You can start planning now!` が出れば OK です。この端末は開いたままにします。

- `No 3D sensor plugin(s) defined for octomap updates` というエラーが出ますが、周りの障害物を見るセンサをまだ設定していないためで、動作には関係ありません。
- 起動のときに、URDF のメッシュの相対パス（`assets/...`）を絶対パスに直した写しを一時ディレクトリに作ります。MoveIt が衝突判定でメッシュを読むためです。元の URDF は変えません。

- `activate.bash` は ROS 2 のドメイン（`ROS_DOMAIN_ID=73`）を箱庭 ROS 2 Bridge とそろえます。ROS 2 を使う端末では、どれも最初に読み込みます。読み込まないと、`/pdu/joint_states` が見えません。

### 6. 端末 C：MoveIt と軌道で動かす

端末 B と同じようにコンテナに 3 つ目の端末（端末 C）を開きます。

```bash
CONTAINER_NAME=hakoniwa-ros2-control-jazzy bash docker/attach.bash jazzy
```

端末 C で実行します。最後の行は、関節名ごとの今の位置を表示します（`/joint_states` の関節の並びは名前の順なので、名前と組にして見ます）。

```bash
source "$HAKONIWA_ROS2_WS/activate.bash"
source "${HAKONIWA_ROS2_WS}-control/install/setup.bash"
ros2 control list_controllers
ros2 run hakoniwa_ros2_control move_to_joints.py --group arm --joints shoulder_pan=0.5 shoulder_lift=-0.3 elbow_flex=0.4 wrist_flex=0.2 wrist_roll=0.0
ros2 run hakoniwa_ros2_control move_to_joints.py --group gripper --joints gripper=1.0
python3 -c "import rclpy; from sensor_msgs.msg import JointState; rclpy.init(); n=rclpy.create_node('peek'); m=[]; n.create_subscription(JointState, '/joint_states', m.append, 10); [rclpy.spin_once(n, timeout_sec=1.0) for _ in range(5) if not m]; print({k: round(v, 3) for k, v in zip(m[-1].name, m[-1].position)})"
```

次のとおりなら OK です。

- `list_controllers` に、3 つのコントローラが `active` で出る。
- 2 回の `move_to_joints.py` が、どちらも `MoveIt error code 1 (SUCCESS)` で終わる。
- 最後の行で、`shoulder_pan` 0.5、`shoulder_lift` -0.3、`elbow_flex` 0.4、`wrist_flex` 0.2、`wrist_roll` 0.0、`gripper` 1.0 の近く（0.01 rad 程度の差）になる。

アームとグリッパーは、ホストの MuJoCo のビューアで動きます。

MoveIt を使わずに、joint_trajectory_controller に直接軌道を送ることもできます。

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.6, -0.4, 0.5, 0.3, -0.5], time_from_start: {sec: 3}}]}}"
```

`Goal finished with status: SUCCEEDED` で終われば OK です。

### 7. 終了する

端末 B と端末 A で Ctrl-C を押し、端末 A を `exit` で閉じます（コンテナが消えます）。端末 C は `exit` で閉じます。ホストの `host-hako` の端末で、Robot Runtime と TCP Bridge を止めます。

```bash
python tools/recipe/so101.py stop
python tools/recipe/so101.py status
```

## 確認した環境

macOS（Apple Silicon）、Docker Desktop、ROS 2 Jazzy（arm64）、SO-101。

| 確認 | 結果 |
|---|---|
| controller_manager の周期 | 100 Hz（`/actuator/shoulder_pan/command` の実測 100.03 Hz） |
| 軌道（joint_trajectory_controller に直接） | `SUCCEEDED`、すべての関節が目標の位置に届いた |
| MoveIt（腕 2 回、グリッパー 1 回） | どれも計画と実行が `SUCCESS`。関節は目標から約 0.01 rad |
| TCP Bridge で捨てられた指令 | 0 回 |
