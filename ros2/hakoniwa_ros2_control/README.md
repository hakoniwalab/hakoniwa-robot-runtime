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

## センサーを付けて物を掴んで置く

上の手順の SO-101 に、作業台の上から下向きの 3D LiDAR と手首の力覚センサーを付けます。MoveIt が点群から障害物の地図を作り、物を掴んで別の場所に置きます。設計は [docs/ros2-control.md](../../docs/ros2-control.md) の 9 章にあります。

```text
ホスト  : Robot Runtime（SO-101、センサー付き）＋ TCP Bridge
Docker  : 箱庭 ROS 2 Bridge ＋ ros2_control ＋ MoveIt 2（点群から障害物の地図）＋ pick_and_place.py
```

| もの | 内容 |
|---|---|
| 3D LiDAR | 作業台の上の筐体から下向き。点群は `/pdu/lidar_scan`（`frame_id` は `overhead_lidar`） |
| 力覚センサー | 手首。`/pdu/wrist_wrench`（`frame_id` は `wrist_ft`）。掴めたか（物の重さ）と、離したかを見る |
| 時刻 | ROS 2 の全ノードを箱庭の時刻で動かす。`hakoniwa_clock.py` が関節の状態の時刻から `/clock` を出す |
| `scripts/add_grasp_object.py` | 物を MoveIt に登録する。`--attach` で手に付け、`--detach` で置いた場所に戻す |
| `scripts/pick_and_place.py` | 掴んで置く流れ。どの段でも失敗したら、その場で止まる |
| `config/so101/pick_and_place.yaml` | 物・机・4 つの姿勢（掴む前・掴む・置く前・置く）の関節の値。hakoniwa-robot-arm の `tools/recipe/so101_grasp_poses.py` が書く |

前提（上の手順の前提に加えて）：

- 3D LiDAR と力覚センサーは `hakoniwa` 実装を使います。そのライブラリは箱庭ラボが提供します（[お問い合わせ](https://hakoniwa-lab.net/contact/)）。受け取ったライブラリのパスを、セミコロンで並べて環境変数 `HAKO_ROBOT_ARM_SENSOR_LIBRARIES` に入れておきます。
- ROS 2 のイメージは、手順 2 の `docker build` で作ったものを使います（MoveIt の点群の部品 `moveit_ros_perception` が入っています）。

### 1. ホスト：センサー付きで起動する

上の手順 1 の `host-hako` の端末で、`configure` と `build` に `--sensors` を付けます。

```bash
cd ../hakoniwa-robot-arm
python tools/recipe/so101.py configure --ros2-control --sensors --realtime-sync-cycle-msec 2
python tools/recipe/so101.py build --sensors
python tools/recipe/so101.py start
python tools/recipe/so101.py status
```

`configure` の表示が `Sensors         : enabled` で、`status` が `"state": "RUNNING"` なら OK です。作業台の上に LiDAR の筐体（黒い円柱）と、緑の物があります。

### 2. ホスト：掴む・置く姿勢を求める（物の位置を変えるときだけ）

物を (0.28, 0) から (0.20, -0.20) に運ぶ設定は、`config/so101/pick_and_place.yaml` に入っています。別の位置にするときは、同じ `host-hako` の端末で次を実行し、出来た設定を手順 4 の `--config` で渡します。

```bash
python tools/recipe/so101_grasp_poses.py --pick 0.28 0.0 --place 0.20 -0.20 --output ../work-host/so101_pick_and_place.yaml
```

4 つの姿勢それぞれに `error` と `tilt` が表示され、最後に `written:` が出れば OK です。手が届かない位置では、エラーで止まります。

### 3. Docker：ROS 2 Bridge と ros2_control（センサー付き）を起動する

上の手順 2〜4 でコンテナに入って ROS 2 Bridge を起動し、手順 5 の端末 B では `sensors:=true` を付けて起動します。

```bash
source "$HAKONIWA_ROS2_WS/activate.bash"
source "${HAKONIWA_ROS2_WS}-control/install/setup.bash"
ros2 launch hakoniwa_ros2_control so101.launch.py so101_urdf:=/workspace/work-host/model-forge/so101/source/Simulation/SO101/so101_new_calib.urdf moveit:=true sensors:=true
```

`Configured and activated` が 3 つ出て、`Listening to '/pdu/lidar_scan'` と `You can start planning now!` が出れば OK です。

### 4. 端末 C：掴んで置く

```bash
source "$HAKONIWA_ROS2_WS/activate.bash"
source "${HAKONIWA_ROS2_WS}-control/install/setup.bash"
ros2 run hakoniwa_ros2_control pick_and_place.py
```

手順 2 で別の設定を作ったときは、`--config /workspace/work-host/so101_pick_and_place.yaml` を付けます。

次のとおりなら OK です（約 1 分）。

- 1〜13 の全段が `OK` で終わる。
- `holding` の力の変化が、物の重さ（0.294 N）に近い。
- `released` の力の変化が、0.1 N 未満。
- 最初と最後の行の LiDAR の点の数が、掴む場所で減り、置く場所で増える。

ホストの MuJoCo のビューアで、物が掴まれて運ばれ、置かれるのが見えます。

うまくいかないときの見どころ：

- 3 か 8 で失敗する：手順 3 の端末 B のログで、`CheckStartStateBounds`（関節の範囲外）か `contact between`（何かとぶつかる）を探します。
- 5 で失敗する：あごが閉じきった（物が間にない）。物の位置と、手順 2 の姿勢を確かめます。

## 点群から物を見つけて掴む

物の位置を設定で与える代わりに、3D LiDAR の点群から見つけます。上の「センサーを付けて物を掴んで置く」の手順 1 と 3 を済ませ、ROS 2 側が動いている状態から始めます。

| もの | 内容 |
|---|---|
| `scripts/detect_objects.py` | 点群（`/pdu/lidar_scan`）を `base_link` に直し、作業台の範囲で机の面より 5 mm 以上高い点を、2 cm のます目で隣り合うものどうしまとめる。まとまりごとに中心・大きさ・向きを出す。ロボットの根元の周り（半径 0.12 m）は除く |
| 腕をよける姿勢 | 腰（`shoulder_pan`）を -1.5 rad。腕が LiDAR の見ていない側（y > 0.25）に向き、物を隠さない |

- MoveIt の自己フィルタ後の点群は使いません。登録した物（机）の周り 2 cm の点まで消えるので、高さ 3 cm の物は上の縁しか残らないためです。腕をよけてから、元の点群を使います。
- 向きは、LiDAR から物の一部の面しか見えないとずれます。今は位置だけを使います（グリッパーは 70 mm まで開くので、3 cm の物なら向きがずれても掴めます）。

### 1. 端末 C：机を登録して、腕をよける

```bash
source "$HAKONIWA_ROS2_WS/activate.bash"
source "${HAKONIWA_ROS2_WS}-control/install/setup.bash"
ros2 run hakoniwa_ros2_control add_grasp_object.py --id table --position 0 0 -0.03 --size 2.4 2.4 0.06
ros2 service call /clear_octomap std_srvs/srv/Empty "{}"
ros2 run hakoniwa_ros2_control move_to_joints.py --group arm --joints shoulder_pan=-1.5 shoulder_lift=0 elbow_flex=0 wrist_flex=0 wrist_roll=0
```

`added table: OK` と `MoveIt error code 1 (SUCCESS)` が出れば OK です。机を先に登録するのは、障害物の地図の上の机（2 cm のます目で 2〜4 cm の厚み）に、腰を回したときの肩が触れるためです。

### 2. 端末 C：物を見つける

```bash
ros2 run hakoniwa_ros2_control detect_objects.py
```

`1 objects` と、`object_0: ... centre [x, y, z]` が出れば OK です。物が初めの位置にあるときは、`centre [0.28, 0.0, 0.015]`、`size [0.029, 0.03, 0.03]` 程度になります。

### 3. 端末 C：見つけた位置で掴む・置く姿勢を求める

手順 2 の `centre` の x と y を `--pick` に入れます。次は、物が (0.28, 0.0) に見つかり、(0.20, -0.20) に置く例です。

```bash
cd /workspace/hakoniwa-robot-arm
python3 tools/recipe/so101_grasp_poses.py --pick 0.28 0.0 --place 0.20 -0.20 --model /workspace/work-host/model-forge/so101/install/so101.environment.xml --output /workspace/work-host/so101_pick_and_place.yaml
```

4 つの姿勢の `error` が数 mm 以内で、`written:` が出れば OK です。

### 4. 端末 C：掴んで置く

```bash
ros2 run hakoniwa_ros2_control pick_and_place.py --config /workspace/work-host/so101_pick_and_place.yaml
```

1〜13 の全段が `OK` で終われば OK です。最後の行で、LiDAR の点が置いた場所に増えます。手順 1〜4 をくり返すと、物を見つけては運ぶことができます。

## 確認した環境

macOS（Apple Silicon）、Docker Desktop、ROS 2 Jazzy（arm64）、SO-101。

| 確認 | 結果 |
|---|---|
| controller_manager の周期 | 100 Hz（`/actuator/shoulder_pan/command` の実測 100.03 Hz） |
| 軌道（joint_trajectory_controller に直接） | `SUCCEEDED`、すべての関節が目標の位置に届いた |
| MoveIt（腕 2 回、グリッパー 1 回） | どれも計画と実行が `SUCCESS`。関節は目標から約 0.01 rad |
| 点群から見つけて掴んで置く（物を (0.20, -0.20) に置いたあと） | 見つけた中心 (0.203, -0.193)、17 点。そこから掴んで (0.28, 0) に戻し、13 段すべて OK。持ち上げたときの力の変化 0.295 N |
| 掴んで置く（センサー付き、作業台を初めに戻して 1 回で） | 13 段すべて OK、約 62 秒。持ち上げたときの力の変化 0.294 N（物の重さ 0.294 N）、離したあと 0.018 N。LiDAR の点は掴む場所 3 → 0、置く場所 0 → 1 |
| TCP Bridge で捨てられた指令 | 0 回 |
