# ros2_control との接続

ROS 2 の ros2_control の標準のコントローラ（joint_trajectory_controller など）と MoveIt 2 で、箱庭のロボットを動かすための設計です。実機と同じコントローラを、シミュレーションでそのまま使えるようにします。

## 1. 方針

- ROS 2 の側に、ros2_control のハードウェア（`hardware_interface::SystemInterface`）として `HakoniwaSystem` を置きます。
- `HakoniwaSystem` は箱庭の PDU を直接触りません。ROS 2 のトピックを読み書きするだけです。PDU との間は箱庭 ROS 2 Bridge（hakoniwa_pdu_ros）がつなぎます。
- Robot Runtime は変えません。今ある関節ごとの指令の受け口と、関節の状態の出力をそのまま使います。
- ros2_control の controller_manager は ROS の時計で回り、箱庭の時間とはゆるくつながります。箱庭の時間と強くそろえることは、この設計の範囲外です。

## 2. 全体の形

```text
ros2_control ── HakoniwaSystem ──(ROS 2 topic)── 箱庭 ROS 2 Bridge ──(PDU)── Robot Runtime
  joint_trajectory_controller      read():  /pdu/joint_states を購読
  joint_state_broadcaster          write(): /actuator/<joint>/command を出す
  MoveIt 2
```

どこで動かすかで、違うのは Bridge のつなぎ方だけです。`HakoniwaSystem` と Robot Runtime は同じです。

| 構成 | Robot Runtime と MuJoCo | ROS 2 の側 | Bridge と PDU のつなぎ |
|---|---|---|---|
| Ubuntu（同じマシン） | ホスト | ホスト | Bridge が箱庭アセットとして共有メモリで読み書きする |
| macOS・Windows のホスト ＋ Docker | ホスト | Docker（Ubuntu、ROS 2 Jazzy） | Docker の Bridge がホストの TCP Bridge に TCP でつなぐ。共有メモリに書くアセットはホストの TCP Bridge |

- MuJoCo のビューアはホストで動くので、どちらの構成でもホストの画面に出ます。
- Docker の中の画面（MoveIt の RViz）は、ホストに出すために画面の転送が要ります。動作の確認はホストのビューアで行い、RViz は必要になったときに扱います。

## 3. Robot Runtime の側（既存の部品）

関節ごとの指令は、アクチュエータごとに次の 1 組で受けます。どれも Robot Runtime の最初の形から入っている部品です。

| 口 | 実装 | 役割 |
|---|---|---|
| 受け口 | `Float64PduEventReader`（`IFloat64EventReader`） | 指令の PDU（`std_msgs/Float64`）の受信イベントで届いた値を、最新の 1 つだけ置く |
| コマンドソース | `ScalarPduCommandSource`（`ICommandSource`） | 1 歩ごとに最新の値を 1 つ取り、箱庭の時刻・通し番号・期限（時刻 ＋ `command_timeout_sec`）を付けて渡す。新しい値が無ければ入力待ち |
| コントローラ | `ScalarPduCommandController`（`IController`） | 入力 1 つを、アクチュエータ 1 つの `ActuatorCommand`（位置・速度・トルクはアクチュエータの設定による）にする。補間はしない。期限までは同じ値を出し続ける |

```text
PDU 書き込み → 受信イベント → Float64PduEventReader → ScalarPduCommandSource::poll
→ ScalarPduCommandController::update → ActuatorCommand → Arbiter → Plant
```

- Arbiter には優先度 0、関節ごとの指令の制御グループとして入ります。手動の操作を優先させることや、物理に入る直前の検査は、ros2_control の指令にもそのまま効きます。
- 指令の PDU 名・受け取る周期・指令の種類は、アクチュエータの設定（`pdu_config`、`spec.type`）で決まります。
- 新しい指令が来るまで歩みを止める受け口（lockstep）もあります。この設計では使いません。

## 4. PDU の書き込みの決まり（受信イベント）

Robot Runtime は、指令の PDU に受信イベントを登録しています。箱庭コアは、受信イベントのある PDU について、次の決まりで書き込みを扱います。

- 書くと、そのチャンネルに受信の印が立ちます。
- 印が立っている間（受け手がまだ受け取っていない間）は、次の書き込みを断ります。
- 受け手（Robot Runtime のアセット）は、箱庭の時間を 1 刻み進めるたびに受信イベントを処理し、印を下ろします。Robot Runtime の 1 歩の中（物理の計算と、実時間に合わせる待ち）では印は下りません。

そのため、PDU に指令を書くもの（Ubuntu では Bridge、ホスト ＋ Docker ではホストの TCP Bridge）は、次のようにします。

- 箱庭アセットとして、箱庭の時間で書く。
- 断られたら指令を捨てず、受け手の 1 歩分を待って同じ指令を書き直す。

macOS のホストで、箱庭アセットから共有メモリに `actuator/shoulder_pan/command` を毎秒 50 回書いて確かめました（SO-101、12 秒）。

| 書き手 | 断られた書き込み | 失われた指令 |
|---|---|---|
| アセットにしない外部からの書き込み | 47 %（913 回中 427 回） | 断られた分すべて |
| 箱庭アセット、断られたら捨てる | 1.5 %（600 回中 9 回） | 9 回 |
| 箱庭アセット、断られたら受け手の 1 歩分待って書き直す | 6 回（開始直後の 1 回を含む） | 0 回 |

最後の形で、関節は指令に最大 0.040 rad の差でついてきました。

### 4.1 時間の設定（刻みをそろえる）

Robot Runtime は、箱庭アセットとして自分のループで動きます（ポーリング）。1 周は次のとおりです。

```text
on_step：世界のロック → 指令を拾う → 物理 1 歩 → 状態を出す → ロックを放す
         → 実時間合わせ（realtime_sync_cycle の境目で、壁時計が追いつくまで眠る）
usleep ：箱庭の時間を 1 刻み進める（世界の時刻に追いつくまで待つ）。進めたときに受信イベントを処理する
```

受信イベントが処理されるのは、時間を進めたときだけです。次の間は処理されず、その間に届いた 2 つ目の指令は断られます。

- 実時間合わせで眠っている間。
- コンダクタの遅れの上限（Dmax）に当たって、ほかのアセットを待っている間。

そのため、次のようにそろえます。

| 項目 | 値 | 理由 |
|---|---|---|
| Robot Runtime の刻み（dT） | MuJoCo の物理の刻み（SO-101・Nova5 は 2 ms） | — |
| 実時間合わせの周期（`--realtime-sync-cycle-msec`） | dT と同じ（2 ms） | 1 回の眠りを 1 歩分にする |
| ホストの TCP Bridge の刻み（`--delta-time-step-usec`） | dT と同じ（2000） | Robot Runtime が Bridge を待つ時間を 1 歩分にする |
| controller_manager の周期 | 100 Hz まで確かめた | 指令が 10 ms ごとに来ても、受け取りが間に合う |

ホストの TCP Bridge の運び方は次のとおりです。

| PDU | 運び方 | 引き金 |
|---|---|---|
| 指令（TCP → 共有メモリ） | `immediate` | TCP で届いたとき（Bridge の刻みとは無関係） |
| 関節の状態（共有メモリ → TCP） | `ticker`（20 ms） | Bridge の刻みごとに判定し、20 ms 経っていれば運ぶ |

ホスト（macOS）と Docker（ROS 2）で、Docker から `ros2 topic pub` で `/actuator/shoulder_pan/command` を出して確かめました（SO-101）。

| Robot Runtime の実時間合わせ | Bridge の刻み | 指令 | Bridge で捨てられた指令 |
|---|---|---|---|
| 50 ms | 20 ms | 50 Hz | 約 4 分の 1 |
| 2 ms | 20 ms | 50 Hz（約 1,000 回） | 127 回（約 13 %） |
| 2 ms | 20 ms | 10 Hz（約 100 回） | 0 回 |
| 2 ms | 2 ms | 50 Hz（約 500 回） | 0 回 |
| 2 ms | 2 ms | 100 Hz（約 1,000 回） | 0 回 |

どの条件でもアームは指令どおりの位置に動き、箱庭の時間はほぼ実時間で進みました。

次の 2 つの案は採りません。

- **実時間合わせをやめる:** 指令は断られなくなります。ただし、実時間の動きが分かりにくくなります。
- **指令の受信イベントを外す:** 断られなくなります。ただし、新しい指令が来たことが分からず、指令の期限という安全の仕組みが働かなくなります。

## 5. ROS 2 のトピック

箱庭 ROS 2 Bridge のトピック名の決まりに従います。

- 持ち主が PDU 側（`pdu_to_ros`）のトピックは、Bridge が `/pdu` を前に付けて出します。
- 持ち主が ROS 側（`ros_to_pdu`）のトピックは、書いた名前のままです。
- `/pdu` の下は PDU 側の写しのために予約されています。

| PDU | 向き | ROS 2 のトピック | 使うもの |
|---|---|---|---|
| `joint_states`（`sensor_msgs/JointState`） | PDU → ROS | `/pdu/joint_states` | `HakoniwaSystem::read()` |
| `actuator/<joint>/command`（`std_msgs/Float64`、関節ごと） | ROS → PDU | `/actuator/<joint>/command` | `HakoniwaSystem::write()` |

- `/joint_states` は ROS 2 側の joint_state_broadcaster が出し、MoveIt が読みます。Bridge の写しは `/pdu/joint_states` なので、ぶつかりません。
- 箱庭の軌道コントローラ向けの `joint_trajectory`（JointTrajectory）は、この構成では Bridge に流しません。軌道は ROS 2 側の joint_trajectory_controller（`/<controller>/follow_joint_trajectory`）が補間します。箱庭の軌道コントローラにも届くと、補間と指令が二重になるためです。

## 6. HakoniwaSystem

- ROS 2 だけに依存するプラグインです。箱庭のライブラリには依存しません。
- `read()`：`/pdu/joint_states` の最新の値から、関節ごとの `position`・`velocity`（`effort` は必要なとき）の State Interface を埋めます。センサの測定値（`joint_states_measured`）を使う選択も持たせます。
- `write()`：関節ごとの Command Interface の値を `/actuator/<joint>/command` に出します。
- URDF の `<ros2_control>` に、関節名、指令の種類（最初は位置）、トピック名を書きます。
- controller_manager の周期は、Robot Runtime が指令を受け取る周期（アクチュエータの設定の `update_rate_hz`）以下にします。指令の期限（`command_timeout_sec`）が、その周期と通信の遅れより長いことも確かめます。

実装は [`ros2/hakoniwa_ros2_control`](../ros2/hakoniwa_ros2_control/README.md) にあります。

- 有効にするときは、関節の状態を待ちません。有効にする処理は controller_manager の受信の処理の中で呼ばれるので、その間は自分の購読が回らないからです。状態と指令は値なし（NaN）で始め、値なしの指令は出しません。
- ROS 2 のドメインは、箱庭 ROS 2 Bridge の環境（`activate.bash`、`ROS_DOMAIN_ID=73`）にそろえます。

## 7. ROS 2 のイメージ

ホスト ＋ Docker の構成の ROS 2 の側は、[`ros2/docker/Dockerfile.jazzy`](../ros2/docker/Dockerfile.jazzy) のイメージで動かします。

- Ubuntu 24.04、ROS 2 Jazzy。amd64 と arm64（Apple Silicon）で作れます。
- 入っているもの：箱庭 ROS 2 Bridge のビルドに要るもの、ros2_control、ros2_controllers、MoveIt 2。
- Robot Runtime と MuJoCo はホストで動くので、それらのビルドに要るものは入れません。

## 8. 進め方

1 つずつ確かめます。

1. 箱庭アセットから指令の PDU を書き、関節が動くこと（macOS のホスト、共有メモリ）。済み（4 章）。
2. Docker の中で箱庭 ROS 2 Bridge をビルドし、ホストの TCP Bridge につないで、`/pdu/joint_states` が届くこと。済み。
3. Docker から `/actuator/<joint>/command` に出した指令で、関節が動くこと。済み（4.1）。
4. `HakoniwaSystem` と joint_trajectory_controller で、軌道を送るとアームが位置制御で動くこと。済み（手順は [ros2/hakoniwa_ros2_control/README.md](../ros2/hakoniwa_ros2_control/README.md)）。
5. MoveIt 2 から動かすこと。関節の目標を渡して計画と実行まで済み（腕とグリッパー、RViz なし）。周りの障害物をセンサで見る（点群・深度画像）ことと、手先の位置・姿勢の目標は、まだ。
6. Ubuntu（同じマシン、Bridge が共有メモリ）で同じことを確かめること。

## 9. センサーと物を掴む

### 9.1 センサー

SO-101 に、作業台の上から下向きの 3D LiDAR と、手首の力覚センサーを付けます。設定は hakoniwa-robot-arm の SO-101 のレシピ（`--sensors`）にあります。

| センサー | 取り付け | 出力 | ROS 2 のトピック | 運ぶ周期 |
|---|---|---|---|---|
| 3D LiDAR（180×16 本、下向き -80°〜-20°、5 Hz） | 筐体 (0.28, 0.25, 0.5) の 0.025 m 下 | `sensor_msgs/PointCloud2` | `/pdu/lidar_scan` | 200 ms |
| 力覚センサー | 手首（MuJoCo の `gripper`、URDF の `gripper_link`） | `geometry_msgs/WrenchStamped` | `/pdu/wrist_wrench` | 20 ms |

- 3D LiDAR は、深度カメラより軽いので選びました。それでも光線の計算で、箱庭の時間は実時間の約 0.75 倍で進みます。
- ホストの TCP Bridge は、出力ごとの周期（出力の設定の `update_rate_hz`）で運びます。同じ点群を何度も送らないためです。
- TF：URDF（`so101.urdf.xacro`、`sensors:=true`）に、センサー名と同じ名前のリンク（`overhead_lidar`、`wrist_ft`）を固定の関節で足します。値はレシピのセンサーの設定と同じです。

### 9.2 箱庭の時刻で動かす

ROS 2 の全ノード（controller_manager、robot_state_publisher、move_group）を、箱庭の時刻（`use_sim_time: true`）で動かします。箱庭の時刻は、`hakoniwa_clock.py` が関節の状態の時刻から `/clock` に出します。

- センサーの出力の時刻は、測った箱庭の時刻です。ROS 2 側が壁時計のままだと、MoveIt の自己フィルタが点群の時刻での腕の位置を TF に見つけられず、腕自身が障害物の地図に入りました。
- 箱庭が実時間より遅いとき（今は約 0.75 倍）も、軌道の時間が箱庭の時間でそろいます。

### 9.3 障害物の地図

MoveIt は、`/pdu/lidar_scan` から障害物の地図（octomap、2 cm のます目、`base_link`）を作ります。腕の形の中の点は除きます（自己フィルタ）。机にめり込む姿勢や物に当たる姿勢は断られ、離れた姿勢は計画と実行ができることを確かめました。

- 地図の上で、机は 2〜4 cm の厚みになります。物も障害物です。そのため、物の近くまで手先を下ろす計画は作られません。

### 9.4 物を掴む進め方

1. 掴む物の位置を、最初は設定で与えて、MoveIt に「掴む対象」として登録する。その周りの点は地図から消す。点群から物を見つける認識は、あとにする。
2. 物の真上の少し高い位置（手先が物の上 5〜8 cm）へ、MoveIt で障害物を避けて動かす。腕が 5 関節なので、目標は関節の値で与える。
3. 下ろす → グリッパーを閉じる → 持ち上げる、は短い動きなので、地図を気にせず joint_trajectory_controller に直接送る。
   - 下ろすときは手首の力覚を見て、接触したら止める。
   - 持ち上げたあと、力覚で物の重さ（30 g、約 0.29 N）が増えたかで、掴めたかを判定する。
