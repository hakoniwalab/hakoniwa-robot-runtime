# センサの取り付けと PDU の対応

Robot Runtime にどのセンサを取り付けられるか、MJCF のどこにどう付けるか、測定値をどの PDU（メッセージ型）のどの欄に入れて送るかの仕様です。

センサは opt-in です。センサを使うビルド（`HAKONIWA_ROBOT_RUNTIME_ENABLE_SENSOR=1`）でだけ使え、センサの実装のライブラリを一緒にリンクします。センサを使わないビルドでセンサの設定を書くと、起動時に止まります。

## 1. 原則：ROS の決まりに従う

PDU の型は ROS 2 のメッセージ型と同じです（PDU 登録集にある型）。欄の使い方も ROS の決まりに従います。そのため、ROS 2 のブリッジを通したときに、ROS の道具（RViz、`ros2_control` の broadcaster を前提にした道具など）がそのまま使えます。

| 決まり | 内容 |
|---|---|
| 時刻 `header.stamp` | 値を**測った**シミュレーション時刻。送った時刻ではない。センサの周期と送信の周期が違っても、測った時刻が入る |
| `header.frame_id` | センサの座標系の名前。Robot Runtime ではセンサ名（`sensors[].name`） |
| 単位と軸（REP-103） | SI 単位。センサの軸は前・左・上（FLU）。箱庭の決まりと同じ |
| 測っていない値 | 空の配列にする（JointState）か、ROS の決まりの印を入れる（Imu の姿勢） |
| 共分散 | 分からないときは 0（ROS で「不明」の意味） |

1 つのメッセージに複数の値をまとめるとき（JointState）は、その中で一番新しい測定時刻を入れます。

## 2. 対応表

| センサ種別（`type`） | Runtime | 取り付け（`mount`） | 真値（ステップの後の状態） | メッセージ型 | 1 つの出力に入るセンサ |
|---|---|---|---|---|---|
| `joint` | 使える | 関節 | 関節の角度・角速度（トルクを測る定義なら、関節を通る伝達トルク） | `sensor_msgs/JointState` | 1 つ以上 |
| `force_torque` | 使える | body | 工具側がロボット側に及ぼす力とトルク | `geometry_msgs/WrenchStamped` | 1 つ |
| `imu` | 使える | body | 角速度と比力（重力込みの加速度） | `sensor_msgs/Imu` | 1 つ |
| `lidar`（2D） | 使える | 測距用の body | ワールドのレイの部品で走査した距離 | `sensor_msgs/LaserScan` または `sensor_msgs/PointCloud2` | 1 つ |
| `lidar`（3D） | 使える | 測距用の body | 同上 | `sensor_msgs/PointCloud2` | 1 つ |
| `rangefinder` | 使える | 測距用の body | 同上（1 本、または円錐） | `sensor_msgs/Range` | 1 つ |
| `mag` | 予定 | body | — | `sensor_msgs/MagneticField` | 1 つ |
| `baro` | 予定 | body | — | `sensor_msgs/FluidPressure` | 1 つ |
| `gps` | 予定 | body | — | `sensor_msgs/NavSatFix`（速度は `geometry_msgs/TwistStamped`） | 1 つ |
| `depth_camera`・`rgb_camera` | 予定 | 測距用の body | — | `sensor_msgs/CompressedImage` と `sensor_msgs/CameraInfo` | 1 つ |

- LiDAR の走査の終わりの姿勢（`geometry_msgs/Pose`）は予定です。
- 予定の型は、箱庭の外部センサプラグインが使っている型と同じにします。
- 出力の設定（`schemas/components/sensor-output.schema.json`）で、`pdu_config.message_type` にこの表の型を書きます。センサの種別と型が合わない出力、3D の LiDAR を LaserScan に出す出力は、起動時に止まります。
- どのセンサ種別をどの実装（`hakoniwa`、`gazebo`、`airsim` など）が持つか、定義ファイルにどの項目を書けるかは、実装一覧 [`catalog/sensor-implementations.json`](../catalog/sensor-implementations.json) が正です。一覧に無い実装・種別・項目は、起動時に止まります。

### 2.1 取り付けの書き方

取り付けはセンサの設定（`schemas/components/sensor-instances.schema.json`）の `mount` に書きます。センサの軸は前・左・上（FLU）です。

| 書き方 | 欄 | 使う種別 |
|---|---|---|
| 関節 | `mjcf_joint`：1 自由度の関節（hinge か slide） | `joint` |
| body | `mjcf_body`、`position_m`（body の座標系での位置 [m]）、`orientation_deg`（`roll`・`pitch`・`yaw` [度]、R = Rz(yaw)·Ry(pitch)·Rx(roll)） | `force_torque`、`imu` など |
| 測距用の body | body と同じ欄に加えて、`exclude_mjcf_body`（省略できる） | `lidar`、`rangefinder` |

- 測距用の body には、ふつうセンサの筐体を付けます。筐体は MJCF の body（形状あり）として先にモデルに入れておきます。レイは、筐体とその下の body には当たりません。
- ロボット自身は既定で見えます（実機と同じ）。`exclude_mjcf_body` に body を書くと、その body とその下にもレイが当たらなくなります（たとえば `base` でアーム全体を隠す）。
- 力覚センサの `mjcf_body` は工具側の根の body です。その body より下が工具側、上がロボット側になります。

## 3. 欄ごとの決まり（実装済みの型）

### 3.1 `sensor_msgs/JointState`（`joint`）

| 欄 | 中身 |
|---|---|
| `header.stamp` | 入っているセンサの、一番新しい測定時刻 |
| `header.frame_id` | 空 |
| `name` | センサを付けた MJCF の関節名（出力の設定の `sensors` の順） |
| `position` | 測定した角度 [rad]（slide 関節は [m]）。関節側の値 |
| `velocity` | 測定した角速度 [rad/s] |
| `effort` | 関節トルクを測るセンサ（定義ファイルに `torque` がある）なら、測定したトルク [N·m]。測らないなら空の配列。1 つの出力の中で、全部測るか全部測らないかにそろえる |

エンコーダのカウント値と状態のフラグは、JointState に欄がないので送りません（ROS でもドライバの中で扱う値です）。

### 3.2 `geometry_msgs/WrenchStamped`（`force_torque`）

| 欄 | 中身 |
|---|---|
| `header.stamp` | 測定時刻 |
| `header.frame_id` | センサ名 |
| `wrench.force` | 工具側がロボット側に及ぼす力 [N]、センサの軸。工具がぶら下がって止まっていると、重さが重力の向きに出る |
| `wrench.torque` | センサ原点まわりのトルク [N·m]、センサの軸 |

### 3.3 `sensor_msgs/Imu`（`imu`）

| 欄 | 中身 |
|---|---|
| `header.stamp` | ジャイロと加速度計のうち、新しいほうの測定時刻 |
| `header.frame_id` | センサ名 |
| `orientation` | (0, 0, 0, 1)。IMU は姿勢を測らない |
| `orientation_covariance` | `[0] = -1`（REP-145：姿勢なし）、ほかは 0 |
| `angular_velocity` | 測定した角速度 [rad/s]、センサの軸 |
| `linear_acceleration` | 測定した比力 [m/s²]、センサの軸。重力を含む（水平に止まっていると上向きに +9.8） |
| `angular_velocity_covariance`・`linear_acceleration_covariance` | 0（不明） |

ジャイロと加速度計の両方が 1 回測るまでは送りません。

### 3.4 `sensor_msgs/LaserScan`（`lidar`、2D）

| 欄 | 中身 |
|---|---|
| `header.stamp` | 走査の最初のレイの測定時刻 |
| `header.frame_id` | センサ名 |
| `angle_min`・`angle_max`・`angle_increment` | センサの z 軸まわりの角度 [rad]。0 が前、左が正 |
| `time_increment`・`scan_time` | レイの間の時間と、走査の間の時間 [s] |
| `range_min`・`range_max` | 測れる範囲 [m] |
| `ranges` | 測定した距離 [m]。範囲の外は REP-117 に従い +inf（遠すぎる・返らない）か -inf（近すぎる） |
| `intensities` | 強度を測らない実装では空 |

1 回の走査が終わるまでは送りません。送る周期が走査より速いときは、同じ走査をもう一度送ります（時刻は同じ）。

3D の LiDAR（定義の `scan.vertical.samples` が 2 以上）を LaserScan に出す設定は、起動時に止まります。

### 3.5 `sensor_msgs/PointCloud2`（`lidar`、2D・3D）

| 欄 | 中身 |
|---|---|
| `header.stamp` | 走査の最初のレイの測定時刻 |
| `header.frame_id` | センサ名 |
| `height`・`width` | 1 と、返った点の数（範囲内の点だけ） |
| `fields` | x, y, z, intensity（float32）、ring（uint16、0 が一番下の段）、time（float32、走査の始まりからの秒） |
| `point_step`・`row_step` | 22 と 22 × 点の数 |
| `is_bigendian`・`is_dense` | false、true |

点の位置はセンサの軸（FLU）で、その点を測った時のものです。

### 3.6 `sensor_msgs/Range`（`rangefinder`）

| 欄 | 中身 |
|---|---|
| `header.stamp` | 測定時刻 |
| `header.frame_id` | センサ名 |
| `radiation_type` | 0 超音波、1 赤外線、2 レーザー（定義の `radiation_type`） |
| `field_of_view` | 円錐の全角 [rad]。1 本のビームなら 0 |
| `min_range`・`max_range` | 測れる範囲 [m] |
| `range` | センサの x 軸方向に測った距離 [m]。範囲の外は REP-117 に従い +inf か -inf |
| `variance` | 実装が知っていれば分散 [m²]、知らなければ 0 |
