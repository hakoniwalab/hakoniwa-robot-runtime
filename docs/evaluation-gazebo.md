# Gazebo との比べ方と結果（SO-101、ros2_control）

同じロボット（SO-101）、同じ ROS 2 のコントローラ、同じ軌道で、箱庭と Gazebo を比べた結果です。箱庭は `HakoniwaAssetSystem`（ros2_control そのものが箱庭アセット、[ros2-control.md](ros2-control.md) の 10 章）、Gazebo は `gz_ros2_control`（controller_manager が Gazebo の中で動く）です。

## 1. 比べた条件

| 項目 | 箱庭 | Gazebo |
|---|---|---|
| 物理 | Robot Runtime（MuJoCo）、画面なし | `gz sim -s`（Harmonic 8.15、サーバーだけ） |
| ros2_control | `HakoniwaAssetSystem`（共有メモリで直結） | `gz_ros2_control`（Gazebo の中） |
| 物理の刻み | 2 ms | 2 ms |
| ロボット | SO-101 の同じ URDF | 同じ |
| コントローラ | joint_trajectory_controller、controller_manager 100 Hz（`ros2_controllers.yaml`） | 同じ |
| 軌道 | 9 秒で 3 点（`tools/compare/measure.py`） | 同じ |
| 画面・ブリッジ | なし | なし |
| 場 | 作業台の環境（床、壁、柱、立方体） | 床だけ |

- 環境：macOS（Apple Silicon）の Docker Desktop（6 コア、8 GB）、Linux arm64、ROS 2 Jazzy。両方とも同じコンテナ（`ros2/docker/Dockerfile.jazzy-gz`）で、1 つずつ動かしました。
- 関節の位置の制御は、作りが違います。MuJoCo は力に上限のある位置のアクチュエータ、`gz_ros2_control` は位置の指令を速度に直して関節を動かします。制御の差には、この違いも入ります。

### 1.1 Gazebo の当たり判定をそろえる

URDF をそのまま Gazebo に入れると、この環境では実時間に届きませんでした（0.9 倍）。原因は当たり判定で、MuJoCo と扱いが違うためです。

| Gazebo（腕だけ、ros2_control なし、できるだけ速く） | 実時間比 |
|---|---|
| URDF の当たり判定のメッシュのまま、地面あり | 0.8〜0.9 |
| URDF のメッシュのまま、地面なし | 22〜28 |
| 凸包、地面あり | 5〜6 |
| 凸包、地面なし | 24〜30 |

- 世界に固定した根元（`base_link`）の当たり判定が、地面（別のモデル）と接したままで、毎刻みその接触を計算していました。MuJoCo は、世界に固定した物どうしの接触を計算しません。
- MuJoCo は当たり判定のメッシュを凸包にして使います。Gazebo はメッシュ（1 つ 1〜5 万面）そのものを使います。

そこで、比べるときの Gazebo は MuJoCo と同じ扱いにそろえました（`so101_gz.launch.py collision_hulls:=`）。

- 当たり判定は、URDF のメッシュの凸包（`tools/so101_collision_hulls.py`）。
- 世界に固定した `base_link` の当たり判定は外す。

URDF のままの Gazebo の値も、参考として並べます。

## 2. 比べ方

箱庭の controller_manager は壁時計で回ります（ros2_control と箱庭は別のスレッドで、最新値でつながる。[ros2-control.md](ros2-control.md) の 10.2）。そのため、制御の精度を保つには実時間で回します。比べ方は次の 3 つに分けました。

| 比べるもの | 条件 | 指標 |
|---|---|---|
| 物理の速さ | 制御なし、物理だけ、できるだけ速く | 実時間比、シミュレーション 1 秒あたりの CPU |
| 制御込みの実力 | ros2_control あり、実時間 | CPU、制御の差、毎回同じか |
| 参考：速くしたときの制御 | ros2_control あり、できるだけ速く | 制御の差 |

| 指標 | 測り方 |
|---|---|
| 実時間比 | 壁時計 5 秒の間に進んだシミュレーション時間 ÷ 5 秒。箱庭は世界の時刻（`hako-cmd pmeta`）、Gazebo は Gazebo 自身の実時間比（`/stats`）、ros2_control ありは `/clock`（`tools/compare/run-physics.sh`、`speed.py`） |
| CPU | コンテナの中の全プロセスの CPU 時間（`/proc`）を、シミュレーション 1 秒あたりに直した値 |
| 制御の差 | joint_trajectory_controller 自身が出す状態（`/arm_controller/controller_state`）の目標と実際の差。腕の 5 関節の最大、軌道の間の最大と平均（`tools/compare/measure.py`） |
| 毎回同じか | 同じ設定で 3 回流し、目標が動き出した時刻をそろえて、関節の値（実際）の違いの最大。どの 2 回の組でも比べる（`tools/compare/repeat.py`） |

どれも 3 回ずつ流しました。ros2_control ありの軌道の結果は、すべて SUCCEEDED でした。

## 3. 結果

### 3.1 物理の速さ（制御なし）

| | 実時間比（3 回） | CPU（シミュレーション 1 秒あたり、CPU 秒） |
|---|---|---|
| 箱庭（Robot Runtime だけ） | 136.4 / 137.1 / 137.5 | 0.015 |
| Gazebo（当たり判定をそろえた、SO-101 だけ） | 25.2 / 25.3 / 25.8 | 0.058 |
| Gazebo（URDF のまま、地面あり、参考） | 0.8〜0.9 | — |

- 箱庭の物理は、当たり判定をそろえた Gazebo の約 5.4 倍の速さで、シミュレーション 1 秒あたりの CPU は約 1/4 でした。
- 場の違い：箱庭は作業台の環境（床、壁、柱、立方体）で、腕は位置のアクチュエータで保持されています。Gazebo は地面なしで、腕は保持されずにぶら下がっています（制御なしで地面があると、腕が地面に倒れて接触だらけになるため）。

### 3.2 制御込みの実力（実時間）

| | 実時間比 | CPU | 制御の差（最大） | 制御の差（平均） | 毎回同じか |
|---|---|---|---|---|---|
| 箱庭 | 1.02 | 1.35〜1.38 | 0.006〜0.007 rad | 0.0016 rad | 0.0033 rad |
| Gazebo（当たり判定をそろえた） | 1.01〜1.08 | 1.35〜1.54 | 0.031 rad | 0.018 rad | 0.0012 rad |

- CPU は同じくらいでした。箱庭の値には、アセットが時刻を進めるときの待ち（箱庭コアの待ちは休まず回る）の CPU が入っています。
- 制御の差には、1 章の位置の制御の作りの違いが入っています。
- 毎回同じか：どちらも、まったく同じにはなりませんでした。Gazebo でも、軌道の受け付け（ROS 2 の action）は物理とは別に非同期で動くので、軌道の始まる瞬間が刻みに対して毎回少しずれます。

### 3.3 参考：ros2_control ありで、できるだけ速く

| | 実時間比 | シミュレーション 1 秒あたりの制御の回数 | 制御の差（最大） | 毎回同じか |
|---|---|---|---|---|
| 箱庭 | 119 | 1 回未満（controller_manager は壁時計で 100 Hz） | 0.07 rad | 0.040 rad |
| Gazebo（当たり判定をそろえた） | 14 | 100 回（controller_manager はシミュレーションの刻みで 100 Hz） | 0.031 rad | 0.022 rad |

- Gazebo は controller_manager をシミュレーションの刻みで回すので、速くしても制御の回数が変わらず、精度も変わりません。
- 箱庭は、速くするとシミュレーション時間あたりの制御の回数が減り、精度が落ちます。この表の実時間比は、制御の仕事量がそろっていないので、速さの比べには使いません。

## 4. 読み方

- **物理の速さ**：箱庭の物理（MuJoCo）は、当たり判定をそろえた Gazebo の約 5.4 倍の速さで、シミュレーション時間あたりの CPU は約 1/4 でした。
- **制御込み（実時間）**：CPU は同じくらいで、どちらも実時間で軌道を正しく実行しました。
- **URDF のまま使うときの注意**：Gazebo は、URDF の当たり判定のまま、固定した根元が地面に接した状態では、この環境で実時間に届きませんでした。
- **速くしたときの制御**：Gazebo は速くしても精度が保たれ、箱庭は落ちます。箱庭で精度を保ったまま速くするには、controller_manager をシミュレーションの刻みで回す形（物理と同じプロセスで、ros2_control が物理を進める）が要ります。
- この結果は、この 1 つの環境（Mac の Docker、6 コア、GPU なし）とこの 1 つのロボット（関節だけ、センサーなし）のものです。センサーと MoveIt を含めた比べ（掴んで置く）は、このあと足します。

## 5. 同じ測定をする

Docker だけの構成（[ros2/hakoniwa_ros2_control/README.md](../ros2/hakoniwa_ros2_control/README.md) の「Docker だけで動かす」）の 1〜3 が済んでいることが前提です。センサーは使わないので、SO-101 の Robot Runtime は `--sensors` なしの設定（`asset-manifest-environment.json`）を使います。

ホストで、`hakoniwa-robot-arm` のリポジトリのルートから、比べるためのイメージを作ってコンテナに入ります。

```bash
docker build -t hakoniwa-ros2-control-gz:jazzy -f ../hakoniwa-robot-runtime/ros2/docker/Dockerfile.jazzy-gz ../hakoniwa-robot-runtime/ros2/docker
IMAGE_NAME=hakoniwa-ros2-control-gz CONTAINER_NAME=hakoniwa-gz-jazzy HAKONIWA_DOCKER_GUI=off bash docker/run.bash jazzy
```

コンテナの端末で、凸包を作り（trimesh が要ります）、測ります。

```bash
python3 -m pip install --break-system-packages -q trimesh
T=/workspace/hakoniwa-robot-runtime/ros2/hakoniwa_ros2_control/tools
python3 $T/so101_collision_hulls.py --urdf /workspace/work-docker-jazzy/model-forge/so101/source/Simulation/SO101/so101_new_calib.urdf --output /workspace/work-docker-jazzy/model-forge/so101/gazebo-hull
$T/compare/run-physics.sh
$T/compare/run-hakoniwa.sh rt 3
$T/compare/run-gazebo.sh rt 3
$T/compare/run-hakoniwa.sh max 3
$T/compare/run-gazebo.sh max 3
$T/compare/run-hakoniwa.sh max 3 speed
$T/compare/run-gazebo.sh max 3 speed
python3 $T/compare/repeat.py /tmp/compare
```

`run-physics.sh` は 3.1 の行（実時間比と CPU）、`speed` を付けた行は 3.3 の実時間比を出します。軌道ごとの結果は `/tmp/compare/<箱庭か Gazebo>-<設定>-<回>.json`（`result`、`ratio`、`cpu_s_per_sim_s`、`error_max`、`error_mean`）です。`repeat.py` は設定ごとの「毎回同じか」を出します。
