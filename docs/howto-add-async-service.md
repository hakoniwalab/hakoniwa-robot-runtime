# 非同期 RPC のサービスをプラグインとして足す

SO-101 の腕に ServoJ のようなサービスを足します。
クライアントが関節の目標を送ると、サービスはすぐに応え、腕はそのあとも目標まで動きます。
プラグインは、HakoCpp の型で要求を受け取り、応答を返す C++ クラスとして書きます。
Robot Runtime のアプリは、起動時に共有ライブラリと宣言 JSON を読み込みます。

## 0. 準備する

Bash を使い、最後まで同じシェルで実行します。
ビルド済みの Foundation（Core Pro と PDU Endpoint）と、`cffi`、`hakoniwa-pdu` 1.7.1 以降が入った Python を用意します。
Robot Arm は `--rpc-service-config` に対応する版（例は `5249ed3`）を使います。
PDU RPC は SHM に対応する main の #91 以降を使います。
Linux では GLFW と OpenGL の開発用ファイル、macOS では Homebrew の GLFW を用意します。
MuJoCo Robots が使う版の展開済み MuJoCo を用意します（Linux の例は 3.16.0）。

下のパスを手元に合わせて書き換えます。
`SO101_MODEL` は Model Forge が出力した `so101.xml` と `assets/` があるディレクトリです。
`MUJOCO_PRECOMPILED` は展開した MuJoCo のルートです。
`WORK` には新しい作業ディレクトリを指定します。

```bash
B=/path/to/checkouts
RUNTIME="$B/hakoniwa-robot-runtime"
ROBOT_ARM="$B/hakoniwa-robot-arm"
MUJOCO_ROBOTS="$B/hakoniwa-mujoco-robots"
SENSOR_ADAPTER="$B/hakoniwa-sensor-adapter"
REGISTRY="$B/hakoniwa-pdu-registry"
RPC="$B/hakoniwa-pdu-rpc"
PREFIX=/path/to/foundation/install
SO101_MODEL=/path/to/model-forge/so101/install
MUJOCO_PRECOMPILED=/path/to/extracted/mujoco
PYTHON=/path/to/venv/bin/python3
WORK="$B/my-servoj"
DEMO="$RUNTIME/examples/service_plugins/servoj_like"
JOBS=4
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$WORK"
```

## 1. 型を決める

要求と応答を ROS の `.srv` の形で書きます。
コメントは `#` から始まる別の行に書きます。
要求の関節配列は SO-101 のマニフェストでアクチュエータが並ぶ順です。

```bash
mkdir -p "$WORK/my_robot_srvs/srv"
cat > "$WORK/my_robot_srvs/srv/ServoJ.srv" <<'SRV'
# joint: 関節の目標 [rad]
float64[6] joint
# t: 目標に着くまでの時間 [s]
float64 t
---
# res: 0 = 受け付けた
int32 res
SRV
```

## 2. PDU の型を用意する

生成済みの ServoJ の型が手元にあれば、そのパスを設定して 2.4 節の確認へ進みます。
`PDU_TYPES` は `my_robot_srvs_msgs/pdu_cpptype_conv_ServoJRequestPacket.hpp` を含む型のルートです。
`PDU_PY` は `servoj_pdu/` の親ディレクトリです。

```bash
PDU_TYPES=/path/to/generated/types
PDU_PY=/path/to/generated/python-parent
```

### 2.1 `.srv` を Hakoniwa の `.msg` に変える

新しい型を生成する場合は、2.1〜2.3 節を実行します。
Jinja2 が使えるホストの `python3` と、C コンパイラを用意します。

```bash
(
  cd "$REGISTRY"
  python3 -m generators.generate_hako_service_msgs.main \
    "$WORK/my_robot_srvs/srv/ServoJ.srv" --out "$WORK/my-pdu-src"
)
```

`$WORK/my-pdu-src/srv_msgs/msg/` に `ServoJRequest`、`ServoJResponse` と両方の `Packet` の `.msg` ができます。

### 2.2 パッケージ名と生成する型を指定する

生成した `srv_msgs` フォルダを、使いたいパッケージ名に変えます。
パケットの 2 型を指定すると、生成器が参照先の型も一緒に生成します。

```bash
mv "$WORK/my-pdu-src/srv_msgs" "$WORK/my-pdu-src/my_robot_srvs_msgs"
cat > "$WORK/selected-types.txt" <<'TYPES'
my_robot_srvs_msgs/ServoJRequestPacket
my_robot_srvs_msgs/ServoJResponsePacket
TYPES
cat > "$WORK/search-paths.txt" <<PATHS
$REGISTRY/idl
$WORK/my-pdu-src
PATHS
```

### 2.3 C++ と Python の型を生成する

生成先は `$WORK/my-pdu` です。

```bash
(
  cd "$REGISTRY"
  python3 -m generators.generate_hako_pdu_msgs.main "$WORK/selected-types.txt" \
    --search-path-file "$WORK/search-paths.txt" --output-dir "$WORK/my-pdu" --incremental
)
PDU_TYPES="$WORK/my-pdu/types"
printf '要求 %s bytes\n' "$(cat "$WORK/my-pdu/pdu_size/my_robot_srvs_msgs/ServoJRequestPacket.txt")"
printf '応答 %s bytes\n' "$(cat "$WORK/my-pdu/pdu_size/my_robot_srvs_msgs/ServoJResponsePacket.txt")"

PDU_PY="$WORK/local-python"
mkdir -p "$PDU_PY/servoj_pdu"
(
  cd "$PDU_PY/servoj_pdu"
  touch __init__.py
  cp "$REGISTRY/pdu/python/binary_io.py" "$REGISTRY/pdu/python/pdu_utils.py" .
  cp -r "$WORK/my-pdu/python/my_robot_srvs_msgs" "$WORK/my-pdu/python/hako_srv_msgs" .
  touch my_robot_srvs_msgs/__init__.py hako_srv_msgs/__init__.py
)
```

ServoJ のパケットサイズは要求が 328 バイト、応答が 272 バイトです。

### 2.4 型を確認する

```bash
test -f "$PDU_TYPES/my_robot_srvs_msgs/pdu_cpptype_conv_ServoJRequestPacket.hpp"
test -f "$PDU_TYPES/my_robot_srvs_msgs/pdu_cpptype_conv_ServoJResponsePacket.hpp"
PYTHONPATH="$PDU_PY" "$PYTHON" -c \
  'from servoj_pdu.my_robot_srvs_msgs import pdu_conv_ServoJRequestPacket; print("ok")'
```

## 3. プラグインと宣言を書く

まずはサンプルの 3 ファイルを作業ディレクトリに写します。

```bash
mkdir -p "$WORK/plugin"
cp "$DEMO/servoj_like.cpp" "$DEMO/CMakeLists.txt" "$DEMO/my_servoj.plugin.json" "$WORK/plugin/"
```

### 3.1 C++ クラスを書く

#### 3.1.1 ソースファイルを用意する

コピーした `$WORK/plugin/servoj_like.cpp` に、`ServoJRobot` の実装と 2 つのマクロがそろっています。
まずはこのファイルを編集します。別の `.cpp` に書く場合は、同じディレクトリに置きます。
`$WORK/plugin/CMakeLists.txt` の `add_library` で `servoj_like.cpp` を新しいファイル名に替えます。

新しい `.cpp` には次のヘッダーを読み込みます。
`RobotPlugin`、`RobotContext`、`StepView`、`StateView` は `hakoniwa::robot_runtime::plugin` にあります。
`HAKO_ROBOT_SERVICE(my_robot_srvs_msgs, ServoJ);` で `ServoJService` を定義します。

```cpp
#include "hakoniwa/robot_runtime/plugin/robot_plugin.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJRequestPacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJResponsePacket.hpp"

#include <cstddef>

using hakoniwa::robot_runtime::plugin::RobotContext;
using hakoniwa::robot_runtime::plugin::RobotPlugin;
using hakoniwa::robot_runtime::plugin::StateView;
using hakoniwa::robot_runtime::plugin::StepView;

HAKO_ROBOT_SERVICE(my_robot_srvs_msgs, ServoJ);
```

#### 3.1.2 クラスを定義してサービスを結び付ける

`RobotPlugin` を継承します。基底クラスのコンストラクタは次の形です。

```cpp
explicit RobotPlugin(const RobotContext& context);
```

`step`、`reset`、`cancel` は決まった名前です。`step` は必須で、`reset` と `cancel` は必要に応じて上書きします。
要求と応答のメソッド名は自由です。末尾に `const` が付かないメンバー関数を使い、`bind` でサービス番号に結び付けます。

次の宣言に沿ってメソッドを実装します。処理の中身は、コピーした `servoj_like.cpp` を参照します。

```cpp
class ServoJRobot final : public RobotPlugin {
    using Service = ServoJService;

public:
    explicit ServoJRobot(const RobotContext& context) : RobotPlugin(context)
    {
        bind<Service>(0, &ServoJRobot::on_goal, &ServoJRobot::reply);
    }

    void step(StepView& step) override;
    void reset() override;
    void cancel(std::size_t service) override;

private:
    bool on_goal(const Service::Request& request);
    Service::Response reply(const StateView& state);
};
```

サンプルのように `cancel` を使わない場合は、その宣言を省きます。
ファイルの末尾で `HAKO_ROBOT_PLUGIN(ServoJRobot, servoj_like)` を無名名前空間の外に書き、読み込み口を公開します。

Runtime は `pdu_robot` ごとにインスタンスを 1 つ作ります。
サービス番号は、マニフェストの `services` のうち、同じ `pdu_robot` の行を上から数えた順です。
先頭は 0 です。この順に `RobotContext::services` に入ります。
サービスが複数ある場合は、各 `bind` に対応する番号を渡します。
`bind` で指定した型は、`service.json` の `type` とそろえます。

#### 3.1.3 状態を読んで指令を出す

| メソッド | サンプルでやること |
| --- | --- |
| コンストラクタ | アクチュエータが 6 つ、サービスが 1 つあることを確認し、サービスを結び付けます。 |
| `on_goal` | `HakoCpp_ServoJRequest` の時間と目標角を確かめます。受け付けるときは目標を保存して `true` を返します。 |
| `step` | `StepView` から開始角と時刻を読み、`finish` で結果を設定し、`command` で位置指令を出します。 |
| `reply` | 物理を進めた後の `StateView` を受け取り、`res = 0` の `HakoCpp_ServoJResponse` を返します。 |
| `reset` | 動きを止めます。 |

`context().actuators[i]` の `minimum` と `maximum` は、アクチュエータ設定の `spec.limit` から来ます。
添字 `i` は、マニフェストのアクチュエータの並びで 0 から数えた位置です。
各関節の範囲、要求の目標、状態の読み取り、出力の指令は、同じ添字を使います。
開始角は `step.position(i)` で読みます。回転関節の単位は rad、直動関節の位置の単位は m です。
シミュレーション時刻は `step.tick().sim_time_ns` で読みます。単位は ns です。
SO-101 の例は `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`、`gripper` の順です。

`step` は要求がないときも、物理ステップごとに 1 回呼ばれます。
サンプルは `Doing` で `Ok` を設定し、`Canceling` で動きを止めて `Canceled` を設定します。
応答が済んでスロットが `Idle` になっても、位置指令は開始角から目標へ補間し、到達後は目標を出し続けます。
最初のステップでは、まだ指令を出していないため `tick.selected` は 0 です。
サンプルは指令を出したときに `commanded_` を `true` にします。
サンプルは、自分が指令を出したあとで Arbiter がほかの制御に切り替えたときに指令を止めます。

```cpp
const auto& tick = step.tick();
if (commanded_ && !tick.selected) {
    active_ = false;
}
```

取り消しの通知ですぐに処理したい場合は、`cancel` を上書きできます。

メソッドはシミュレーションのスレッドで同期して動きます。
`StepView`、`StateView` と要求への参照は、その呼び出しの間だけ使えます。
生成されたコンバーターが要求を読み、Core の応答ヘッダーを保ったまま応答の本体とメタデータを書きます。

### 3.2 宣言を書く

`my_servoj.plugin.json` の `service_types` には扱える RPC 型を並べます。

```json
{
  "service_types": ["my_robot_srvs_msgs/ServoJ"]
}
```

`plant_operations` にはプラグインが出す Plant 操作を並べます。
指定できる操作は `surface_friction`、`body_motion`、`humanoid_motion` です。
ServoJ が出すのは関節指令だけなので、この項目は省略します。

### 3.3 C の関数テーブルを使う（上級者向け）

C で書く場合やパケットを直接扱う場合は、[service_plugin.h](../include/hakoniwa/robot_runtime/plugin/service_plugin.h)
の `HakoServicePluginV1` を実装し、`HAKO_ROBOT_PLUGIN_ENTRY` で公開します。
入出力はメタデータを含む PDU パケットです。
バッファのサイズを確認し、Core の応答ヘッダーを保ち、応答に合うメタデータを書きます。
宣言 JSON とビルド後の配置は同じです。

## 4. プラグインをビルドする

自分で `CMakeLists.txt` を書く場合は、コピーしたファイルと同じく `cxx_std_20` で C++20 を指定します。
インクルード先には Robot Runtime の `include`、自分で生成した型のルート、PDU Registry の `pdu/types` の 3 つを指定します。

```bash
cmake -S "$WORK/plugin" -B "$WORK/build-plugin" -DCMAKE_BUILD_TYPE=Release \
  -DHAKO_RUNTIME_INCLUDE_DIR="$RUNTIME/include" \
  -DHAKO_SERVOJ_PDU_TYPES="$PDU_TYPES" \
  -DHAKO_PDU_REGISTRY_TYPES="$REGISTRY/pdu/types"
cmake --build "$WORK/build-plugin" --parallel "$JOBS"
PLUGIN="$WORK/build-plugin/libmy_servoj.so"
ls -l "$PLUGIN" "$WORK/build-plugin/libmy_servoj.plugin.json"
```

サンプルは CMake の MODULE を使うので、Linux と macOS でライブラリは `libmy_servoj.so` です。
CMake はライブラリの拡張子を `.plugin.json` に替えた名前で、その隣に宣言をコピーします。
ライブラリと宣言は同じディレクトリに置いて配布します。

## 5. マニフェストと `service.json` を書く

6.2 節の `run.py` は両方のファイルを毎回作ります。
手で設定するときは、この節と `run.py` の `prepare()` を参照します。

### 5.1 マニフェスト

Robot Arm の `recipes/so101/asset-manifest.json` の `services` に次の行を足します。

```json
"services": [
  { "name": "ArmService/ServoJ/SO101", "type": "plugin", "pdu_robot": "SO101",
    "plugin": { "library": "/abs/path/to/libmy_servoj.so" },
    "lockstep": false, "priority": 16 }
]
```

`plugin.library` は手順 4 の共有ライブラリです。
相対パスで書くと、マニフェストのディレクトリから解決されます。
同じ `pdu_robot` のプラグイン行を増やすと、1 台のロボットが複数のサービスを持ちます。
その行の `plugin.library`、`plugin.config`、`priority` はそろえます。
ServoJ はロックステップを無効にし、優先度は手動の 20 と軌道の 10 の間の 16 にします。

### 5.2 Core の `service.json`

```json
{
  "pduMetaDataSize": 24,
  "services": [{
    "name": "ArmService/ServoJ/SO101", "type": "my_robot_srvs_msgs/ServoJ", "maxClients": 1,
    "pduSize": {
      "server": { "baseSize": 336, "heapSize": 0 },
      "client": { "baseSize": 280, "heapSize": 0 }
    },
    "clients": [{ "name": "ServoJClient", "client_endpoint": {
      "nodeId": "servoj-python", "protocol": "shm", "assetName": "ServoJClientAsset",
      "deltaTimeUsec": 2000, "coreConfig": "/abs/path/to/core.json" } }]
  }]
}
```

マニフェストと `service.json` の `name` をそろえます。
`service.json` の `type` は RPC の型で、宣言 JSON の `service_types` に含めます。
Runtime は起動時にこの型と、宣言した Plant 操作を使えるかどうかを確かめます。
宣言ファイルが無い場合や、宣言と設定が合わない場合は起動が止まります。
`server.baseSize` は要求の 328 ＋ 8 バイト、`client.baseSize` は応答の 272 ＋ 8 バイトです。
クライアントの `client_name` と `node_id` は、それぞれ `clients[].name` と `nodeId` に合わせます。
`coreConfig` と `HAKO_CONFIG_PATH` には同じ Core 設定ファイルを指定します。

## 6. 動かして呼ぶ

### 6.1 PDU RPC と Robot Arm をビルドする

Linux では Foundation の `libshakoc.so`、macOS では `libshakoc.dylib` をリンクします。
センサありのビルドでは GLFW を使うため、画面なしで動かす場合も `HAKO_ARM_ENABLE_VIEWER=ON` にします。
macOS では、`include/mujoco/` と `lib/mujoco.framework/` を含む MuJoCo の展開先を `MUJOCO_PRECOMPILED` に指定します。

```bash
SHLIB=so
SEARCH_PREFIX="$PREFIX"
if [[ $(uname -s) == Darwin ]]; then
  SHLIB=dylib
  SEARCH_PREFIX="$PREFIX;$(brew --prefix glfw)"
  cmake -E copy_directory "$MUJOCO_PRECOMPILED" "$WORK/build-arm/_deps/mujoco_precompiled-src"
fi

cmake -S "$RPC" -B "$WORK/build-rpc" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DHAKONIWA_PDU_REGISTRY_DIR="$REGISTRY" \
  -DHAKO_PDU_RPC_ENABLE_SHM=ON -DHAKO_PDU_RPC_BUILD_TESTS=OFF
cmake --build "$WORK/build-rpc" --target hakoniwa_pdu_rpc_shared --parallel "$JOBS"

cmake -S "$ROBOT_ARM" -B "$WORK/build-arm" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH="$SEARCH_PREFIX" \
  -DHAKONIWA_ROBOT_RUNTIME_ROOT="$RUNTIME" -DHAKONIWA_MUJOCO_ROBOTS_ROOT="$MUJOCO_ROBOTS" \
  -DHAKONIWA_FOUNDATION_PREFIX="$PREFIX" -DHAKO_ROBOT_ARM_CONTROLLER_PLUGIN=ON \
  -DHAKO_ARM_ENABLE_VIEWER=ON -DHAKO_ROBOT_ARM_ENABLE_SENSOR=ON \
  -DHAKO_ROBOT_ARM_SENSOR_IMPLEMENTATIONS=reference \
  -DHAKO_ROBOT_ARM_SENSOR_ADAPTER_ROOT="$SENSOR_ADAPTER" \
  "-DCMAKE_CXX_STANDARD_LIBRARIES=\"$PREFIX/lib/libshakoc.$SHLIB\"" \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  -DFETCHCONTENT_SOURCE_DIR_MUJOCO_PRECOMPILED="$MUJOCO_PRECOMPILED"
cmake --build "$WORK/build-arm" --target robot-arm-hakoniwa-asset --parallel "$JOBS"
```

### 6.2 動かす

`run.py` はマニフェストと設定を書き、腕とクライアントを起動します。
両方が登録されたら `hako-cmd start` で始め、5 回呼び終えると止めます。
実行ごとに新しいディレクトリを指定します。

```bash
export ROBOT_ARM PREFIX SO101_MODEL PLUGIN
export ARM_ASSET="$WORK/build-arm/bin/robot-arm-hakoniwa-asset"
export HAKO_PDU_RPC_LIBRARY="$WORK/build-rpc/src/libhakoniwa_pdu_rpc.$SHLIB"
export PYTHONPATH="$RPC/python:$PDU_PY:$REGISTRY${PYTHONPATH:+:$PYTHONPATH}"
if [[ $SHLIB == dylib ]]; then
  export DYLD_LIBRARY_PATH="$WORK/build-rpc/src:$PREFIX/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

"$PYTHON" "$DEMO/run.py" "$WORK/run-$(date +%Y%m%d-%H%M%S)" --no-viewer
```

macOS では SHM 対応の PDU RPC を使えるよう、`DYLD_LIBRARY_PATH` の先頭を `build-rpc/src` にしています。
デスクトップで MuJoCo ウィンドウを開くには、別の実行ディレクトリを指定し、`--no-viewer` を外します。

クライアントは応答を受けてから実測角を読み、目標の 0.01 rad 以内に 0.2 秒とどまるまで待ちます。
Linux での出力例は次の 5 行です。

```text
res=0 shoulder_pan reached=+0.500001 rad (goal=+0.50)
res=0 shoulder_pan reached=-0.500001 rad (goal=-0.50)
res=0 shoulder_pan reached=+0.000001 rad (goal=+0.00)
res=0 elbow_flex reached=+0.800327 rad (goal=+0.80)
res=0 elbow_flex reached=+0.000454 rad (goal=+0.00)
```

自分のクライアントを書くときは [client.py](../examples/service_plugins/servoj_like/client.py) を参照してください。
`create_request()` で要求の `joint` と `t` を設定し、`call()` が返す応答の `res` を読みます。
