# 非同期 RPC のサービスをプラグインとして足す（手順書・ServoJ のようなサービスを例に）

この文書は、Robot Runtime に、自分の RPC のサービスを**外部のプラグイン（共有ライブラリ）**として足す人のための手順書です。例として、Dobot の `ServoJ` のようなサービス（関節の目標を受け取って、すぐ応え、そのあと目標まで動かす）を、SO-101 の腕に足します。

**Robot Runtime のソースは、1 行も変えません。** 自分のプラグイン（`.so`）を作り、マニフェストに書くだけで、Robot Runtime が起動のときに自動で読み込みます。

> **状態（2026-11-26）**：第 3 稿。プラグインの仕組みは Robot Runtime の main に入っています（hakoniwalab/hakoniwa-robot-runtime#67）。この手順書のもとになったサンプル（`examples/service_plugins/servoj_like/`）は、Linux と macOS（Apple Silicon）で、本物の hakoniwa-core-pro と SO-101 で 5 つの動きが通ることを確かめています。この第 3 稿は、初めて読む人が手順書だけを見て macOS（Apple Silicon）で最後まで追い、画面なしで 3 回とも期待どおりの出力になることを確かめています（Linux での手順書の確かめは、まだです）。

仕組みの説明は [design-async-rpc.md](design-async-rpc.md)、マニフェストの書き方は [configuration.md](configuration.md) の 2.4 節にあります。

## 0. 全体の流れと前提

```text
1. 型を決める          ServoJ.srv を書く
2. PDU の型を作る      手元で生成する（hakoniwa-pdu-registry の本体には足さない）
3. プラグインを書く    service_plugin.h の関数を実装した C/C++ のファイル 1 つ
4. プラグインを作る    CMake で共有ライブラリにする
5. 書く                マニフェストの services と、core-pro の service.json
6. 動かして呼ぶ        Robot Runtime（腕）→ Python のクライアント → hako-cmd start
```

**前提**
- Business Pack の Foundation（hakoniwa-core-pro `e0fa56f` 以降、hakoniwa-pdu-endpoint、`hakoniwa-pdu` 1.7.1 以降、Python と cffi）が入っていること。Foundation は Business Pack の手順で作ります。手で作ると、つまずきやすい点が多いためです（付録 B）。
- 並べて clone しておくもの：hakoniwa-robot-runtime、hakoniwa-robot-arm、hakoniwa-mujoco-robots、hakoniwa-sensor-adapter、hakoniwa-pdu-registry、hakoniwa-pdu-rpc。どれも main で確かめています。
- SO-101 のモデル（Business Pack の Model Forge の出力。`so101.xml` と `assets/`）。
- Docker（手順 2.3 で使う）。macOS では Homebrew の `glfw`。

この手順書では、場所を次の変数で書きます。自分の環境に合わせて、最初に一度だけ設定してください（すべて絶対パス）。**最後まで、同じシェルで続けてください。** 後の節は、前の節で決めた変数（`SRC_REF`、`PDU_TYPES`、`PDU_PY`、`PLUGIN`、`SHLIB`）を使います。別の端末で続けるときは、6.2 の頭の「変数のまとめ」を貼り直してください。

**待ち時間を減らすには**：6.1 のビルド（数分かかる）は、2〜5 節に依存しません。別の端末で、0 節の変数を設定してから先に走らせておけます。

```bash
B=$HOME/work                                   # リポジトリを並べた場所
RUNTIME=$B/hakoniwa-robot-runtime
ROBOT_ARM=$B/hakoniwa-robot-arm
MUJOCO_ROBOTS=$B/hakoniwa-mujoco-robots
SENSOR_ADAPTER=$B/hakoniwa-sensor-adapter
REGISTRY=$B/hakoniwa-pdu-registry
RPC=$B/hakoniwa-pdu-rpc
PREFIX=$B/hakoniwa-business-pack/work/foundation/install       # Foundation
SO101_MODEL=$B/hakoniwa-business-pack/work/model-forge/so101/install
PYTHON=$PREFIX/python/bin/python3
WORK=$B/my-servoj                              # 自分の作業の場所
mkdir -p "$WORK"
```

**できあがり**：Python から `ArmService/ServoJ/SO101` を呼ぶと、すぐ `res = 0` が返り、そのあと SO-101 の関節が目標へ動きます。

## 1. 型を決める（ServoJ.srv）

要求と応答を、ROS の `.srv` の形で書きます。配列は長さを決めます（固定長の PDU にするため）。

```bash
mkdir -p "$WORK/my_robot_srvs/srv"
cat > "$WORK/my_robot_srvs/srv/ServoJ.srv" <<'EOF'
# joint: 関節の目標 [rad]
float64[6] joint
# t: 目標に着くまでの時間 [s]
float64 t
---
# res: 0 = 受け付けた
int32 res
EOF
```

- **コメントは、行の頭に `#` を置いた別の行に書いてください。** 欄の行の後ろに書くと、手順 2.1 の変換の道具が `ValueError: Invalid field line` で止まります。
- パッケージ名と型名は、自分のものにします。Hakoniwa の既存の名前（`hako_srv_msgs` など）は使いません。

## 2. PDU の型を手元で作る

自分の型だけを生成します。hakoniwa-pdu-registry の追跡されるファイル（`pdu/`、`config/`）は変わりません。

### 2.1 .srv を Hakoniwa の .msg に変える

2 節の `python3` は、ホストの Python です（Foundation の `$PYTHON` ではありません）。要るのは 2.3 の PyYAML だけです。

```bash
cd "$REGISTRY"
python3 -m generators.generate_hako_service_msgs.main \
  "$WORK/my_robot_srvs/srv/ServoJ.srv" --out "$WORK/my-pdu-src"
```

`$WORK/my-pdu-src/srv_msgs/msg/` に、`ServoJRequest.msg`、`ServoJRequestPacket.msg`、`ServoJResponse.msg`、`ServoJResponsePacket.msg` の 4 つができます（フォルダの名前は、いつも `srv_msgs` になります）。

### 2.2 ROS のパッケージの形にして、手元の git に入れる

```bash
cd "$WORK/my-pdu-src"
mv srv_msgs my_robot_srvs_msgs
cat > my_robot_srvs_msgs/package.xml <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_srvs_msgs</name>
  <version>0.0.0</version>
  <description>ServoJ packet definitions.</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>
</package>
EOF
git init -q
git add -A
git -c user.name="Your Name" -c user.email=you@example.com commit -q -m "ServoJ PDU types"
SRC_REF=$(git rev-parse HEAD)
```

### 2.3 生成する

「どこから取るか」の設定は、registry の外に置けます。

```bash
cat > "$WORK/my_robot_srvs_msgs.yaml" <<EOF
schema_version: 1
source:
  type: git
  repository: file://$WORK/my-pdu-src
  ref: $SRC_REF
packages:
  - my_robot_srvs_msgs
EOF

cd "$REGISTRY"
rm -rf .cache/my-pdu      # 前の生成の残りを消す（古い型が混ざらないように）
python3 -c 'import yaml' 2>/dev/null || {
  python3 -m venv "$WORK/venv-fetch" && "$WORK/venv-fetch/bin/pip" install -q -r tools/requirements-fetch.txt
  export PATH="$WORK/venv-fetch/bin:$PATH"
}
python3 tools/generate_from_sources.py \
  --sources "$WORK/my_robot_srvs_msgs.yaml" \
  --output-dir .cache/my-pdu \
  my_robot_srvs_msgs/ServoJRequestPacket \
  my_robot_srvs_msgs/ServoJResponsePacket
# 道具が registry の中に取ってきた .msg の写し（git に追跡されない）を片付ける
rm -rf "$REGISTRY/output_msgs/my_robot_srvs_msgs"
git status --short -- pdu/ config/                    # 何も出なければ、registry は変わっていない
```

- このコマンドは、ホスト（Docker の外）で動かします。生成は、道具が中で Docker を使って行います。
- Docker が見えるのは registry の中だけなので、出力先は registry の中の `.cache/...` です。
- PyYAML がシステムの Python に入っていて、pip が `externally-managed-environment` で断る場合は、上のように venv を使います。

### 2.4 出てきたもの

```bash
cat "$REGISTRY/.cache/my-pdu/pdu_size/my_robot_srvs_msgs/ServoJRequestPacket.txt"    # 328
cat "$REGISTRY/.cache/my-pdu/pdu_size/my_robot_srvs_msgs/ServoJResponsePacket.txt"   # 272
PDU_TYPES=$REGISTRY/.cache/my-pdu/types
```

**Python で使う形にする**：生成された Python の変換は、1 つ上のフォルダの `pdu_utils` と `binary_io` を相対で読みます。親のパッケージを 1 つ作って、次の形に写します。

```bash
PDU_PY=$WORK/local-python
mkdir -p "$PDU_PY/servoj_pdu"
cd "$PDU_PY/servoj_pdu"
touch __init__.py
cp "$REGISTRY/pdu/python/binary_io.py" "$REGISTRY/pdu/python/pdu_utils.py" .
cp -r "$REGISTRY/.cache/my-pdu/python/my_robot_srvs_msgs" "$REGISTRY/.cache/my-pdu/python/hako_srv_msgs" .
touch my_robot_srvs_msgs/__init__.py hako_srv_msgs/__init__.py
PYTHONPATH="$PDU_PY" "$PYTHON" -c \
  'from servoj_pdu.my_robot_srvs_msgs import pdu_conv_ServoJRequestPacket; print("ok")'
```

## 3. プラグインを書く

プラグインは、公開ヘッダー `include/hakoniwa/robot_runtime/plugin/service_plugin.h` の関数の表（`HakoServicePluginV1`）を実装した、C/C++ のファイル 1 つです。Robot Runtime のソースを変える必要はありません。

**一番の近道は、サンプルを写して直すことです。**

```bash
mkdir -p "$WORK/plugin"
cp "$RUNTIME/examples/service_plugins/servoj_like/servoj_like.cpp" \
   "$RUNTIME/examples/service_plugins/servoj_like/CMakeLists.txt" "$WORK/plugin/"
```

表の関数と、ServoJ のようなサービスでやること（サンプルの中身）：

| 関数 | いつ | サンプルでやること |
|---|---|---|
| `create(args, error, size)` | 起動のとき 1 回 | `args->joint_names`（マニフェストの、その腕のアクチュエータの名前。マニフェストの順）から、6 つの関節の位置を探して覚える。足りなければ `error` に書いて `nullptr` |
| `on_request(inst, packet, size)` | 要求が来たとき | パケットのメタデータを確かめ、`Hako_ServoJRequest` を読む。`t` と関節の範囲を確かめ、よければ目標を覚えて `1`。おかしければ `0`（INVALID で応える） |
| `on_step(inst, tick, states, server_state, commands)` | 要求を受け付けたステップ | 今の関節の角度と時刻を「動きの始まり」として覚え、**すぐ `HAKO_SERVICE_STEP_OK`** を返す（0 ステップで応える）。取り消し中（`CANCELING`）なら止めて `CANCELED` |
| `on_response(inst, tick, states, reply)` | 応える直前（物理の後） | 応答の中身（`res = 0`）を書く。**頭（`reply->header`）は読むだけで、書き換えない。** メタデータの `heap_off`・`total_size` を、応答の全体に合わせる。中身の欄は毎回すべて書く |
| `update(inst, tick, states, …, commands, …)`（任意） | **毎刻み**（要求がない間も） | 動きの途中なら、始まりから目標までを `t` 秒かけて直線で結んだ位置を、6 つの関節に指令する。手動の操作に優先を奪われたら（`tick->selected` が 0）、やめる |
| `reset(inst)` | リセットのとき | 動きを止める |
| `destroy(inst)` | 終わりのとき | 片付ける |
| `on_cancel(inst)`（任意） | 受け付けたあとの取り消し | サンプルでは使わない（`nullptr`） |

- **「すぐ応え、動きは続く」**のは、`on_step` ですぐ `OK` を返し、動き（指令）は `update` が毎刻み出す、という分け方で作ります。
- 表は、入口の関数 `hako_robot_runtime_plugin_v1()` が返します。表の頭には、種類 `HAKO_PLUGIN_KIND_SERVICE`、ABI の版 `HAKO_SERVICE_PLUGIN_ABI_VERSION`、表の大きさを入れます（サンプルの最後を参照）。
- 関数は、シミュレーションのスレッドから順に呼ばれます。待たずにすぐ戻ってください。

## 4. プラグインを作る

```bash
cmake -S "$WORK/plugin" -B "$WORK/build-plugin" -DCMAKE_BUILD_TYPE=Release \
  -DHAKO_RUNTIME_INCLUDE_DIR="$RUNTIME/include" \
  -DHAKO_SERVOJ_PDU_TYPES="$PDU_TYPES" \
  -DHAKO_PDU_REGISTRY_TYPES="$REGISTRY/pdu/types"
cmake --build "$WORK/build-plugin" --parallel 4
PLUGIN=$WORK/build-plugin/libmy_servoj.so
ls -l "$PLUGIN"
```

- サンプルの CMake は、Robot Runtime の**公開ヘッダーだけ**を使います。Robot Runtime をビルドし直す必要はありません。
- macOS でも、ファイル名は `.so` になります（CMake の MODULE ライブラリのため）。

## 5. マニフェストと service.json を書く

**サンプルの `run.py`（6.2）を使う場合、ここは手で書かなくてかまいません。** `run.py` が、実行のたびに、この 2 つを自動で書きます。この節は、何が書かれるのかを知るため、また自分の腕で手で書くときのために読んでください。

### 5.1 マニフェスト

腕のマニフェスト（SO-101 なら robot-arm の `recipes/so101/asset-manifest.json`）を写し、`services` を足します。`library` には、手順 4 のプラグインを書きます（相対パスなら、マニフェストのある場所から解かれます）。

```json
"services": [
  { "name": "ArmService/ServoJ/SO101", "type": "plugin", "pdu_robot": "SO101",
    "plugin": { "library": "/abs/path/to/libmy_servoj.so" },
    "lockstep": false, "priority": 16 }
]
```

- `priority` は、Arbiter の優先の順です。SO-101 の例では、手動 20 ＞ **このプラグイン 16** ＞ 軌道 10 です。
- ServoJ のようなサービスは、ロックステップにしません（`lockstep: false`）。

### 5.2 service.json（core-pro と、Python のクライアントが読む）

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

- **大きさの向き**：`server` には**要求**の大きさ（328 ＋ 8 ＝ 336）、`client` には**応答**の大きさ（272 ＋ 8 ＝ 280）を入れます。逆にしても読み込めてしまい、最初の呼び出しで `status=3, result_code=1` で失敗します。
- `maxClients` は 1 です（core-pro のサーバは、1 度に 1 クライアントの番しか読まない。hakoniwalab/hakoniwa-core-pro#115）。
- `clients[].name` は Python の `client_name` と、`nodeId` は Python の `node_id` と同じにします。
- `coreConfig` は core-pro の設定です（例：`{"shm_type":"mmap","core_mmap_path":"/abs/path/to/mmap","asset_timeout_usec":600000000}`）。環境変数 `HAKO_CONFIG_PATH` にも、同じファイルを指定します。

サンプルの `run.py` は、5.1 と 5.2 を実行のたびに自動で書きます。手で書くときの見本として、`run.py` の `prepare()` も見てください。

## 6. 動かして呼ぶ

### 6.1 腕（robot-arm）と、Python 用の pdu-rpc をビルドする（最初の 1 回）

```bash
SHLIB=so; SEARCH_PREFIX="$PREFIX"; RPC_LINK=()
if [[ $(uname -s) == Darwin ]]; then
  SHLIB=dylib
  SEARCH_PREFIX="$PREFIX;$(brew --prefix glfw)"
  # macOS：core-pro の関数をすべて libshakoc から取る（libassets と 2 つの写しになるのを防ぐ）
  RPC_LINK=("-DCMAKE_SHARED_LINKER_FLAGS=$PREFIX/lib/libshakoc.dylib")
fi

cmake -S "$RPC" -B "$WORK/build-rpc" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DHAKONIWA_PDU_REGISTRY_DIR="$REGISTRY" \
  -DHAKO_PDU_RPC_ENABLE_SHM=ON -DHAKO_PDU_RPC_BUILD_TESTS=OFF "${RPC_LINK[@]}"
cmake --build "$WORK/build-rpc" --target hakoniwa_pdu_rpc_shared --parallel 4

cmake -S "$ROBOT_ARM" -B "$WORK/build-arm" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH="$SEARCH_PREFIX" \
  -DHAKONIWA_ROBOT_RUNTIME_ROOT="$RUNTIME" -DHAKONIWA_MUJOCO_ROBOTS_ROOT="$MUJOCO_ROBOTS" \
  -DHAKONIWA_FOUNDATION_PREFIX="$PREFIX" -DHAKO_ROBOT_ARM_CONTROLLER_PLUGIN=ON \
  -DHAKO_ARM_ENABLE_VIEWER=ON -DHAKO_ROBOT_ARM_ENABLE_SENSOR=ON \
  -DHAKO_ROBOT_ARM_SENSOR_IMPLEMENTATIONS=reference \
  -DHAKO_ROBOT_ARM_SENSOR_ADAPTER_ROOT="$SENSOR_ADAPTER"
cmake --build "$WORK/build-arm" --target robot-arm-hakoniwa-asset --parallel 4
```

- プラグインを読み込む機能は、`HAKO_ROBOT_ARM_CONTROLLER_PLUGIN=ON` で入ります。
- センサありの構成は、画面を使わないときも `HAKO_ARM_ENABLE_VIEWER=ON` でビルドします（GLFW のため）。起動のときに `--no-viewer` を付ければ、画面なしで動きます。
- robot-arm のアプリは、`--rpc-service-config <service.json>` でサービスの定義を受け取ります（hakoniwalab/hakoniwa-robot-arm#19 以降）。

### 6.2 動かす

サンプルの `run.py` が、5.1・5.2 を書き、腕を起動し、クライアントの登録を待って `hako-cmd start` し、`client.py` で 5 回呼んで、止めるまでを、1 回で行います。

```bash
# 変数のまとめ（同じシェルで続けているなら、そのままでよい）
PDU_PY=$WORK/local-python
PLUGIN=$WORK/build-plugin/libmy_servoj.so
SHLIB=so; [[ $(uname -s) == Darwin ]] && SHLIB=dylib

export ROBOT_ARM PREFIX SO101_MODEL PLUGIN
export ARM_ASSET="$WORK/build-arm/bin/robot-arm-hakoniwa-asset"
export HAKO_PDU_RPC_LIBRARY="$WORK/build-rpc/src/libhakoniwa_pdu_rpc.$SHLIB"
export PYTHONPATH="$RPC/python:$PDU_PY:$REGISTRY${PYTHONPATH:+:$PYTHONPATH}"
if [[ $SHLIB == dylib ]]; then
  # build-rpc を先に：Foundation にも SHM のない libhakoniwa_pdu_rpc.dylib があり、
  # macOS は名前だけで探すため、先にあるほうが使われてしまう
  export DYLD_LIBRARY_PATH="$WORK/build-rpc/src:$PREFIX/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
DEMO=$RUNTIME/examples/service_plugins/servoj_like

"$PYTHON" "$DEMO/run.py" "$WORK/run-$(date +%Y%m%d-%H%M%S)" --no-viewer   # 画面なし（まずこちらで）
# "$PYTHON" "$DEMO/run.py" "$WORK/run-$(date +%Y%m%d-%H%M%S)"             # ビューアあり（ウィンドウが開く）
```

1 回およそ 10 秒です。core-pro の INFO の行に混じって、最後のほうに次の 5 行が出ます。`Warning: Log message truncated to fit 256 byte limit.` は何度も出ますが、core-pro のログが長いだけなので、気にしなくてかまいません。

期待される出力：

```text
res=0 shoulder_pan reached=+0.500000 rad (goal=+0.50)
res=0 shoulder_pan reached=-0.500000 rad (goal=-0.50)
res=0 shoulder_pan reached=+0.000000 rad (goal=+0.00)
res=0 elbow_flex reached=+0.800326 rad (goal=+0.80)
res=0 elbow_flex reached=+0.000453 rad (goal=+0.00)
```

- 実行の場所は、毎回新しくしてください（core-pro の mmap を使い回すと、前のロックが残って起動が止まります）。
- 自分のクライアントを書くときは、`client.py` が見本です。要は、`RpcClient(..., endpoint_config_path=None)` → `make_typed_client(..., package="servoj_pdu.my_robot_srvs_msgs")` → `client.start()` → `hako-cmd start` を待つ → `create_request()`（`.joint`、`.t`）→ `call()`（`.res`）です。`create_request()` は要求の中身を、`call()` は応答の中身を返します。

## 付録 A. 自分の腕に足すとき

- `create` で、`args->joint_names` から自分の腕の関節を探してください（サンプルは SO-101 の 6 つの名前と範囲を書いています）。
- 関節の数が違う場合は、手順 1 の `float64[6]` の長さも合わせます。
- 範囲（`minimum`・`maximum`）は、マニフェストのアクチュエータの範囲と合わせてください。

## 付録 B. Foundation を手で作る場合の落とし穴

Business Pack の Foundation を使えば、次のことは要りません。

- core-pro：`HAKO_PYTHON_EXECUTABLE`・`HAKO_PYTHON_INSTALL_DIR` を自分の venv に向ける（でないと、システムの dist-packages に入れようとして失敗する）。
- pdu-endpoint：core-pro 込み（`HAKO_PDU_ENDPOINT_ENABLE_HAKONIWA_CORE=ON`）で作る。システムに古い core-pro があると、それを拾ってリンクで失敗するので、`-DHAKONIWA_assets_LIB=$PREFIX/lib/libassets.so -DHAKONIWA_shakoc_LIB=$PREFIX/lib/libshakoc.so` を明示する。
- hakoniwa-mujoco-robots：`git submodule update --init thirdparty/nolman thirdparty/hakoniwa-mbody-registry`。
