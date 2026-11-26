# 非同期 RPC のサービスを 1 つ足す（手順書・ServoJ を例に）

この文書は、Robot Runtime に新しい RPC のサービスを足す人のための手順書です。例として、Dobot の `ServoJ`（関節の目標を受け取り、すぐ応える指令）を、SO-101 の腕に足します。

> **状態（2026-11-26）**：第 2 稿。第 1 稿を、Codex が手順書だけを見て追い、本物の core-pro（`e0fa56f`）で Python から `ArmService/ServoJ/SO101` を呼んで `res = 0` を受け取り、腕が目標まで動くところまで確かめました。第 2 稿は、そのとき補ったことをすべて入れたものです。第 2 稿そのものの追試は、これからです。
>
> 今の Robot Runtime では、サービスは **runtime の中に C++ で足します**（runtime を直す必要があります）。runtime を直さずにプラグインとして足せる形（C の境目、設計書の段 E）は、次の回に作ります。そのときは、手順 3〜5 が変わります。

仕組みの説明は [design-async-rpc.md](design-async-rpc.md)、マニフェストの書き方は [configuration.md](configuration.md) の 2.4 節にあります。ここでは「何を、どの順で」に絞ります。

## 0. 全体の流れと前提

```text
1. 型を決める        ServoJ.srv を書く
2. PDU を作る        手元で生成する（hakoniwa-pdu-registry の本体には足さない）
3. サービスを書く    IAsyncService（応答）と IController（動き）
4. 登録する          サービスの種類、マニフェスト、工場、優先の順
5. ビルドに足す      runtime のソースの一覧と、使う側（robot-arm）
6. サービスの定義    core-pro の service.json（大きさの向きに注意）
7. 動かして呼ぶ      Robot Runtime → Python のクライアント → hako-cmd start
8. テスト
```

**前提**
- Linux（Ubuntu 24.04、ROS 2 Jazzy のコンテナで確かめました）。Docker が使えること（手順 2.3 で使う）。
- hakoniwa-core-pro は `e0fa56f` 以降、`hakoniwa-pdu` は 1.7.1 以降。
- hakoniwa-robot-runtime、hakoniwa-robot-arm、hakoniwa-pdu-registry、hakoniwa-pdu-rpc を、並べて clone しておく。
- この手順書では、Foundation（core-pro などを入れた場所）を `$PREFIX`、作業の場所を `$WORK` と書きます。

**できあがり**：Python から `ArmService/ServoJ/SO101` を呼ぶと、すぐ `res = 0` が返り、そのあと SO-101 の関節が目標へ動きます。

## 1. 型を決める（ServoJ.srv）

要求と応答を、ROS の `.srv` の形で書きます。PDU は大きさが決まっている（固定長）ほうが扱いやすいので、配列は長さを決めます。

```text
# $WORK/my_robot_srvs/srv/ServoJ.srv
# joint: 関節の目標 [rad]
float64[6] joint
# t: 目標に着くまでの時間 [s]
float64 t
---
# res: 0 = 受け付けた
int32 res
```

- **コメントは、行の頭に `#` を置いた別の行に書いてください。** 欄の行の後ろに書くと（`float64 t  # …`）、手順 2.1 の変換の道具が `ValueError: Invalid field line` で止まります。
- 名前（パッケージ名 `my_robot_srvs`、型名 `ServoJ`）は、自分のものにします。Hakoniwa の既存の名前（`hako_srv_msgs` など）は使いません。
- 実機の Dobot の srv（`dobot_msgs_v4/srv/ServoJ`）と同じ欄にしたい場合は、そちらに合わせます（ROS 2 から同じ型で呼ぶため）。

## 2. PDU を手元で作る（hakoniwa-pdu-registry の本体には足さない）

自分の型だけを、registry の中の追跡されないフォルダ（`.cache/`）に生成します。registry の `pdu/` と `config/` は変わりません。道具の詳しい説明は、registry の `docs/external-message-fetch.md` にあります。

### 2.1 .srv を Hakoniwa の .msg に変える

```bash
cd $WORK/hakoniwa-pdu-registry
python3 -m generators.generate_hako_service_msgs.main \
  $WORK/my_robot_srvs/srv/ServoJ.srv --out $WORK/my-pdu-src
find $WORK/my-pdu-src -type f | sort
```

`$WORK/my-pdu-src/srv_msgs/msg/` に、次の 4 つができます（フォルダの名前は、元の .srv の置き場所によらず `srv_msgs` になります）。

```text
ServoJRequest.msg  ServoJRequestPacket.msg  ServoJResponse.msg  ServoJResponsePacket.msg
```

### 2.2 ROS のパッケージの形にして、手元の git に入れる

生成の道具は、`package.xml` のある ROS のパッケージから `msg/*.msg` を拾います。フォルダの名前を自分のパッケージ名に変えて、`package.xml` を置きます。

```bash
cd $WORK/my-pdu-src
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
git commit -q -m "ServoJ PDU types"     # git の名前とメールが未設定なら、先に git config で設定する
git rev-parse HEAD                      # この番号を 2.3 で使う
```

- パケットの `.msg` の中は、同じパッケージの型を名前だけで指しているので、書き換えは要りません。

### 2.3 生成する

registry の中に、「どこから取るか」の設定を 1 つ置きます。registry の追跡されるファイルは変わりません（この yaml も追跡されません）。

```yaml
# $WORK/hakoniwa-pdu-registry/sources/my_robot_srvs_msgs.yaml
schema_version: 1
source:
  type: git
  repository: file:///<$WORK の絶対パス>/my-pdu-src
  ref: <2.2 の番号>
packages:
  - my_robot_srvs_msgs
```

```bash
cd $WORK/hakoniwa-pdu-registry
python3 -m pip install -r tools/requirements-fetch.txt   # PyYAML（入っていなければ）
python3 tools/generate_from_sources.py \
  --sources sources/my_robot_srvs_msgs.yaml \
  --output-dir .cache/my-pdu \
  my_robot_srvs_msgs/ServoJRequestPacket \
  my_robot_srvs_msgs/ServoJResponsePacket
```

- このコマンドは、ホスト（Docker の外）で動かします。`file://` の git は、ホストで取りに行きます。生成そのものは、道具が中で Docker を使って行います。
- Docker が見えるのは registry の中だけなので、出力先は registry の中の `.cache/...` にします。
- 依存する型（`hako_srv_msgs/ServiceRequestHeader` など）は、自動で一緒に生成されます。

### 2.4 出てきたものと、その使いどころ

```bash
cat .cache/my-pdu/pdu_size/my_robot_srvs_msgs/ServoJRequestPacket.txt    # 328
cat .cache/my-pdu/pdu_size/my_robot_srvs_msgs/ServoJResponsePacket.txt   # 272
git status --short -- pdu/ config/                                        # 何も出なければ、registry は変わっていない
```

| 出てきたもの | 使うところ |
|---|---|
| `.cache/my-pdu/types/my_robot_srvs_msgs/`（C++ の型と変換） | 手順 3・5。ヘッダーの探し先に `.cache/my-pdu/types` を足す |
| `.cache/my-pdu/python/my_robot_srvs_msgs/`、`.cache/my-pdu/python/hako_srv_msgs/`（Python の変換） | 手順 7。下の形に置き直す |
| `.cache/my-pdu/pdu_size/my_robot_srvs_msgs/*.txt`（大きさ） | 手順 6 |

**Python の置き方**：生成された Python の変換は、1 つ上のフォルダの `pdu_utils` と `binary_io` を相対で読みます。そのため、生成先を `PYTHONPATH` に入れるだけでは import できません。親のパッケージを 1 つ作り、次の形に写します。

```bash
mkdir -p $WORK/local-python/servoj_pdu
cd $WORK/local-python/servoj_pdu
touch __init__.py
cp $WORK/hakoniwa-pdu-registry/pdu/python/binary_io.py .
cp $WORK/hakoniwa-pdu-registry/pdu/python/pdu_utils.py .
cp -r $WORK/hakoniwa-pdu-registry/.cache/my-pdu/python/my_robot_srvs_msgs .
cp -r $WORK/hakoniwa-pdu-registry/.cache/my-pdu/python/hako_srv_msgs .
touch my_robot_srvs_msgs/__init__.py hako_srv_msgs/__init__.py

# 確かめ
PYTHONPATH=$WORK/local-python python3 -c \
  'from servoj_pdu.my_robot_srvs_msgs import pdu_conv_ServoJRequestPacket, pdu_conv_ServoJResponsePacket; print("ok")'
```

```text
local-python/
└── servoj_pdu/                 （親のパッケージ。名前は自由。手順 7 の package= と合わせる）
    ├── __init__.py             （空）
    ├── binary_io.py            （registry の pdu/python/ から）
    ├── pdu_utils.py            （registry の pdu/python/ から）
    ├── hako_srv_msgs/          （生成されたもの。__init__.py は空）
    └── my_robot_srvs_msgs/     （生成されたもの。__init__.py は空）
```

## 3. サービスを書く

ServoJ は「受け付けたらすぐ応え、動きは続く」指令です。ここでは、**応答**と**動き**を分けて作ります（設計書の案 A）。

- `ServoJService`（`IAsyncService`）：要求を読んで確かめ、目標を「共有の目標」に置き、**0 ステップで `Ok`** を返す。応答の中身（`res = 0`）を書く。
- `ServoJController`（`IController`）：共有の目標が来たら、今の関節の角度から目標までの 2 点の軌道を作り、既存の `JointTrajectoryController` に渡して動かし続ける。

2 つは、`std::shared_ptr<std::optional<ServoJGoal>>`（共有の目標）でつなぎます。

足すファイル（runtime の中）：

```text
src/adapters/endpoint/servoj_packet.hpp     # パケットのメタデータを確かめる
src/adapters/endpoint/servoj_service.hpp
src/adapters/endpoint/servoj_service.cpp
tests/test_servoj_service.cpp               # 手順 8
```

ファイルの全文は、[付録 A](#付録-a-サービスのコード) にあります。要点：

| フック | ServoJ でやること |
|---|---|
| `on_request(要求)` | メタデータを確かめ、パケットの `body` を `Hako_ServoJRequest` として読む。`t` は有限で 1 µs〜60 s、関節は有限でアクチュエータの範囲（`limits`）の中か確かめる。おかしければ `false`（INVALID で応える）。よければ共有の目標に置いて `true` |
| `on_step(…)` | 何も指令を出さず、すぐ `Ok` を返す（動きはコントローラが受け持つ） |
| `on_response(状態, 時刻, 入れ物)` | `res = 0` を書く。**頭（サービスの header）は書き換えない**。メタデータの `heap_off`・`total_size` が応答の全体より小さければ、広げて書き戻す。中身の欄は毎回すべて書く |
| `on_cancel()` | 書かない（すぐ終わるので） |
| `reset()` | 共有の目標を捨てる |

`ServoJController` の要点：
- `update()` で共有の目標を 1 回だけ取り出し（`std::exchange`）、来ていれば 2 点の軌道を `JointTrajectoryController` に渡す。来ていなければ、前の軌道を続ける（終点を保つ）。
- このコントローラには入力源がないので、`source_id()` は空を返し、出す指令の `source_id` も空にします（空でないと、Runtime が指令を受け付けず止まります）。

## 4. 登録する

| どこ | 何を足すか |
|---|---|
| `src/runtime/runtime_definition.hpp` | `enum class RuntimeServiceType` に `ServoJ` |
| `src/runtime/runtime_factory_manifest.cpp` | `"servo_j"` を `RuntimeServiceType::ServoJ` に読み分ける。ServoJ で `lockstep: true` は誤りとして断る |
| `schemas/runtime/asset-manifest.schema.json` | `$defs/service/properties/type/enum` に `"servo_j"` |
| `src/factory/manifest_factory.cpp` | (1) サービスの名前を確かめる（`ArmService/ServoJ/<robot>`）。(2) 同じ `pdu_robot` の位置のアクチュエータを、マニフェストの順に 6 つ集め、その `limits` と一緒に `ServoJService` と `ServoJController` を作る |
| `src/runtime/runtime_factory.{hpp,cpp}` | `build_runtime` に `ServoJController` を渡す引数を足し、優先の順を付けて登録する |

差分の全文は、[付録 B](#付録-b-登録の差分) にあります。

**優先の順**（Arbiter）：SO-101 の例では、手動 20 ＞ **ServoJ 16** ＞ SO-101 のプラグイン 15 ＞ 軌道 10 にしました。手で動かすと ServoJ の軌道は捨てられ、ServoJ はほかの自動の指令より優先します。腕に合わせて決めてください。

**関節の割り当て**：パケットの `joint[0..5]` は、マニフェストの、その `pdu_robot` の位置のアクチュエータの**記載の順**に割り当てます。SO-101 では `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` です。6 つちょうどでなければ、起動のときに止めます。

**マニフェスト**：

```json
"services": [
  { "name": "ArmService/ServoJ/SO101", "type": "servo_j", "pdu_robot": "SO101", "lockstep": false }
]
```

- ServoJ はロックステップにしません（実機と同じく、要求は好きなときに来て、すぐ応える）。
- この例では、1 つのアセットに ServoJ の腕は 1 台までにしています。

## 5. ビルドに足す

### 5.1 runtime のソースの一覧

`servoj_service.cpp` を、runtime のソースの一覧 `cmake/HakoniwaRobotRuntimeSources.cmake` に足します（新しいまとまりにするか、endpoint のまとまりに入れる）。robot-arm と Urban は、この一覧を読んでビルドします。足し忘れは `cmake -P tests/check_runtime_sources.cmake` が教えてくれます。

生成した型（手順 2）を使うので、ServoJ は**生成した型があるときだけ**ビルドに入れ、`HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ=1` を定義します。

### 5.2 robot-arm

robot-arm の `CMakeLists.txt` に、生成した型の場所を受け取る設定を足します。

```cmake
set(HAKO_SERVOJ_PDU_TYPES "" CACHE PATH "Generated local ServoJ C/C++ include root")
if(HAKO_SERVOJ_PDU_TYPES)
    target_sources(hako-robot-arm-endpoint-adapter PRIVATE
        ${HAKO_ROBOT_RUNTIME_SRC_DIR}/adapters/endpoint/servoj_service.cpp)
    target_include_directories(hako-robot-arm-endpoint-adapter PRIVATE "${HAKO_SERVOJ_PDU_TYPES}")
    target_compile_definitions(hako-robot-arm-endpoint-adapter PUBLIC
        HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ=1)
endif()
```

> **注意（2026-11-26 の時点）**：今の robot-arm のアプリには、サービスを使うための次の 2 つがまだありません。ServoJ に限らず、腕にサービスを足すときには必ず要ります。
> - アプリが、サービスの定義ファイルを受け取る口（`--rpc-service-config <file>`）と、それを `ManifestFactoryConfig::async_rpc_service_definition_path` に渡すこと（`apps/common/robot_arm_app.cpp`、`manifest_runtime_app.{hpp,cpp}`）
> - `hako-robot-arm-hakoniwa-runner` を `hakoniwa-core::shakoc` にリンクすること
>
> robot-arm に入るまでは、[付録 C](#付録-c-robot-arm-の差分) の差分を当ててください。

ビルド：

```bash
cmake -S $WORK/hakoniwa-robot-arm -B $WORK/build-arm -DCMAKE_BUILD_TYPE=Release \
  -DHAKONIWA_FOUNDATION_PREFIX=$PREFIX -DHAKO_ARM_ENABLE_VIEWER=OFF -DHAKO_ARM_BUILD_EXCAVATOR=OFF \
  -DBUILD_TESTING=OFF \
  -DHAKO_SERVOJ_PDU_TYPES=$WORK/hakoniwa-pdu-registry/.cache/my-pdu/types
cmake --build $WORK/build-arm --target robot-arm-hakoniwa-asset -j 4
```

- Foundation の pdu-endpoint は、core-pro 込み（`HAKO_PDU_ENDPOINT_ENABLE_HAKONIWA_CORE=ON`）でビルドされている必要があります。

### 5.3 SHM を有効にした pdu-rpc（Python から呼ぶため）

```bash
cmake -S $WORK/hakoniwa-pdu-rpc -B $WORK/build-rpc -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$PREFIX -DHAKONIWA_PDU_REGISTRY_DIR=$WORK/hakoniwa-pdu-registry \
  -DHAKO_PDU_RPC_ENABLE_SHM=ON -DHAKO_PDU_RPC_BUILD_TESTS=OFF
cmake --build $WORK/build-rpc --target hakoniwa_pdu_rpc_shared -j 3
```

## 6. core-pro のサービスの定義（service.json）

core-pro と Robot Runtime と Python のクライアントの 3 つが、**同じ 1 つのファイル**を読みます（hakoniwa-pdu-rpc の `docs/design/shm/README.md` 第 5 節）。

```json
{
  "pduMetaDataSize": 24,
  "services": [
    {
      "name": "ArmService/ServoJ/SO101",
      "type": "my_robot_srvs_msgs/ServoJ",
      "maxClients": 1,
      "pduSize": {
        "server": { "baseSize": 336, "heapSize": 0 },
        "client": { "baseSize": 280, "heapSize": 0 }
      },
      "clients": [
        {
          "name": "ServoJClient",
          "client_endpoint": {
            "nodeId": "servoj-python",
            "protocol": "shm",
            "assetName": "ServoJClientAsset",
            "deltaTimeUsec": 2000,
            "coreConfig": "/path/to/core.json"
          }
        }
      ]
    }
  ]
}
```

- **大きさの向きに気をつけてください。** `server` には**要求**（`ServoJRequestPacket`）の大きさ、`client` には**応答**（`ServoJResponsePacket`）の大きさを入れます。ここでは、要求 328 ＋ 8 ＝ 336、応答 272 ＋ 8 ＝ 280 です。`+ 8` の余裕は、車のサービスの定義を作る道具（`tools/gen_rpc_service_config.py`）と同じです。
- 向きを逆にしても `service.json` は読み込めてしまい、最初に呼んだときに `status=3, result_code=1` で失敗します（要求がメタデータ込みで 352 バイトあり、逆の向きだと要求の枠が 304 バイトしかないため）。
- `maxClients` は 1 にします（core-pro のサーバは、1 度に 1 クライアントの番しか読まない。hakoniwalab/hakoniwa-core-pro#115）。
- `clients[].name` は Python の `client_name` と、`client_endpoint.nodeId` は Python の `node_id` と、同じにします。
- `coreConfig` は core-pro の設定のファイルです。例：`{"shm_type":"mmap","core_mmap_path":"/path/to/mmap","asset_timeout_usec":600000000}`。環境変数 `HAKO_CONFIG_PATH` にも、同じファイルを指定します。

## 7. 動かして呼ぶ

順番が大事です。[付録 D](#付録-d-起動の台本-runsh) に、通しで動く台本があります。

0. **準備**
   - **mmap の場所は、毎回新しくします。** 古いものを使うと、core が前の実行のロックの持ち主を見つけ、起動が止まります。
   - 環境：`HAKO_CONFIG_PATH`（core-pro の設定）、`LD_LIBRARY_PATH`（`$PREFIX/lib` と MuJoCo のライブラリ）、`HAKO_PDU_RPC_LIBRARY`（`$WORK/build-rpc/src/libhakoniwa_pdu_rpc.so`）、`PYTHONPATH`（`$WORK/hakoniwa-pdu-rpc/python` と `$WORK/local-python`）。
1. **Robot Runtime（サーバ）を起動する**
   ```bash
   $WORK/build-arm/bin/robot-arm-hakoniwa-asset --manifest asset-manifest.json \
     --rpc-service-config service.json --no-viewer --realtime-sync-cycle-msec 2 &
   ```
   ログに `[INFO] Starting Hakoniwa Runner:` が出るまで待ちます。
2. **Python のクライアントを起動する**（下のコード）。`client.start()` まで進めて待たせます。
3. **登録を確かめる**：`$PREFIX/bin/hako-cmd jmeta` に `ServoJClientAsset` が出るまで待ちます。
4. **開始する**：`$PREFIX/bin/hako-cmd start`。そのあと、クライアントに呼んでよいと知らせます。
5. **呼ぶ**：`res = 0` がすぐ返り、そのあと関節が目標へ動きます。
6. **止める**：`$PREFIX/bin/hako-cmd stop`。

Python のクライアント（要のところ）：

```python
import os
from hakoniwa_pdu_rpc import RpcClient, make_typed_client

with RpcClient(
    library_path=os.environ["HAKO_PDU_RPC_LIBRARY"],
    node_id="servoj-python",              # service.json の client_endpoint.nodeId
    client_name="ServoJClient",           # service.json の clients[].name
    service_config_path="service.json",
    endpoint_config_path=None,            # shm だけなら要らない
) as client:
    servoj = make_typed_client(
        client, service_name="ArmService/ServoJ/SO101",
        service_type="my_robot_srvs_msgs/ServoJ",
        package="servoj_pdu.my_robot_srvs_msgs",   # 手順 2.4 の親のパッケージ
    )
    client.start()
    # ここで、hako-cmd start を待つ（手順 7 の 3・4）
    request = servoj.create_request()
    request.joint = [0.30, 0.0, 0.0, 0.0, 0.0, 0.0]
    request.t = 0.4
    response = servoj.call(request, timeout_usec=2_000_000)
    print("res =", response.res)          # 0
```

- 型つきの API では、`create_request()` が要求の中身（`.joint`、`.t`）を返し、`call()` が応答の中身（`.res`）を返します。
- 範囲の外の要求は、`TypedRpcServiceError`（`result_code == 3`、INVALID）になります。そのあとの正しい要求は、ふつうに通り、目標は置き換わります。
- ROS 2 から呼ぶ場合は、hakoniwa-pdu-ros の service-server に、同じ `service.json` を `service.rpc_service_config` として渡します。ROS 2 の `ServoJ` の srv の型が、手順 1 と同じ欄である必要があります（この手順書では、まだ確かめていません）。

## 8. テスト

| 何を | どう |
|---|---|
| サービス単体（要求の確かめ、0 ステップの `Ok`、応答の頭を保つ、応答のあとの軌道、手動で奪われたとき、reset） | `tests/test_servoj_service.cpp`（[付録 A](#付録-a-サービスのコード)）。生成した型と robot-arm のライブラリをつないでビルドする（下） |
| 受け口と合わせて | 既存の `tests/test_async_service_zero_step.cpp` |
| 本物の core-pro で | 手順 7 の台本を、範囲の外の要求と目標の置き換えまで入れて流す |

```bash
c++ -std=c++20 -I$WORK/hakoniwa-robot-runtime/src -I$WORK/hakoniwa-robot-runtime/include \
  -I$WORK/hakoniwa-pdu-registry/.cache/my-pdu/types -I$PREFIX/include \
  $WORK/hakoniwa-robot-runtime/tests/test_servoj_service.cpp \
  $WORK/build-arm/libhako-robot-arm-endpoint-adapter.a $WORK/build-arm/libhako-robot-arm-runtime.a \
  -o test_servoj_service && ./test_servoj_service
```

確かめた結果（2026-11-26）：単体は通過。本物の core-pro で `res = 0`、`shoulder_pan` は応答のとき 0.000130 rad、応答のあと 0.300311 rad（目標 0.30）まで動いた。範囲の外は INVALID（3）、次の要求で目標が置き換わった。

## 変える場所の一覧（まとめ）

| リポジトリ | ファイル |
|---|---|
| 自分のもの | `ServoJ.srv`、`my-pdu-src/`（手順 1・2）、`local-python/`（手順 2.4）、`service.json`、マニフェスト、クライアント |
| hakoniwa-pdu-registry | 追跡されるファイルは変わらない（`sources/…yaml` と `.cache/` は追跡されない） |
| hakoniwa-robot-runtime | `servoj_service.{hpp,cpp}`・`servoj_packet.hpp`・`tests/test_servoj_service.cpp`（新規）、`runtime_definition.hpp`、`runtime_factory_manifest.cpp`、`runtime_factory.{hpp,cpp}`、`manifest_factory.cpp`、マニフェストのスキーマ、ソースの一覧 |
| hakoniwa-robot-arm | `CMakeLists.txt`（生成した型の場所）。robot-arm に入るまでは、付録 C も |
| hakoniwa-pdu-rpc | 変えない（SHM を ON にしてビルドするだけ） |

---

## 付録 A. サービスのコード

### src/adapters/endpoint/servoj_packet.hpp

```cpp
#pragma once

#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJRequestPacket.hpp"
#include "my_robot_srvs_msgs/pdu_cpptype_conv_ServoJResponsePacket.hpp"

#include <cstddef>
#include <cstring>
#include <optional>
#include <span>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

inline std::optional<HakoPduMetaDataType> servoj_packet_metadata(
    std::span<const std::byte> packet, std::size_t base_size)
{
    if (packet.size() < sizeof(HakoPduMetaDataType)) {
        return std::nullopt;
    }
    HakoPduMetaDataType metadata {};
    std::memcpy(&metadata, packet.data(), sizeof(metadata));
    const bool valid = metadata.magicno == HAKO_PDU_META_DATA_MAGICNO
        && metadata.version == HAKO_PDU_META_DATA_VERSION
        && metadata.base_off == HAKO_PDU_META_DATA_SIZE()
        && metadata.heap_off >= metadata.base_off + base_size
        && metadata.heap_off <= metadata.total_size && metadata.total_size <= packet.size();
    return valid ? std::optional {metadata} : std::nullopt;
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
```

### src/adapters/endpoint/servoj_service.hpp

```cpp
#pragma once

#include "runtime/controller/joint_trajectory_controller.hpp"
#include "runtime/service/async_service.hpp"

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

struct ServoJGoal {
    std::array<double, 6> joint;
    std::uint64_t duration_usec;
};

using ServoJLimits = std::array<runtime::ActuatorLimits, 6>;

class ServoJService final : public runtime::IAsyncService {
public:
    ServoJService(std::shared_ptr<std::optional<ServoJGoal>> goal, ServoJLimits limits);

    bool on_request(std::span<const std::byte> request) override;
    runtime::AsyncServiceStepResult on_step(
        const runtime::RobotState&, runtime::AsyncServiceState,
        const runtime::RuntimeStepContext&) override;
    bool on_response(const runtime::RobotState&, const runtime::RuntimeStepContext&,
        std::span<std::byte> packet) override;
    void reset() override;

private:
    std::shared_ptr<std::optional<ServoJGoal>> goal_;
    ServoJLimits limits_;
};

/** Turns each accepted goal into a trajectory from the current simulated pose. */
class ServoJController final : public runtime::IController {
public:
    ServoJController(std::string id,
        std::shared_ptr<std::optional<ServoJGoal>> goal,
        std::vector<runtime::TrajectoryActuatorBinding> joints);

    std::string_view id() const noexcept override;
    std::string_view source_id() const noexcept override;
    bool accepts(const runtime::IControllerInput&) const noexcept override;
    runtime::ControllerOutput update(
        std::shared_ptr<const runtime::IControllerInput>,
        const runtime::RobotState& state, const runtime::RuntimeStepContext& context) override;
    void reset(const runtime::RobotState& state) override;

private:
    std::shared_ptr<std::optional<ServoJGoal>> goal_;
    std::vector<runtime::TrajectoryActuatorBinding> joints_;
    runtime::JointTrajectoryController trajectory_;
};

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
```

### src/adapters/endpoint/servoj_service.cpp

```cpp
#include "adapters/endpoint/servoj_service.hpp"

#include "adapters/endpoint/servoj_packet.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace hakoniwa::robot_runtime::adapters::hakoniwa {

ServoJService::ServoJService(
    std::shared_ptr<std::optional<ServoJGoal>> goal, ServoJLimits limits)
    : goal_(std::move(goal)), limits_(limits)
{
}

bool ServoJService::on_request(std::span<const std::byte> request)
{
    constexpr auto body_offset = offsetof(Hako_ServoJRequestPacket, body);
    constexpr auto body_size = offsetof(Hako_ServoJRequest, t) + sizeof(Hako_ServoJRequest::t);
    if (!servoj_packet_metadata(request, body_offset + body_size)) {
        return false;
    }
    Hako_ServoJRequest wire {};
    std::memcpy(&wire, request.data() + HAKO_PDU_META_DATA_SIZE() + body_offset, body_size);
    HakoCpp_ServoJRequest decoded {};
    cpp_pdu2cpp_ServoJRequest(nullptr, wire, decoded);

    // This simulation example accepts a duration between one microsecond and 60 seconds.
    if (!std::isfinite(decoded.t) || decoded.t < 1e-6 || decoded.t > 60.0) {
        return false;
    }
    for (std::size_t index = 0; index < decoded.joint.size(); ++index) {
        const auto position = decoded.joint[index];
        if (!std::isfinite(position) || position < limits_[index].minimum
            || position > limits_[index].maximum) {
            return false;
        }
    }
    *goal_ = ServoJGoal {decoded.joint,
        static_cast<std::uint64_t>(std::llround(decoded.t * 1'000'000.0))};
    return true;
}

runtime::AsyncServiceStepResult ServoJService::on_step(
    const runtime::RobotState&, runtime::AsyncServiceState,
    const runtime::RuntimeStepContext&)
{
    return {runtime::AsyncServiceStepOutcome::Ok, {}};
}

bool ServoJService::on_response(
    const runtime::RobotState&, const runtime::RuntimeStepContext&, std::span<std::byte> packet)
{
    constexpr auto full_size = HAKO_PDU_META_DATA_SIZE()
        + HAKO_ALIGN_SIZE(sizeof(Hako_ServoJResponsePacket), HAKO_ALIGNMENT_SIZE);
    auto metadata = servoj_packet_metadata(packet, sizeof(Hako_ServiceResponseHeader));
    if (!metadata || packet.size() < full_size) {
        return false;
    }
    HakoCpp_ServoJResponse response {};
    response.res = 0;
    Hako_ServoJResponse wire {};
    PduDynamicMemory memory;
    if (!cpp_cpp2pdu_ServoJResponse(response, wire, memory)) {
        return false;
    }
    if (metadata->heap_off < full_size) {
        metadata->heap_off = full_size;
        metadata->total_size = full_size;
        std::memcpy(packet.data(), &*metadata, sizeof(*metadata));
    }
    std::memcpy(packet.data() + HAKO_PDU_META_DATA_SIZE()
        + offsetof(Hako_ServoJResponsePacket, body), &wire, sizeof(wire));
    return true;
}

void ServoJService::reset()
{
    goal_->reset();
}

ServoJController::ServoJController(std::string id,
    std::shared_ptr<std::optional<ServoJGoal>> goal,
    std::vector<runtime::TrajectoryActuatorBinding> joints)
    : goal_(std::move(goal)), joints_(std::move(joints)),
      trajectory_(id, id + ":goal", joints_)
{
}

std::string_view ServoJController::id() const noexcept { return trajectory_.id(); }
std::string_view ServoJController::source_id() const noexcept { return {}; }
bool ServoJController::accepts(const runtime::IControllerInput&) const noexcept { return false; }

runtime::ControllerOutput ServoJController::update(
    std::shared_ptr<const runtime::IControllerInput>,
    const runtime::RobotState& state, const runtime::RuntimeStepContext& context)
{
    const auto goal = std::exchange(*goal_, std::nullopt);
    std::shared_ptr<const runtime::IControllerInput> input;
    if (goal) {
        runtime::JointTrajectory trajectory;
        runtime::JointTrajectoryPoint start;
        runtime::JointTrajectoryPoint finish;
        for (const auto& binding : joints_) {
            const auto actuator = std::find_if(state.actuators.begin(), state.actuators.end(),
                [&binding](const auto& value) { return value.actuator_id == binding.actuator_id; });
            if (actuator == state.actuators.end()) {
                trajectory_.reset(state);
                return {std::string(id()), {std::string(id()), runtime::ComponentState::Error,
                    "missing joint state for " + binding.actuator_id}, {}};
            }
            trajectory.joint_names.push_back(binding.joint_name);
            start.positions.push_back(actuator->position_rad);
        }
        finish.positions.assign(goal->joint.begin(), goal->joint.end());
        finish.time_from_start_usec = goal->duration_usec;
        trajectory.points = {std::move(start), std::move(finish)};
        runtime::ControllerInputMetadata metadata {std::string(trajectory_.source_id()), 0,
            context.simulation_time_usec, std::nullopt};
        input = std::make_shared<runtime::JointTrajectoryInput>(
            std::move(metadata), std::move(trajectory));
    }
    auto output = trajectory_.update(std::move(input), state, context);
    // The wrapped trajectory has an internal source ID. Runtime checks the
    // outward commands against this controller's empty source_id().
    for (auto& command : output.commands) {
        command.source_id.clear();
    }
    return output;
}

void ServoJController::reset(const runtime::RobotState& state)
{
    goal_->reset();
    trajectory_.reset(state);
}

} // namespace hakoniwa::robot_runtime::adapters::hakoniwa
```

### tests/test_servoj_service.cpp

```cpp
#undef NDEBUG

#include "adapters/endpoint/servoj_packet.hpp"
#include "adapters/endpoint/servoj_service.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>

using namespace hakoniwa::robot_runtime::adapters::hakoniwa;
using namespace hakoniwa::robot_runtime::runtime;

namespace {

using Packet = std::array<std::byte, 512>;

Packet request_packet(double position, double duration)
{
    Packet wire {};
    HakoCpp_ServoJRequestPacket request {};
    request.body.joint.fill(position);
    request.body.t = duration;
    assert(hako::pdu::msgs::my_robot_srvs_msgs::ServoJRequestPacket {}.cpp2pdu(
        request, reinterpret_cast<char*>(wire.data()), wire.size()) > 0);
    return wire;
}

void check_response(ServoJService& service)
{
    Packet wire {};
    HakoCpp_ServiceResponseHeader header {};
    header.request_id = 19;
    header.status = 3;
    header.client_name = "ServoJClient";
    header.service_name = "ArmService/ServoJ/SO101";
    assert(hako::pdu::msgs::hako_srv_msgs::ServiceResponseHeader {}.cpp2pdu(
        header, reinterpret_cast<char*>(wire.data()), wire.size()) > 0);
    constexpr auto body_offset =
        HAKO_PDU_META_DATA_SIZE() + offsetof(Hako_ServoJResponsePacket, body);
    std::fill_n(wire.begin() + body_offset, sizeof(Hako_ServoJResponse), std::byte {0xFF});
    const auto before = wire;
    assert(service.on_response({}, {}, wire));
    assert(std::equal(before.begin() + HAKO_PDU_META_DATA_SIZE(),
        before.begin() + body_offset, wire.begin() + HAKO_PDU_META_DATA_SIZE()));
    assert(std::all_of(wire.begin() + body_offset,
        wire.begin() + body_offset + sizeof(Hako_ServoJResponse),
        [](std::byte value) { return value == std::byte {0}; }));
}

} // namespace

int main()
{
    auto goal = std::make_shared<std::optional<ServoJGoal>>();
    ServoJLimits limits {};
    limits.fill({-1.0, 1.0});
    ServoJService service(goal, limits);

    const auto valid = request_packet(0.5, 0.4);
    assert(!service.on_request(std::span(valid).first(40)));
    assert(!service.on_request(request_packet(1.5, 0.4)));
    assert(!service.on_request(request_packet(0.5, 0.0)));
    assert(!service.on_request(request_packet(std::numeric_limits<double>::quiet_NaN(), 0.4)));
    assert(!goal->has_value());
    assert(service.on_request(valid));
    assert(goal->value().duration_usec == 400'000);
    assert(!service.on_request(request_packet(1.5, 0.4)));
    assert(goal->value().joint.front() == 0.5);

    const auto step = service.on_step({}, AsyncServiceState::Doing, {});
    assert(step.outcome == AsyncServiceStepOutcome::Ok && step.commands.empty());
    check_response(service);

    RobotState state;
    std::vector<TrajectoryActuatorBinding> joints;
    for (int index = 0; index < 6; ++index) {
        const auto name = "joint" + std::to_string(index);
        joints.push_back({name, name, ActuatorCommandType::Position});
        state.actuators.push_back({name, 0.0});
    }
    ServoJController controller("motion", goal, std::move(joints));
    RuntimeStepContext context {.simulation_time_usec = 0, .delta_time_usec = 2'000};
    assert(controller.update(nullptr, state, context).commands.at(0).value == 0.0);
    assert(!goal->has_value());
    context.previous_selected_control_id = "motion";
    context.simulation_time_usec = 200'000;
    auto motion = controller.update(nullptr, state, context);
    assert(motion.commands.size() == 6 && motion.commands.at(0).value == 0.25);
    assert(motion.commands.at(0).source_id == controller.source_id());
    context.simulation_time_usec = 400'000;
    assert(controller.update(nullptr, state, context).commands.at(0).value == 0.5);
    context.previous_selected_control_id = "manual";
    assert(controller.update(nullptr, state, context).commands.empty());
    assert(service.on_request(valid));
    service.reset();
    controller.reset(state);
    assert(!goal->has_value());
    assert(controller.update(nullptr, state, context).commands.empty());

    std::cout << "ServoJ unit passed\n";
}
```

## 付録 B. 登録の差分

```diff
--- a/schemas/runtime/asset-manifest.schema.json
+++ b/schemas/runtime/asset-manifest.schema.json
-        "type": { "enum": ["ackermann_step", "person_step"] },
+        "type": { "enum": ["ackermann_step", "person_step", "servo_j"] },

--- a/src/runtime/runtime_definition.hpp
+++ b/src/runtime/runtime_definition.hpp
-enum class RuntimeServiceType { AckermannStep, PersonStep };
+enum class RuntimeServiceType { AckermannStep, PersonStep, ServoJ };

--- a/src/runtime/runtime_factory_manifest.cpp
+++ b/src/runtime/runtime_factory_manifest.cpp
         } else if (type == "person_step") {
             config.type = RuntimeServiceType::PersonStep;
+        } else if (type == "servo_j") {
+            config.type = RuntimeServiceType::ServoJ;
         } else {
             return fail(error, "unknown service type " + type + " for " + config.name);
         }
 ...
+        if (config.type == RuntimeServiceType::ServoJ && config.lockstep) {
+            return fail(error, "ServoJ requires lockstep: false for " + config.name);
+        }

--- a/src/runtime/runtime_factory.hpp
+++ b/src/runtime/runtime_factory.hpp
     , const AckermannServiceFactory& ackermann_service_factory = {}
-    , std::vector<AsyncServiceComponents> person_services = {}
+    , std::vector<AsyncServiceComponents> independent_services = {}
     , std::shared_ptr<IStatePublisher> person_state_publisher = {}
+    , std::shared_ptr<IController> servoj_controller = {}
     );

--- a/src/runtime/runtime_factory.cpp
+++ b/src/runtime/runtime_factory.cpp
-    , std::vector<AsyncServiceComponents> person_services
+    , std::vector<AsyncServiceComponents> independent_services
     , std::shared_ptr<IStatePublisher> person_state_publisher
+    , std::shared_ptr<IController> servoj_controller
 ...
-    for (auto& service : person_services) {
+    // Poll independent RPC services before the remaining input sources.
+    for (auto& service : independent_services) {
         sources.push_back(std::move(service.source));
         controllers.push_back(std::move(service.controller));
     }
+    if (servoj_controller) {
+        // Higher than trajectory (10) and the SO101 plugin (15), below manual (20).
+        priorities.push_back({std::string(servoj_controller->id()), 16, {}});
+        controllers.push_back(std::move(servoj_controller));
+    }
 ...
-    for (auto& service : person_services) {
+    for (auto& service : independent_services) {
         publishers.push_back(std::move(service.publisher));
     }

--- a/src/factory/manifest_factory.cpp
+++ b/src/factory/manifest_factory.cpp
 #include "adapters/endpoint/person_step_service.hpp"
+#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ
+#include "adapters/endpoint/servoj_service.hpp"
+#endif
 ...
     std::unordered_set<std::string> rpc_requested;
     std::vector<std::string> people;
+#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ
+    bool has_servoj = false;
+#endif
     for (const auto& service : runtime_definition.services) {
         const auto& robot = service.pdu_robot;
         const bool ackermann = service.type == runtime::RuntimeServiceType::AckermannStep;
-        const auto expected_name = ackermann
+        const bool servoj = service.type == runtime::RuntimeServiceType::ServoJ;
+        const auto expected_name = servoj ? "ArmService/ServoJ/" + robot : (ackermann
             ? adapters::hakoniwa::ackermann_step_service_name(robot)
-            : adapters::hakoniwa::person_step_service_name(robot);
+            : adapters::hakoniwa::person_step_service_name(robot));
         if (service.name != expected_name) {
             throw std::invalid_argument("service " + service.name
                 + " must use the name " + expected_name + " for " + robot);
         }
+        if (servoj) {
+#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ
+            if (std::exchange(has_servoj, true)) {
+                throw std::invalid_argument("only one ServoJ-controlled arm per asset is supported");
+            }
+            continue;
+#else
+            throw std::invalid_argument("ServoJ service is disabled in this build");
+#endif
+        }
 ...
             runtime::AckermannServiceFactory ackermann_service_factory;
-            std::vector<runtime::AsyncServiceComponents> person_services;
+            std::vector<runtime::AsyncServiceComponents> independent_services;
+            std::shared_ptr<runtime::IController> servoj_controller;
 ...
             for (std::size_t index = 0; index < definition.services.size(); ++index) {
                 const auto& service = definition.services[index];
+#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ) && HAKONIWA_ROBOT_RUNTIME_ENABLE_SERVOJ
+                if (service.type == runtime::RuntimeServiceType::ServoJ) {
+                    std::vector<runtime::TrajectoryActuatorBinding> joints;
+                    adapters::hakoniwa::ServoJLimits limits {};
+                    // Packet order follows the manifest's position actuators for this robot.
+                    for (const auto& actuator : definition.actuators) {
+                        if (actuator.pdu_robot == service.pdu_robot
+                            && actuator.command_type == runtime::ActuatorCommandType::Position) {
+                            if (joints.size() == limits.size()) {
+                                throw std::invalid_argument("ServoJ requires exactly six position actuators");
+                            }
+                            limits[joints.size()] = actuator.limits;
+                            joints.push_back({actuator.joint_name, actuator.component_id,
+                                actuator.command_type});
+                        }
+                    }
+                    if (joints.size() != limits.size()) {
+                        throw std::invalid_argument("ServoJ requires exactly six position actuators");
+                    }
+                    auto goal = std::make_shared<std::optional<adapters::hakoniwa::ServoJGoal>>();
+                    servoj_controller = std::make_shared<adapters::hakoniwa::ServoJController>(
+                        service.name + ":motion", goal, std::move(joints));
+                    independent_services.push_back(make_service_components(index,
+                        std::make_shared<adapters::hakoniwa::ServoJService>(std::move(goal), limits)));
+                    continue;
+                }
+#endif
                 if (service.type != runtime::RuntimeServiceType::PersonStep) {
                     continue;
                 }
                 const auto& person = service.pdu_robot;
-                person_services.push_back(make_service_components(index,
+                independent_services.push_back(make_service_components(index,
 ...
-            if (!person_services.empty()) {
+            if (!people.empty()) {
 ...
                 , ackermann_service_factory
-                , std::move(person_services)
+                , std::move(independent_services)
                 , std::move(person_state_publisher)
+                , std::move(servoj_controller)
                 );
```

## 付録 C. robot-arm の差分

robot-arm に入るまで、当ててください（手順 5.2 の注意）。

```diff
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ target_link_libraries(hako-robot-arm-hakoniwa-runner PRIVATE
     ${HAKO_ASSETS_LINK_TARGET_IMPORTED}
     ${HAKO_CONDUCTOR_LINK_TARGET_IMPORTED}
     ${HAKO_PDU_ENDPOINT_LINK_TARGET_IMPORTED}
+    hakoniwa-core::shakoc
 )

--- a/apps/common/manifest_runtime_app.hpp
+++ b/apps/common/manifest_runtime_app.hpp
@@ struct ManifestRuntimeAppConfig {
+    std::string rpc_service_config;
 };

--- a/apps/common/manifest_runtime_app.cpp
+++ b/apps/common/manifest_runtime_app.cpp
 #else
+        ::hakoniwa::robot_runtime::factory::ManifestFactoryConfig factory_config;
+        factory_config.asset_name = config.asset_name;
+        factory_config.endpoint_name = config.endpoint_name;
+        factory_config.realtime_sync_cycle_msec = config.realtime_sync_cycle_msec;
+        factory_config.async_rpc_service_definition_path = config.rpc_service_config;
         runtime_runner = ::hakoniwa::robot_runtime::factory::ManifestFactory::create(
-            manifest_path, {config.asset_name, config.endpoint_name, config.realtime_sync_cycle_msec});
+            manifest_path, std::move(factory_config));
 #endif

--- a/apps/common/robot_arm_app.cpp
+++ b/apps/common/robot_arm_app.cpp
@@ struct LaunchOptions {
     std::uint64_t realtime_sync_cycle_msec {0};
+    std::string rpc_service_config;
 };
@@ void print_usage(const char* program)
-           " [--realtime-sync-cycle-msec <0..>]\n\n"
+           " [--realtime-sync-cycle-msec <0..>] [--rpc-service-config <file>]\n\n"
@@ bool parse_options(const int argc, char** argv, LaunchOptions& options)
+        if (argument == "--rpc-service-config") {
+            if (++index >= argc || argv[index][0] == '-') {
+                throw std::invalid_argument("--rpc-service-config expects a path");
+            }
+            options.rpc_service_config = argv[index];
+            continue;
+        }
@@ int run_robot_arm_app(int argc, char** argv)
             options.realtime_sync_cycle_msec,
+            options.rpc_service_config,
         });
```

## 付録 D. 起動の台本（run.sh）

手順 7 の 0〜6 を、そのまま流せる形にしたものです。`$PREFIX`・`$WORK` と、MuJoCo のライブラリの場所は、自分の環境に合わせてください。引数は、新しいログの場所（毎回新しくする）です。

```bash
#!/usr/bin/env bash
set -euo pipefail

prefix=$PREFIX
work=$WORK
logs=$1
if [[ -e "$logs" ]]; then
    echo "Choose a new run directory to avoid reusing Core mmap state: $logs" >&2
    exit 2
fi
mkdir -p "$logs/mmap"
export HAKO_CONFIG_PATH="$logs/core.json"
export LD_LIBRARY_PATH="$prefix/lib:$work/build-arm/_deps/mujoco_precompiled-src/lib"
export HAKO_PDU_RPC_LIBRARY="$work/build-rpc/src/libhakoniwa_pdu_rpc.so"
export PYTHONPATH="$work/hakoniwa-pdu-rpc/python:$work/local-python"
printf '{"shm_type":"mmap","core_mmap_path":"%s/mmap","asset_timeout_usec":600000000}\n' \
    "$logs" > "$HAKO_CONFIG_PATH"
# service.json の coreConfig は、この $HAKO_CONFIG_PATH を指すように書いておく
cp service.json "$logs/service.json"
sed -i "s#/path/to/core.json#$HAKO_CONFIG_PATH#" "$logs/service.json"

pids=()
started=0
cleanup() {
    if (( started )); then
        timeout 5 "$prefix/bin/hako-cmd" stop >"$logs/stop.log" 2>&1 || true
    fi
    for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT

wait_for_log() {
    local file=$1 pattern=$2 pid=$3
    for ((attempt=0; attempt<150; ++attempt)); do
        grep -qF -- "$pattern" "$file" 2>/dev/null && return
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    echo "Missing '$pattern' in $file" >&2
    cat "$file" >&2
    return 1
}

# 1. Robot Runtime
"$work/build-arm/bin/robot-arm-hakoniwa-asset" --manifest asset-manifest.json \
    --rpc-service-config "$logs/service.json" --no-viewer --realtime-sync-cycle-msec 2 \
    >"$logs/arm.log" 2>&1 &
server=$!; pids+=("$server")
wait_for_log "$logs/arm.log" "[INFO] Starting Hakoniwa Runner:" "$server"

# 2. Python のクライアント（client.start() のあと "CLIENT transport started" を出し、
#    $logs/start-go ができるまで待ってから呼ぶように書いておく）
python3 client.py "$logs/service.json" "$logs/start-go" >"$logs/client.log" 2>&1 &
client=$!; pids+=("$client")
wait_for_log "$logs/client.log" "CLIENT transport started" "$client"

# 3. 登録を確かめる
for ((attempt=0; attempt<100; ++attempt)); do
    "$prefix/bin/hako-cmd" jmeta >"$logs/registered.json"
    grep -qF ServoJClientAsset "$logs/registered.json" && break
    sleep 0.1
done
grep -qF ServoJClientAsset "$logs/registered.json"

# 4. 開始して、クライアントに知らせる
"$prefix/bin/hako-cmd" start >"$logs/start.log" 2>&1
started=1
: > "$logs/start-go"

# 5. 呼び出しの結果を待つ
wait "$client"
cat "$logs/client.log"
```
