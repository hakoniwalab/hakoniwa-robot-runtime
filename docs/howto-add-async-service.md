# 非同期 RPC のサービスを 1 つ足す（手順書・ServoJ を例に）

この文書は、Robot Runtime に新しい RPC のサービスを足す人のための手順書です。例として、Dobot の `ServoJ`（関節の目標を受け取り、すぐ応える指令）を使います。

> **状態（2026-11-26）**：下書き。**[未確認]** の付いた手順は、まだ実際に流して確かめていません。確かめたら印を外します。
>
> 今の Robot Runtime では、サービスは **runtime の中に C++ で足します**（runtime を直す必要があります）。runtime を直さずにプラグインとして足せる形（C の境目、設計書の段 E）は、次の回に作ります。そのときは手順 3〜5 が変わります。

仕組みの説明は [design-async-rpc.md](design-async-rpc.md)、マニフェストの書き方は [configuration.md](configuration.md) の 2.4 節にあります。ここでは「何を、どの順で」に絞ります。

## 0. 全体の流れ

```text
1. 型を決める        ServoJ.srv を書く
2. PDU を作る        手元で生成する（hakoniwa-pdu-registry には足さない）
3. サービスを書く    IAsyncService を実装した C++ のクラス
4. 登録する          サービスの種類、マニフェスト、工場
5. ビルドに足す      ソースの一覧
6. サービスの定義    core-pro の service.json
7. 呼んで確かめる    Python のクライアント → ROS 2
8. テスト
```

前提：Business Pack の Foundation が入っていること（hakoniwa-core-pro は #123 を含む版、`hakoniwa-pdu` 1.7.1 以降）。

## 1. 型を決める（ServoJ.srv）

要求と応答を、ROS の `.srv` の形で書きます。PDU は大きさが決まっている（固定長）ほうが扱いやすいので、配列は長さを決めます。

```text
# my_robot_srvs/srv/ServoJ.srv
float64[6] joint      # 関節の目標 [rad]
float64 t             # 目標に着くまでの時間 [s]
---
int32 res             # 0 = 受け付けた
```

- 名前（パッケージ名 `my_robot_srvs`、型名 `ServoJ`）は、自分のものにします。Hakoniwa の既存の名前（`hako_srv_msgs` など）は使いません。
- 実機の Dobot の srv（`dobot_msgs_v4/srv/ServoJ`）と同じ欄にしたい場合は、そちらに合わせます（ROS 2 から同じ型で呼べるようにするため。手順 7）。

## 2. PDU を手元で作る（hakoniwa-pdu-registry を大きくしない）

hakoniwa-pdu-registry の本体には足さず、**自分の型だけを、追跡されないフォルダに生成**します。registry には、外の ROS のパッケージから選んだ型だけを作る道具があります（`tools/fetch.py`、`tools/generate_from_sources.py`。詳しくは registry の `docs/external-message-fetch.md`）。

### 2.1 .srv を Hakoniwa の .msg に変える [未確認]

サービスは、要求・応答と、それを包むパケット（`…RequestPacket`、`…ResponsePacket`）の 4 つの `.msg` になります。registry の変換の道具を使います。

```bash
cd hakoniwa-pdu-registry
python3 -m generators.generate_hako_service_msgs.main \
  /path/to/my_robot_srvs/srv/ServoJ.srv --out /path/to/my-pdu-src
```

`/path/to/my-pdu-src/<パッケージ>_msgs/msg/` に、次の 4 つができます（パッケージの名前の付き方は、出てきたもので確かめる）。

```text
ServoJRequest.msg  ServoJResponse.msg  ServoJRequestPacket.msg  ServoJResponsePacket.msg
```

### 2.2 ROS のパッケージの形にして、手元の git に入れる [未確認]

`fetch.py` は、`package.xml` のある ROS のパッケージから `msg/*.msg` を拾います。そこで、できた `msg/` を 1 つのパッケージにまとめ、手元の git のリポジトリにします。

```text
my-pdu-src/
└── my_robot_srvs_msgs/
    ├── package.xml     # <name>my_robot_srvs_msgs</name> と、ライセンスを書く
    └── msg/            # 2.1 でできた 4 つ
```

```bash
cd /path/to/my-pdu-src
git init -q && git add -A && git commit -q -m "ServoJ PDU types"
git rev-parse HEAD     # この番号を 2.3 で使う
```

### 2.3 生成する [未確認]

registry の中に、自分の型の「どこから取るか」の設定を置きます。registry に足すのは、この小さな設定ファイルだけです（足したくなければ、registry の外に置き、パスで渡す）。

```yaml
# sources/my_robot_srvs_msgs.yaml
schema_version: 1
source:
  type: git
  repository: file:///path/to/my-pdu-src
  ref: <2.2 の番号>
packages:
  - my_robot_srvs_msgs
```

```bash
cd hakoniwa-pdu-registry
python3 tools/generate_from_sources.py \
  --sources sources/my_robot_srvs_msgs.yaml \
  --output-dir .cache/my-pdu \
  my_robot_srvs_msgs/ServoJRequestPacket \
  my_robot_srvs_msgs/ServoJResponsePacket
```

- 出力は registry の中の、追跡されないフォルダ（`.cache/my-pdu`）にします（Docker が registry の中しか見えないため）。registry の `pdu/` は変わりません。
- 依存する型（`hako_srv_msgs/ServiceRequestHeader` など）は、registry の今ある定義から自動で解かれます。

### 2.4 出てきたものの使いどころ [未確認]

| 出てきたもの | 使うところ |
|---|---|
| C++ の型と変換（`pdu_cpptype_ServoJRequestPacket.hpp`、`pdu_cpptype_conv_*.hpp`） | 手順 3 のサービス。ビルドのヘッダーの探し先に `.cache/my-pdu/types` などを足す |
| Python の変換（`hakoniwa_pdu` の形） | 手順 7 の Python のクライアント。`PYTHONPATH` に足す |
| 大きさの表（`pdu_size`） | 手順 6 の `service.json` の `pduSize` |

## 3. サービスを書く（IAsyncService）

例として、車のサービス（`src/adapters/endpoint/ackermann_step_service.{hpp,cpp}`）が一番近い形です。これを写して直すのが早いです。

```text
src/adapters/endpoint/servoj_service.hpp
src/adapters/endpoint/servoj_service.cpp
src/adapters/endpoint/servoj_packet.hpp     # 要求・応答のパケットの読み書き（ackermann_step_packet.hpp を手本に）
```

フック（`src/runtime/service/async_service.hpp` の `IAsyncService`）でやること：

| フック | ServoJ でやること |
|---|---|
| `on_request(要求)` | パケットを読んで、関節の目標 6 つと `t` を取り出す。値が有限で範囲の中か確かめる。おかしければ `false`（INVALID で応える） |
| `on_step(状態, サーバの状態, 時刻)` | 目標へのアクチュエータの指令（`ActuatorCommand`）を返す。**ServoJ は受け付けたらすぐ応える**ので、最初の刻みで `Ok` を返す |
| `on_response(状態, 時刻, 入れ物)` | 応答の中身（`res = 0`）を書く。頭（header）は書き換えない。中身の欄は毎回すべて書く |
| `on_cancel()` | 何もしなくてよい（すぐ終わるので） |
| `reset()` | 持っている目標を捨てる |

**決まっていない点（大事）**：ServoJ は「すぐ応え、動きは続く」指令です。応えたあと、関節を目標へ動かし続けるのはだれかを決める必要があります。

- 案 A：サービスは目標を「サービスのコントローラ」に渡すだけ。コントローラが毎刻み目標へ動かす（今の軌道のコントローラに近い）。
- 案 B：サービスが `on_step` の中で動かし続ける（要求が終わったあとも）。

設計書（第 1.3 節・第 8 節）で、まだ決めていません。手順書では、まず **案 A** で書くことを勧めます（応答と動きが分かれて、実機と同じ形になるため）。

## 4. 登録する（サービスの種類、マニフェスト、工場）

| どこ | 何を足すか |
|---|---|
| `src/runtime/runtime_definition.hpp` | `enum class RuntimeServiceType` に `ServoJ` を足す |
| `src/runtime/runtime_factory_manifest.cpp` | マニフェストの `type` の文字（例 `"servo_j"`）を読み分ける（今は `"ackermann_step"` と `"person_step"`） |
| `schemas/runtime/asset-manifest.schema.json` | `$defs/service/properties/type/enum` に `"servo_j"` を足す |
| `src/factory/manifest_factory.cpp` | サービスの種類ごとに、決まった名前か確かめ（例 `ArmService/ServoJ/<robot>`）、`ServoJService` を作って受け口に登録する分岐を足す（今は車と人の 2 つ） |
| Arbiter | ServoJ の指令と、腕のほかのコントローラ（軌道など）の優先の順を決める |

マニフェストの書き方：

```json
"services": [
  { "name": "ArmService/ServoJ/nova5", "type": "servo_j", "pdu_robot": "nova5", "lockstep": false }
]
```

- ServoJ はロックステップにしません（実機と同じく、要求は好きなときに来て、すぐ応える）。

## 5. ビルドに足す

新しいソース（`servoj_service.cpp` など）を、ビルドのソースの一覧に足します。

- 2026-11-26 から、runtime がソースの一覧を CMake のファイルとして出す作業（案 B）をしています。それが入れば、runtime の一覧に 1 行足すだけで、robot-arm と Urban にも届きます。
- 入るまでは、使う側（robot-arm など）の CMake の一覧にも足す必要があります。
- ヘッダーの探し先に、手順 2 で生成したフォルダを足します。

## 6. core-pro のサービスの定義（service.json） [未確認]

core-pro は、サービスの名前、型、PDU の大きさを `service.json` で知ります。

```json
{
  "pduMetaDataSize": 24,
  "services": [
    {
      "name": "ArmService/ServoJ/nova5",
      "type": "my_robot_srvs_msgs/ServoJ",
      "maxClients": 1,
      "pduSize": {
        "server": { "baseSize": <ServoJResponsePacket の大きさ + 8>, "heapSize": 0 },
        "client": { "baseSize": <ServoJRequestPacket の大きさ + 8>, "heapSize": 0 }
      }
    }
  ]
}
```

- 大きさは、手順 2 で出てきた大きさの表から取ります（車のサービスの `tools/gen_rpc_service_config.py` が、同じ計算をしています。`+ 8` はその道具と同じ余裕）。
- `maxClients` は 1 にします（core-pro のサーバは、1 度に 1 クライアントの番しか読まない。hakoniwalab/hakoniwa-core-pro#115）。
- 車と人は `gen_rpc_service_config.py` で作れます。ServoJ も、その道具に足すと楽です。

## 7. 呼んで確かめる [未確認]

1. **Python のクライアントで呼ぶ**：hakoniwa-pdu-rpc の Python（`hakoniwa_pdu_rpc`）で、`ArmService/ServoJ/nova5` を呼びます。shm の口を使えば、core-pro に直接つながります（hakoniwa-pdu-rpc の `docs/design/shm/README.md` 第 5 節）。応答の `res` が 0 で、関節が目標へ動くことを確かめます。
2. **ROS 2 から呼ぶ**：hakoniwa-pdu-ros の service-server に、`ServoJ` のサービスを足します（`service.rpc_service_config` に、同じ RPC の定義のファイルを渡す）。ROS 2 の `ServoJ` の srv の型が、手順 1 と同じ欄である必要があります。

## 8. テスト

車のサービスのテストを手本にします。

| 何を | 手本 |
|---|---|
| サービス単体（要求を読む、指令、応答の中身） | `tests/test_ackermann_step_service.cpp` |
| 受け口と合わせて（0 ステップで終わる、キャンセル） | `tests/test_async_service_zero_step.cpp` |
| 本物の core-pro で | `tests/integration/test_ackermann_step_service.bash` |

## 変える場所の一覧（まとめ）

| リポジトリ | ファイル |
|---|---|
| 自分のもの | `ServoJ.srv`、生成の元のパッケージ（手順 1・2） |
| hakoniwa-pdu-registry | 生成の設定 1 つ（`sources/…yaml`）だけ。外に置けば 0 |
| hakoniwa-robot-runtime | `servoj_service.*`・`servoj_packet.hpp`（新規）、`runtime_definition.hpp`、`runtime_factory_manifest.cpp`、`manifest_factory.cpp`、マニフェストのスキーマ、ソースの一覧、テスト |
| 使う側（robot-arm など） | マニフェスト、`service.json`（案 B が入るまでは CMake の一覧も） |
| hakoniwa-pdu-ros（ROS 2 から呼ぶとき） | service-server の設定 |
