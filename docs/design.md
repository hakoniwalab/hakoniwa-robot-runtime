# hakoniwa-robot-runtime Design

この文書は、`hakoniwa-robot-runtime` の設計と責務境界を説明する正本です。

本 Runtime は、特定のロボット機種を実装するためのパッケージではありません。Robot Pack や Application から与えられたモデル・設定・外部 I/O を組み合わせ、箱庭上でアクチュエータ型ロボットを実行するための共通実行層です。

現在の実装はロボットアームで検証されていますが、設計上の中心は「Arm」ではなく、**Command を actuator へ適用し、physical state を返す Runtime** です。

## 1. Runtime の位置づけ

```text
Robot Pack / Application
        |
        | manifest path
        v
+-----------------------------+
| ManifestFactory             |
| - top-level document        |
| - path resolution           |
| - concrete composition      |
+-------------+---------------+
              |
       +------+--------------------+
       |                           |
       v                           v
 RuntimeFactory               Adapters
       |                    Endpoint / MuJoCo
       v                           |
 RuntimeDefinition                 |
       |                           |
       +-------------+-------------+
                     |
                     v
              ActuatorRuntime
                     |
                     v
 CommandSource -> Controller -> Arbiter -> Plant -> Publisher
                     |
                     v
                   Runner
                     |
                     v
               Hakoniwa Asset
```

`ManifestFactory` が最上位の composition root です。

Runtime 固有の manifest semantics は `RuntimeFactory` へ委譲し、transport / physics backend / Runner implementation の具体選択は `ManifestFactory` 側で行います。

独立した巨大な `Config` / `ManifestLoader` layer は置きません。

## 2. Runtime core の5責務

Runtime core は次の5分類で構成します。

```text
CommandSource -> Controller -> Arbiter -> Plant -> Publisher
```

`ActuatorRuntime` はこの5分類の一つではありません。5責務を **1 simulation step の順序で呼び出す orchestrator** です。

### 2.1 CommandSource

外部の独立した非同期 input stream を current Runtime step へ取り込みます。

主な実装:

- `ScalarPduCommandSource`
- `JointTrajectoryCommandSource`
- `JoyCommandSource`

原則:

- 1 `ICommandSource` = 1 independent input channel / stream
- `poll()` は non-blocking
- transport receive 自体を `poll()` 内で待たない
- receive callback 等で受信済みの最新値を current step へ移す
- 新着なしは error ではない
- Controller / Arbitration / physics の判断はしない

Endpoint adapter は、非同期 receive event を channel-local mailbox に保持し、CommandSource はその値を消費します。

### 2.2 Controller

CommandSource input、現在の `RobotState`、`RuntimeStepContext` から `ActuatorCommand` 候補を生成します。

主な実装:

- `ScalarPduCommandController`
- `JointTrajectoryController`
- `ManualController`
- `HoldController`

原則:

- physical backend へ直接依存しない
- 他 Controller の内部 state を直接参照しない
- 前 step の selection feedback は `RuntimeStepContext` から受け取る
- time-based trajectory は preempt 後に古い軌道を暗黙継続しない
- control handoff 時の position target continuity を維持する

`ManualController` と `HoldController` は、必要に応じて previous selected position command を継承します。

### 2.3 Arbiter

現在 step で Controller 群が生成した候補から、一つの **logical control / control group** を選択します。

```text
ControllerOutput[*]
       |
       v
PriorityCommandArbiter
       |
       v
selected ActuatorCommand[*]
```

原則:

- stateless
- AUTO / MANUAL / HOLD のような mode state を保持しない
- actuator ごとに異なる Controller の command を mix しない
- logical control 単位で選択する
- physical safety validation はしない

現在の標準 priority は次です。

```text
Manual          20
JointTrajectory 10
scalar direct    0
Hold           -10
```

同一 priority の複数 logical control が競合した場合、登録順で勝者を決めず conflict として扱います。

### 2.4 Plant

Arbiter が選択した `ActuatorCommand` を physical backend へ適用し、physical state と simulation time を提供する境界です。

```text
selected ActuatorCommand[*]
        |
        v
IActuatorPlant
  - actuator binding
  - command guard
  - fallback / clamp
  - physical apply
  - physics step
        |
        v
RobotState
```

Plant 入口で最終的に guard するもの:

- unknown actuator ID
- duplicate command
- missing command
- command type mismatch
- future / expired command
- NaN / Inf
- actuator limit

基本 fallback:

- position: current position
- velocity: 0
- effort: 0

Safety を Controller や Arbiter に分散させず、physical binding と現在 state を知っている Plant 入口で最終保証します。

現在の具象実装は `MujocoActuatorPlant` です。

### 2.5 Publisher

physical step 完了後の `RobotState` を外部表現へ変換して送信します。

現在の主な実装は `JointStatePublisher` です。

原則:

- Plant state を変更しない
- Control 判断をしない
- rate limiting は simulation time で行う
- transport 固有 write は adapter へ委譲する
- publish failure によって完了済み physical step を rollback しない

### 2.6 Mirror extension（opt-in）

Mirror は、外部の simulation / Plant が所有する物体を、本 Runtime
の physical world に表示・接触させるための代理物体です。Mirror 自身の運動は
本 Runtime が判断せず、外部から受信した pose / velocity を反映します。

Mirror も既存の Source / Controller / Plant / Publisher の責務分類に載せます。
ただし、Mirror command は制御候補ではないため Arbiter の対象にしません。
フラグで Arbiter の判断を無効化するのではなく、異なる command type と経路によって
構造的に分離します。

```text
Normal control path:
  CommandSource -> Controller -> Arbiter --------+
                                                   |
Mirror path:                                      v
  MirrorSource  -> MirrorController ------------> Plant -> Publisher
```

責務:

- `MirrorSource`: pose / velocity PDU の最新値を non-blocking に取り込む
- `MirrorController`: 受信値を physical backend 非依存の Mirror command へ正規化する
- `Arbiter`: Mirror command を受け取らず、選択・競合判定を行わない
- `Plant`: Mirror command を backend の pose / velocity へ反映し、physics step
  完了後の接触情報を生成する
- `Publisher`: 接触情報から送信ポリシーを適用し、Impulse PDU を外部 Plant
  へ送信する

Mirror velocity は相対法線速度の計算に必要です。velocity PDU がある場合は
それを正本とし、`world` / `body` frameを設定で明示します。ない場合だけ
pose の simulation-time 差分から推定します。
pose 反映時に無条件で velocity をゼロクリアしてはいけません。reset 時は
最後の pose / time と接触履歴をクリアします。

#### 2.6.1 External Drone PDU contract

Mirror Runtimeは特定のDrone実装には依存せず、外部Droneとの標準接続契約として
次のPDUの組み合わせを使用します。Drone Core / Proは、この契約を既存PDUのまま
満たす実装の一つです。`pdu_robot` は対象ドローンのRobot名であり、以下では
`Drone-1` を例とします。

| 方向 | PDU channel | message type | Mirrorでの用途 |
|---|---|---|---|
| Drone -> Mirror | `Drone-1/pos` | `geometry_msgs/Twist` | pose入力 |
| Drone -> Mirror | `Drone-1/velocity` | `geometry_msgs/Twist` | velocity入力 |
| Mirror -> Drone | `Drone-1/impulse` | `hako_msgs/ImpulseCollision` | 接触Impulse出力 |

`pos` は必須で、`Twist.linear` をworld-frameのposition、`Twist.angular` を
roll / pitch / yaw（XYZ Euler angle）として解釈します。`velocity` は
標準ではbody-frame値として受け取り、MirrorControllerがMuJoCo
freejointの規約（world-frame並進速度 / body-frame角速度）へ正規化します。
`velocity` PDUを利用できない構成ではpose差分から推定できますが、Drone連携では
`velocity` PDUを使用することを標準とします。Car等との相互衝突を扱う構成では、
外部Droneが `impulse` PDUを受信・適用できることも契約に含みます。

したがって、将来Drone Pro以外を接続する場合も同じ3 PDUを使用します。異なる
内部APIやtransportを持つDrone実装は、adapterでこの契約へ変換します。

takeoff / move / landなどのFleet指令はMirror経路の責務外です。外部Drone Plantが
従来の指令を処理し、その結果としてpublishする `pos` / `velocity` をMirrorが
追従します。Mirrorから飛行指令を送信することはありません。

#### 2.6.2 接触境界

本 Runtime が Impulse の送信対象とするのは、Mirror と本 Runtime 所有の
physical Plant body との接触です。physical Plant body は複数存在して構いません。

```text
Mirror x local physical Plant body  -> Impulse 対象
Mirror x Mirror                     -> 対象外
Mirror x Environment                -> 対象外
```

Environment（地面・建物等）と Mirror 所有物体との接触応答は、外部 Plant 側の
責務です。分類は body 名の命名規則に依存せず、Manifest から解決された
Mirror / local physical / Environment の所有関係に基づきます。

同一 Mirror が同一 step で複数の対象と接触した場合、初期実装では
最も深い接触を1件選びます。複数 Impulse の同時送信は将来拡張とします。

#### 2.6.3 Impulse 送信ポリシー

Impulse は継続的な力ではなく、接触開始時の one-shot event として送信します。
`hakoniwa-mujoco-robots` の ball sample と同じ基本方式を使用します。

- 接触の inactive -> active 遷移時のみ送信する
- 接触継続中は再送信しない
- 相対法線速度が threshold 未満なら抑制する
- 前回送信から cooldown 経過前の再接触は抑制する
- 接触履歴と cooldown は Mirror / local physical body のペアごとに持つ
- cooldown は step 数ではなく Plant の simulation time で判定する

Impulse対応の外部Droneは、Impulseを消費したstepで `collision=false` をPDUへ
書き戻します。Drone Core / Proもこの規約に従うため、latest-value transport上でも
同じImpulseは繰り返し適用されません。

Publisher のポリシーは component config から外部設定します。

```json
{
  "$schema": "https://hakoniwa.dev/schemas/impulse-collision-output.schema.json",
  "schema_version": 1,
  "spec": {
    "mirror_component": "drone-1-mirror"
  },
  "pdu_config": {
    "pdu_name": "impulse",
    "message_type": "hako_msgs/ImpulseCollision"
  },
  "policy": {
    "restitution_coefficient": 0.3,
    "relative_normal_speed_threshold_mps": 0.2,
    "cooldown_sec": 0.1
  }
}
```

`cooldown_sec` は正の有限値とし、`cooldown_steps` は公開設定にしません。
これにより physics timestep から独立した送信間隔を保ちます。接触対象の所有関係は
このポリシーに混ぜず、Mirror / Plant binding として定義します。

#### 2.6.4 opt-in build boundary

Mirror extension は既存 Runtime への回帰を避けるため、Ackermann / mobile-base
extension と同じ opt-in 方式で導入します。

```cpp
#if defined(HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR) \
    && HAKONIWA_ROBOT_RUNTIME_ENABLE_MIRROR
// Mirror-only implementation
#endif
```

未定義または `0` の場合:

- Mirror 用 Source / Controller / Plant extension / Publisher を構成しない
- Mirror 用 Manifest / component config を解釈しない
- Mirror / Impulse PDU 固有の link dependency を要求しない
- 既存の Runtime step、ABI、標準テストの挙動を変更しない

Mirror config を使う Application だけがマクロを `1` とし、対応 source と
adapter を build target に組み込みます。

## 3. ActuatorRuntime

`ActuatorRuntime` は5責務を1 stepとして順序実行します。

```text
Read Plant State / Time
        |
        v
Build RuntimeStepContext
        |
        v
CommandSource
        |
        v
Controller
        |
        v
Arbiter
        |
        v
Plant
        |
        v
Publisher
```

概念的には次の処理です。

```cpp
const auto current_state = plant_->read_state();
const auto context = prepare_step_context(current_state);
const auto inputs = poll_sources(context, ...);
const auto outputs = update_controllers(inputs, current_state, context, ...);
auto arbitration = arbiter_->arbitrate(outputs, current_state, context);
const auto next_state = plant_->step(arbitration.selected_commands);
const auto next_context = complete_step(next_state, arbitration);
publish_state(next_state, next_context, ...);
```

Runtime 自身は `simulation_time += delta` のような独立 clock update を行いません。

## 4. Time Model

Runtime の時間設計では、physical backend の時刻を Asset-local な Source of Truth とします。

```text
Hakoniwa Core world time
  = distributed simulation global scheduling time

Hakoniwa Asset-local time
  = this Asset's completed local step time

Physical Plant time
  = Runtime が参照する local physical time
```

MuJoCo backend では:

- `model->opt.timestep` = fixed delta の Source of Truth
- `mjData::time` = physical / Asset-local time の Source of Truth
- Runtime は独立した Clock object を持たない
- Conductor ownerの場合だけ、同じdeltaでHakoniwa Asset / Conductorを構成する
- wall-clock pacing は Runner 側の責務

## 5. RuntimeFactory

`RuntimeFactory` は Runtime module の composition を担当します。

```text
ManifestFactory から委譲された Runtime 記述
        |
        v
RuntimeFactory
        |
        +--> parse / validation
        |       |
        |       v
        |  RuntimeDefinition
        |
        +--> Runtime object composition
                |
                v
          ActuatorRuntime
```

責務:

- actuator semantics の解釈
- trajectory semantics の解釈
- manual semantics の解釈
- state-output semantics の解釈
- component 間の cross-reference validation
- `RuntimeDefinition` への正規化
- Source / Controller / Arbiter / Publisher の構成
- 注入された `IActuatorPlant` の接続
- reader / writer factory を介した I/O seam の接続

非責務:

- transport implementation の選択
- physics backend の選択
- Runner implementation の選択
- top-level Application lifecycle

## 6. ManifestFactory

`ManifestFactory` は、manifest path から application-facing `IRunner` までを構成する最上位 composition root です。

```text
manifest file
      |
      v
ManifestFactory
  ├─ top-level document/path resolution
  ├─ RuntimeFactory
  ├─ Endpoint + Endpoint adapters
  ├─ Physics adapter
  └─ RunnerFactory -> IRunner
```

公開入口:

```text
include/hakoniwa/robot_runtime/factory/manifest_factory.hpp
```

## 7. Adapter Boundary

外部技術と Runtime 内部 interface の変換は Adapter に隔離します。

```text
src/adapters/
├── endpoint/
│   ├── Float64 receive
│   ├── JointTrajectory receive
│   ├── Joy receive
│   └── JointState publish
└── physics/
    └── mujoco/
        └── MujocoActuatorPlant
```

Endpoint adapter は Hakoniwa PDU Endpoint と Runtime の reader / writer seam を接続します。

Runtime core は SHM / TCP など transport の違いを直接扱いません。transport 差分は Endpoint / Bridge 側へ閉じ込めます。

Physics adapter は `IActuatorPlant` と具体 physics backend を接続します。現在は MuJoCo を実装しています。

## 8. Runner Boundary

Runner は `ActuatorRuntime` を **どの execution lifecycle で反復実行するか**を担当します。

Application-facing interface:

```text
include/hakoniwa/robot_runtime/runner/runner.hpp
```

現在の具象実装は `HakoniwaRunner` です。

```text
HakoniwaRunner
  ├─ Endpoint lifecycle
  ├─ Hakoniwa Conductor lifecycle（owner時のみ）
  ├─ pause / reset / stop
  ├─ model synchronization
  ├─ wall-clock pacing
  ├─ HakoniwaAssetDriver
  └─ ActuatorRuntime::step() / reset()
```

`HakoniwaAssetDriver` は Hakoniwa C Asset API の low-level helper で、callback wiring と manual timing loop を隔離します。

`HakoniwaRunnerConfig::owns_conductor` はConductorの所有境界を明示します。
単体アセットでは既定値 `true` とし、RunnerがConductorをstart / stopします。
複数アセット構成では全体でownerを1つだけ選び、それ以外は `false` として
外部Conductorへ参加します。例えばDrone + Car構成ではDroneがownerとなり、
Car側RunnerはConductorを起動・停止しません。Launcherは複数プロセスの起動順を
管理しますが、Conductor ownerを複数に増やしてはいけません。

現行 `IRunner` は Viewer 連携のため `mjModel*` / `mjData*` accessor を持っています。そのため application-facing Runner interface には現在 MuJoCo 依存が一部残っています。将来別 physics / presentation backend を導入する場合は、view capability を execution contract から分離する余地があります。

## 9. External Interface

Runtime core は ROS 2 API や physical Gamepad を直接扱いません。

代表的な input 経路:

```text
ROS 2 Application
      |
      v
ROS / PDU Bridge
      |
      v
Hakoniwa PDU Endpoint
      |
      v
CommandSource
```

```text
Physical Gamepad
      |
      v
Gamepad Frontend
      |
      v
Joy PDU
      |
      v
JoyCommandSource
```

代表的な output 経路:

```text
MuJoCo
  -> Plant
  -> RobotState
  -> JointStatePublisher
  -> Endpoint Writer
  -> Hakoniwa PDU
  -> Bridge / Application
```

ROS 2 bridge、Gamepad frontend、Launcher は本 Runtime の外側です。

## 10. Source Layout

```text
include/hakoniwa/robot_runtime/
├── adapters/
├── factory/
└── runner/

src/
├── runtime/
│   ├── actuator_runtime.*
│   ├── runtime_definition.hpp
│   ├── runtime_factory*
│   ├── source/
│   ├── controller/
│   ├── arbiter/
│   ├── plant/
│   └── publisher/
├── adapters/
│   ├── endpoint/
│   └── physics/mujoco/
├── runner/
│   ├── runner_factory.*
│   └── hakoniwa/
└── factory/
    ├── manifest_factory.cpp
    ├── manifest_factory_internal.hpp
    └── manifest_factory_manifest.cpp
```

`src/runtime/` 内の interface / class は Runtime 内部契約です。Application は可能な限り `ManifestFactory` と `IRunner` を入口として利用します。

## 11. Robot Pack との責務分離

`hakoniwa-robot-runtime` は共通実行層です。Robot Pack はロボット固有の構成を所有します。

```text
hakoniwa-robot-runtime
  - Runtime core
  - RuntimeFactory
  - ManifestFactory
  - Runner
  - Endpoint / MuJoCo adapters

Robot Pack / Application
  - robot model
  - robot-specific actuator / controller config
  - PDU definition / endpoint config
  - Recipe / Launcher
  - demo / frontend
  - robot-specific controller
```

この分離により、個別ロボットや個別ユースケースで作成するモデル・Recipe・設定と、共通 Runtime 実装を独立して保守できます。

## 12. 現在のスコープ

現在の Runtime は joint / scalar actuator を中心とした実装です。

実装済みの代表機能:

- scalar actuator command
- JointTrajectory command
- Joy manual control
- Hold control
- priority arbitration
- Plant-side command guard
- MuJoCo physical update
- JointState output
- Hakoniwa Asset Runner

特定のロボット機種、運動学 solver、歩容生成、移動ロボットの navigation などは Runtime core の責務ではありません。

車輪型・脚型などへ展開する場合も、まず既存の `ActuatorCommand` / `IActuatorPlant` / Controller 境界で表現できるかを確認し、必要な拡張だけを追加します。

## 13. Build / Integration の現状

現在 `hakoniwa-robot-runtime` は source ownership を持ちますが、top-level CMake composition は consumer 側から行う構成です。

consumer は本リポジトリの source / public header を build target に組み込み、Robot Pack / Application 側のモデル・設定・Recipeと組み合わせて利用します。

将来 Runtime 側が独立した CMake target / package export を持つ場合でも、Runtime / Adapter / Runner / Factory の責務境界は維持します。
