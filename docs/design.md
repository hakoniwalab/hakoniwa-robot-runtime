# hakoniwa-robot-runtime Design

この文書は、`hakoniwa-robot-runtime` の設計と責務境界を説明する正本です。

本 Runtime は、特定のロボット機種を実装するためのパッケージではありません。Robot Pack や Application から与えられたモデル・設定・外部I/Oを組み合わせ、箱庭上でアクチュエータ型ロボットを実行するための共通実行層です。

現在の実装はロボットアームで検証されていますが、設計上の中心は「Arm」ではなく、**Command を actuator へ適用し、physical state を返す Runtime** です。

---

## 1. Runtime の位置づけ

全体構造は次のように捉えます。

```text
Robot Pack / Application
        |
        | manifest path
        v
+-----------------------------+
| ManifestFactory             |
|                             |
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

---

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

`ManualController` と `HoldController` は、必要に応じて previous selected position command を継承します。これにより control handoff 時に target が不必要に実測位置へ飛ぶことを避けます。

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

現在の標準的な priority は次です。

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

基本 fallback は次です。

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

---

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

---

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
- 同じ delta で Hakoniwa Asset / Conductor を構成する
- wall-clock pacing は Runner 側の責務

これにより Runtime 内部に「Hakoniwa timeとは別の独自 simulation clock」を作りません。

---

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

Runtime 固有 manifest parser は責務別に分割し、一つの巨大 parser へ戻さない方針です。

---

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

`ManifestFactory` が知るのは、具体的な技術をどの組み合わせで使うかです。

一方、Runtime step の control policy や Hakoniwa C callback の詳細をここへ持ち込みません。

公開入口:

```text
include/hakoniwa/robot_runtime/factory/manifest_factory.hpp
```

---

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

### Endpoint Adapter

Hakoniwa PDU Endpoint と Runtime の reader / writer seam を接続します。

Runtime core は SHM / TCP など transport の違いを直接扱いません。transport 差分は Endpoint / Bridge 側へ閉じ込めます。

### Physics Adapter

`IActuatorPlant` と具体 physics backend を接続します。

現在は MuJoCo を実装していますが、Runtime core 自体は `IActuatorPlant` を通して physical backend と接続します。

---

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
  ├─ Hakoniwa Conductor lifecycle
  ├─ pause / reset / stop
  ├─ model synchronization
  ├─ wall-clock pacing
  ├─ HakoniwaAssetDriver
  └─ ActuatorRuntime::step() / reset()
```

`HakoniwaAssetDriver` は Hakoniwa C Asset API の low-level helper で、callback wiring と manual timing loop を隔離します。

将来 Hakoniwa Asset を使わない standalone execution を追加する場合は、Runtime core を変更するのではなく別 Runner implementation として追加する方針です。

### 現在の注意点

現行 `IRunner` は Viewer 連携のため `mjModel*` / `mjData*` accessor を持っています。そのため application-facing Runner interface には現在 MuJoCo 依存が一部残っています。

これは Runtime core の必須依存ではなく、将来別 physics / presentation backend を導入する場合は view capability を Runner execution contract から分離できる設計余地として扱います。

---

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

---

## 10. Source Layout

現在の source layout は次です。

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

---

## 11. Robot Pack との責務分離

`hakoniwa-robot-runtime` は共通実行層です。Robot Pack はロボット固有の構成を所有します。

```text
hakoniwa-robot-runtime
  - Runtime core
  - RuntimeFactory
  - ManifestFactory
  - Runner
  - Endpoint / MuJoCo adapters

Robot Pack
  - robot model
  - actuator config
  - controller config
  - PDU definition
  - manifest
  - Recipe
  - demo / frontend
  - robot-specific controller
```

現在の主要 consumer は `hakoniwa-robot-arm-pack` です。

この分離により、個別案件・個別ロボットで作成するモデルや Recipe と、箱庭ラボが共通資産として保守する Runtime 実装を分離します。

---

## 12. 現在のスコープと拡張方針

現在の正本は joint / scalar actuator を中心とする Runtime です。

検証済みの中心はロボットアームですが、Runtime の core pipeline は特定機種名や Arm 専用 API を前提にしていません。

ただし、「どのロボットにもそのまま対応済み」という意味ではありません。車輪型・脚型・ハンド等へ展開する場合、次を Robot Pack または新しい共通 Component として追加する必要があります。

- robot-specific actuator / state binding
- control semantics
- kinematics / locomotion specific Controller
- model / sensor configuration
-必要に応じた新しい Publisher / Adapter

共通化するときは、まず Robot Pack 側で具体的な要求と実装を成立させ、その後、複数ロボットで再利用可能と確認できた責務だけを Runtime へ昇格させます。

**先に抽象化範囲を広げるのではなく、実利用で共通性が確認できた境界を共通 Runtime に取り込む**ことを基本方針とします。

---

## 13. Build / Integration の現状

現在 `hakoniwa-robot-runtime` は source ownership を持ちますが、top-level CMake composition は consumer 側から行っています。

`hakoniwa-robot-arm-pack` では sibling checkout の `hakoniwa-robot-runtime` を参照し、このリポジトリの source / public header を build target に組み込んでいます。

将来 Runtime 側が独立した CMake target / package export を持つ場合でも、上記の Runtime / Adapter / Runner / Factory の責務境界は維持します。
