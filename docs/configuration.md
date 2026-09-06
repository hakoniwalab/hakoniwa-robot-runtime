# Robot Runtime Configuration Contract

この文書は、`hakoniwa-robot-runtime` が解釈する設定ファイルの **format / semantics / validation contract** を説明する正本です。

Robot Pack / Application は、各ロボット固有の値を持つ設定インスタンスを所有します。一方、`asset-manifest.json`、Runtime config、actuator / controller / state-output config が **何を意味し、どの参照関係を満たす必要があるか** は本 Runtime の責務です。

```text
Robot Runtime
  owns:
    - configuration format / semantics
    - path resolution rules
    - component cross-reference rules
    - Runtime-side validation

Robot Pack / Application
  owns:
    - concrete robot model
    - concrete config instances / values
    - Recipe / Launcher / demo
    - robot-specific extensions
```

JSON Schema は構文・型・必須項目の静的検証を補助します。Runtime 実装はこれに加えて、ファイル存在、component 間参照、PDU binding、joint / actuator 対応、値の意味的整合などを実行時に検証します。

## 1. Configuration 全体像

Runtime の入口は Asset Manifest です。

```text
asset-manifest.json
  |
  +-- model --------------------------> physical model
  +-- pdu_def ------------------------> PDU definition
  +-- endpoint -----------------------> Hakoniwa PDU Endpoint config
  +-- runtime_config -----------------> Runtime common config
  |
  +-- components[]
       |
       +-- actuator ------------------> actuator config
       +-- controller ----------------> controller config
       +-- state_output --------------> state output config
```

`ManifestFactory` は Asset Manifest の top-level path を解決し、`RuntimeFactory` に Runtime 固有の定義を渡します。

## 2. Asset Manifest

### 2.1 必須 top-level fields

現在の Runtime は次の項目を必須とします。

| field | 意味 |
| --- | --- |
| `name` | Asset / Robot の論理名 |
| `model` | physical model file への path |
| `pdu_def` | PDU definition file への path |
| `endpoint` | Endpoint config file への path |
| `runtime_config` | Runtime common config file への path |
| `components` | Runtime component 定義の配列 |

`model` / `pdu_def` / `endpoint` / `runtime_config` の相対 path は **Asset Manifest 自身のディレクトリ** を基準に解決します。参照先は通常ファイルとして存在する必要があります。

`$schema`、`schema_version`、`description` 等の metadata を付与しても構いませんが、現在の Runtime 実装が直接意味解釈する必須 field は上表です。

### 2.2 `components[]`

各 component は少なくとも次を持ちます。

| field | 意味 |
| --- | --- |
| `id` | manifest 内で一意の component ID |
| `kind` | `actuator` / `controller` / `state_output` 等の分類 |
| `type` | Runtime が解釈する component type |
| `config` | component config file への path |
| `pdu_robot` | PDU binding に使用する robot 名。現在の標準 actuator / controller / state-output で使用 |

`config` の相対 path は **Asset Manifest のディレクトリ** を基準に解決します。

`id` は manifest 内で重複できません。

### 2.3 現在の標準 component type

Runtime が現在標準で解釈する主な component は次です。

| kind | type | 役割 |
| --- | --- | --- |
| `actuator` | `joint_position_actuator` | position command を受ける scalar joint actuator |
| `actuator` | `joint_velocity_actuator` | velocity command を受ける scalar joint actuator |
| `actuator` | `joint_torque_actuator` | effort / torque command を受ける scalar joint actuator |
| `controller` | `joint_trajectory_controller` | `trajectory_msgs/JointTrajectory` を scalar actuator 群へ展開 |
| `controller` | `joy_manual_controller` | `sensor_msgs/Joy` を manual joint command へ変換 |
| `state_output` | `joint_state` | physical state を `sensor_msgs/JointState` として出力 |

Robot Pack 固有の component type は上位側で追加できますが、Runtime core の標準 contract とは分けて管理します。

## 3. Runtime Config

Runtime common config は Asset Manifest の `runtime_config` から参照します。

現在の標準形式は次です。

```json
{
  "$schema": "https://hakoniwa.dev/schemas/robot-arm-actuator-runtime.schema.json",
  "actuators": {
    "joint1": { "command_timeout_sec": 0.1 },
    "joint2": { "command_timeout_sec": 0.1 }
  }
}
```

### 3.1 `actuators`

`actuators` は actuator component ID を key とする object です。

各 scalar actuator について `command_timeout_sec` が必要です。

```text
components[].id
      |
      +--------------------------+
                                 v
runtime_config.actuators.<id>.command_timeout_sec
```

`command_timeout_sec` は command の有効期限を simulation time で表します。正の有限値で、Runtime 内部の microsecond 表現へ変換可能である必要があります。

Runtime は独立した wall-clock を Source of Truth とせず、Plant の simulation time に基づいて command expiration を判定します。

Schema:

```text
schemas/runtime/actuator-runtime.schema.json
```

## 4. Scalar Joint Actuator Config

標準 scalar actuator config は次の3領域を持ちます。

```json
{
  "spec": {
    "joint_name": "joint1",
    "type": "position",
    "limit": { "lower": -3.14, "upper": 3.14 }
  },
  "mjcf_binding": {
    "actuator_name": "joint1_motor"
  },
  "pdu_config": {
    "pdu_name": "actuator/joint1/command",
    "update_rate_hz": 50,
    "message_type": "std_msgs/Float64"
  }
}
```

### 4.1 `spec`

| field | 意味 |
| --- | --- |
| `joint_name` | physical joint 名 |
| `type` | command semantics |
| `limit` | Plant 入口で適用する command limit |

component type と `spec.type` は次の組合せで一致する必要があります。

| component type | `spec.type` | limit |
| --- | --- | --- |
| `joint_position_actuator` | `position` | `lower`, `upper` |
| `joint_velocity_actuator` | `velocity` | 正の `velocity` |
| `joint_torque_actuator` | `torque` | 正の `effort` |

position の `lower < upper`、velocity / effort の magnitude は正の有限値である必要があります。

同じ physical `joint_name` に複数の標準 scalar actuator component を割り当てることはできません。

### 4.2 `mjcf_binding`

`mjcf_binding.actuator_name` は Runtime actuator と MuJoCo actuator を対応付けます。

省略時は現在 `joint_name` を actuator 名として使用します。

この field の意味は現在の MuJoCo adapter に依存します。Runtime core 自体の logical actuator ID と physical backend の binding は分離されています。

### 4.3 `pdu_config`

標準 scalar actuator command は `std_msgs/Float64` を使用します。

| field | contract |
| --- | --- |
| `pdu_name` | `pdu_robot` 内に存在する PDU channel 名 |
| `message_type` | `std_msgs/Float64` |
| `update_rate_hz` | 正の有限値 |

Runtime は PDU definition と照合し、対象 channel の存在、message type、必要最小 PDU size を検証します。

現在 scalar `Float64` channel は 32 byte 以上を要求します。

### 4.4 JSON Schema の配置

Joint actuator schema は現在 `hakoniwa-mujoco-robots` 側にも既存 contract として配置されています。

```text
config/actuator/schema/joint-actuator.schema.json
```

Runtime が実際に解釈する actuator semantics と cross-reference validation は本 Runtime の contract です。Schema ownership / 配置の整理は別途段階的に行います。

## 5. JointTrajectory Controller Config

標準 JointTrajectory Controller の例:

```json
{
  "$schema": "https://hakoniwa.dev/schemas/robot-arm-joint-trajectory-controller.schema.json",
  "input": {
    "pdu_name": "joint_trajectory",
    "update_rate_hz": 50,
    "message_type": "trajectory_msgs/JointTrajectory"
  },
  "joints": [
    { "name": "joint1", "actuator": "joint1" },
    { "name": "joint2", "actuator": "joint2" }
  ]
}
```

### 5.1 `input`

| field | contract |
| --- | --- |
| `pdu_name` | JointTrajectory input channel |
| `message_type` | `trajectory_msgs/JointTrajectory` |
| `update_rate_hz` | 正の有限値 |

### 5.2 `joints`

各 entry は外部から見える joint 名と actuator component ID を結びます。

```text
JointTrajectory joint name
          |
          v
joints[].name
          |
          +--> joints[].actuator
                    |
                    v
             actuator component ID
```

次を満たす必要があります。

- `joints` は1件以上
- `name` は controller 内で一意
- `actuator` は controller 内で一意
- `actuator` は manifest 内の既知 scalar actuator component を参照
- PDU definition 上の channel type が `trajectory_msgs/JointTrajectory`

Schema:

```text
schemas/components/joint-trajectory-controller.schema.json
```

## 6. Joy Manual Controller Config

標準 Joy Manual Controller は、logical Joy layout と joint binding を組み合わせます。

```text
Joy PDU
  |
  v
logical Joy layout
  |
  +-- axes[] / buttons[]
  |
  v
manual controller banks
  |
  v
trajectory joint name
  |
  v
scalar actuator
```

主な領域:

```json
{
  "schema_version": 1,
  "id": "manual-jog-v1",
  "joy_layout": "path/to/logical-joy-v1.json",
  "input": {
    "pdu_name": "joy",
    "message_type": "sensor_msgs/Joy",
    "update_rate_hz": 20.0,
    "timeout_sec": 0.25
  },
  "spec": {
    "manual_enable_button": "l1",
    "quit_button": "options",
    "deadzone": 0.10,
    "expo": 0.35,
    "banks": []
  }
}
```

### 6.1 `joy_layout`

`joy_layout` の相対 path は **Manual Controller config 自身のディレクトリ** を基準に解決します。

現在の Runtime は layout に次を要求します。

- `schema_version: 1`
- `axes`: string の配列
- `buttons`: string の配列
- axis 名 / button 名は空文字不可・重複不可

### 6.2 `input`

| field | contract |
| --- | --- |
| `pdu_name` | Joy input channel |
| `message_type` | `sensor_msgs/Joy` |
| `update_rate_hz` | 正の有限値 |
| `timeout_sec` | 正の有限値。microsecond 表現へ変換可能 |

Runtime は Joy layout の axis / button 数から必要最小 PDU size を算出して PDU definition と照合します。

### 6.3 `spec`

- `manual_enable_button` は Joy layout の既知 button
- `quit_button` は Joy layout の既知 button
- `deadzone` は `0 <= deadzone < 1`
- `expo` は `0 <= expo <= 1`
- `banks` は1件以上
- `select_button: null` の default bank は **ちょうど1件**
- 非 null の `select_button` は Joy layout の既知 button

各 binding は次を満たします。

- `axis` は Joy layout の既知 axis
- `joint` は JointTrajectory Controller が定義する既知 joint
- `velocity_rad_s` は正の有限値
- `invert` は任意の boolean

現在 Manual Controller は JointTrajectory Controller の joint-to-actuator mapping を利用して manual command の actuator を決定します。

Schema:

```text
schemas/components/joy-manual-controller.schema.json
```

## 7. JointState Output Config

標準 state output は `sensor_msgs/JointState` を publish します。

```json
{
  "spec": {
    "type": "joint_state",
    "name": "joint_states",
    "joints": [
      { "name": "joint1" },
      { "name": "joint2" }
    ]
  },
  "mjcf_binding": {
    "joints": [
      { "name": "joint1", "mjcf_joint": "joint1" }
    ]
  },
  "pdu_config": {
    "pdu_name": "joint_states",
    "update_rate_hz": 50.0,
    "message_type": "sensor_msgs/JointState"
  }
}
```

### 7.1 `spec.joints`

- 1件以上
- output joint name は重複不可
- 各 output joint は最終的に既知 scalar actuator の physical joint と対応する必要があります

### 7.2 `mjcf_binding.joints`

任意です。

指定した場合、output 上の `name` を physical MJCF joint 名へ変換します。

```text
output name
   |
   +--> mjcf_binding.joints[].mjcf_joint
             |
             v
       physical joint
             |
             v
       scalar actuator
```

binding が無い joint は、output `name` と physical joint 名が同一であるものとして解釈します。

### 7.3 `pdu_config`

| field | contract |
| --- | --- |
| `pdu_name` | JointState output channel |
| `message_type` | `sensor_msgs/JointState` |
| `update_rate_hz` | 正の有限値 |

Runtime は PDU definition 上の channel の存在と type を照合します。

JointState output schema は現在 `hakoniwa-mujoco-robots` 側にも既存 schema として配置されています。Runtime semantics と cross-reference validation は本 Runtime が所有します。

## 8. PDU Contract

### 8.1 PDU definition format の所有者

PDU definition / PDU types の構文そのものは `hakoniwa-pdu-endpoint` 側で管理します。

Schema directory:

- https://github.com/hakoniwalab/hakoniwa-pdu-endpoint/tree/main/config/schema

現在の compact format に対応する主な schema:

- `pdudef.schema.json`
- `pdutypes.schema.json`

### 8.2 Robot Runtime が要求する contract

`hakoniwa-pdu-endpoint` の schema は legacy / compact の双方を扱いますが、**現在の Robot Runtime の RuntimeFactory は compact PDU definition を要求します**。

概念例:

```json
{
  "paths": [
    { "id": "robot-pdutypes", "path": "robot-pdutypes.json" }
  ],
  "robots": [
    { "name": "Robot", "pdutypes_id": "robot-pdutypes" }
  ]
}
```

`pdutypes` は channel 配列です。

```json
[
  {
    "channel_id": 0,
    "pdu_size": 2048,
    "name": "joint_states",
    "type": "sensor_msgs/JointState"
  }
]
```

Runtime は component config の `pdu_robot` / `pdu_name` と PDU definition を cross-reference し、少なくとも次を確認します。

- `pdu_robot` が `robots[].name` に存在
- `robots[].pdutypes_id` が `paths[].id` に解決可能
- `paths[].path` の pdutypes JSON を読める
- `pdu_name` に対応する channel が存在
- channel `type` が component の期待 message type と一致
- `pdu_size` が component ごとの必要最小サイズ以上

`paths[].path` の相対 path は **PDU definition file のディレクトリ** を基準に解決します。

PDU format 自体の完全な仕様は `hakoniwa-pdu-endpoint` の schema を正本とし、本 Runtime ではここに複製しません。

## 9. Endpoint Contract

### 9.1 Endpoint config format の所有者

Endpoint / Cache / Comm の構文と transport-specific semantics は `hakoniwa-pdu-endpoint` の責務です。

Schema directory:

- https://github.com/hakoniwalab/hakoniwa-pdu-endpoint/tree/main/config/schema

主な schema:

- `endpoint_schema.json`
- `endpoint_container_schema.json`
- `cache_schema.json`
- `comm_schema.json`
- `pdudef.schema.json`
- `pdutypes.schema.json`

### 9.2 Robot Runtime 側の扱い

Asset Manifest の `endpoint` は Endpoint config file を指します。

`ManifestFactory` は参照先の存在を確認した後、その path を `hakoniwa::pdu::Endpoint::open()` へ渡します。

```text
asset-manifest.endpoint
        |
        v
endpoint config path
        |
        v
hakoniwa::pdu::Endpoint::open()
```

そのため、次は `hakoniwa-pdu-endpoint` の責務です。

- `cache` config の構文 / semantics
- `comm` config の構文 / semantics
- SHM / TCP / Zenoh / MQTT 等の transport-specific option
- Endpoint config 内の相対 path 解決

Robot Runtime は Endpoint 内部設定を独自に再定義しません。

## 10. Path Resolution Rules

Runtime と関連 subsystem の相対 path 基準をまとめます。

| path | 基準 directory |
| --- | --- |
| Asset Manifest の `model` | Asset Manifest directory |
| Asset Manifest の `pdu_def` | Asset Manifest directory |
| Asset Manifest の `endpoint` | Asset Manifest directory |
| Asset Manifest の `runtime_config` | Asset Manifest directory |
| `components[].config` | Asset Manifest directory |
| Manual Controller の `joy_layout` | Manual Controller config directory |
| compact PDU definition の `paths[].path` | PDU definition directory |
| Endpoint config の `cache` / `comm` / `pdu_def_path` | Endpoint config directory。詳細は `hakoniwa-pdu-endpoint` schema |

## 11. Validation Boundary

### 11.1 JSON Schema が担当するもの

- JSON structure
- field type
- required property
- primitive range
- `additionalProperties` 制約等

### 11.2 Runtime semantic validation が担当するもの

- referenced file existence
- component ID uniqueness
- component type と actuator `spec.type` の整合
- actuator limit の意味的妥当性
- Runtime timeout と actuator ID の対応
- trajectory joint / actuator uniqueness
- referenced actuator existence
- Manual Joy layout と axis / button / bank の対応
- state-output joint と physical actuator の対応
- PDU robot / channel existence
- PDU message type consistency
- PDU minimum size

Schema validation が通っても Runtime semantic validation が失敗する場合があります。

## 12. 最小構成例

```text
robot/
├── asset-manifest.json
├── model.xml
└── config/
    ├── runtime.json
    ├── pdu/
    │   ├── robot-pdudef.json
    │   └── robot-pdutypes.json
    ├── endpoint/
    │   ├── robot_endpoint.json
    │   ├── cache_buffer.json
    │   └── comm_shm.json
    ├── actuator/
    │   └── joint/
    │       └── joint1.json
    ├── controller/
    │   └── trajectory.json
    └── sensors/
        └── joint_state.json
```

Asset Manifest:

```json
{
  "name": "Robot",
  "model": "model.xml",
  "pdu_def": "config/pdu/robot-pdudef.json",
  "endpoint": "config/endpoint/robot_endpoint.json",
  "runtime_config": "config/runtime.json",
  "components": [
    {
      "id": "joint1",
      "kind": "actuator",
      "type": "joint_position_actuator",
      "config": "config/actuator/joint/joint1.json",
      "pdu_robot": "Robot"
    },
    {
      "id": "trajectory_controller",
      "kind": "controller",
      "type": "joint_trajectory_controller",
      "config": "config/controller/trajectory.json",
      "pdu_robot": "Robot"
    },
    {
      "id": "joint_states",
      "kind": "state_output",
      "type": "joint_state",
      "config": "config/sensors/joint_state.json",
      "pdu_robot": "Robot"
    }
  ]
}
```

この example の具体値は Robot Pack が所有します。各 field の意味と cross-reference contract は本 Runtime が所有します。
