# `runtime_config` の現在の位置づけ

`runtime_config` は現在の `hakoniwa-robot-runtime` では必須の設定ファイルです。ただし、独立した設定ファイルとしての責務はまだ限定的であり、今後の Runtime 設計に応じて再評価する対象とします。

## 現在の役割

現行実装では、`runtime_config` が保持する Runtime-side setting は scalar actuator ごとの `command_timeout_sec` です。

```json
{
  "$schema": "https://hakoniwa.dev/schemas/robot-arm-actuator-runtime.schema.json",
  "actuators": {
    "joint1": { "command_timeout_sec": 0.1 },
    "joint2": { "command_timeout_sec": 0.1 }
  }
}
```

この値は未使用の metadata ではありません。RuntimeFactory で `RuntimeActuatorConfig.command_timeout_usec` に解決され、`ScalarPduCommandSource` が command の有効期限を simulation time 上で決めるために使用します。

## Asset Manifest との関係

`runtime_config.actuators` の key は physical joint name ではなく、Asset Manifest の actuator component `id` です。

```text
asset-manifest.json
  components[]
    id: shoulder_actuator
    kind: actuator
    type: joint_position_actuator
            |
            +----------------------------+
                                         v
runtime_config
  actuators.shoulder_actuator.command_timeout_sec
```

component の identity / composition の Source of Truth は Asset Manifest です。`runtime_config` は、その component ID を参照して Runtime 上の制御挙動を追加設定します。

そのため現状では、actuator component ID が Asset Manifest と `runtime_config` の双方に現れます。これは現在の設計上の重複であり、意図的に固定化するものではありません。

## Component Config との考え方の違い

概念上は次のように分けます。

```text
Asset Manifest
  = 何が存在するか / component identity と composition

Component Config
  = その component は何者か / physical・I/O binding

Runtime Config
  = Runtime がその component をどう扱うか
```

現在の `command_timeout_sec` は「component の物理仕様」ではなく、受信した scalar command を Runtime がどれだけ有効として扱うかという制御挙動です。この意味では Runtime-side setting として妥当です。

一方、現在この設定しか存在しないため、独立した `runtime_config` ファイルを維持する必要性までは確定していません。

## 将来の判断

将来、実際に Runtime-wide / cross-component な設定が必要になった場合は、独立した `runtime_config` を維持する意味が強くなります。候補としては、watchdog、arbitration、fallback、その他 Runtime 全体の実行・制御設定などがあります。

逆に、そのような要求が現れず `command_timeout_sec` が component-local な唯一の設定であり続ける場合は、actuator component config への統合と `runtime_config` 自体の削除を検討できます。

仮説上の拡張性だけを理由に format を維持・拡張せず、具体的な Runtime-wide configuration requirement が現れた時点で再評価します。

追跡 Issue: [#4 Reevaluate the role of runtime_config](https://github.com/hakoniwalab/hakoniwa-robot-runtime/issues/4)

現在有効な format / semantics は [`configuration.md`](configuration.md) を正本とします。この文書は、その独立ファイルとしての設計判断が未確定であることを補足する design note です。
