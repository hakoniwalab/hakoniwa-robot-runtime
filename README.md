# hakoniwa-robot-runtime

`hakoniwa-robot-runtime` は、箱庭上でアクチュエータ型ロボットを実行するための共通 Runtime です。

ロボット固有のモデルや Recipe を持つのではなく、外部入力を受け取り、制御候補を生成・選択し、物理 Plant へ適用し、更新後の状態を外部へ返す実行層を提供します。

現在の Runtime core は、次の5つの責務で構成されています。

```text
CommandSource -> Controller -> Arbiter -> Plant -> Publisher
```

`ActuatorRuntime` はこれらを 1 simulation step の順序で実行する orchestrator です。

## 位置づけ

```text
Robot Pack / Application
        |
        v
   ManifestFactory
        |
        +--> RuntimeFactory
        |       |
        |       v
        |  ActuatorRuntime
        |       |
        |       v
        |  Source -> Controller -> Arbiter -> Plant -> Publisher
        |
        +--> Endpoint / Physics Adapters
        |
        +--> Runner
                |
                v
          Hakoniwa Asset
```

このリポジトリが担当するのは、主に以下です。

- Runtime core
- RuntimeDefinition / RuntimeFactory
- Hakoniwa Runner
- Hakoniwa PDU Endpoint adapter
- MuJoCo Plant adapter
- Manifest から Runtime / Adapter / Runner を組み立てる ManifestFactory
- Asset Manifest / Runtime config / actuator / controller / state-output の **format / semantics / cross-reference validation**

一方、次は Robot Pack や上位アプリケーション側の責務です。

- ロボット固有の MuJoCo モデル
- Runtime contract に従う **具体的な config instance / robot-specific value**
- Recipe / demo / Launcher 構成
- ROS 2 bridge
- Gamepad frontend
- ロボット固有の追加 Controller

PDU Definition / Endpoint config の構文そのものは `hakoniwa-pdu-endpoint` 側の contract を利用します。本 Runtime は、それらを独自に再定義せず、Runtime component から見た binding 条件だけを定義・検証します。

本 Runtime は、これらの Robot Pack / Application から共通実行層として利用することを想定しています。

## Documentation

- [docs/design.md](docs/design.md) — Runtime architecture / responsibility boundary
- [docs/configuration.md](docs/configuration.md) — Asset Manifest、Runtime config、actuator / controller / state-output、PDU / Endpoint contract
- [schemas/README.md](schemas/README.md) — Runtime-owned JSON Schemas

## 設計上の重要点

- Runtime は独立した simulation clock を持ちません。
- 物理 Plant の時刻と timestep を Runtime の Source of Truth とします。
- Controller は物理 backend に直接依存しません。
- Arbiter は AUTO / MANUAL / HOLD のような mode state を保持せず、現在 step の logical control を選択します。
- command type / expiration / finite value / limit / fallback などの最終 guard は Plant 入口で行います。
- transport、physics backend、execution policy は Runtime core から分離します。

## 現在のスコープ

現在の実装は joint / scalar actuator を中心とした Runtime を正本としています。ロボットアームで実運用・検証されていますが、Runtime 自体は特定のロボット機種やアーム専用 API を前提にしない構造です。

車輪型・脚型などへの展開は可能な設計ですが、各ロボット固有の運動学・制御方式・モデル構成まで本 Runtime が提供するものではありません。

## License

MIT License
