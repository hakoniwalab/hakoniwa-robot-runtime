# SO-101 で ServoJ のようなサービスを試す

SO-101 の肩を +0.5、-0.5、0 rad に振り、肘を +0.8 rad まで曲げて 0 rad に戻します。
各動作はシミュレーション時間で 1.5 秒です。
クライアントは目標の 0.01 rad 以内に落ち着いたとき、`res=0` と実測角を表示します。
続いて Stop と GetAngle を使い、肩の途中停止、保持、JointState との一致、再始動、無効な停止モード、時刻の増加を確認します。

## 1. 準備する

[非同期 RPC の手順書](../../../docs/howto-add-async-service.md) の 0〜2 節で、Foundation、SO-101 モデル、各パスを用意します。
ServoJ、Stop、GetAngle の PDU 型は同じ `my_robot_srvs_msgs` パッケージに生成します。
関節配列は SO-101 のマニフェストのアクチュエータ順です。

## 2. C++ プラグインを見る

`servoj_like.cpp` に `ServoJRobot` の実装と、サービス型と入口を定義するマクロがそろっています。
自分のプラグインを書くときは、手順書の [3.1 節](../../../docs/howto-add-async-service.md#31-c-クラスを書く) に沿ってコピーを編集します。
別の `.cpp` を使う場合は、コピー先の `CMakeLists.txt` の `add_library` でソース名を替えます。
`bind` でサービス型に HakoCpp の要求・応答メソッドを登録します。
`step` で使うサービス番号は `bind` が返します。

```cpp
servo_service = bind<ServoJService>(&ServoJRobot::on_goal, &ServoJRobot::reply_goal);
stop_service = bind<StopService>(&ServoJRobot::on_stop, &ServoJRobot::reply_stop);
get_angle_service = bind<GetAngleService>(&ServoJRobot::on_get_angle, &ServoJRobot::reply_get_angle);
```

`step` はマニフェストのアクチュエータ順に `step.position(i)` で開始角（rad）を読み、位置指令を出します。
時刻は `step.tick().sim_time_ns`（ns）で読みます。
Stop の `mode = 0` は補間を打ち切り、そのステップの物理更新前の角度を保持目標にします。それ以外のモードは `INVALID` です。
GetAngle は同じく物理更新前の角度とそのステップの時刻を保存し、ステップの末尾に返します。
目標角は各アクチュエータの範囲内で受け付けます。
初回は `tick.selected` が 0 です。指令を出したあとに `commanded_ && !tick.selected` が成り立つと、指令を止めます。
公開する型は `HAKO_ROBOT_PLUGIN` に並べます。

```cpp
HAKO_ROBOT_PLUGIN(ServoJRobot, servoj_like, ServoJService, StopService, GetAngleService)
```

左右の腕などで同じ型を複数使う場合は `bind<Service>(index, …)` で区別します。
番号は、そのロボットの `services` の先頭が 0 です。
C の関数テーブルを直接使う場合は、手順書の [3.3 節](../../../docs/howto-add-async-service.md#33-c-の関数テーブルを使う上級者向け) を参照してください。

## 3. プラグインをビルドする

独自の CMake でも、サンプルと同じく `cxx_std_20` を指定します。
Robot Runtime の `include`、生成した型のルート、PDU Registry の `pdu/types` をインクルード先に指定します。

```bash
cmake -S "$DEMO" -B "$WORK/build-plugin" -DCMAKE_BUILD_TYPE=Release \
  -DHAKO_SERVOJ_PDU_TYPES="$PDU_TYPES" -DHAKO_PDU_REGISTRY_TYPES="$REGISTRY/pdu/types"
cmake --build "$WORK/build-plugin" --parallel "$JOBS"
PLUGIN="$WORK/build-plugin/libmy_servoj.so"
ls -l "$PLUGIN"
```

## 4. 腕と RPC をビルドして動かす

手順書の [6.1 節](../../../docs/howto-add-async-service.md#61-pdu-rpc-と-robot-arm-をビルドする) で、
SHM 対応の PDU RPC と、センサを有効にした Robot Arm をビルドします。
手順書の [6.2 節](../../../docs/howto-add-async-service.md#62-動かす) で環境変数を設定し、`run.py --no-viewer` で 5 つの動作と S1〜S6 を確かめます。
デスクトップで MuJoCo ウィンドウを開くには、新しい実行ディレクトリを指定し、`--no-viewer` を外します。
`run.py` は実行ディレクトリにマニフェスト、RPC 設定、Core のログを書き、動作後に止めます。

```json
"robots": [
  {
    "name": "SO101", "plugin": { "library": "/abs/path/to/libmy_servoj.so" }, "priority": 16,
    "services": [
      { "name": "ArmService/ServoJ/SO101", "lockstep": false },
      { "name": "ArmService/Stop/SO101", "lockstep": false },
      { "name": "ArmService/GetAngle/SO101", "lockstep": false }
    ]
  }
]
```

`run.py` はサービス名と型宣言から Core の `service.json` を生成します。
`PDU_OUT` に `pdu_size/` がある生成先を指定します。
設定ファイルの生成だけなら、次のコマンドを使います。

```bash
"$PYTHON" "$RUNTIME/tools/gen_rpc_service_config.py" \
  --manifest "$RUN_DIR/asset-manifest.json" --pdu-out "$PDU_OUT" \
  --clients "$RUN_DIR/clients.json" --output "$RUN_DIR/service.json"
```

Core の `service.json` はサービスごとにクライアントを 1 件登録します。各 `baseSize` は生成した `pdu_size` に 8 バイトを足した値です。

| サービス | `server.baseSize` | `client.baseSize` |
| --- | ---: | ---: |
| ServoJ | 336 | 280 |
| Stop | 280 | 280 |
| GetAngle | 280 | 336 |

`lockstep: true` を設定できるのは、シミュレーションの時計を進めるクライアントだけです。
ロックステップのサービスは毎ステップ送る必要があり、送らないとシミュレーションが止まります。
この例は 3 サービスとも非ロックステップです。Stop と GetAngle は非ロックステップのまま使います。
非ロックステップの要求には次に実行されるステップの末尾で応答します。
Stop が効くステップは到着時刻で決まるため、実行間で同じ停止角は再現できません。GetAngle は物理状態を変えません。
