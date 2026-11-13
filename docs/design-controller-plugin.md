# 設計：Robot Runtime のプラグイン（制御プラグインとデータ変換プラグイン）

状態：設計案（レビュー用）。今回作るのは共通部分（4 章）と制御プラグイン（5 章）。データ変換プラグイン（6 章）は詳細を決めない。読み込みで種類（`kind`）を見て分けるので、あとで詳細を決めても全体が壊れないことだけを決めておく。

## 1. 目的

Robot Runtime の本体を外部の仕組みに依存させずに、外部の制御やデータの形式を、共有ライブラリ（プラグイン）として同じプロセスの中に足せるようにする。

最初の使い道は ros2_control。ドローンの Control Link と同じ形にする。

```text
ドローン：Control Link のプロセス = 物理 + 制御 = 箱庭アセット
          ROS で流したいデータだけ、そこから pub/sub する

アーム　：Robot Runtime のプロセス = 物理 + ros2_control（プラグイン）= 箱庭アセット
          ROS で流したいデータ（センサー、関節の状態、/clock、MoveIt との軌道のやり取り）だけ、
          プラグインが pub/sub する
```

| | 今の形：ros2_control が別のアセット | この設計：ros2_control を組み込む |
|---|---|---|
| つなぎ方 | ros2_control が別の箱庭アセット（`HakoniwaAssetSystem`）として、PDU で Robot Runtime とつながる | ros2_control をプラグインとして Robot Runtime が読み込み、刻みのたびに呼ぶ |
| 制御の時刻 | 壁時計（controller_manager は実時間の 100 Hz） | シミュレーションの時刻 |
| 速く回したとき | 制御の回数が減り、精度が落ちる | 制御の回数は変わらず、精度も変わらない |
| アセットの数 | 2 つ（実時間で待ち合わせる） | 1 つ |

- MoveIt は、ros2_control の軌道のコントローラと ROS 2 で話すだけなので、組み込みの形だけで足りる。
- 箱庭の共有メモリは、ros2_control との間ではなく、アセットとアセットの間（見た目、ほかのロボット、ドローンなど）で使う。Robot Runtime は今までどおり箱庭アセット。
- 今の形（`HakoniwaAssetSystem`）は、組み込みの形が同じ作業を通ることを確かめるまで、比べる相手として残す。確かめたら「古い形」として新しい手順から外し、消すかどうかはそのあと決める。

## 2. 方針

1. **境界は C だけ**。プラグインとの間を通るのは C の型だけ。C++ のオブジェクト、標準ライブラリの型、例外、片側で確保して反対側で解放するメモリは通さない。Robot Runtime とプラグインを別のコンパイラ・設定で作ってもよい。見本は外部センサープラグイン（`hakoniwa-sensor-adapter` の `external_sensor_plugin.h`）。
2. **境界に外部のデータ型を出さない**。ROS の型やメーカー独自の型は、プラグインの中だけにある。境界を通るのは、Robot Runtime の正規のデータ（4.4）と、箱庭の PDU のバイト列だけ。
3. **ROS に依存するものは `ros2/` の下に閉じる**。境界のヘッダ、読み込みの仕組み、Robot Runtime 側の部品は ROS に依存しない。ros2_control のプラグインは `ros2/hakoniwa_ros2_control/` の下に、別のディレクトリで置く。
4. **時刻を決めるのは Robot Runtime**。いつプラグインを呼ぶかは Robot Runtime が刻みの番号で決める。プラグインは壁時計に頼らない。だから毎回同じ順で呼ばれる。

## 3. プラグインの 2 つの種類

| | 制御プラグイン（5 章、今回作る） | データ変換プラグイン（6 章、詳細は後で決める） |
|---|---|---|
| 役目 | 制御の流れごと引き受ける。状態を受け取り、指令を返す | 処理の流れの入口と出口で、データの形式を変換する。制御はしない |
| 呼ばれるとき | 制御の周期ごと（刻みの整数倍）、シミュレーションのスレッドで | 後で決める |
| 例 | ros2_control、独自の制御則 | メーカー製のサーボ指令の形式を関節の指令にする、状態を独自の形式で外に出す |

2 つの種類は、読み込み（入口の関数、`kind` での分かれ道、版の確認）と、4 章の共通部分を同じにする。関節の指令を作るなら形（4.4 の `HakoJointCommandsV1`）も同じにするので、Robot Runtime から見ると指令の出どころが違うだけになる。

## 4. 共通部分（今回作る）

`include/hakoniwa/robot_runtime/plugin/plugin_common.h`（C）と、読み込みの仕組み（`src/plugin/`）。

### 4.1 入口、種類、版

- 入口の関数は、どの種類のプラグインでも 1 つ（`hako_robot_runtime_plugin_v1`）。入口は、表の先頭（`HakoPluginHeaderV1`）へのポインタを返す。
- 読み込む側は、まず先頭の `kind` を見て、その種類の表として扱う。種類ごとの表は、先頭に `HakoPluginHeaderV1` を置く。

```c
#define HAKO_ROBOT_RUNTIME_PLUGIN_ENTRY_V1 "hako_robot_runtime_plugin_v1"
/* 構造体には型と同じ名前のタグを付ける（前方宣言と同じ型になる） */
typedef const struct HakoPluginHeaderV1* (*HakoRobotRuntimePluginEntryV1)(void);

#define HAKO_PLUGIN_KIND_CONTROLLER   1u
#define HAKO_PLUGIN_KIND_DATA_ADAPTER 2u   /* 予約（6 章） */
```


```c
typedef struct HakoPluginHeaderV1 {
    uint32_t struct_size;          /* この表の大きさ（後ろに足しても古い側が読める） */
    uint32_t abi_version;          /* 種類ごとの ABI の版 */
    uint32_t kind;                 /* HAKO_PLUGIN_KIND_CONTROLLER / _DATA_ADAPTER */
    const char* interface_revision;
    const char* plugin_id;
} HakoPluginHeaderV1;
```

- 読み込む側は、`kind`、`abi_version`、`struct_size` を確かめてから使う。知らない `kind`、合わない版、小さすぎる表は、名前と理由を出して読み込みを断る。
- マニフェストの部品が求める種類（制御の部品なら制御プラグイン）と `kind` が違うときも断る。

### 4.2 寿命と呼び出しの決まり

どの種類も、次の順で呼ぶ。

| 関数 | いつ | どのスレッドで |
|---|---|---|
| `create` | 1 回、シミュレーションが止まっている間 | 作る側のスレッド |
| `on_start` | 開始のとき | シミュレーションのスレッド |
| （種類ごとの関数） | 動いている間 | シミュレーションのスレッド |
| `on_stop` | 停止のとき | シミュレーションのスレッド |
| `reset` | リセットのとき（`create` 直後の状態に戻す） | シミュレーションのスレッド |
| `destroy` | 1 回、最後に | 作る側のスレッド |

- プラグインは自分のスレッドを持ってよい（通信など）。ただし、Robot Runtime の関数は呼ばない。Robot Runtime が渡すのは、4.3 の記録の口だけ。
- 共有ライブラリは、そのライブラリから作ったものが 1 つでも残っている間は閉じない（Drone PRO のプラグインの読み込みと同じ）。

### 4.3 エラーと記録

- どの関数も、成功で 0、失敗で 0 以外を返し、呼び出し側が用意した `error` に文字列を書く（`error_size` で切る）。
- `create` の引数で、記録の口（`void (*log)(void* user, int32_t level, const char* message)` と `user`）を渡す。プラグインの記録は Robot Runtime の記録と同じところに出る。

### 4.4 正規のデータの形

```c
typedef struct HakoJointStatesV1 {                   /* 関節の状態（読み取りだけ、呼び出しの間だけ有効） */
    uint32_t count;
    const double* position;        /* rad または m */
    const double* velocity;
    const double* effort;
} HakoJointStatesV1;

typedef struct HakoJointCommandsV1 {                   /* 関節の指令（呼び出し側が用意し、プラグインが書く） */
    uint32_t count;
    uint8_t* has_command;          /* 0：この関節の指令なし */
    uint8_t* command_type;         /* 0：位置 1：速度 2：力 */
    double* value;
} HakoJointCommandsV1;

typedef struct HakoTickV1 {                   /* 刻みの情報 */
    uint32_t struct_size;
    uint64_t sim_time_ns;
    uint64_t step_index;
    uint64_t physics_step_ns;
    uint8_t  selected;             /* 制御プラグイン：前回この部品の指令が選ばれていたら 1 */
    uint8_t  reset;                /* リセットの直後の最初の呼び出しなら 1 */
} HakoTickV1;

typedef struct HakoPduViewV1 {                   /* 箱庭の PDU のバイト列（呼び出しの間だけ有効） */
    uint32_t channel;              /* create で渡した一覧の番号 */
    uint32_t size;
    const void* data;
    uint64_t sample_time_ns;       /* 測った時刻（シミュレーションの時刻） */
} HakoPduViewV1;
```

- 関節の並びは、`create` で渡す名前の一覧（Robot Runtime のアクチュエータの名前）の順。
- メモリは、確保した側が解放する。境界を通るポインタは、その呼び出しの間だけ有効。

### 4.5 Robot Runtime 側の読み込み

- `PluginLibrary`（新規、`src/plugin/`）：パスの解決（マニフェストからの相対パス）、`dlopen`／`LoadLibrary`、入口の取り出し、4.1 の確認、寿命の管理。どの種類でも使う。
- マニフェストでは、プラグインを使う部品の設定ファイルに、`plugin.path`（共有ライブラリ）と `plugin.config`（プラグイン自身の設定ファイル。中身はプラグインが決める）を書く。

## 5. 制御プラグイン（今回作る）

### 5.1 境界

`include/hakoniwa/robot_runtime/plugin/controller_plugin.h`。入口は 4.1 の共通の入口で、`kind` は `HAKO_PLUGIN_KIND_CONTROLLER`。

```c

typedef struct HakoControllerCreateV1 {
    uint32_t struct_size;
    const char* config_path;              /* プラグイン自身の設定ファイル */
    uint32_t joint_count;
    const char* const* joint_names;
    uint64_t physics_step_ns;
    uint32_t pdu_channel_count;           /* 渡すセンサーの PDU */
    const char* const* pdu_names;
    const char* const* pdu_types;         /* "sensor_msgs/PointCloud2" など */
    void (*log)(void* user, int32_t level, const char* message);
    void* log_user;
} HakoControllerCreateV1;

typedef struct HakoControllerPluginV1 {
    HakoPluginHeaderV1 header;
    void* (*create)(const HakoControllerCreateV1* args, char* error, size_t error_size);
    uint64_t (*update_period_ns)(void* plugin);   /* 物理の刻みの整数倍。0：毎刻み */
    int32_t (*on_start)(void* plugin, char* error, size_t error_size);
    int32_t (*update)(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                      uint32_t pdu_count, const HakoPduViewV1* pdus,
                      HakoJointCommandsV1* commands, char* error, size_t error_size);
    int32_t (*on_stop)(void* plugin, char* error, size_t error_size);
    int32_t (*reset)(void* plugin, char* error, size_t error_size);
    void (*destroy)(void* plugin);
} HakoControllerPluginV1;
```

- `update` は、制御の周期にあたる刻みだけ呼ぶ（刻みの番号が周期の倍数のとき）。周期の間の刻みは、前の指令を使い続ける。
- `pdus` は、前回の `update` から新しくなったセンサーの PDU だけ。PDU の種類ごとに最新の 1 つを、測った時刻つきで渡す。制御より速いセンサー（制御 100 Hz に対する 1 kHz の IMU など）の途中の値は渡らない（ROS のトピックを深さ 1 で受けるのと同じ）。履歴を渡す形は、構造体の大きさで版を広げて後で足す。
- 関節の状態をどこから取るかは、部品の設定（`joint_state_source`）で選ぶ。既定は、別プロセスの形で ros2_control に渡している関節の状態（`joint_states` の出力）と同じ出どころ。ノイズや測る周期を入れた値を渡せば、エンコーダーの観測値で制御できる。どちらの形でも同じ条件で比べられるように、既定をそろえる。
- 制御の横取り：`tick.selected` で、前回この部品の指令が選ばれたかを伝える（Robot Runtime は前の刻みで選ばれた制御を覚えている：`previous_selected_control_id`）。選ばれなかった間も `update` は呼び続け、状態は流し続ける。
- リセット：`reset` のあと最初の `update` は `tick.reset` が 1。プラグインは、今の位置を保つ指令を返す。

### 5.2 Robot Runtime 側：今の流れのまま、4 段目に入れる

Robot Runtime の 1 刻み（`ActuatorRuntime::step()`）は、次の段で動いている。制御プラグインは、この流れを変えずに、4 段目のコントローラの 1 つとして入る。

```text
1. 状態を読む                read_current_state()
2. 刻みの情報を作る          prepare_step_context()
3. 指令の受け口を見る        poll_sources()           ← 将来のデータ変換プラグインの入口はここ
4. コントローラの更新        update_controllers()     ← 制御プラグインはここ（PluginController）
5. 指令を選ぶ                arbiter（優先度で、この刻みの制御のやり方を 1 つ選ぶ）
6. 物理を進める・センサー    step_plant()、sensor_stage->step()
7. 次の刻みへ引き継ぐ        complete_step()
8. 配信                      publish_state()          ← 将来のデータ変換プラグインの出口はここ
```

**受け口のないコントローラとして入る**

- 今のコントローラ（軌道、手動）は、3 段目の受け口（箱庭の PDU を読む）と組になっている。制御プラグインは、自分の入力（ros2_control なら MoveIt からの軌道など、ROS のメッセージ）を、プラグインの中の通信スレッドで自分で受け取る。ROS の型を Robot Runtime の中に入れないため、3 段目の受け口は使わない。
- なので、`PluginController` は受け口の名前（`source_id`）が空のコントローラになる。今の作りは、これをそのまま扱える（確かめたこと）。
  - 組み立て時の確認は、受け口の名前が空でないときだけ受け口の存在を確かめる（`actuator_runtime.cpp` の検査）。
  - 4 段目は、受け口の名前が空なら入力なしで `update` を呼ぶ（`update_controllers()`）。
  - 5 段目は、受け口と関係なく、指令を出したコントローラの優先度で選ぶ（`PriorityCommandArbiter`）。
- プラグインは毎回指令を出す（止まっているときも、今の位置を保つ指令）。より優先度の高い出どころ（手動の割り込みなど）が指令を出した刻みだけ、そちらが選ばれる。今の軌道のコントローラと同じ扱い。
- 同じ優先度のコントローラが同時に指令を出すと「衝突」になり、どれも選ばれない。プラグインと今の軌道のコントローラを同じロボットに置くときは、優先度を変える。
- ROS のメッセージは通信スレッドで刻みと関係なく届き、`update` のときに届いている最新のものが使われる。軌道が始まる刻みは毎回少しずれうる（Gazebo の gz_ros2_control と同じ）。

**足すもの**（1〜8 段の流れには手を入れない）

1. `PluginLibrary`（4.5）：読み込みと種類・版の確認。
2. `PluginController`（`IController` の実装）：受け口の名前は空。周期の刻みだけプラグインの `update` を呼び、それ以外の刻みは前の指令を返す。状態の中のセンサーの値を、配信と同じ部品で PDU にして渡す。
3. マニフェストの読み込み（`type: "plugin"` を読む関数）と、組み立て（`build_runtime()`）の分かれ道 1 つ。優先度は、今の軌道のコントローラと同じくマニフェストで持たせる。
4. 開始と停止の合図：今のコントローラの口には `reset` しかないので、何もしない既定の動きを持つ `on_start` / `on_stop` をコントローラの口に足し、ランナーのイベントからつなぐ。
5. 失敗の扱い：プラグインの `update` の失敗は、コントローラの状態（`ComponentStatus`）で返す。ランナーがこれを止める理由として扱っているかを確かめ、扱っていなければ足す。

- 箱庭あり（Hakoniwa のランナー）でも、箱庭なしでも、同じように動く（プラグインはランナーを知らない）。

**呼び出しの流れ**（実行の主体は Robot Runtime。時刻を進める、実時間に合わせる、箱庭と同期するのは Robot Runtime で、プラグインは呼ばれたときだけ動き、眠ったり待ったりしない）

```text
1. 起動（メインのスレッド）
   マニフェストを読む
     部品 {kind: "controller", type: "plugin", config: "ros2_control_plugin.json"}
   build_runtime() が PluginController を作る
     PluginLibrary：dlopen → 入口の関数 → 表の先頭 → kind と版を確かめる
     table->create(関節の名前, センサーの PDU の名前, 物理の刻み, プラグインの設定)
     table->update_period_ns() → 「5 刻みに 1 回」と決める
   ActuatorRuntime のコントローラの一覧に登録する（指令の優先度つき）

2. 開始（シミュレーションのスレッド）
   table->on_start()

3. 毎刻み（シミュレーションのスレッド。ランナーの on_step → runtime->step()）
   状態を読む → 指令の受け口を見る
   → コントローラの更新
        PluginController::update()
          周期の刻みだけ table->update(時刻, 関節の状態, 新しいセンサーの PDU) → 指令
          それ以外の刻みは、前の指令をそのまま返す
   → 指令を選ぶ → 物理を 1 刻み進める → センサー → 配信
   → （ランナーが）実時間に合わせる、箱庭の時刻を進める

4. 停止・リセット（シミュレーションのスレッド）
   table->on_stop()、table->reset()

5. 終了（メインのスレッド）
   table->destroy() → 最後の 1 つがなくなったら dlclose
```

- プラグインの中のスレッド（ROS 2 の通信、センサーの配信役）は、プラグインが自分で持つ。Robot Runtime の関数は呼ばない。
- 部品の設定ファイル（`ros2_control_plugin.json`）には、プラグインの共有ライブラリの場所、プラグイン自身の設定ファイル（URDF、コントローラの設定）、渡すセンサーの PDU の名前を書く。

```json
{ "id": "ros2_control", "kind": "controller", "type": "plugin",
  "config": "config/controller/ros2_control_plugin.json" }
```

部品の設定ファイルの中身（SO-101 の例：`ros2/hakoniwa_ros2_control/config/so101/plugin/so101-plugin-controller.json`）：

```json
{
  "plugin": { "path": "libhakoniwa_ros2_control_plugin.so", "config": "plugin.yaml" },
  "joints": ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
  "priority": 15,
  "joint_state_source": "truth",
  "pdus": ["lidar_scan", "wrist_wrench"],
  "stop_on_error": true
}
```

| 項目 | 必須 | 意味 |
|---|---|---|
| `plugin.path` | はい | プラグインの共有ライブラリ。相対パスは、この設定ファイルのディレクトリから |
| `plugin.config` | いいえ | プラグイン自身の設定ファイル。`create` にそのまま渡す（中身はプラグインが決める）。相対パスは同じく |
| `joints` | はい | プラグインが状態を受け取り、指令を返す関節（アクチュエータの id）。この順で渡す |
| `priority` | いいえ（既定 10） | 指令を選ぶときの優先度（Arbiter）。大きいほうが勝つ |
| `joint_state_source` | いいえ（既定 `"truth"`） | 関節の状態の出どころ。`"truth"` は物理の真値。`"observed"`（関節センサーの測定値）は未実装で、書くと読み込み時に失敗する |
| `pdus` | いいえ | `update` に渡すセンサーの PDU の名前（センサー付きのビルドのみ）。型はセンサーの出力の設定から決まる。新しくなったときだけ、種類ごとに最新の 1 つを渡す |
| `stop_on_error` | いいえ（既定 `true`） | `true`：プラグインがエラーを返したら（`on_start`、`update`、`on_stop`）、実行を止める。`update` のエラーでは、その刻みの物理を進めない。`false`：エラーは状態として報告し、ほかの制御（Hold Controller など）で続ける |

### 5.3 ros2_control プラグイン

`ros2/hakoniwa_ros2_control/controller_plugin/`（新規）。

- `create`：`rclcpp` を初期化し（プロセスで一度。二つ目は失敗を返す）、ノードと `controller_manager::ControllerManager` を作る。ロボットの記述（URDF）とコントローラの設定は、プラグインの設定ファイルで渡す。`use_sim_time` は真。
- 箱庭用のハードウェアの口（新規の `SystemInterface`）は、プラグインの中の構造体から状態を読み、指令を書く。PDU も箱庭アセットも使わない。
- `update_period_ns`：コントローラの設定の `update_rate` から求める（物理の刻みの整数倍でなければ `create` で失敗）。
- コントローラを動かし始める・止める（`switch_controller`）は、プラグインの中の別のスレッドから頼む。ros2_control では、切り替えは `update()` の中で行われ、頼んだ側はそれが終わるのを待つ。`create` や `on_start` の中（シミュレーションのスレッド）で頼むと、だれも `update()` を回していないので待ちぼうけになる（試作で確かめた：1 秒で時間切れ）。`on_start` は頼みを出すだけで戻り、切り替えは Robot Runtime が刻みごとに呼ぶ `update` の中で済む。
- 試作で確かめたこと：MuJoCo で物理を回すホストが、`ControllerManager` を作るプラグインを `dlopen`（`RTLD_LOCAL`）で読み込み、物理 1000 刻み・制御 200 回を回して、きれいに終われた。ros2_control が実行中に読み込むハードウェアの口とコントローラも、同じプロセスでぶつからなかった（同じ名前で版の違う共有ライブラリはない。MuJoCo は中に組み込んだ部品の名前を外に出さない）。
- `update`：状態を写す → `/clock` を配信 → `read(t, period)` → `update(t, period)` → `write(t, period)` → 指令を写して返す。センサーの PDU は `pdu2ros` で ROS のメッセージにして配信する（受け取り先は使い回す）。
- 通信は、プラグインの中の別のスレッドの executor で回す。コントローラの更新は、シミュレーションのスレッドの `update` の中だけで行う（gz_ros2_control と同じ分け方）。
- 横取りされたとき（`tick.selected` が 0 に変わったとき）：動いている軌道を取り消し、今の位置を保つ（コントローラを止めて動かし直す）。手動の割り込みなどから戻ったとき、古い軌道を再開しない。今の軌道のコントローラ（`JointTrajectoryController`、ほかの制御に切り替わると軌道を取り消す）と同じ挙動にそろえる。
- `reset`：コントローラを止め、受け付け中の MoveIt の軌道を捨てる。次の `update`（`tick.reset` が 1）で、今の状態を受け取ってからコントローラを動かし直し、今の位置を保つ指令を返す。リセット直後に関節が勝手に動かない。

### 5.4 ros2_control とは何で、どう呼ばれるか

- ros2_control の中心は `controller_manager::ControllerManager`（C++ のクラス）。ふだんは `ros2_control_node` という実行ファイルがこれを作り、`read()` → `update()` → `write()` を一定の周期で回している。
- この 3 つは外から呼べる。Gazebo の gz_ros2_control も、自分で `ControllerManager` を作り、物理の刻みのたびに呼んでいる。
- ここでは、ros2_control プラグインが `ControllerManager` を作り、Robot Runtime がプラグインの `update` を呼ぶたびに、プラグインがこの 3 つを呼ぶ。

```text
Robot Runtime（シミュレーションのスレッド）
  PluginController::update()                      ← 制御の周期の刻みだけ
    └─ plugin->update(tick, states, pdus, commands)   C の境界
         └─ ros2_control プラグイン
              ├─ 状態を受け渡し場所に写す
              ├─ ControllerManager::read()   → HakoniwaPluginSystem::read()：受け渡し場所 → 状態の口
              ├─ ControllerManager::update() → コントローラ（joint_trajectory_controller など）
              ├─ ControllerManager::write()  → HakoniwaPluginSystem::write()：指令の口 → 受け渡し場所
              └─ 受け渡し場所の指令を commands に写す
```

- 関節は、ros2_control の「ハードウェアの口」（`hardware_interface::SystemInterface`）を通る。その口（`HakoniwaPluginSystem`、新規）を、プラグインの中の受け渡し場所（`PluginBridge`）につなぐ。PDU も箱庭アセットも使わない。
- 組み込むのは制御（ros2_control）だけ。センサーの配信のしかたは、別プロセスの形と同じ。
  - 今の配信役（`HakoniwaSensorPublisher`：PDU を `pdu2ros` で ROS のメッセージにして `/pdu/<PDU の名前>` に配信する）を、ラッパーの中でそのまま使う。トピックの名前と周期も同じ。
  - 変わるのは PDU の届き方だけ。別プロセスの形では共有メモリから読むが、組み込みの形では ros2_control 側は箱庭アセットではない（アセットでないプロセスは PDU を読めない）ので、Robot Runtime が作ったセンサーの PDU を `update` の引数で受け取る。
  - 配信役は自分のスレッドで変換と配信をする。`update` は、新しい PDU を配信役に置く（最新の値だけを残す）だけで戻る。制御の流れは、点群の変換や配信を待たない。
  - 点群の時刻は、PDU の中の時刻（シミュレーションの時刻）をそのまま使う。
  - センサーは `HakoniwaPluginSystem`（ハードウェアの口）を通さない。ハードウェアの口は、制御の周期で呼ばれる数値の状態と指令だけを扱う。
  - 力センサを ros2_control の状態の口（`force.x` など）として出し、コントローラから読めるようにするのは、必要になったときに足す。
- 確かめたこと（Jazzy）：`ControllerManager(executor, urdf, activate_all, name, ns, options)` で URDF の文字列から作れる。`read(time, period)`、`update(time, period)`、`write(time, period)` は公開されている。細部（引数の既定値など）は、作るときに Jazzy の版で確かめる。

### 5.5 境界を通るもの（SO-101 の例）

| 向き | 境界の形 | SO-101 での中身 | プラグインの中での行き先 |
|---|---|---|---|
| Robot Runtime → プラグイン | `create` の `joint_names` | `shoulder_pan` `shoulder_lift` `elbow_flex` `wrist_flex` `wrist_roll` `gripper`（6 つ） | URDF の `<ros2_control>` の関節と同じ名前で対応づける |
| Robot Runtime → プラグイン | `create` の `pdu_names` / `pdu_types` | `lidar_scan` / `sensor_msgs/PointCloud2`、`wrist_wrench` / `geometry_msgs/WrenchStamped` | 配信先のトピックと、変換の方法を決める |
| Robot Runtime → プラグイン | `update` の `tick` | シミュレーションの時刻、刻みの番号、刻み 2 ms | `/clock`、`read` / `update` / `write` の時刻と周期 |
| Robot Runtime → プラグイン | `update` の `states` | 6 関節の位置・速度・力 | ハードウェアの口の状態（`position`、`velocity`、`effort`） |
| Robot Runtime → プラグイン | `update` の `pdus` | 新しくなったときだけ、LiDAR（2 Hz）と力センサ（50 Hz）の PDU | 配信役が `/pdu/lidar_scan`、`/pdu/wrist_wrench` に配信（別プロセスの形と同じ） |
| プラグイン → Robot Runtime | `update` の `commands` | 6 関節の位置の指令 | ハードウェアの口の指令（`position`）から写す |

制御の周期は、コントローラの設定の `update_rate`（100 Hz）から 10 ms。物理の刻み 2 ms の 5 回に 1 回、`update` が呼ばれる。

### 5.6 実装例

骨組みだけを示す（エラー処理の一部、`on_start` などは省く）。

**(1) プラグインの入口と表**（`ros2/hakoniwa_ros2_control/controller_plugin/src/plugin_entry.cpp`）

```cpp
extern "C" {

static void* create(const HakoControllerCreateV1* args, char* error, size_t error_size);
static uint64_t update_period_ns(void* plugin);
static int32_t update(void* plugin, const HakoTickV1* tick, const HakoJointStatesV1* states,
                      uint32_t pdu_count, const HakoPduViewV1* pdus,
                      HakoJointCommandsV1* commands, char* error, size_t error_size);
/* on_start, on_stop, reset, destroy も同じように */

static const HakoControllerPluginV1 kPlugin = {
    {sizeof(HakoControllerPluginV1), 1u, HAKO_PLUGIN_KIND_CONTROLLER, "2026-11", "hakoniwa_ros2_control"},
    create, update_period_ns, on_start, update, on_stop, reset, destroy,
};

HAKO_PLUGIN_EXPORT const HakoPluginHeaderV1* hako_robot_runtime_plugin_v1(void)
{
    return &kPlugin.header;
}

}  // extern "C"
```

**(2) 刻みごとの `update`**（同じファイル。例外を境界の外に出さない）

```cpp
static int32_t update(void* p, const HakoTickV1* tick, const HakoJointStatesV1* states,
                      uint32_t pdu_count, const HakoPduViewV1* pdus,
                      HakoJointCommandsV1* commands, char* error, size_t error_size)
{
    auto* self = static_cast<Ros2ControlPlugin*>(p);
    try {
        if (tick->reset || (self->was_selected && !tick->selected)) {
            self->hold_current_position(*states);          // 軌道を取り消し、今の位置を保つ
        }
        self->was_selected = tick->selected;
        self->bridge.set_states(*states);                  // 受け渡し場所へ
        for (uint32_t i = 0; i < pdu_count; ++i) {
            self->sensors.post(pdus[i]);                   // 配信役に置くだけ（変換と配信は配信役のスレッド）
        }
        const rclcpp::Time now(static_cast<int64_t>(tick->sim_time_ns), RCL_ROS_TIME);
        self->publish_clock(now);
        self->cm->read(now, self->period);                 // HakoniwaPluginSystem::read()
        self->cm->update(now, self->period);               // コントローラ
        self->cm->write(now, self->period);                // HakoniwaPluginSystem::write()
        self->bridge.get_commands(*commands);              // 受け渡し場所から
        return 0;
    } catch (const std::exception& e) {
        snprintf(error, error_size, "%s", e.what());
        return -1;
    }
}
```

**(3) ハードウェアの口**（`hakoniwa_plugin_system.cpp`。ros2_control が URDF の `<hardware><plugin>` から作る）

```cpp
hardware_interface::return_type HakoniwaPluginSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    const PluginBridge& bridge = PluginBridge::instance();  // プロセスに 1 つ（5.3）
    for (std::size_t j = 0; j < joints_.size(); ++j) {
        position_[j] = bridge.position(joint_index_[j]);
        velocity_[j] = bridge.velocity(joint_index_[j]);
        effort_[j] = bridge.effort(joint_index_[j]);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HakoniwaPluginSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    PluginBridge& bridge = PluginBridge::instance();
    for (std::size_t j = 0; j < joints_.size(); ++j) {
        bridge.set_position_command(joint_index_[j], command_[j]);
    }
    return hardware_interface::return_type::OK;
}
```

- `HakoniwaPluginSystem` は、プラグインと同じ共有ライブラリに入れる。ros2_control（pluginlib）が同じパスのライブラリを開くので、`PluginBridge` は Robot Runtime が開いたものと同じ 1 つになる。
- `read` と `write` は、プラグインの `update` の中、シミュレーションのスレッドでだけ呼ばれる。受け渡し場所にロックは要らない。

**(4) Robot Runtime 側**（`src/runtime/controller/plugin_controller.cpp`）

```cpp
ControllerOutput PluginController::update(std::shared_ptr<const IControllerInput>,
                                          const RobotState& state, const RuntimeStepContext& context)
{
    if (context.step_index % steps_per_update_ != 0) {
        return last_output_;                                // 周期の間は前の指令
    }
    copy_states(joint_state_source_, state, position_, velocity_, effort_);  // 真値か観測値（設定で選ぶ）
    const HakoJointStatesV1 states{count_, position_.data(), velocity_.data(), effort_.data()};
    HakoTickV1 tick{};
    tick.struct_size = sizeof(tick);
    tick.sim_time_ns = context.simulation_time_usec * 1000;
    tick.step_index = context.step_index;
    tick.physics_step_ns = step_ns_;
    tick.selected = context.previous_selected_control_id == control_group_id_;  // 横取りされていないか
    tick.reset = std::exchange(just_reset_, false);
    collect_new_pdus(state, pdus_);                         // 前回から新しくなったセンサーの PDU
    HakoJointCommandsV1 commands{count_, has_.data(), type_.data(), value_.data()};
    char error[256] = {};
    if (table_->update(plugin_, &tick, &states, static_cast<uint32_t>(pdus_.size()), pdus_.data(),
                       &commands, error, sizeof(error)) != 0) {
        return fail(error);
    }
    last_output_ = to_controller_output(commands);         // ActuatorCommand の並びに
    return last_output_;
}
```

## 6. データ変換プラグイン（詳細は決めない）

- `kind` は `HAKO_PLUGIN_KIND_DATA_ADAPTER` を予約する。表の形、呼ばれるとき、受け渡すものは、作るときに決める。
- 決めておくのは、4 章の共通部分（入口、種類、版、寿命、エラーと記録、メモリの決まり、正規のデータの形）に従うことだけ。関節の指令を作る場合は 4.4 の `HakoJointCommandsV1` を使い、Robot Runtime から見て指令の出どころの 1 つになる。
- 今の読み込みの仕組みは、この種類を知らないので断る。作るときに、読み込みの分かれ道を 1 つ足す。

## 7. 時刻と速さ

- 制御は、シミュレーションの時刻で、物理の刻みの整数倍ごとに回る。実時間に合わせるかどうかは、Robot Runtime の既存の設定（`--realtime-sync-cycle-msec`）で決める。速く回しても、制御の回数は変わらない。
- 2 つを分けて言う。
  - **制御の周期は決まっている**（保証する）：どの刻みでプラグインが呼ばれるかは、刻みの番号だけで決まる。
  - **シミュレーション全体が毎回同じ結果になる**（保証しない）：MoveIt からの軌道の受け付けは通信のスレッドで起きるので、軌道が始まる刻みは毎回少しずれうる（Gazebo の gz_ros2_control も同じ）。ここまで同じにしたい場合は、別の課題として扱う。

## 8. 決めたこと（レビューで見直す）

1. センサーの PDU は、制御プラグインの境界を通す（ROS 以外の型を出さずに、組み込みの形でもセンサーを ROS に配信できる）。
2. 制御の周期は、プラグインが申告し、Robot Runtime が物理の刻みの整数倍であることを確かめる。
3. 境界の決まりとしては、プラグインを複数作ってよい。ros2_control プラグインは、自分で 1 プロセスに 1 つに限る。
4. 境界のヘッダは Robot Runtime の `include/` に置く。ほかのシミュレーターでも使うことになったら、センサーの境界と同じく、独立したリポジトリに移す。

## 9. 確かめ方

- 単体：見本の制御プラグイン（ROS なし。目標の位置をそのまま返すもの）で、読み込み、版の確認、周期、指令の受け渡し、PDU の受け渡しを確かめる。
- 結合：横取り（手動の割り込みのあと、古い軌道が再開しないこと）、リセット（直後に関節が動かないこと、時刻の巻き戻しのあと受け付け中の軌道が捨てられること）。
- 結合：SO-101 で、Gazebo との比較（`docs/evaluation-gazebo.md`）と同じ測り方をする。速く回したときの制御の差、毎回同じか、掴んで置く作業、CPU を、別プロセスの形と Gazebo と並べる。
