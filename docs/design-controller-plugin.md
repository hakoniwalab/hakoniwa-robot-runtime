# 設計：Robot Runtime のプラグイン（制御プラグインとデータ変換プラグイン）

状態：設計案（レビュー用）。今回作るのは共通部分（4 章）と制御プラグイン（5 章）。データ変換プラグイン（6 章）は詳細を決めない。読み込みで種類（`kind`）を見て分けるので、あとで詳細を決めても全体が壊れないことだけを決めておく。

## 1. 目的

Robot Runtime の本体を外部の仕組みに依存させずに、外部の制御やデータの形式を、共有ライブラリ（プラグイン）として同じプロセスの中に足せるようにする。

最初の使い道は ros2_control。今は ros2_control が別の箱庭アセット（`HakoniwaAssetSystem`）として PDU でつながり、壁時計で回っている。これをプラグインとして Robot Runtime の中に入れ、物理と同じシミュレーションの刻みで回す。

| | 今の形：別プロセス | この設計：組み込み |
|---|---|---|
| つなぎ方 | ros2_control が別の箱庭アセットとして PDU でつながる | ros2_control をプラグインとして Robot Runtime が読み込み、刻みのたびに呼ぶ |
| 制御の時刻 | 壁時計（controller_manager は実時間の 100 Hz） | シミュレーションの時刻 |
| 速く回したとき | 制御の回数が減り、精度が落ちる | 制御の回数は変わらず、精度も変わらない |
| 向いている場面 | 実機に近い構成、ほかのアセットやツールとつなぐ | 速く回す、毎回同じ結果がほしい、たくさん回す |

両方の形を残す。同じロボット、同じコントローラの設定、同じスクリプトのまま、どちらでも動かせることを目標にする（ドローンの SITL と Control Link の関係と同じ）。

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
typedef const struct HakoPluginHeaderV1* (*HakoRobotRuntimePluginEntryV1)(void);

#define HAKO_PLUGIN_KIND_CONTROLLER   1u
#define HAKO_PLUGIN_KIND_DATA_ADAPTER 2u   /* 予約（6 章） */
```


```c
typedef struct {
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
typedef struct {                   /* 関節の状態（読み取りだけ、呼び出しの間だけ有効） */
    uint32_t count;
    const double* position;        /* rad または m */
    const double* velocity;
    const double* effort;
} HakoJointStatesV1;

typedef struct {                   /* 関節の指令（呼び出し側が用意し、プラグインが書く） */
    uint32_t count;
    uint8_t* has_command;          /* 0：この関節の指令なし */
    uint8_t* command_type;         /* 0：位置 1：速度 2：力 */
    double* value;
} HakoJointCommandsV1;

typedef struct {                   /* 刻みの情報 */
    uint64_t sim_time_ns;
    uint64_t step_index;
    uint64_t physics_step_ns;
} HakoTickV1;

typedef struct {                   /* 箱庭の PDU のバイト列（呼び出しの間だけ有効） */
    uint32_t channel;              /* create で渡した一覧の番号 */
    uint32_t size;
    const void* data;
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

typedef struct {
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

typedef struct {
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
- `pdus` は、前回の `update` から新しくなったセンサーの PDU だけ。

### 5.2 Robot Runtime 側

- マニフェストの部品に `kind: "controller"`、`type: "plugin"` を足す。設定ファイルに、4.5 の 2 つと、配信するセンサーの PDU の名前を書く。
- `PluginController`（`IController` の実装、新規）。指令を選ぶときの優先度は、軌道のコントローラと同じ扱い（設定で変えられる）。
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

### 5.3 ros2_control プラグイン

`ros2/hakoniwa_ros2_control/controller_plugin/`（新規）。

- `create`：`rclcpp` を初期化し（プロセスで一度。二つ目は失敗を返す）、ノードと `controller_manager::ControllerManager` を作る。ロボットの記述（URDF）とコントローラの設定は、プラグインの設定ファイルで渡す。`use_sim_time` は真。
- 箱庭用のハードウェアの口（新規の `SystemInterface`）は、プラグインの中の構造体から状態を読み、指令を書く。PDU も箱庭アセットも使わない。
- `update_period_ns`：コントローラの設定の `update_rate` から求める（物理の刻みの整数倍でなければ `create` で失敗）。
- `update`：状態を写す → `/clock` を配信 → `read(t, period)` → `update(t, period)` → `write(t, period)` → 指令を写して返す。センサーの PDU は `pdu2ros` で ROS のメッセージにして配信する（受け取り先は使い回す）。
- 通信は、プラグインの中の別のスレッドの executor で回す。コントローラの更新は、シミュレーションのスレッドの `update` の中だけで行う（gz_ros2_control と同じ分け方）。
- `reset`：コントローラを止めて状態を戻し、動かし直す。

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
    copy_states(state, position_, velocity_, effort_);      // アクチュエータの並びで
    const HakoJointStatesV1 states{count_, position_.data(), velocity_.data(), effort_.data()};
    const HakoTickV1 tick{context.simulation_time_usec * 1000, context.step_index, step_ns_};
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
- MoveIt からの軌道の受け付けは通信のスレッドで起きるので、軌道が始まる刻みは毎回少しずれうる（Gazebo も同じ）。ここまで同じにしたい場合は、別の課題として扱う。

## 8. 決めたこと（レビューで見直す）

1. センサーの PDU は、制御プラグインの境界を通す（ROS 以外の型を出さずに、組み込みの形でもセンサーを ROS に配信できる）。
2. 制御の周期は、プラグインが申告し、Robot Runtime が物理の刻みの整数倍であることを確かめる。
3. 境界の決まりとしては、プラグインを複数作ってよい。ros2_control プラグインは、自分で 1 プロセスに 1 つに限る。
4. 境界のヘッダは Robot Runtime の `include/` に置く。ほかのシミュレーターでも使うことになったら、センサーの境界と同じく、独立したリポジトリに移す。

## 9. 確かめ方

- 単体：見本の制御プラグイン（ROS なし。目標の位置をそのまま返すもの）で、読み込み、版の確認、周期、指令の受け渡し、PDU の受け渡しを確かめる。
- 結合：SO-101 で、Gazebo との比較（`docs/evaluation-gazebo.md`）と同じ測り方をする。速く回したときの制御の差、毎回同じか、掴んで置く作業、CPU を、別プロセスの形と Gazebo と並べる。
