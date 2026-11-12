# 設計：Robot Runtime のプラグイン（制御プラグインとデータ変換プラグイン）

状態：設計案（レビュー用）。今回作るのは制御プラグイン（5 章）と共通部分（4 章）。データ変換プラグイン（6 章）は、置き場所と形だけを決めておく。

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

| | 制御プラグイン（5 章、今回作る） | データ変換プラグイン（6 章、後で作る） |
|---|---|---|
| 役目 | 制御の流れごと引き受ける。状態を受け取り、指令を返す | 処理の流れの入口と出口で、データの形式を変換する。制御はしない |
| 呼ばれるとき | 制御の周期ごと（刻みの整数倍）、シミュレーションのスレッドで | データが来たとき（入口）、データを出すとき（出口） |
| 受け取るもの | 関節の状態、センサーの PDU | 入口：外部の形式のバイト列。出口：関節の状態、センサーの出力 |
| 返すもの | 関節の指令 | 入口：関節の指令（など）。出口：外部の形式のバイト列 |
| 例 | ros2_control、独自の制御則 | メーカー製のサーボ指令の形式を関節の指令にする、状態を独自の形式で外に出す |

2 つの種類は、指令の形（4.4 の `HakoJointCommandsV1`）を共通にする。制御プラグインが返す指令と、データ変換プラグインが入口で作る指令は同じ形なので、Robot Runtime から見ると指令の出どころが違うだけになる。

## 4. 共通部分（今回作る）

`include/hakoniwa/robot_runtime/plugin/plugin_common.h`（C）と、読み込みの仕組み（`src/plugin/`）。

### 4.1 入口と版

- 種類ごとに入口の関数が 1 つある（`hako_controller_plugin_v1` など）。入口は、関数の表へのポインタを返す。
- 表の先頭は、どの種類も同じ形にする。

```c
typedef struct {
    uint32_t struct_size;          /* この表の大きさ（後ろに足しても古い側が読める） */
    uint32_t abi_version;          /* 種類ごとの ABI の版 */
    uint32_t kind;                 /* HAKO_PLUGIN_KIND_CONTROLLER / _DATA_ADAPTER */
    const char* interface_revision;
    const char* plugin_id;
} HakoPluginHeaderV1;
```

- 読み込む側は、`kind`、`abi_version`、`struct_size` を確かめてから使う。合わなければ、名前と理由を出して止める。

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

`include/hakoniwa/robot_runtime/plugin/controller_plugin.h`。

```c
#define HAKO_CONTROLLER_PLUGIN_ENTRY_V1 "hako_controller_plugin_v1"

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

### 5.3 ros2_control プラグイン

`ros2/hakoniwa_ros2_control/controller_plugin/`（新規）。

- `create`：`rclcpp` を初期化し（プロセスで一度。二つ目は失敗を返す）、ノードと `controller_manager::ControllerManager` を作る。ロボットの記述（URDF）とコントローラの設定は、プラグインの設定ファイルで渡す。`use_sim_time` は真。
- 箱庭用のハードウェアの口（新規の `SystemInterface`）は、プラグインの中の構造体から状態を読み、指令を書く。PDU も箱庭アセットも使わない。
- `update_period_ns`：コントローラの設定の `update_rate` から求める（物理の刻みの整数倍でなければ `create` で失敗）。
- `update`：状態を写す → `/clock` を配信 → `read(t, period)` → `update(t, period)` → `write(t, period)` → 指令を写して返す。センサーの PDU は `pdu2ros` で ROS のメッセージにして配信する（受け取り先は使い回す）。
- 通信は、プラグインの中の別のスレッドの executor で回す。コントローラの更新は、シミュレーションのスレッドの `update` の中だけで行う（gz_ros2_control と同じ分け方）。
- `reset`：コントローラを止めて状態を戻し、動かし直す。

## 6. データ変換プラグイン（後で作る。形だけ決める）

`include/hakoniwa/robot_runtime/plugin/data_adapter_plugin.h`（後で）。

- 入口（外 → 内）：外部の形式のバイト列を受け取り、関節の指令（4.4 の `HakoJointCommandsV1`）などにする。今の指令の受け口（軌道や手動の指令の受け口）と同じ位置に入り、指令の出どころの 1 つになる。
- 出口（内 → 外）：関節の状態やセンサーの出力を受け取り、外部の形式のバイト列にする。今の状態の出力と同じ位置に入る。
- 運ぶのは Robot Runtime（PDU、非同期の RPC）。プラグインは形式の変換だけをし、通信の手段を知らない。
- 外から次々に届く指令（メーカー製のサーボ指令など）を受けるには、非同期の RPC の受け口が整っている必要がある。データ変換プラグインは、その後に作る。

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
