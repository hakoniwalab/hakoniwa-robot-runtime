# SO-101 で ServoJ のようなサービスを試す

SO-101 の肩を +0.5、-0.5、0 rad に振り、肘を +0.8 rad まで曲げて 0 rad に戻します。
各動作はシミュレーション時間で 1.5 秒です。
クライアントは目標の 0.01 rad 以内に落ち着いたとき、`res=0` と実測角を表示します。

## 1. 準備する

[非同期 RPC の手順書](../../../docs/howto-add-async-service.md) の 0〜2 節で、Foundation、SO-101 モデル、ServoJ の型、各パスを用意します。
要求には `float64[6] joint` と `float64 t`、応答には `int32 res` を使います。
要求の関節配列は SO-101 のマニフェストのアクチュエータ順です。

## 2. C++ プラグインを見る

`servoj_like.cpp` に `ServoJRobot` の実装と、サービス型と入口を定義するマクロがそろっています。
自分のプラグインを書くときは、手順書の [3.1 節](../../../docs/howto-add-async-service.md#31-c-クラスを書く) に沿ってコピーを編集します。
別の `.cpp` を使う場合は、コピー先の `CMakeLists.txt` の `add_library` でソース名を替えます。
`bind` で HakoCpp の要求・応答メソッドを登録します。サービス番号は同じ `pdu_robot` のマニフェスト行の順で、0 から始まります。
`step` はマニフェストのアクチュエータ順に `step.position(i)` で開始角（rad）を読み、位置指令を出します。
時刻は `step.tick().sim_time_ns`（ns）で読みます。
目標角は各アクチュエータの範囲内で受け付けます。
初回は `tick.selected` が 0 です。指令を出したあとに `commanded_ && !tick.selected` が成り立つと、指令を止めます。
`HAKO_ROBOT_PLUGIN` が入口を作り、生成したコンバーターが PDU を読み書きします。
C の関数テーブルを直接使う場合は、手順書の [3.3 節](../../../docs/howto-add-async-service.md#33-c-の関数テーブルを使う上級者向け) を参照してください。

## 3. プラグインをビルドする

独自の CMake でも、サンプルと同じく `cxx_std_20` を指定します。
Robot Runtime の `include`、生成した型のルート、PDU Registry の `pdu/types` をインクルード先に指定します。

```bash
cmake -S "$DEMO" -B "$WORK/build-plugin" -DCMAKE_BUILD_TYPE=Release \
  -DHAKO_SERVOJ_PDU_TYPES="$PDU_TYPES" -DHAKO_PDU_REGISTRY_TYPES="$REGISTRY/pdu/types"
cmake --build "$WORK/build-plugin" --parallel "$JOBS"
PLUGIN="$WORK/build-plugin/libmy_servoj.so"
ls -l "$PLUGIN" "$WORK/build-plugin/libmy_servoj.plugin.json"
```

CMake は `my_servoj.plugin.json` を `libmy_servoj.plugin.json` として共有ライブラリの隣にコピーします。
宣言の `service_types` は `my_robot_srvs_msgs/ServoJ` です。
ライブラリを移すときは宣言ファイルも一緒に移します。

## 4. 腕と RPC をビルドして動かす

手順書の [6.1 節](../../../docs/howto-add-async-service.md#61-pdu-rpc-と-robot-arm-をビルドする) で、
SHM 対応の PDU RPC と、センサを有効にした Robot Arm をビルドします。
手順書の [6.2 節](../../../docs/howto-add-async-service.md#62-動かす) で環境変数を設定し、`run.py --no-viewer` で 5 つの動作を確かめます。
デスクトップで MuJoCo ウィンドウを開くには、新しい実行ディレクトリを指定し、`--no-viewer` を外します。
`run.py` は実行ディレクトリにマニフェスト、RPC 設定、Core のログを書き、動作後に止めます。
