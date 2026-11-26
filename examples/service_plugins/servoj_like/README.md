# SO-101 で ServoJ のようなサービスを試す

SO-101 の肩を +0.5、-0.5、0 rad に振り、肘を +0.8 rad まで曲げて 0 rad に戻します。
各動作はシミュレーション時間で 1.5 秒です。
クライアントは目標の 0.01 rad 以内に落ち着いたとき、`res=0` と実測角を表示します。

## 1. 準備する

[非同期 RPC の手順書](../../../docs/howto-add-async-service.md) の 0〜2 節で、Foundation、SO-101 モデル、ServoJ の型、各パスを用意します。
要求には `float64[6] joint` と `float64 t`、応答には `int32 res` を使います。
要求の関節配列は SO-101 のマニフェストのアクチュエータ順です。

## 2. プラグインをビルドする

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

## 3. 腕と RPC をビルドして動かす

手順書の [6.1 節](../../../docs/howto-add-async-service.md#61-pdu-rpc-と-robot-arm-をビルドする) で、
SHM 対応の PDU RPC と、センサを有効にした Robot Arm をビルドします。
手順書の [6.2 節](../../../docs/howto-add-async-service.md#62-動かす) で環境変数を設定し、`run.py --no-viewer` で 5 つの動作を確かめます。
デスクトップで MuJoCo ウィンドウを開くには、新しい実行ディレクトリを指定し、`--no-viewer` を外します。
`run.py` は実行ディレクトリにマニフェスト、RPC 設定、Core のログを書き、動作後に止めます。
