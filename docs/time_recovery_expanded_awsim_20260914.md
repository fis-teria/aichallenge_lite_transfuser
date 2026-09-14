# 復帰データ統合モデルのAWSIM比較

ユーザー依頼により、`graneple@192.168.3.10`で追加学習前後の復帰性能を確認する。
Windowsを編集正本とし、Linuxでの試験・保存記録の解析はnative WSLを使用する。
AWSIM実行は指定された`.10`で行い、既存のdirty checkoutと過去の実験を保全する。

## 固定する条件と実行順序

- 追加前checkpoint SHA256: `c2fdb6fd1daf525524fe44d3f793d84b482760aa1f9f1fa84d5315222ab9fbe0`。
- 追加後checkpoint SHA256: `7ec445b6ab955e76d0d71efbd8f2f1dd3066ebe365516df38df8cf18adb56f44`。
- 生の時間軌道30点、固定目標5km/h、`stopping_preview_segment_v1`、
  `awsim_understeer_v1`、既存の操舵応答補償・停止領域監視を共通にする。
- まず通常のE2E走行を各1回。1周、監視停止、進捗停止、既存600s上限のいずれかまで記録する。
- 次に左右各1回ずつ両モデルを比較する。教師PPで同じコーナーへ進み、既存の有限操舵パルスで
  小さな横ずれ・向きずれを作った後、E2Eへ一度だけ制御を渡す。引継ぎ後はE2Eが走行する。
  教師が準備した区間とモデル自身の走行区間は明示的に分け、教師区間をE2E完走と数えない。
- 初回は通常2回＋左右復帰4回を上限とし、走行失敗の無条件再試行や監視条件の緩和はしない。
  準備・接続失敗は原因を確認して修正し、走行成績に混ぜない。
- 走行経路は通常のAutoware RVizに`/visualization/time_path/raw_path`を表示する。
  参照経路・評価用自己位置はモデル入力へ加えない。

## 記録と判定

走行前にsource/install/checkpoint/sceneのhash、GPU、ROS publisherの所有権、RVizの購読、
空き容量を確認する。隔離ROSの実モデルPath・異常制動smokeを実行し、simulation起動前に閉じる。
閉じた走行記録をWSLへ移してhashを確認し、同じ制御計算の再生と復帰区間の横ずれ・向き・
復帰時間・停止理由を集計する。制御が成立したことだけで復帰成功とは判定しない。
新しい実行記録を学習splitへ自動追加せず、元のtestと評価予約は未使用のまま保持する。

実行前の環境はGPU正常、動作中containerなし、空き約22GB。
既存checkoutのHEADは`4af395eee10f928c7fc7225760adfa04c4c07ff4`で、既存の編集を保持する。
新しい専用deploymentを使い、今回のowned process/containerだけを終了する。

実行コマンド・実測結果・未解決事項は以下に追記する。
