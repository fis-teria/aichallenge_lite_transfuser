# 発進5%配分モデルのAWSIM到達範囲試験

ユーザーの「一回、AWSIMでどこまで走れるかためしてみましょう」に基づく1回の探索試験。前回の[オフライン判定](time_launch_protection_20260916.md)は不合格のまま保全し、正式採用・自動昇格とは扱わない。

- 実行先: `graneple@192.168.3.10`。
- モデル: `time_launch_protection_v2_20260916/launch_balanced/epoch_03.pt`。SHA256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。
- 設定: `configs/control/time_path_launch_probe_20260917.json`。前回corner試験からcheckpointのSHAとepochだけを変更する。
- 固定目標5km/h、既存PP・先読み・操舵応答補償・速度上限・センサ監視・停止領域監視を維持する。拒否時の制動も保持する。
- 通常RVizにE2Eの生予測経路を表示する。AWSIMの全ファイルを試験前後でハッシュ照合し、AWSIM本体は編集しない。
- 1走行。1周、監視停止または既存の有限上限で終了する。走行上限はsim/wall各600秒、外側720秒。停止確認後、今回のプロセスだけを終了する。
- 到達距離、Judge区間／周回、停止理由、停止付近の予測／制御を確認する。ログのオフライン解析はnative WSLで行う。

## 実行手順

Windowsで設定をcommitし、`tools/sync_to_wsl.ps1`の公式手順で同期する。学習・実行モデルの出力一致と設定をnative WSLのworktree lock内で確認する。既存全pytestの成功記録と今回の実装が同一であることを照合し、新しいcheckpoint/configについて実行側のsmokeを行う。

実行operatorはWindowsの`tmp/time_launch_model_lap_20260917`に配置する。記録用コピーは実行後に証跡へ保存する。

```powershell
python tmp/time_launch_model_lap_20260917/manage.py package
python tmp/time_launch_model_lap_20260917/manage.py prepare
python tmp/time_launch_model_lap_20260917/manage.py start
python tmp/time_launch_model_lap_20260917/manage.py status
python tmp/time_launch_model_lap_20260917/finish_and_evaluate.py
```

## 結果

準備中。実走・解析後に到達距離と停止理由を追記する。
