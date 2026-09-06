# README_REVIEW: Tiny新ROS package / CONTROL_METHOD

今回の主題は、新package `aic_tiny_sim_test` / `CONTROL_METHOD=tiny_lidar_net_guarded` の
接続・停止構成と、GUI起動失敗後の修正範囲を確認すること。
**今回の実走・一周は未達。修正版のROS起動も未確認。** 追加駆動の許可書ではない。

1. `report/tiny_gui_control_method_20260907_results.md` を最初に読む。
2. `versions.json` の実行5237612と修正d0b86e3を区別する。
3. `attempts/gui_short_5237612_01/host_summary.json` / `host.jsonl` が失敗・cleanupの一次記録。
4. `budget_before_authorization.json` / `budget_authorization_change.json` / `budget_after.json` を照合する。
   Tiny600/20sim秒は保守計上で、観測推論・走行ではない。旧試行は台帳上のみ含み、旧生ログを再同梱した完全履歴packetではない。
5. `source/` は修正版。`diff/hostname_fix.patch` が実行版から修正版への全差分。
6. `evidence/` は限定98testの2回分、単独packageビルド、非ROS X11 probe、同期の警告と再実行記録。
7. `PACKAGE_MANIFEST.json` は自己除外のfile hash。ZIP hashは外側のreceipt。

実画面動画、重み、raw/sensor/Dataset、旧V4解析物は含めない。
modelのload-only testと実scan forwardを混同しない。
新methodは既存make dev recipeをincludeするが、専用compose・新launchを使用し、通常Autoware制御stackではない。

## Astra Proへ渡すプロンプト

以下を添付ZIPとともに渡してください。

```text
TinyLidarNetの新ROS packageとCONTROL_METHODの限定レビューをお願いします。
リポジトリ: https://github.com/fis-teria/aichallenge_lite_transfuser
branch: codex/windows-wsl-training-sync
今回は自動pushしていないため、未公開差分は添付ZIPのsource/とversions.jsonを正本にしてください。

実行版: 52376121d52a44f678e13a20a87127e1532e203a
修正版・最新98test: d0b86e3cc89bbcecabfd841c7c9a772352043566
新package: aic_tiny_sim_test
新選択肢: CONTROL_METHOD=tiny_lidar_net_guarded

最初にREADME_REVIEW、結果レポート、versions、manifestを読み、次を区別してレビューしてください。
・実装/限定test/単独ROS buildで確認できた範囲
・Docker hostname/network競合で制御container未作成だったGUI attempt
・修正版の非ROS X11接続成功と、未確認の両GUI表示/推論/走行/制動
・実測と保守予算計上（Tiny600/駆動1/20sim秒は実測ではない）

静的確認の重点:
1. 公式Tiny coreの利用、唯一の制御送信元、既存controllerとの競合回避。
2. デスクトップ表示の所有PID・IsViewable確認、Ready fence、scan更新、ARM判定。
3. モデル停止時にも働くhost watchdog、部分起動cleanup、pause後unpauseなし。
4. hostname修正がnetwork共有/Xauthority契約と整合し、AWSIM設定を変更しないこと。
5. 旧予算/失敗/未知消費の保持、powered4/4後に自動再試行しないこと。

不明はUNKNOWNとし、根拠file/関数/ログfieldを付けてください。
HOST_MONITOR_STALEの前回実原因が確定したとは扱わないでください。
V4/S1/参照fit/学習改善の再監査へ広げないでください。
新しい実行・推論・走行はせず、必要なら次の有限短試験の具体的条件を提案してください。
重大な未解決があれば少数に絞り、修正済みと未検証を区別してください。
このレビューやpacketは、追加試行枠・実車制御・期限延長の承認ではありません。
```
