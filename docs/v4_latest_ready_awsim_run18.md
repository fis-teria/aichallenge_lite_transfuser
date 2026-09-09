# Latest-ready修正版 AWSIM再試験18

## 結果

2026-09-09、1回実施。**走行開始前にV4のpose同時刻競合検査で停止。完走は未達。**
AWSIM状態記録はspawnedのみ、Start/Finishはない。
実速度最大絶対値は約3.79e-7m/sで、走行成功やブレーキ停止成功ではない。
経路4件が出たことを、走行中の性能改善とは扱わない。

## 使用版・実行

- Windows正本 `E:/workspace/e2e_lite_transfuser`、branch `codex/windows-wsl-training-sync`。
- 実行commit `c4f7a82088e2f58a64910c3bb81e7b05367d04e8`、join変更は `513a4ec`。
- 専用host `graneple@192.168.3.10`、既存repo
  `/home/graneple/git/autononous_ai/aichallenge-racingkart` のdirtyを保全。
- 配布先 `/home/graneple/e2e_autonomous/v4_pp_shadow_18`。
- source.tar SHA256 `137f1cb694f25eebd8d30abc8641ce579825426ae948d182cbe81f12263316e0`、
  Windows/remote一致。join source SHA256
  `07c4a33ebef06faeb007daaf7184dbf32e0ba52c5bd1e1e11912896762aa3030`。
- 固定image `sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7`。
  network noneのcolcon build: 1 package、1.21s、CONFIG_OK_NO_INFERENCE。
- 既定CheckOnly/同期成功。Dataset固定root存在判定のみ、内容未読。
  最初のCheckOnlyは前回生成したWSL JUnitの未追跡状態で停止。
  自分が生成したJUnitだけを `runs/v4_latest_ready_513a4ec/junit.xml` へ保全移動し、再実行した。
  WindowsのJUnitは従来の `tmp/v4_latest_ready_junit.xml` に保持。

```bash
# 前回の所有instance専用wrapperを新run IDへ変更して1回実行。
timeout --signal=TERM --kill-after=15s 300s python3 /home/graneple/e2e_autonomous/v4_pp_shadow_18/run.py
# wrapper内部。通常make dev、PPが制御し、V4はshadowのみ。
make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1 \
  RUN_ID=v4_pp_shadow_18 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/v4_pp_shadow_18/evidence
```

前回同様、外側300wall秒、駆動240sim秒以内、1周Finishで終了する1試行。
旧budget/usedは保持し、新budgetへ累積を引き継いだ。forward10000の保守予約は
実推論数4とは異なる。実wall31.6384秒、log7,052,859bytes。

開始前に稼働container/ROS/AWSIMなし、hostの/dev/vcuと/dev/gnssなし、
現composeのmount/deviceとDDSのlo限定を確認。既存host network/privileged構成は維持。
observerは駆動前に起動。最終command送信元は既存 `/simple_pure_pursuit_node` を監視。
V4の制御送信なし。AWSIM本体・scene・sensor・PP設定は変更していない。

## 停止原因

`JOIN_FAULT role=pose header_ns=1029999976 epoch=0 reason=CONFLICTING_TIMESTAMP:pose`
の後、workerがValueErrorで終了し、SESSION_END `WORKER_EXIT:1`。
hostは `V4_ENDED` で所有AWSIMをfreeze/KILLし終了した。
最後のobserver INPUT_STALEとTRANSPORT_CLOSEDは終了に伴うもので、一次原因とは分ける。

独立observerにも同時刻1.029999976sim秒のOdometryが2件あり、xは
`89631.41620806887` と `89631.41620806868` m、yは両方 `43127.80851333334` m。
x差は約1.9e-10mと極小だが、新しい検査は完全一致でなければ競合停止する。
これは実際に大きな位置ずれが発生した証拠ではない。
observerのtraceはyawを保存しておらず、V4が比較した全pose値の差はUNKNOWN。

今回の失敗は「最新成立入力を選べない」ではなく、その前段に追加した
**同時刻poseの完全一致検査が実際のlocalization出力で停止を起こすこと**。
次は同一送信元・同一headerのposeをどう扱うか（再通知／推定更新）を明確にし、
既に確定した入力を遡及変更せず、availabilityを保持する限定処理を検討する。
無根拠な許容差や、新しいtimestampへの書換えは今回追加していない。

## 推論・遅延（停止中の少数例）

MODEL_LOADED cuda:0、RTX4060 Laptop GPU、PyTorch2.3.1+cu121。
FORWARD_STARTED4、PLAN4。source時刻0.310、0.730、0.835、0.940sim秒。
4件のcamera受信→joinは24.24 / 68.79 / 23.04 / 31.76ms。
推論呼出しは382.96 / 45.35 / 26.79 / 54.74ms（初回を含む）。
SUPERSEDED_BY_READY4件。最新成立候補の選択は実入力で動作した。
JOIN_WAIT15、JOIN_FAULT1。短い停止中の4件なので、前run17の一周366件と比較して
改善率や定常Hzを断定しない。

## 保存・終了確認

Windows `tmp/v4_pp_shadow_18/` とremote同名runにsource、run.py、設定、budget、
shadow.jsonl、motion.jsonl、vehicle_states.jsonl、make/ROS/observerログ、resultを保存。
raw resultはFAILED/BOUNDED_MONITOR_STOP_DURING_MAKEのまま保全。
result保存時にはone-off command containerが残存と記録されたが、その後の独立確認では
当該projectの全container、全稼働container、ROS/AWSIM processはいずれも残存なし。
自動再試行・自動pushなし。実車接続なし。無接触・経路品質・完走性能の検証は未成立。
