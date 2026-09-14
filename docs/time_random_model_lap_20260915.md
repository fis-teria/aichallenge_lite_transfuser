# 再学習モデルの通常AWSIM完走試験（2026-09-15）

状態: **WSL検証・配布準備完了、実行先の接続復旧待ち。AWSIM走行は未実施（0回）**。
ユーザーが再学習後のAWSIM走行試験を依頼し、合格ラインを通常完走と指定した。
今回の実行は新しい依頼に基づく。前タスクの学習・offline比較限定という範囲とは別の試験である。

## 条件と変更の必要性

- 実行先 `graneple@192.168.3.10`。新しい専用ディレクトリ `/home/graneple/e2e_autonomous/time_random_model_lap_20260915` を使用する。
- 通常走行1回 `codex-time-random-lap01`。停止状態からE2E単独で発進し、外乱や教師運転を使用しない。
- 合格はAWSIMの順序付き区間通過と `Lap completed` による1周完了。走行距離・offline誤差だけでは合格にしない。
- 1周、既存監視停止、進捗停止、走行600 s（sim/wall各々）のいずれかまで。外側は710 s + KILL猶予10 s。無条件の再試行をしない。
- 新設定 `configs/control/time_path_random_lap_20260915.json` は前回通常走行設定に対してcheckpoint SHAだけを変更する。新モデルを実際にロードするために必要な変更である。
- epoch3、SHA256 `53e1962b97cfaa47acae3e2ad4687fac96c80672905406fdbe1514abd9563da2`。元ファイルはnative WSL `runs/time_recovery_random_update_20260915/best.pt`。
- 固定目標5 km/h、生の時間軌道30点、`stopping_preview_extended_v1`、`awsim_understeer_v1`、既存操舵応答補償を維持する。実測速度は別に報告する。
- 通常Autoware RVizの `/visualization/time_path/raw_path` にE2E出力を表示。観測時base_linkから現在rear axleへ変換する既存座標処理、SI単位、sim/monotonic時刻、ROS topic/QoSを維持する。
- 既存の単一command publisher、停止領域監視、入力鮮度、操舵制限、watchdogを維持。故障時は今回のsimulatorをfreezeしてowned環境を終了する。元のcheckout・過去container・RViz設定を前後照合し保全する。

新設定以外のモデル・制御・監視ロジックは変更しない。通常完走という同じ指標で新モデルを検証し、失敗時には実測の停止理由を残す。旧設定・重みを上書きしないので切り戻し可能。
nominal test4、r48/r49は未使用のまま保全し、本試験を学習データへ自動追加しない。

## 実行コマンド

Windowsで設定・本書をcommitし、`tools/sync_to_wsl.ps1 -CheckOnly`、同スクリプトの通常同期を完了後:

```bash
# native WSL
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q

# .10: source/install/checkpoint/scene照合と隔離ROS smoke後に1回だけ実行
timeout --signal=TERM --kill-after=10s 710s python3 \
  "$deployment/$source/tools/run_time_path_awsim_trial.py" \
  --deployment "$deployment" --run-id codex-time-random-lap01 --display :1 \
  --config configs/control/time_path_random_lap_20260915.json

# rawをnative WSLへ転送し、全ファイルのSHA256とサイズ照合後
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_awsim_trial.py \
  --run ../runs/time_random_model_lap_20260915/raw/codex-time-random-lap01 \
  --output ../runs/time_random_model_lap_20260915/evaluation
```

## 現在までの確認

- 設定・試験計画source `69acd5340278ff8e484c118b88b3077534aa22ec` をWindowsでcommitし、WSLへ同一SHAを同期済み。
- native WSLの共有lock下で全pytestを実施: **2,488 passed / 4 skipped / 65 warnings、84.76 s**。新設定を既存validatorへ通し、旧設定からの差分がcheckpoint SHAのみであることも確認した。
- 新checkpointのSHA256をWSLで再照合。現在のcontrol/runtime、推論node、制御node、AWSIM runnerは前回通常走行source `edb00cf682641c28d7fd7b5766d64fe47ee8d2d2` と同一。
- commit由来の569ファイル、4,587,520 bytesのsource archiveとGit blob manifestを準備済み。
- 最初の`.10`確認はGPU正常（RTX 4060 Laptop / driver 595.91.07）、動作中containerなし、空き約18 GiB、通常desktop `:1`、新deploymentなし。
- その後SSH・pingが応答しなくなり、Windows側のARPもIncomplete。現地のスリープ・電源・ネットワーク確認をユーザーへ依頼中。原因はまだ確定していない。
- **`.10`への新規配置・ビルド・ROS smoke・走行開始はまだ行っていない**。今回のremoteデータ削除や既存環境の変更はない。合否は未判定。

小さい証跡は [evidence](evidence/time_random_model_lap_20260915) に保存。native WSLの検証出力は `/home/thistle/e2e_autonomous/runs/time_random_model_lap_20260915`。重み・raw・ROS build出力はGitへ追加しない。

接続復旧後は本書と実行先の稼働状況を再確認し、準備済みsourceを新deploymentへ配置して既存のbuild・隔離ROS smokeを通す。
Windowsのtask driver `tmp/time_random_model_lap_20260915/manage.py` の `test` と `package` は実行済みで、再実行しない。
`prepare` は新deploymentが存在しないことを確認してから実施し、成功後に `start` を1回だけ実施、`status` で監視する。
以前のモデル・設定・実験記録は保全し、今回の新しい試行だけを評価する。
