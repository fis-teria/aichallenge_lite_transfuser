# 学習方法を変更したモデルのAWSIM走行比較（2026-09-15）

## 固定した比較計画

実行先は `graneple@192.168.3.10`、解析・評価はnative WSLとする。
目標速度5 km/h、Pure Pursuit、操舵応答補償、停止領域監視、シーン、通常RVizへのE2E生経路表示を共通にする。
合格条件は公式JudgeLogによる1周完走と、その後の停止確認。オフライン誤差の改善だけでは合格にしない。

比較対象は同じ初期重み・更新数で学習した次の3モデル。制御設定の差分はcheckpoint SHA-256だけとする。

|記号|モデル|変更点|
|---|---|---|
|A|uniform_l1|現行モデル|
|B|balanced_l1|外向き復帰状態の提示配分を増加|
|D|balanced_geometry|提示配分と復帰軌道の損失を変更|

実行順は **A, D, B, B, D, A**。各2回で順序を反転し、各試行を初期状態から開始する。
各試行は1周、既存の停止条件、走行600秒、外側720秒のいずれかまで。
これは6試行の限定比較であり、再現性や一般化を十分に推定する母数ではない。
モデルの監視停止は結果として記録し、インフラ不具合・後片付け失敗があれば原因確認まで次の試行を開始しない。
失敗後の制御調整、追加学習、旧評価データの訓練への混入はこの比較に含めない。

機械可読計画: `configs/control/time_objective_driving_comparison_20260915.json`。
各モデルのオフライン比較は `docs/time_recovery_objective_comparison_20260915.md` を参照。

## データ収集案を決める観点

既存の厳密な外向き状態は学習35点/11 run、検証12点/3 run。
ランダム外乱の成功収集は2 run/6イベントで、復帰教師516点のうち厳密な外向き状態は13点。
データ容量や連続フレーム数よりも、独立したイベント・runと、左右・コーナー位相・ずれ状態の広がりが課題となる。

走行比較後、通常走行のどの位置・向きから復帰予測が不足したかを確認し、採取範囲を具体化する。
ずれた位置でのCamera/LiDAR/ego履歴と、その状態から教師が戻す連続3秒の実測軌道を組にする。
左右対称の外乱、コーナー入口/中間/出口、直線を分け、run・seed単位でtrain/validationを事前固定する。
今回の走行比較ログは評価用として保持する。収集の実施は次の作業として扱う。

## 実行・評価コマンド

各専用deploymentで既存ランナーを使用する。

```bash
python3 <deployment>/source_<sha>/tools/run_time_path_awsim_trial.py \
  --deployment <deployment> --run-id <run_id> --display :0 --config <model_config>

cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_awsim_trial.py --run <verified_raw_run> --output <evaluation>
```

## 結果

準備中。走行結果・読み込んだ重みの検証・環境復元・不足データ案は実測後に追記する。

## 走行前に検出・修正した保存メタデータ不具合

初回source `fa426a3` のROS smokeでDが `TEACHER_RUNTIME_CONTRACT_MISMATCH` を報告し、AWSIM開始前に停止した。
比較学習の保存処理で `teacher_manifest.contract` の転記が欠けていた。元キャッシュには正しい30点・0.1秒・観測時base_link・50ms確定の契約があり、保存処理へ転記を追加した。
runtime側の拒否条件、モデル構造、推論処理、PP・安全監視は変更していない。

既存B/Dは元ファイルを保全し、`tools/export_time_recovery_runtime_checkpoint.py` で元キャッシュhash・split・入力設定と結び付けた別ファイルへexportした。
学習キャッシュ13,637,154,115 bytesを全hash照合し、全215 model state entryの一致と、各モデル12個の既存検証入力（正常11・復帰1）でCUDA推論の完全一致（最大差0m）を確認した。再学習は行っていない。
元の学習checkpoint SHAとexport SHAは比較計画内に両方残す。新規export回帰テストを含む重点テスト16件が成功した。
初回のROS失敗記録とテストのimport修正履歴は保全し、初回deploymentを上書きせず `_r2` の専用環境へ配置する。走行の順序・回数は変更しない。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/export_time_recovery_runtime_checkpoint.py \
  --checkpoint <original_best.pt> --checkpoint-sha256 <original_sha256> \
  --cache /home/thistle/e2e_autonomous/datasets/cache/time_recovery_random_update_20260915 \
  --output <fresh_runtime.pt>
```
