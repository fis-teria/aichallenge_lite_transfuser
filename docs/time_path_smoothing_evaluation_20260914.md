# 保存した時間軌道の平滑化評価

ユーザー依頼: 平滑化前後の操舵余裕と元の経路からのずれを評価する。
Windows正本で実装し、同期後のnative WSLで保存済みデータを評価する。
実行前の比較条件をこの文書とコードで固定する。実車/AWSIMの走行条件変更や再学習は含まない。

## 固定比較条件

- raw、mean3、mean5、d2_lambda1、d2_lambda10、d2_lambda100の6条件。
- mean3/5: 予測時刻方向の対称移動平均。端付近は対称に窓を縮める。
- d2: `sum ||q-p||² + lambda * sum ||D2(q)||²` の制約付き最小二乗。
  lambdaは0.1秒間隔の点列に対する無次元重み。物理的な加速度上限ではない。
- 既知の観測原点(0,0)と3秒先の終点を固定し、30点の0.1秒時刻対応を保持する。
  したがって終点誤差は不変。途中の教師誤差、点列の二階差分、同時刻の位置変化を評価する。
- 平滑化は各予測の内部だけで行い、他時刻のセンサ、他予測、教師を使わない。
  曲率・先読み点の採用上限・安全監視の設定は変更しない。

## 評価母集団と範囲

1. 保存AWSIM旧モデル `codex-time-recovery-model-base01` と候補 `codex-time-recovery-model-candidate01`。
   元の制御計算を既存replayで再確認し、その同じpose/速度で平滑化した予測をPPへ通す。
   全記録可能区間と、走行許可後の最初の5.2秒を分ける。pose等が記録されていない指令は分母を明示する。
2. 各試験で制御に参照されたunique予測について、全30点の同時刻位置変化と暗黙の速度変化を測る。
   「位置変化」は教師誤差・車線偏差・障害物余裕ではない。
3. 既存の候補epoch3 validation予測12,346件と元の6run教師を比較する。
   input欠損・教師maskを維持し、cache/order/hashと元のADE・件数の再現を確認する。test splitは使用しない。

同じ予測の平滑化前後は対応する比較だが、新旧モデル同士は別AWSIM試験である。
PP成立は制御計算だけの結果。変えた指令による車両状態の推移、操舵応答、scan衝突判定は再現しない。
結果から発進・完走・安全な走行の成功を主張しない。比較条件の事後調整も行わない。

## 実行コマンド

Windowsでcommit後、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`。
native WSL `/home/thistle/e2e_autonomous/e2e_lite_transfuser` で以下を実行する。
出力directoryは新規のみ。既存出力は上書きしない。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python tools/evaluate_time_smoothing.py \
  --records ../runs/time_recovery_finetune_evidence_20260914 \
  --cache ../datasets/cache/time_recovery_20260914 \
  --training ../runs/time_recovery_finetune_20260914 \
  --output ../runs/time_path_smoothing_evaluation_20260914
```

## 実行状態

比較実装・テスト作成済み。WSL実行前。
