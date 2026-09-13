# 時間予測経路とPure Pursuit先読み条件の整合性調査

ユーザー依頼: 修正を適用する前に、予測経路とPP先読み条件の整合性を調査・解析する。
Windows正本で診断処理とテストを追加し、同期したnative WSLで既存記録を解析する。
制御・ROS・モデル・速度目標・AWSIM設定は変更しない。

## 調査計画

- 対象は既存baseline/candidate試験。元の制御replayとsource/config hashを確認する。
- 実装上の距離帯は `Dmin=max(1, 0.4+0.5v+v²/2)`、`Dmax=Dmin+0.5`（m）。
  PP要求角は `atan((1.087+0.045v²)*2y/(x²+y²))`（rad、vはm/s）。
- 現行のfloat32離散点選択と、同じ折れ線上の連続位置を比較する。
  各線分の半径条件・前方条件・操舵条件の二次方程式境界を求め、狭い区間を間引きで見逃さない。
  補間した中点をfloat32に変換し、同じ距離上限・操舵上限で再確認して既存PP計算へ通す。
  経路の端点、時刻順、形状、探索距離上限を変更しない。外挿・平滑化・操舵上限の緩和は含まない。
- float64離散点、時刻による点の除去を省いた場合、後輪offsetを0にした場合を感度比較する。
  最後の2条件は原因切り分けであり、正しいframe/time契約の変更案ではない。
- 発進時に保存された予測/姿勢を固定し、計算速度のみ0/1/3/5km/hとした感度も比較する。
  この感度計算は車両状態を進めるシミュレーションではない。
- geometry検査、実タイヤ角、ROS入力角、応答補償、学習目的の各制約が何を保証するかを整理する。

## 実行コマンド

Windowsでcommit後、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`。
native WSL `/home/thistle/e2e_autonomous/e2e_lite_transfuser` で以下を実行する。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python tools/audit_time_lookahead.py \
  --records ../runs/time_recovery_finetune_evidence_20260914 \
  --output ../runs/time_lookahead_alignment_audit_20260914
```

出力は新規directoryに限定し、原本と旧評価は保全する。
診断でnominal PPが成立しても、操舵応答・停止領域・動いた後の新しいモデル予測は別の検証が必要。

## 状態

診断実装・テスト作成済み。WSL実行前。
