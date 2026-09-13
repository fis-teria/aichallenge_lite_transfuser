# コーナー進入・予測経路・教師走行の比較

対象は固定目標5km/hの `codex-time-segment01`。最初の `STOPPING_SWEEP_OCCUPIED` より前を解析する。
再学習・制御設定変更・追加AWSIM走行は今回の比較に含めない。

## 評価前に固定する方法

- native WSLの共有worktree lock内で評価する。Windowsを編集・Git正本とする。
- `.13`で収集した通常教師のvalidation 4周（5kmh 03/06、8kmh 06/09）と、
  `.10`で収集した復帰走行validation 2周（r22/r23）の実測位置を使う。
  比較の主参照は同じ実行先のr23。対応地点の収集phaseを確認する。
- 同じmap座標の同方向の位置を対応づけ、位置合わせの最適化を行わない。
  走行速度が異なるためrun間の経過時間は合わせない。横ずれは左正、角度はradで保存する。
- 位置のcapture stampを使用し、50ms超の補間や曖昧なstampをまたがない。
  予測の原点は各planの観測時刻のbase_link。現在のposeへ付け替えない。
- 実走行の横ずれ・向き、元の予測の1/2/3秒点と教師ラインのずれ、
  予測時刻の1/2/3秒後の実測位置を別々に評価する。
  時間点の誤差とポリラインへの距離を分け、速度差を横追従誤差と混同しない。
- 最初の監視拒否後の未来は評価しない。継続的な再計画が入るため、予測を固定した因果実験ではない。
- 現モデルepoch3の保存済みvalidation予測と同一観測の正解を、停止対応地点の手前40m〜先8mで比較する。
  checkpoint、cache、順序を確認する。予測npyは現在のhashを記録し、過去のbyte封印とは扱わない。
- 新旧runの対応scanを同じmapへ投影し、座標差の参考として6m以内の最近傍検出距離を示す。
  この値は真値・壁からの距離・車体衝突の測定ではない。AWSIMのビルド差は残る。
- 学習に採用した復帰6runの全アンカーの位置も確認し、このコーナーの復帰状態を含むかを数える。
  test splitのデータ本体は読まない。隣接フレームを独立標本と解釈しない。

## 再現

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python tools/compare_time_corner_tracking.py \
  --root /home/thistle/e2e_autonomous \
  --output /home/thistle/e2e_autonomous/runs/time_corner_teacher_comparison_20260914
```

出力先は未使用のディレクトリを指定する。
全フレーム結果と原本はWSLに保持し、小さな集計・図・実行ログをWindowsへ戻す。
