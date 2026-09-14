# 通常走行の予測経路・PP追従・操舵応答の切り分け

対象は `codex-time-lap-candidate01` の通常走行、最初の停止領域拒否まで。目標は引き続き通常 E2E の完走であり、通常ラインへの横位置差を新しい合格条件にはしない。

## 比較方法

- 実行先 `.10` で同じ車両・コース・目標 5 km/h により完走した教師 PP の r30/r31 を測定ラインとして使用する。教師 PP と E2E PP の実装が同一だったとは扱わない。
- 原本の転送 manifest と、使用する control / result / bag の SHA256・サイズを確認する。車両 DLL・設定・scene の hash を今回の走行と照合する。
- 教師の元 Odometry `/localization/kinematic_state` を使用する。55–142 m の初回通過に対応する capture 時間で切り出し、重複・曖昧な時刻・50 ms 超の区間を既存 RecordedLine / PoseIndex 契約で扱う。表示用 PoseStamped で置き換えず、コース位置合わせの最適化もしない。
- 今回の発進許可から 55 s 以降を、進入 55–65 s、旋回 65–80 s、戻り 80–90 s、末尾 90 s–最初の拒否に分ける。区間は先行する位置推移の観察に基づく診断用であり、独立標本や一般的成功率の区分ではない。
- 各予測を元の観測時刻の base_link から map へ変換し、1/2/3 s の予測点と通常ラインの横位置差を測る。
- 1/2/3 s 後の実測位置と予測を、通常ライン上の**同じ進行位置**で比較する。同じ接線の法線方向で `実測−通常ライン = 予測−通常ライン + 実測−予測` を計算し、前後方向の速度差を横追従誤差に含めない。予測進捗の逆行・範囲外・曖昧な未来・最初の停止後は除外理由を残し、外挿しない。
- この分解は幾何学的な恒等式であり、原因の寄与率ではない。実走では後続予測に切り替わるため、実測−予測には PP だけでなく再計画の影響も含まれる。
- 同じ現在位置・速度・残り時間・距離探索帯・物理タイヤ角上限で、通常ラインを PP の目標にした場合の角度を診断する。通常ライン上の先頭点と、横にずれた自車原点は一致しないため、**これは完全な TimePlan や正しい復帰教師を生成する処理ではない**。geometry の感度比較であり、制御・scan まで通った走行許可を意味しない。
- 元の要求タイヤ角と実測角、rate limit、停止監視余裕を合わせて確認する。実走・推論・学習・監視条件の変更は行わない。評価予約 r48/r49 は読まない。

## 再現

Windows でコミットし `tools/sync_to_wsl.ps1` で同一 commit を同期してから native WSL で実施する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q

tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python tools/analyze_time_normal_lap_attribution.py \
  --trial /home/thistle/e2e_autonomous/runs/time_normal_lap_20260914/raw/codex-time-lap-candidate01 \
  --reference-runs \
    /home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914/codex-time-recovery-speedbase-r30 \
    /home/thistle/e2e_autonomous/raw/time_recovery_speed_20260914/codex-time-recovery-speedbase-r31 \
  --output /home/thistle/e2e_autonomous/runs/time_normal_lap_attribution_20260914
```

出力ディレクトリは新規のものを使用する。全予測・全指令の比較表は WSL に置き、小さい集計と図だけを Windows へ戻す。結果は解析終了後に追記する。
