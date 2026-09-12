# 35 km/h条件でのCMA-ES再開

通常条件を従来の目標10 m/sから35 km/h（35/3.6 m/s）へ変更し、
ハンデ条件はネイティブ1位・目標7.5 m/sを維持する。
前回の `continuous/round-004` の確定採用経路を出発点とする。
旧continuousの時間切れで未完了になった候補・評価・チェックポイントは保全する。
計算モデルの横加速度仮定は採用判定に使わず、AWSIMの実走結果で評価する。

新しい `continuous35_20260913` で、まず通常用の採用ラインを4台で再計測する。
4台とも35 km/hの実行速度設定、完走、接触なし、OT侵入なしを満たした場合だけ
新条件の基準として保存する。通常CMAは採用ラインの16アンカーを中心に
sigma 0.16 mで初期化する。旧10 m/s条件のCMA分布・成績は流用しない。
ハンデ側は完了済みの第18世代のCMA分布と乱数状態を引き継ぐ。

速度条件は選択経路の実測メタデータと結び付け、後続ラウンド、キャッシュ済み評価、
4台の各実行コマンドへ引き継ぐ。速度だけを変更した旧スコアの再利用は拒否する。
ダッシュボードも実際の目標速度を表示する。
35 km/hは制御の実行速度上限であり、計測実速度の過渡的な超過まで保証する値ではない。
実速度の最大値と速度指令の最大値は別々に記録する。

通常は同一コース・D1の同一位置から4台ゴーストで走り、各個体のCSVを個別に読み込む。
他車入力は従来どおりplanner/recoveryとも空にする。
ハンデは独立した4 AWSIMで、各車にネイティブ1位ハンデを継続適用する。
コーナーの加速hold上限0.6 m/s²は維持する。

予算は初回の基準4評価を含め、最大3時間・6ラウンド・388台分。
各ラウンドは通常24候補とハンデ24候補、それぞれ新旧4回ずつの再検証。
締切後は新しいバッチを開始せず、開始済みの有限バッチを完了して保存する。
採用には完走・接触なし・密なOT判定を4回とも満たし、中央値0.05秒以上改善、
4比較中3勝以上を要求する。改善しなければその条件の現採用経路を継続する。

## 実行

Windows正本でコミットし、変更ファイルをSI26の凍結実験用toolsへ転送する。
学習中のWSL正本を変更せず、独立したnative Linux checkoutで同一コミットを検証する。
検証はそのcheckoutの `tools/with_wsl_training_lock.sh` を通す。
SSH先からのgit pushは行わない。

```bash
python3 -u /home/si26-pc008/cma_mppi_20260912/tools/restart35.py \
  --previous-subdir continuous/round-004 \
  --campaign-subdir continuous35_20260913 --episode-prefix c35 \
  --hours 3 --max-rounds 6
```

初回スクリプトは新規ディレクトリだけを受け付ける。
基準再計測後、同じ締切を渡してcontinuous supervisorへ自動移行する。
停止要求は次のファイルで行い、開始済み評価を破棄しない。

```bash
touch /home/si26-pc008/cma_mppi_20260912/continuous35_20260913/stop_requested
```

ビューアは当日の有効なXauthorityを使い、新しいstateを指定する。

```bash
python3 /home/si26-pc008/cma_mppi_20260912/tools/show_live.py \
  /home/si26-pc008/cma_mppi_20260912 --state-subdir continuous35_20260913 \
  --xauthority /run/user/1000/.mutter-Xwaylandauth.V5NQV3
```

```bash
tools/with_wsl_training_lock.sh /path/to/existing/.venv/bin/python -m pytest -q \
  tests/test_mppi_cma_restart35.py tests/test_mppi_cma_round_resume.py \
  tests/test_mppi_cma_continuation.py tests/test_mppi_cma_dashboard.py \
  tests/test_mppi_cma_refinement.py tests/test_mppi_cma_viewer.py
tools/with_wsl_training_lock.sh /path/to/existing/.venv/bin/python -m pytest -q
```

検証対象は速度条件の持ち越し、旧スコアの誤流用拒否、通常のみのCMA初期化、
ハンデ分布の保存、基準走行失敗時の停止、ソース保存、有限予算と中断再開、
別名campaignのダッシュボード表示である。
