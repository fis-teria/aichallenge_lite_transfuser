# 時間基準モデルの初回実データ学習

2026-09-13のユーザー指示で、検証・分割済み20周コーパスを使う本学習を開始する。
初回はcommand OFF/ONの各10epoch。重みはscratch、seed42、同じ初期tensor・epochごとの提示順。
train12周36,726アンカー（支持35,360件）、validation4周12,237アンカーを使い、test4周は学習/モデル選定から隔離する。

## 学習条件

- TimePathV1 B0、未来3秒・0.1秒刻み30XY点、`base_link_at_observation`。
- Camera4時刻、LiDAR4時刻、ego10時刻、過去送出指令10時刻（OFF/ONで有無のみ切替）。
- 画像受信後50msの入力確定条件、run/epoch・到着cut・教師maskを固定する。
- batch32、AdamW lr=0.0003 / weight_decay=0.0001、勾配norm上限1.0。
- 有効点平均→支持アンカー平均のL1[m]。入力欠損も提示数に含める。支持0ではoptimizer/schedulerを進めない。
- cosine LR（最小lrは初期の0.1倍）、最大11,480 optimizer更新/arm、最大367,260提示/arm。
- CUDA BF16 forward、重み・損失はfloat32。GradScalerなし。TF32無効、cuDNN benchmark無効・deterministic有効。
- 250更新ごと・epoch境界に再開checkpoint。optimizer/scheduler/RNG、epochの提示順hash/cursor、入力契約・教師/source identityを保存する。
- validationのrun-macro 3秒位置誤差でbestを選び、全点ADE・0.5/1/2/3秒誤差・worst run・支持数・欠損分母を併記。
- ゼロ移動/実測定速baselineを同じ条件で評価する。最終bestを再ロードし、保存時のvalidation予測と全件一致を検証する。

epoch内の順序はseedとepochから固定する。train内の順序変更であり、frameランダムsplitではない。
同コース・同条件のrun holdoutで、未知コースの汎化やAWSIM追従を証明する評価ではない。
B0は停止意図/stop probabilityを学習しない。実推論側への50ms契約接続とcontroller座標変換は後続。

## キャッシュ

原本bagと教師/参照ファイルのhashを再確認してから、train/validationだけをキャッシュする。
画像はPillowの同じBILINEAR処理で一度だけresizeしたRGB uint8、LiDARは共通range/validity前処理のfloat32。
同じ画像を履歴ごとに複製せずmemory mapで読む。欠損履歴は正規化後のゼロで保持する。
小さいego/command/timing配列は共通入力関数で組み立て、各runの実センサ再構成と全tensor一致を確認する。
予期しない入力除外や既存監査との採否不一致は失敗として停止し、`.partial`を保全する。

## Windows / WSL

編集・コミットはWindows、実行はnative WSL checkoutと共有lock経由。大きなcache/重みはWSLのF:側に保存する。

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q \
  tests/test_time_training_cache_v1.py tests/test_time_batched_evaluation_v1.py \
  tests/test_time_corpus_runner_v1.py tests/test_time_checkpoint_p1.py tests/test_time_pipeline_p1.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_corpus.py prepare-cache \
  --corpus ../datasets/processed/time_teacher_20laps_20260913 \
  --cache ../datasets/cache/time_training_20260913
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_corpus.py train \
  --corpus ../datasets/processed/time_teacher_20laps_20260913 \
  --cache ../datasets/cache/time_training_20260913 \
  --output ../runs/time_p1_20laps_20260913 --epochs 10 --batch-size 32 --workers 4 --precision bf16
```

中断時は同じコマンドに`--resume`を付ける。既存の予算・config・source identityが違えば拒否する。
各armの`status.json`、`history.json`、`last.pt`、`best.pt`、`best_validation.json`と、両arm終了後の`comparison.json`を確認する。

## 事前GPU確認

WSLのPyTorch2.7.1+cu128、RTX4080 16GB。モデル12,251,426パラメータ。
合成入力のforward/backwardでは、FP32 batch32はwarm後約0.25秒・peak5.8GB、BF16 batch32は約0.16秒・peak3.0GB。
これはデータ読込・optimizer・検証を含まない事前計測であり、本学習速度ではない。

検証結果・学習成果は実行後に追記する。
