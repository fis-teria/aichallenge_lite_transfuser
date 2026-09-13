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
  --cache ../datasets/cache/time_training_20260913_v2
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_corpus.py train \
  --corpus ../datasets/processed/time_teacher_20laps_20260913 \
  --cache ../datasets/cache/time_training_20260913_v2 \
  --output ../runs/time_p1_20laps_20260913 --epochs 10 --batch-size 32 --workers 4 --precision bf16
```

中断時は同じコマンドに`--resume`を付ける。既存の予算・config・source identityが違えば拒否する。
各armの`status.json`、`history.json`、`last.pt`、`best.pt`、`best_validation.json`と、両arm終了後の`comparison.json`を確認する。

## 事前GPU確認

WSLのPyTorch2.7.1+cu128、RTX4080 16GB。モデル12,251,426パラメータ。
合成入力のforward/backwardでは、FP32 batch32はwarm後約0.25秒・peak5.8GB、BF16 batch32は約0.16秒・peak3.0GB。
これはデータ読込・optimizer・検証を含まない事前計測であり、本学習速度ではない。

## 実行確認

- 学習コード: `2137dab87037636441587add6cfa36508db93696`。
- WSL全体pytest: **1,987 passed / 4 skipped / 63 warnings**（75.19秒）。
- CPU合成データの有限2epochを途中で中断し、optimizer/scheduler/RNG/cursor復元後の検証予測が連続実行と全件完全一致する回帰テストを実施済み。CUDA学習の中断再開完全一致を検証したという意味ではない。
- cache identity: `f8ac104391d7a7c483d53aa06fcf058bc06cc6a5a540c6b940d1b4db821f75f5`。
- train/validationの16周全件の採否照合と、各周2アンカーの実センサ再構成で全input tensorの完全一致を確認。
- 初回cacheは収録開始境界の除外理由不一致で停止した。`audit_anchor`の判定順を共通入力処理と揃えて回帰テストを追加した。
- 既存の固定corpusは保存し、cacheでは`5kmh_run09` / `8kmh_run10`の最初の各1アンカーに限り、旧`CURRENT_SENSOR_MISSING`と再生時`ANCHOR_OUTSIDE_EPOCH`を両方記録した。元のepoch bounds外・全履歴参照空・採用/usable falseを条件にした明示的な移行であり、教師・mask・採否・母数は不変。他の不一致は引き続きエラーにする。
- 初回の不完全cacheは`../datasets/cache/time_training_20260913.partial`に保全し、学習には完了済みの`time_training_20260913_v2`を使用する。

実行ログは`/home/thistle/e2e_autonomous/runs/time_p1_training_evidence_20260913/`、
学習結果は`/home/thistle/e2e_autonomous/runs/time_p1_20laps_20260913/`。
学習完了後の最良checkpoint・比較値は追記する。

## 学習中に確認した旧V4データ

ユーザーは「今回の20周パックとは別の、V4-10/V4-20で使った旧データ」の再利用可否を質問した。
今回の固定OFF/ON比較は20周コーパスのtrain/validationだけで継続し、途中でデータを追加しない。

- `../datasets/d1log_0902_all_v3`: 11 run、旧形式48,946サンプル。
- `../datasets/recovery_20260904_v3`: 15 run、旧形式23,751サンプル。
- `../datasets/d1log_recovery_mixed_20260904_v3`: 上記の統合26 run、72,697サンプル。3セットを足して重複計上しない。
- `../datasets/raw/d1log_0902_all_v3/`にraw MCAP、`bag_inventory.json`、`bag_validation.json`、各runの`metadata.yaml`が残る。
- metadata上、Camera/LiDAR、`/localization/kinematic_state`のOdometry、velocity、`/clock`、control command/TFを確認。再教師化の材料は存在する。
- 旧`trajectory_path`だけから時間教師を作らず、rawの観測poseとclockから未来3秒を再構成する必要がある。receipt availability、50ms freeze、epoch、欠損・介入区間、split重複の監査は未実施。
- 旧形式のサンプル件数は、TimePathの採用可能件数を示さない。recovery全rawと公開データ`aic_real_dataset_v2`の時間教師への適合性も未確定。

## 実行テスト先

現在の実データ学習はnative WSLで継続する。runtime/AWSIMの実行テストを行う場合の指定先は、`graneple@192.168.3.10` とする。今回のTimePath checkpointは既存V3 loaderと互換ではないため、専用の30点XY loader、因果4/4/10履歴、receipt＋50ms freeze契約が必要になる。

runtime/shadowへ進む前に、`base_link`からrear axleへの校正済み変換も確認する。これらの条件が揃った実行テストはまだ実施していない。`192.168.3.10:22`への初回SSH BatchMode接続確認はConnectTimeout 8秒でタイムアウトしており、現在の接続状態を示すものではない。SSH先でのruntime操作やAWSIM試験を完了済みとは扱わない。
