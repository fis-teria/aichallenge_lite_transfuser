# AWSIM 2環境と12・20・40・60cm教師の再学習

ユーザー指定: AWSIM本体・シーン・車両・センサファイルは変更しない。
実行先は `graneple@192.168.3.10`、学習・教師検証はnative WSL。

並列環境は既存CLI `--ros2-base-domain` で車両domainを1/2に設定する。
各環境に専用Docker network namespaceを持たせ、既存のdomain 0管理通信も隔離する。
通常の `make dev` / 公式開始処理を使い、外側のcompose override、収集ノード、
保存先、CPU割当だけを変更する。AWSIMの実行バイナリや設定の差し替えは行わない。
2環境の初期試験は左右60cmを各1周・各3イベントとし、無制限の連続収集は開始しない。
既存のセンサ・計算・操舵・停止監視は維持する。通常RVizは環境別の新規ウィンドウを識別する。

再学習は、既存の通常走行＋復帰cacheへ、検証済みの4収集群を追記する。
「12cm」は操舵外乱収集の通称であり、全アンカーが正確に12cmずれている意味ではない。
全既存splitを維持し、failed run/eventを除外する。追加教師は未来3秒・30点の実測XY。
rawを再ハッシュし、採用された全入力履歴を原bagから再生成してprepared配列と比較する。

初期重みは従来と同じcommand-off epoch10、3 epochs、batch32、float32、seed42、lr3e-5。
従来の通常36,726枠と復帰8,920枠/epochを維持し、復帰枠の25%を既存の外向き教師と
新規イベントの採用区間先頭1秒へ配分する。残りの枠でも全unique復帰アンカーを提示する。
補助損失は既存のPP操舵相当・遠方横位置のgeometry設定を維持する。
best選択は従来6runの3秒誤差、追加validationは選択後診断のみ。封印testは読まない。
比較対象は従来の `balanced_geometry`。追加データと提示配分の変更を伴う1 seedの比較であり、
データ量だけの効果やAWSIM完走率を推定する実験ではない。
並列試験でこれから収集するデータは、この学習の途中には混ぜない。

Windowsでコミット後、公式sync処理を通して同一コミットをWSLへ同期する。
以下はnative WSL repoから、各コマンドを `tools/with_wsl_training_lock.sh` で囲んで実行する。

```bash
env PYTHONPATH=src .venv/bin/python -m pytest -q
env PYTHONPATH=src .venv/bin/python -u tools/train_time_multiscale_recovery.py audit \
  --plan configs/data/recovery_multiscale_sources_20260916.json --root ..
# auditのresolved_plan.jsonをWindowsのconfigs/time_path_p1へ保存・commit・再同期してから:
env PYTHONPATH=src .venv/bin/python -u tools/train_time_multiscale_recovery.py prepare \
  --plan configs/time_path_p1/recovery_multiscale_20260916.json --root ..
timeout --signal=TERM --kill-after=20s 7200s env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_multiscale_recovery.py train \
  --plan configs/time_path_p1/recovery_multiscale_20260916.json --root ..
env PYTHONPATH=src .venv/bin/python -u tools/train_time_multiscale_recovery.py compare \
  --plan configs/time_path_p1/recovery_multiscale_20260916.json --root ..
```

開始時点の状態: 実装中。2環境の実動分離・完周・教師検証、再学習と比較の結果は未確定。
