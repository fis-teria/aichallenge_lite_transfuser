# 復帰データ追加学習とAWSIM比較

ユーザー依頼: 採用した復帰データで再学習し、graneple@192.168.3.10でAWSIMテストを実施する。
この文書を実行状態の正本とする。現在は変換・学習準備段階で、走行結果は未取得。

既存の学習入口は20周固定のscratch OFF/ON比較専用であり、復帰区間の除外と追加runの固定split、
既存checkpointからの追加学習を扱えない。データ変換と学習開始点のみを拡張する。
元の20周splitを内包し、元のtrain/validation/test割当とraw hashの非重複を強制する。
新形式以外の6/2/2制約は変えない。旧checkpoint/cache/rawと旧実験部署は保全し、出力は新規パスに限定する。
判定指標は通常走行・復帰holdoutの3秒誤差とAWSIM進捗/停止理由。ラベルshape/mask/履歴、split境界、
finetune初期重みをテストし、実センサ再構成の一致も確認する。安全監視やcontrollerは変更しない。

## 固定計画

- 起点: command OFF epoch10、SHA e857db4b67d3d9f6a77c2865cfc1fa53e9903e7a1d2d8a371407fbe009a1a44f。
- 元のtrain12周、validation4周、test4周の割当を保持。testは未使用。
- 復帰train: r19右20cm、r20左20cm、r21直線左40cm、r25右カーブ左40cm。
- 復帰validation: r22直線右40cm、r23右カーブ左20cm。1runをsplit間で分割しない。
- r24/r26の補助、部分周回、診断失敗は今回の教師に含めない。
- recovery開始150ms後から、因果的な入力と全30点の観測未来を持つアンカーのみ。
  50ms受信freeze、yaw異常を含む履歴の除外、phase maskを収集監査と再照合する。
- train復帰アンカーを40回提示し、通常走行に埋もれないよう約20%の提示割合とする。
  独立データ数は増えない。validationは繰り返さず、全6runの等重み3秒誤差でepochを選ぶ。
- AdamW lr3e-5、3epoch、batch32、seed42、float32/TF32無効。optimizer/RNGは新規開始。
  約136,938提示/4,281更新の有限予算（実測採用数で確定）、学習outer timeout 2時間。
- AWSIMはbaselineと候補各1周を目標、各720秒outer/600秒走行、最大4試行。
  同一の物理的失敗を2回繰り返したら打ち切り、記録して判断する。
- 目標固定5km/h、通常RVizのモデル経路、既存vehicle_model設定・指令制限・停止監視を維持。
  raw poseは推定自己位置、bag receiptは前処理完了時刻の実測ではない。

## 再現コマンド

Windows正本でcommit後、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`。
native WSL `/home/thistle/e2e_autonomous/e2e_lite_transfuser` で実行する。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_recovery.py prepare \
  --plan configs/time_path_p1/recovery_20260914.json \
  --cache ../datasets/cache/time_recovery_20260914
bash tools/with_wsl_training_lock.sh timeout --signal=TERM --kill-after=20s 7200s \
  env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_recovery.py train \
  --plan configs/time_path_p1/recovery_20260914.json \
  --cache ../datasets/cache/time_recovery_20260914 \
  --output ../runs/time_recovery_finetune_20260914
```

大きなデータ/cache/重みはnative WSLのF:上に保持する。Windows E:とGitには置かない。
AWSIMへはcommit済みsourceと選定重みを新規deploymentへ転送し、hash・install・asset・RViz接続を確認する。
完了判定は追加学習完了、候補のオフライン比較、AWSIM実行と結果分類、停止/環境後確認である。
完走できない場合も失敗を隠さず、offlineの改善と閉ループの成立を分けて記録する。
