# コーナー復帰データの統合・追加学習とAWSIM完走試験

残る16条件の追加収集を区切り、採用済みの教師を既存の12/20/40/60cm統合cacheへ追加する。
Windowsを編集正本とし、source commitを公式同期して、native WSLのworktree lock内で監査・学習・評価する。
旧cache、モデル、収集証跡は保持する。未充足条件を充足済みに変更しない。

## 固定する条件

- 7収集の採用分を追加する。train 6,495、validation 5,800サンプル。既存runのsplitは変更しない。
- 完走・無fault・停止確認・bag閉鎖を確認し、元rawのhashと実入力履歴を再照合する。
- 1教師は実観測入力と実測将来3秒、30点のxy座標（m）。準備用経路や失敗走行は教師に追加しない。
- 最新の多段階復帰epoch3を初期重みとして3epoch追加学習する。バッチ32、float32、learning rate 3e-5。
- 通常走行36,726サンプルは毎epoch1回。復帰11,941サンプルに23,882提示枠を確保し、全件を含める。
  復帰枠の25%を既存の重点状態と追加イベントの最初の1秒に配分し、残りの教師も省略しない。
- 合計60,608提示/epoch、最大5,682 optimizer updates。既存の8,920復帰枠を維持すると全件を含められないため拡大する。
  今回は追加学習であり、前回と同じ更新回数によるデータ追加だけの因果比較ではない。
- モデル構造、入力、教師、損失の定義を維持する。旧6 validation runだけでepoch選択する。
  追加validationは選択後の比較専用。封印testは開かない。
- AWSIMはgraneple@192.168.3.10。固定目標5km/h・既存PP・操舵応答補償・停止監視を維持する。
  通常RVizへE2Eの生予測経路を表示する。AWSIM本体を変更しない。
- 有限予算は追加学習1回と通常走行試験1回。走行はsim/wall各600秒以内、監視停止で終了。
  合格は公式Judgeの順序付き区間通過・1周完了・停止確認。未完走もそのまま報告する。

## 再現コマンド

同期後、native WSL repoで実行する。出力は新規ディレクトリのみ。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/train_time_corner_recovery.py audit \
  --plan configs/time_path_p1/recovery_corner_20260916.json
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/train_time_corner_recovery.py prepare \
  --plan ../runs/time_corner_retraining_20260916/resolved_plan.json
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=30s 10800s .venv/bin/python -u tools/train_time_corner_recovery.py train \
  --plan ../runs/time_corner_retraining_20260916/resolved_plan.json
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/train_time_corner_recovery.py compare \
  --plan ../runs/time_corner_retraining_20260916/resolved_plan.json
```

進行・結果はこの文書へ追記する。監査、再学習、オフライン比較、AWSIM完走は別々に判定する。

## 読込ワーカーの中断と再実行

初回は1epoch目の1,050更新を記録後、4つの読込ワーカーにbus errorが発生し、WSL自体が停止した。
ログは共有メモリ不足の可能性を示すが、VM停止の直接原因は確定していない。
再起動後のWSLはRAM上限16GiB、swap4GiB、`/dev/shm`約7.9GiB。元データと中断checkpointは保全する。

入力読込だけをworkers=0へ変更し、ワーカー間の共有メモリ転送を使わずに再実行する。
数学的な学習設定とデータは維持し、同じ前回モデルから新しい出力先へ3epochを実行する。
中断した初回と再実行の更新数・時間は別記する。途中checkpointからの厳密resumeとは扱わない。
教師manifestと学習planには実際のworkers=0を記録する。追加収集・モデル構造・PPの変更はない。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=30s 10800s .venv/bin/python -u tools/train_time_corner_recovery.py train \
  --plan ../runs/time_corner_retraining_20260916/resolved_plan.json --loader-workers 0 \
  --training-output ../runs/time_corner_retraining_20260916/training_serial
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/train_time_corner_recovery.py compare \
  --plan ../runs/time_corner_retraining_20260916/resolved_plan.json --loader-workers 0 \
  --training-output ../runs/time_corner_retraining_20260916/training_serial
```
