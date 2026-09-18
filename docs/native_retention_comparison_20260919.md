# 701窓・旧予測保持の4条件比較

ユーザー承認済みの比較。通常/発進/復帰の旧モデルと設定を保全し、4条件を独立保存する。
AWSIM試験・標準モデルへの自動昇格は含めない。

| arm | 旧提示/更新 | native提示/更新 | 旧出力保持 | native補助係数 |
|---|---:|---:|---:|---:|
| old_only | 32 | 0 | 0 | 0 |
| focused | 32 | 8 | 0 | 0 |
| retained | 32 | 8 | 1.0 | 0 |
| retained_weak_geometry | 32 | 8 | 1.0 | 0.1 |

共通: 初期checkpoint SHA256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`、
seed42、FP32、AdamW、LR1e-6、weight decay1e-4、勾配clip1.0、BatchNormは従来どおり更新。
各512更新、合計2048更新。128/256/512更新で12候補を保存し、初期モデル込み13回評価する。
時間上限2時間、重み保存見込み約1.8GB。完了・例外・timeoutでプロセス終了。
途中評価では乱数状態を復元し、再びtrainモードへ戻して続行する。

旧データは各条件16,384提示で順番を一致させる。混合条件はnative4,096提示を追加。
native8は前方4＋全体4、701窓/17runのtrain教師を使用する。841窓は混ぜない。
前回focusedのスケジュールhash一致を要求し、512更新後の701予測を前回と1e-5m以内で照合する。
旧データ入力欠損は従来どおり除外し、提示数・支持数は別に記録する。
これは全旧データを一巡する1epochではない。

旧出力保持は旧trainの支持窓だけを対象とする。凍結した初期モデルのeval予測を一度保存し、
anchor/run・split・checkpoint・予測hashを結び付ける。ターゲットは30点XY（m）。
Smooth L1 beta=0.02m、係数1.0、各窓内の有効点/XYを平均し、窓ごとの損失を合計。
既存L1＋既存復帰補助＋native補助＋旧出力保持を、混合バッチ全体の支持窓数で割る。
新native入力に旧モデルの直進予測を蒸留しない。validation/testを蒸留に使わない。

評価は通常11,505窓/4run、復帰8,962窓/42run、発進210ケース。
通常/復帰ADE・3秒先・復帰run別・発進操舵余裕の既存閾値は維持する。
native701窓/前方239窓は訓練内適合であり、未学習障害物や実走の成功を示さない。
旧のみの条件でも同じ評価を行い、回避を追加しない継続学習による変化を切り分ける。

## 実行

Windowsで対象ファイルだけをcommitし、既存差分を保全したclean cloneから同期する。
専用WSL checkout: `/home/thistle/e2e_autonomous/native_bn_validation_20260919`。

```powershell
git -C E:/workspace/e2e_native_bn_source_20260919 pull --ff-only
& E:/workspace/e2e_native_bn_source_20260919/tools/sync_to_wsl.ps1 `
  -WslRepository /home/thistle/e2e_autonomous/native_bn_validation_20260919 `
  -WindowsRepositoryInWsl /mnt/e/workspace/e2e_native_bn_source_20260919
```

```bash
cd /home/thistle/e2e_autonomous/native_bn_validation_20260919
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=30s 7200s .venv/bin/python -u \
  tools/compare_native_retention_losses.py --root /home/thistle/e2e_autonomous \
  --output runs/native701_retention_losses_20260919
```

出力先は排他的に新規作成。再試験は別outputを指定する。
学習ログ・checkpoint・全評価・selection・completion・progressをWSLの出力先へ保存する。
例外やtimeout時は完了扱いにせず、保存済み候補までと停止理由を報告する。
