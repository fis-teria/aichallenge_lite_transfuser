# 観測教師の近距離適合比較（2026-09-19）

ユーザー承認の「旧出力保持を残して回避開始・前方1～3mの適合を改善する」を実施する。
通常設定・runtime・運用checkpointは変更せず、学習データと損失の範囲を明示する。

## 監査と不足

701窓のうち障害物が前方にある239窓/13runを再監査した。
教師と旧候補の両方が前方1/2/3mの補間を支持する224窓では、教師の平均|Y|は2.64/10.28/22.04cm、
旧候補は2.95/10.84/22.65cm。単に曲がり量が一律に小さいという問題ではない。
Y誤差は3.08/10.56/21.10cm、|教師Y|>=0.1mの窓の逆符号数は0/18/34。

記録配置と姿勢に基づく診断では、物体前方0～6m・横±1mの教師は0窓。
0～8m・横±1.5mでも1窓だけで、そこでの前方3mのYは教師0.369mに対し予測-0.025mだった。
これは全周囲の物理離隔の証明ではなく、既存の姿勢/配置診断に基づく集計である。
追加のgapfill/feasible収集記録でも正面0～6m・横±1mは0、箱回避教師は未採用と確認した。
本比較でデータに存在しない正面近接回避や箱回避まで学習できたとは扱わない。

## 事前に固定する比較

開始checkpointは`retained_step0512.pt`、SHA256
`e7afdab5d05873de0dbed454e1086b32f4d4f1b67f884a9417a349d7db726ad2`。
旧出力保持ターゲットと保持ゲートの基準は元運用モデル
`1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`のまま。
継続を繰り返して許容誤差を累積させない。

| 条件 | 旧保持係数 | 既存native幾何係数 | 新しい近距離XY係数 |
|---|---:|---:|---:|
| continue_retained | 1 | 0 | 0 |
| near_1 | 1 | 0 | 1 |
| near_3 | 1 | 0 | 3 |

全条件で新しいAdamW、LR1e-6、weight decay1e-4、FP32、seed42、clip1、動的BatchNorm。
各512追加更新、合計1536。旧32＋native8（前方4＋全体4）の提示順を前回と一致させる。
128/512更新を保存し、初期/開始候補を含む8全件評価。時間上限5400秒、snapshot6件。
これは同一seed・固定データの損失比較で、別seed再現性や全epoch学習ではない。

新しい損失は、native前方窓の**観測された教師軌道がX=1/2/3mへ達する時刻**を固定し、
その同じ時刻の予測XYと教師XYにSmooth L1(beta0.02m)をかける。
教師・予測は30点、dt0.1秒、観測base_link座標m。3地点×XYの平均を各窓で取り、
対象窓の和を重み付けし、既存損失と合わせてバッチ全体の支持窓数で割る。
予測自身のX到達時刻で評価しないため、前進を遅らせて横誤差だけを小さくする逃げを避ける。
全点支持、3mまで単調前進、1m以前の実測点がある教師のみ対象。外挿・架空の原点・人工回避軌道は使わない。
旧train以外への蒸留、validation/testの教師利用、nativeへの旧直進予測蒸留は行わない。

評価は元の通常11505窓/4run、復帰8962窓/42run、発進210ケースとrun別保持基準を維持。
新候補には開始候補に対する近距離XY誤差5%以上改善、近距離X誤差悪化0.005m以内を追加要求し、
通過した新候補の近距離XY誤差で選ぶ。未通過なら開始候補を保全する。
近距離は固定教師時刻のtrain適合であり、障害物との車体離隔や未学習配置の成功率ではない。
runtime昇格や追加AWSIM走行はこの比較には含めない。

## 実行

Windowsの対象ファイルのみcommit後、clean cloneから既定の同期ツールでnative WSLへ同期する。

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
  timeout --signal=TERM --kill-after=30s 5400s .venv/bin/python -u \
  tools/compare_native_near_field.py --root /home/thistle/e2e_autonomous \
  --output runs/native_near_field_fit_20260919/comparison
```

既存出力は上書きしない。設定、教師対象/参照hash、全学習ログ、8評価、選択理由、終了記録を保存する。
