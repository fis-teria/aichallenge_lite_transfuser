# Spatial Path V4 固定step500：限定train/validation評価結果

## 結論と判定の範囲

固定train64の全20点XY予測は前段と完全一致した（最大/平均差0、許容rtol1e-5/atol1e-6m）。
同じ固定checkpointでvalidation 5 runsのmain160件・観察20件の評価を完了。
学習、optimizer構築/step、BN更新、checkpoint変更、sensor介入、再推論・再選択は行っていない。

| 指標 | モデル | 直線baseline | train平均template | 原点baseline |
|---|---:|---:|---:|---:|
| train再現64件 ADE m | 0.013010 | 0.040131 | 0.040327 | 0.812469 |
| val main160件 ADE m | 0.025181 | 0.037103 | 0.037129 | 0.718003 |
| val観察20件中18支持あり ADE m | 0.058042 | 0.060287 | 0.059804 | 0.419181 |

val mainのpaired比較は直線baselineに103勝/57敗/同値0。
ただし直線shapeでは7勝/52敗で劣る。全shapeに優位とは言えない。
未見runのこの固定部分集合では有用な改善を観測したが、広い汎化・物理精度・安全性の証明ではない。
特に遠方誤差、限定教師支持、候補適格条件による選択バイアスを残す。

## 版・実行運用

Repo: https://github.com/fis-teria/aichallenge_lite_transfuser

Branch: `codex/windows-wsl-training-sync`

- 今回開始版：`dfedd6de00eb3592b3e0566a7fc99b7c1f08ea5c`
- 前段モデル学習実行版：`f33b197df9eb1e6ae2af70c9661d8b1d88ab4ef0`
- 今回テスト・推論実行版：`153a22a8b85ebcf21436abf9ab99c94800687cea`（dirtyなし）
- 注釈のみ後処理版：`2386f0fba6295278edd3599426f1e5d8774752ae`
- 結果追記/梱包版：ZIPの`provenance/changed_files.json`に実SHAを記録。

Run ID：`spatial_validation_v4_20260906_153a22a`

保存先：`/home/thistle/e2e_autonomous/runs/spatial_validation_v4_20260906_153a22a`

Windowsで今回差分のみcommit→CheckOnly→sync→同一SHAのnative WSL checkoutでlock付き実行。
今回の自動pushは禁止のため未push。以前の依頼でpushしたのは前段dfedd6dまで。
reset/stash/cleanup/process停止/lock削除なし。元Dataset、前段run、重みを上書きしていない。

今回新設は評価専用adapter/metrics/CLI/config/tests/docs/packageと注釈後処理。
前段の入力・教師・モデル・trainer・runtime・controller・Safety・S1/S2/Ledgerは変更していない。
旧trainerは実行しておらず、適格条件の比較用静的資料としてZIPへ収録するだけ。

## 固定identity

| 対象 | SHA-256 / 内部identity |
|---|---|
| Dataset内部 | `181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388` |
| manifestファイル | `d625f42ca05a18ea76952376c6392268191c4895d6e605e0c49ceaa66dcbe1de` |
| splitファイル | `7d0e433dbd032ad695227051573e7d8d17072fa4ea3b4e28f4c44f56fde27b4f` |
| input contract | `77fff3f9c5875b6f116cebe35fde88d42d459aa04c195676c44e5c6caca5edc7` |
| teacher contract | `cdb668834d4baf60c31fa5f934d01d8782f02544bc6fa29053d5a850432cc344` |
| 前段selection | `698aba59f8cb7779006a8b070b6b00b406e8c93922f5fade252d19205907096b` |
| 今回val selection | `59393d98ad4e55a59da515ff324a147995889e98b48b3a7ac676660400e3851f` |
| 固定final.pt | `0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f` |

checkpointは明示された前段run/checkpoints/final.ptのみ。hash検査後weights_only=True、
保存format、全state key/shape/dtype/有限性を検査してstrict復元。scratch/別重みfallbackなし。
前段LAST_STEP_NOT_BEST/500 steps、resolved config、旧ソースhash、contract identityを照合。
12,221,128 parameter要素の同一architectureをeval/inference_modeで使用。
parameter/buffer全stateの前後hash一致。checkpointファイルも終了時hash一致。

前段target/prediction/baseline等6artifactの期待hashは、前段検証済みreview packetから固定。
val平均ではなくtrain64/maskからtemplateを再作成し、前段baseline配列と完全一致。
全距離にtrain templateの支持があり、今回は未定義距離の補完は生じなかった。

## 読取範囲・選択母数

構造metadataは26 runs/72,697 anchors。val母数13,641 anchors。
runとsource_hashのsplit跨ぎはmetadata上0。セッション/シーン独立性や原本一意性はUNKNOWN。
testの構造metadataはidentity確認だけで、test future/sensorにはアクセスしていない。

| validation run | 元母数 | future候補 | main適格 | main選択 | 観察選択 |
|---|---:|---:|---:|---:|---:|
| 20260902-131505 | 4594 | 256 | 212 | 32 | 4 |
| 20260902-132822 | 4301 | 256 | 209 | 32 | 4 |
| recovery_left_far_early_r03 | 2487 | 256 | 241 | 32 | 4 |
| recovery_left_near_early_r01 | 1131 | 256 | 222 | 32 | 4 |
| recovery_right_far_early_r04 | 1128 | 256 | 217 | 32 | 4 |
| 合計 | 13641 | 1280 | 1101 | 160 | 20 |

seed42、候補はrunごと最大256、mainはshape round robinと同run0.5秒gap、各32上限。
適格条件は前段のcurrent ego/moving/0.5m支持/flag/cutを維持。
画像/LiDAR読取はtrain64＋選択val180の4/4/10/10履歴に必要なものだけ。
unique sensor assetsは2,499（画像/LiDAR/validityを別assetで計数）。
future実体はtrain64＋val候補1,280。選択val180の元futureを元bytesで保存した。
元val全件の評価ではない。支持なし/欠損/停止/反転/holdは観察・除外理由へ残す。

mainは160支持あり、2,140点。観察20件は18支持あり/133点、2件支持なし。
原点とinvalid tailをADEに入れず、支持なし2件を偽ゼロ教師へ変換していない。
全選択件を推論済み、未処理0、非有限予測0。未知tailは利用可能性を意味しない。

## 注釈読取の不具合と、非再実験の補足

実行版はcoverage ledgerのsplitを`validation`で検索したが、実ファイルは`val`表記だった。
そのため元selection/metricsのnormal_recovery/collection_sliceがUNKNOWNになった。
この実装不備を隠さず、元実行版・元selection identity・元metricsをそのまま保存した。

`artifacts/annotation_addendum.json`はhash固定の既存ledgerから、既に固定した180 IDsへ
run/splitを照合して注釈を結合した別資料。予測値・教師・role・shape・ID順序は変更なし。
選択処理も推論も再実行していない。main/run/shape/ADE/勝敗の数値は元結果のまま。
元selection/metricsのbytes不変も確認。normal/recoveryとcollection_sliceの集計は補足を参照。
実行版の注釈読取コード自体は再評価せず残しているため、将来利用時にalias対応が必要。

補足上のmainはnormal64件、recovery96件。
normal ADE 0.039131m、recovery 0.015880m。
recoveryには1.5m/2.0m支持が0件であり、短距離支持の平均とnormalの長距離平均を
単純に「recoveryの方が優秀」と比較できない。

## run/shape別baseline比較

すべてmain内、単位m、anchor平均。roleを混ぜたgroup値を使っていない。

| run（各32件） | model | straight | train mean |
|---|---:|---:|---:|
| 20260902-131505 | 0.039959 | 0.064040 | 0.064842 |
| 20260902-132822 | 0.038304 | 0.057147 | 0.057506 |
| recovery_left_far_early_r03 | 0.017377 | 0.021359 | 0.021589 |
| recovery_left_near_early_r01 | 0.015123 | 0.020386 | 0.019728 |
| recovery_right_far_early_r04 | 0.015140 | 0.022582 | 0.021979 |

| shape | 件数 | model | straight | train mean | 対straight勝/敗 |
|---|---:|---:|---:|---:|---:|
| left | 48 | 0.036866 | 0.056059 | 0.057970 | 45 / 3 |
| right | 53 | 0.025072 | 0.052659 | 0.049737 | 51 / 2 |
| straight | 59 | 0.015771 | 0.007707 | 0.008847 | 7 / 52 |

この5 runsの平均では全runでbaselineを下回る誤差だが、直線shapeの弱点は前段同様に残る。
5 runsから強いCI/安全主張はしない。形状分類はfuture幾何でありroute intentではない。

| 距離grid | main支持母数 | model点誤差m |
|---|---:|---:|
| 0.5m | 160 | 0.010380 |
| 1.0m | 114 | 0.024902 |
| 1.5m | 60 | 0.057444 |
| 2.0m | 58 | 0.102357 |

2m地点はnormalだけの58件。全体ADE2.52cmを「全20点/全runで2.52cm精度」と読まない。
最悪main：`20260902-131505__epoch0000__76292918933`、left、ADE0.126394m、
2m地点誤差0.308963m。教師の曲がりより予測が弱い図を収録。
最悪観察：`20260902-131505__epoch0000__563792918933`、ADE0.791533m、支持7点。
観察をmainへ混ぜず、除外flag/未知停止意図等と一緒に検討する。

proper crossingは今回のmain/観察とも全20点0件、teacher支持prefix0件。
原点集中0、全出力有限。ただしcollinear/touchingを検出しないため、これを安全判定にしない。
前段観察のtail交差結果と今回の別集合を混同しない。曲率/車体clearance/controller追従未検証。

## テスト・ログ・実行時間

推論版の非学習tests **48 passed / 7 warnings / 9.07s**。
内訳は新evaluation tests、既存純粋teacher幾何、package安全検証。
optimizer構築を禁止するfixtureあり。既存合成optimizer tests、全pytest、raw/ROS testは未実行。
注釈後処理testsは別版で **6 passed / 2.16s**。別JUnitと生ログを保存。

実行コマンド（native checkout cwd、stdout/stderrは別logへ保存）：

```bash
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -vv tests/test_spatial_diagnostic_validation_v4.py tests/test_spatial_diagnostic_geometry_v4.py tests/test_spatial_diagnostic_package_v4.py --junitxml=/home/thistle/e2e_autonomous/runs/spatial_v4_validation_logs_153a22a/junit.xml
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python tools/evaluate_spatial_diagnostic_validation_v4.py --config configs/spatial_diagnostic_validation_v4.yaml --output /home/thistle/e2e_autonomous/runs/spatial_validation_v4_20260906_153a22a
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -vv tests/test_spatial_validation_annotation_addendum_v4.py --junitxml=/home/thistle/e2e_autonomous/runs/spatial_validation_v4_20260906_153a22a/logs/annotation_junit.xml
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python tools/annotate_spatial_validation_results_v4.py --run /home/thistle/e2e_autonomous/runs/spatial_validation_v4_20260906_153a22a --ledger /home/thistle/e2e_autonomous/runs/spatial_v4_coverage_full_v2_20260905/anchor_audit_ledger.csv
```

本評価開始UTC 2026-09-05 23:30:03.848734、終了23:31:06.067679。
active58.5467s（上限1800s、前処理を含む）、wall約62.22s。最終metrics/hash/図I/Oはactive外。
RTX4080、float32/CUDA、GPU peak allocated153,855,488 bytes。詳細版はenvironment.json。
time-limit停止なし。annotation後処理は別の報告工程で、追加推論は0。

自己点検で244futureのbyte hashと教師再変換、11 NPZのnon-object、
全model/baselineのper-anchor ADE、train template一致を確認した。
独立レビュー担当が実行した検証ではなく、`logs/numerical_selfcheck.log`は実装者の再計算記録。

## 成果物と未確認範囲

新runに全244選択future、train再現・全差分、val全予測・全baseline・XY/mask/grid、
candidate/processing ledger、各role/groupとpaired指標、代表val2件の実input tensors/履歴hash、
最大8図、checkpoint load map/全state前後hash、生stdout/stderr/JUnit、注釈補足を保存。
レビューZIPは実行版repo/と後処理版source/diffを分離する。

重み本体、root manifest、原sensor、全Dataset/rawは非同梱。
第三者がcheckpoint再推論や原sensorからの前処理再構築をできない限界は残る。
元futureと数値だけの再計算は可能。梱包hash/展開検証は原本の物理正当性を証明しない。
ZIPの正確なサイズ/hash/entry検証は別verification logと最終引渡しに示す。

teacher_adoption_approved: false

runtime_deployment_approved: false

new_training_optimizer_steps: 0

raw_reads_performed: 0

test_asset_reads: 0

## 次の1判断

**本学習・走行への移行はまだ承認せず、直線shapeの系統誤差を次の限定診断候補にする。**
直線baseline未達はtrainと今回valの両方で観測されたが、その原因が最適化・表現・教師品質の
どれかはUNKNOWN。既存の固定教師/予測の残差から、距離別・左右偏りを調べることを提案する。
新規データ不足を今回だけで断定せず、収集・MPC変更・再学習・追加評価は自動実行しない。
