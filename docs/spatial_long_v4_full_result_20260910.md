# 全監査済みtrain教師による20 m継続学習

## 完了内容と判断

未使用1,530件を含む、監査済みtrain教師**全1,786件を16 epoch**使用した。
毎epoch1,786件を重複・取りこぼしなく1回ずつ学習し、**3,584更新 / 28,576提示**を完了。
入力キャッシュ、教師照合、学習、固定validation、全train評価、保存後検証はnative WSLで実施。

最終16 epochのvalidationは開始時に対して近距離20.81%、遠距離11.80%、20 m地点5.02%改善。
ただし**中距離は20.41%悪化し、遠方は12 epochから16 epochで悪化**した。
全件利用は実行できたが、学習を長くすれば全指標が改善するという結果ではない。
12 epochと16 epochの重みを両方保全し、bestへの自動選び直しやruntime昇格は行っていない。

## データ・条件

今回の全件とは、既存2,048候補の限定監査で適格だったtrain1,786件。
元canonical dataset全体を監査・学習した意味ではない。新しい候補監査や最終test読込は実施していない。
元の監査済み教師を全件利用したため、前回の0.5秒間隔による間引きは適用していない。
隣接anchorは相関し、1,786件を独立した1,786場面と解釈しない。

- Train: 通常走行766 / 復帰1,020、16 run。復帰比率は前回54/256から1,020/1,786へ変化。
- Validation: 前回と同じ64件・5 run。学習やcheckpoint選択には使用していない。
- 46点、0.1–2 m/2.5–10 m/11–20 mの観測済みXY/mask。原点・未観測遠方は採点外。
- Camera + 2D LiDAR + ego + 過去外部command。future/teacher/maskをforwardへ渡さない。
- 初期化: 前回 `expanded256_2000.pt` のmodelとoptimizerをstrictに継続。
  元optimizer更新数2,500。scratchではない。RNGは44へ設定し直した。
- model/loss/LRを維持。等観測距離帯SmoothL1 beta0.1 m、AdamW LR1e-4/1e-3、clip1、
  batch8、microbatch2、float32、TF32無効、scheduler/augmentationなし。
- 最後のbatch2件も残し、勾配蓄積を実際のbatch件数で正規化。

| 距離帯・地点 | Train anchors / points / runs | Validation anchors / points / runs |
|---|---|---|
| 0–2 m | 1,786 / 23,686 / 16 | 64 / 980 / 5 |
| >2–10 m | 635 / 7,979 / 7 | 33 / 457 / 2 |
| >10–20 m | 350 / 2,549 / 7 | 21 / 169 / 2 |
| 20 m地点 | 165 / 165 / 7 | 12 / 12 / 2 |

validationの遠方評価は実質2 run。以前から観測した開発用validationであり、新しい独立最終評価ではない。

## 固定validationの推移

単位m。anchorごとに観測された帯内点のEuclidean誤差を平均し、anchor間で平均。
自己交差は観測支持区間内のproper crossing検出で、接触/collinear交差を網羅しない。

| 追加epoch | 近距離 | 中距離 | 遠距離 | 20 m地点 | 自己交差 |
|---|---:|---:|---:|---:|---:|
| 0（前回重み） | 0.05394 | 0.30177 | 1.91192 | 2.95686 | 0/64 |
| 4 | 0.05589 | 0.35006 | 1.59566 | 1.94237 | 1/64 |
| 8 | 0.05403 | 0.32319 | 1.26167 | 1.79220 | 1/64 |
| 12 | 0.04987 | 0.26663 | **1.05815** | **1.32976** | 0/64 |
| 16（最終） | **0.04271** | 0.36337 | 1.68628 | 2.80850 | 1/64 |

12 epochは今回保存した固定snapshotの中で中・遠距離の誤差が小さい。
これは観測後の比較であり、12 epochを独立評価済みの最適モデルとして採用した意味ではない。
最終16 epochの近距離は直線基準0.04182 mと近いが、まだ僅かに高い。

8 epochではrun別の観測点平均誤差が5 validation runsすべてで開始時より低下。
最終16 epochでは通常run `20260902-132822` が開始時0.21119→0.25384 mへ悪化し、
ほか4 runは低下した。単一runや平均値だけで全体成功とは判断しない。

## 全train集合の最終評価

全1,786件にも最終重みで推論・評価した。母数は上表のtrain列。
近0.03387 m / 中0.30277 m / 遠0.94878 m / 20 m地点1.80647 m。
支持区間内の自己交差10/1,786。前回のtrain128件とは母集団が異なるため直接比較しない。
比較用の元train128件についても、開始時と4/8/12/16 epochの予測・指標を別途保存している。

## 保存先と再現

Windows正本実行commit: `bd31c0cf135d6897687de9b534c04554e831c4e4`。
Windows commit → CheckOnly → 通常sync → WSL共有lockの順で実行し、実行worktreeはclean。
コマンドと有限予算は `docs/spatial_long_v4_full_20260910.md`。

- WSL run: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_full_20260910_run01`
- WSL logs: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_full_20260910_logs01`
- 入力cache: run内 `inputs/`、1,850件（train1,786＋val64）、7,689,567,550 bytes。
  float32前処理tensorを保存し、CPU上は最大16件のLRU。cache hashと入力履歴を保全。
- 最終checkpoint: run内 `epoch_16.pt`。
- 比較候補: `epoch_04.pt`, `epoch_08.pt`, `epoch_12.pt` も保全。
- Windows小成果物: `tmp/spatial_long_v4_full_20260910/summary.json`, `execution.json`, `learning_curve.png`。

| Checkpoint | SHA256 |
|---|---|
| epoch_04.pt | `df1deb3a7f454a44195f2f8564095a140c2c5d13649b1a6999d29eea23b77dd7` |
| epoch_08.pt | `edb860ba0e32f9c339f79620630de7bec8f700a98b39ff18e685a4668539e785` |
| epoch_12.pt | `07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33` |
| epoch_16.pt | `a7ffdf4604ae7385d6e003f30486b83a146d213548ad130330414924e5d696f0` |

## 検証と残る課題

- WSL限定17 passed、全体 **1,829 passed / 4 skipped / 52 warnings、70.96秒**。
  skipは既存のOSQP、完全なDraft2020 validator、jsonschema、任意公式package不足。
- 全epochで訪問集合が1,786件・全件の訪問回数がepoch番号と一致することを実行時確認。
  保存ログでも各epoch224更新、末尾2件を含む1,786提示、計28,576提示を再確認。
- 使用futureをhash照合して全教師を再生成・完全一致。代表2実anchorでcache/rebuildの
  全9入力tensorをrtol=atol=0で照合。teacherはcacheフィールドに含めない。
- 最終時に元asset、manifest/split、親checkpoint、全1,850cacheのhashを再確認してPASS。
- 保存後、別scriptで20成果物hash、6組の保存予測の距離帯指標を再計算してPASS。
- optimizer更新数6,084（親2,500＋追加3,584）。開始時の前回予測再現、最終checkpoint
  strict reload後の代表2件再現はともに最大差0.0 m。実装者の自己点検で、独立レビューではない。
- 学習/評価/保存後照合active1,436.114秒（約23分56秒、上限2,400秒）、wall1,540.807秒。
  RTX4080、PyTorch2.7.1+cu128、CUDA12.8。GPU peak allocated546,563,584 bytes
  （PyTorch allocator値で、GPU全体占有量ではない）。NaN/OOM/時間停止なし。

code/data/teacher/cache/checkpoint hashesはexecution.json/input_provenance.json、
固定集合はselection.json、全更新はtraining.jsonl、再計算はlogs内verify_and_plot.pyに保存。
重み・データをGitへ追加せず、既存2 mモデル・control/ROS/AWSIM・最終testを保全。pushなし。

残る課題は12→16 epochの中遠距離悪化と近距離改善の両立、固定LRやbatch順序の影響、
通常/復帰比率の変化、未見run多様性、教師の物理正当性。過学習や最適化の揺れは候補で、
この実験だけで原因を確定していない。追加の学習率比較やcheckpoint選択は自動実施していない。
今回も提案モデルのoffline学習結果であり、経路安全性・障害物回避・停止・closed-loop完走は未評価。
