# Spatial Path V4 最初の診断学習：実行結果

## 結論

- IMPLEMENTATION_COMPLETE: YES（今回の限定仕様）。
- OVERFIT_EXECUTED: YES（独立smoke 1 step、本診断500 optimizer steps、skip 0）。
- FIT_TARGET_MET: YES（固定train 64 anchorsのADE 0.90707765 → 0.01300991 m、98.5657%改善）。
- teacherの物理正当性、未見run汎化、停止発進、安全性、controller追従：すべてUNKNOWN。

これは保存済み観測教師への適合診断であり、「走れるモデルが完成した」結果ではない。
独立レビューは未実施。下記の再計算は実装者による自己点検である。

## 版と許可範囲

Repo: https://github.com/fis-teria/aichallenge_lite_transfuser

Branch: `codex/windows-wsl-training-sync`

開始版: `66749534ef3b0b3dc4c82143734240c25aff149f`

実装初版: `e49e363e06cf24ecd71e0334c6ef7f312970324a`

実装・最終tests・学習実行版: `f33b197df9eb1e6ae2af70c9661d8b1d88ab4ef0`、dirtyなし。
初版からの追加変更はreview packageの相対import依存収録だけ。両版で限定48 testsが通過。
本レポートは学習後の別commit。正確な結果追記・梱包commitはZIPの
`provenance/changed_files.json`に記録する。ZIPのrepo/は実行版のGit bytes。

Windowsでcommit→CheckOnly→通常syncし、同一SHAのnative WSL checkoutでlock付き実行。
Windows/WSLからpushしていない。process停止、lock削除、reset、既存変更破棄なし。

変更は診断view/input adapter/path model/loss/train/eval/tests/config/docs/packaging。
旧V3は`forward_features`の抽出だけ変更し、既定head出力とstate互換をテストした。
「旧V3を一切変更していない」とは主張しない。S1/S2/Ledger、runtime、MPC、Safety、
ROS、既存filterのコード・設定は変更なし。診断modelはruntimeの出力interfaceを持たない。

新規raw/MCAP/bagの存在確認を含むアクセスなし。val/test sensor/futureアクセスなし。
外部データ/weights取得、正式teacher採用、active checkpoint登録、本学習、再実験なし。

## Datasetと選択

Dataset: `/home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3`

実測metadata: 26 runs / 72,697 anchors、run split 16 train / 5 validation / 5 test。
train metadata 45,190件。metadata source_hashのsplit間重複は0。
これはシーン独立性やraw record一意性の検証ではない。

| identity | SHA-256 |
|---|---|
| Dataset内部 | `181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388` |
| manifestファイル | `d625f42ca05a18ea76952376c6392268191c4895d6e605e0c49ceaa66dcbe1de` |
| splitファイル | `7d0e433dbd032ad695227051573e7d8d17072fa4ea3b4e28f4c44f56fde27b4f` |
| selection | `698aba59f8cb7779006a8b070b6b00b406e8c93922f5fade252d19205907096b` |
| teacher contract | `cdb668834d4baf60c31fa5f934d01d8782f02544bc6fa29053d5a850432cc344` |
| input contract | `77fff3f9c5875b6f116cebe35fde88d42d459aa04c195676c44e5c6caca5edc7` |

既存coverage ledgerのファイルhashも期待値と照合。候補futureを2,048件読んで固定選択。
sensor読取は選択80件のpast-only履歴のみ。終了時にmanifest/splitと既読assetを再hashし、
変化なしを確認した。元canonicalは書き換えていない。原本を再検証した意味ではない。

学習64件は16 train runsすべてから選択。同run内0.5秒以上の間隔。
normal 37 / recovery 27、幾何左22 / 右21 / 直線21。
collection sliceはunknown 37、offset_left_far 9、offset_left_near 9、offset_right_far 9。
shapeは将来観測幾何の診断分類で、Reference/route intent/収集annotationではない。
annotation不足、source品質、confirmed episodeのUNKNOWNはselectionに残した。

学習loss対象64 anchors / 977点。観察専用16件のうち支持あり13件 / 165点。
支持なし3件はゼロ教師にせず採点外。観察集合には停止、短支持、hold、反転を保持。
観察集合は同じtrain split由来で、validationではない。自動的にoptimizerへ加えていない。

## 教師・入力・model/loss

教師はh30 [30,8]の連続prefixをbase_link@t_obsのまま弧長0.1m刻み20点に変換。
後輪軸への推測移動、欠損越し接続、X単調条件、外挿、終点複製なし。
5mm低速処理は最後の信頼点からの累積であり、毎frameの小移動を全消去しない。
時間/jump/hold/角切り閾値は暫定。raw/processed/resampled長、endpoint XY、切断理由を保存。
teacher maskは観測支持だけ。原点、invalid tail、支持なしanchorをloss/ADEへ入れない。

入力は4/4/10/10、RGB 224x384、LiDAR 750、ego 4特徴、過去command 3特徴。
代表2記録anchorで9 tensor fieldsをV3 lazy materializerとrtol=atol=0で照合してPASS。
同じepoch keysにおける限定照合であり、runtime全体のparityではない。
診断inputはgap/resetでもepochを切る。sensor_dtは保持するが既存backboneでは融合しない。
未来state/教師mask/IDをforwardへ入れない。teacherのみ変更して出力不変のテストあり。

初期化はSCRATCH。具体的な信頼済みcheckpointパスがないため探索・downloadなし。
V3のencoder/fusion構造を使い、時間trajectory/speed/旧補助headは除外。
新しいMLP path head [B,20,2]。trainableは137 parameter tensors、12,221,128要素。
allowlistロード機構は合成テスト済みだが実学習でV3 weightsをロードしてはいない。
転移学習の成功/失敗を判定する実験ではない。

SmoothL1 beta 0.1mのみ。演算前にmask外を安全化し、有効点をanchor内平均→anchor間平均。
旧speed/control/stop/behavior/plan consistency loss、曲率強制、経路伸長lossなし。
float32、seed42、fresh AdamW wd0、backbone LR 1e-4 / head 1e-3、clip norm1、
microbatch2 x accumulation4。augmentation/schedulerなし。設定/選択は学習前固定。

## 実行記録

Run ID: `spatial_diagnostic_v4_20260906_f33b197`

Native run: `/home/thistle/e2e_autonomous/runs/spatial_diagnostic_v4_20260906_f33b197`

開始UTC: 2026-09-05 17:46:07.502175、終了UTC: 17:48:59.373076。
学習・評価active 144.3996秒（上限1,800秒）。前処理/保存を含むwall約171.87秒。
RTX4080、PyTorch 2.7.1+cu128、CUDA12.8、GPU peak allocated 552,288,768 bytes。
GPU memoryはPyTorch allocator計測値で、GPU全体の使用量ではない。
Python等の実測版・空き容量は`provenance/environment.json`。

smoke1 stepはmodel複製＋別optimizer。mainへ継承せずfresh optimizerで500 steps。
50 stepsごとの同一集合評価を保存。最終はstep500、best snapshot選択なし。
NaN/OOM/入力不一致なし、skip0、time-limit停止なし。自動再試行なし。

実行コマンド（native checkoutをcwd、stdout/stderrを同名logへ別々に保存）：

```bash
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -vv tests/test_spatial_diagnostic_geometry_v4.py tests/test_spatial_diagnostic_training_v4.py tests/test_spatial_diagnostic_package_v4.py tests/test_full_control_lite_v3_shape.py tests/test_runtime_input_history_v3.py --junitxml=/home/thistle/e2e_autonomous/runs/spatial_v4_logs_f33b197/junit.xml
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 bash tools/with_wsl_training_lock.sh .venv/bin/python tools/train_spatial_diagnostic_v4.py --config configs/spatial_diagnostic_v4.yaml --output /home/thistle/e2e_autonomous/runs/spatial_diagnostic_v4_20260906_f33b197
```

最終実行版tests: **48 passed、8 warnings、4.09秒**。warningsはTransformer nested tensor設定。
全pytest/実raw/ROSテストは依頼の限定範囲に従って未実行。
`logs/tests_stdout.log`, `tests_stderr.log`, `junit.xml`, `train_stdout.log`,
`train_stderr.log`, `metrics.jsonl`が生記録。正常時stderrも空ファイルとして保存。

## 同一集合の結果

ADEはanchor平均Euclidean距離、単位m。有効maskを予測によって縮めていない。

| 評価 | 学習64件 | 観察16件中13件 |
|---|---:|---:|
| 初期 | 0.907078 | 0.820856 |
| 最終step500 | 0.013010 | 0.104641 |
| 全点原点 | 0.812469 | 0.677254 |
| train距離別平均template | 0.040327 | 0.110157 |
| 常時直線 x=s, y=0 | 0.040131 | 0.109784 |

学習SmoothL1 0.484929 → 0.000586974。原点集中0。全20出力点有限。
学習予測のproper self-intersectionは64→0、観察は16→2。
この判定はcollinear/touching交差を網羅しない。曲率/接線は未算出（null）、ゼロとはしない。

| 教師距離 | train支持母数 | 最終点誤差m |
|---|---:|---:|
| 0.5m | 64 | 0.009098 |
| 1.0m | 52 | 0.012216 |
| 1.5m | 35 | 0.015826 |
| 2.0m | 33 | 0.018581 |

最終train slice ADE: left_far 0.012404 / left_near 0.011920 /
right_far 0.010990 / unknown 0.013914 m（unknownが最悪、37件）。
最悪train anchorは`20260902-152210__epoch0000__359190514258`、ADE 0.021109m。
run/shape別全数値、各anchorのraw/processed/resampled/予測支持内長はJSON/CSVに保存。

学習後のCamera/LiDAR cyclic swapを1回実施。全20点の平均出力変化0.049348m、
train ADE 0.046981mへ増加。センサ入力への感度は観測されたが、ego/command shortcutの
不在やセンサ理解を証明しない。追加学習はしていない。

形状分類は3種類あるものの、直線baselineでも約4cm。幾何の曲がりが穏やかな集合であり、
難しい回避・強い曲率の獲得とは言えない。最終はbaselineより良いが、観察集合では
改善は小さくADE約10.5cm。観察は停止/反転等を含む別分布で、汎化性能とは呼ばない。

## レビュー成果物と限界

80件の元future.npyは元bytesのまま、対象metadata、XY/mask/grid、初期/最終の全予測、
3baseline、代表2件の実input tensorsと履歴ID/hash、合成fixture5件、生ログ、最大8図を収録。
自己点検で80futureのhash、80教師の再変換、11 NPZのnon-object、全予測ID/有限性、
初期/最終ADEの再計算一致を確認。`logs/numerical_selfcheck.log`に記録。
これは独立監査ではない。元sensor例を省略し、前処理を第三者が原画像から再構成できる
範囲には限界がある。root manifestと全Dataset/rawも非同梱。

初期/最終weightsはnative run/checkpoints内だけで保存、各約49MB、ZIP非同梱。

- initial SHA: `168328a33a795a67c67995779b8a15c62fad9dd15b8861541132e673181ff0d9`
- final SHA: `0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`

checkpoint非同梱なので第三者の再推論はできない。hashは教師の物理正当性を証明しない。
packageは実行版ソースと結果追記版を分離し、依頼付録の独立レビュー文を実値だけ埋めて保存。
AGENTS/依頼/コマンドは履歴資料であり自動実行許可ではない。
ZIP実サイズ/hash/展開検証結果は最終引渡しとZIP外の検証ログに記載する。

## 次のボトルネックを1つ

**UNKNOWN**。小集合への適合不能は今回観測されず、teacher品質・入力情報・汎化の
どれが次の支配的制約かをこの実験だけでは特定できない。
次の判断材料として固定run分割の限定評価を検討できるが、本タスクでは実行しない。
本学習、新規収集、停止発進、runtime/controller接続は自動開始せず、独立レビューへ渡して止める。
