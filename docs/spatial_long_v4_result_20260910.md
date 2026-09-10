# 20 m版：教師生成・初回WSL診断学習の結果

20 m/46点の診断教師生成、独立少数例100 step、本診断500 step、
run分離validation、checkpoint保存・再読込まで完了した。
これは**提案モデルの初回offline診断**であり、走行可能な20 mモデルの完成ではない。
近距離の基準値超過、遠距離誤差と自己交差が残るためruntimeへ昇格していない。

## 実行版・保存先

- 実行commit: `9bc02287c862511f36bdb31f634bf9ade3b55d3a`、実行WSLはclean。
- Windows正本: `E:\workspace\e2e_lite_transfuser`。
- 途中で入った2 m側commit `1ab0ad6`を含む親から実装。control/ROS/AWSIM側は編集していない。
- WSL出力: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01`。
- WSLログ: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_logs01`。
- 最終重み: 出力内 `final.pt`。初期重み `initial.pt` も保全。Windowsへの重みコピー・Git追加なし。
- 最終重みSHA256: `0cb82d3dcfba81e11fbd89c51b9e793eca00849d1cb551c33c5398568f8394f6`。
- Windows小成果物: `tmp/spatial_long_v4_20260910/` のmetrics、execution、verification、PNG。

Windows commit → CheckOnly → 通常sync → WSL共有lockで実行した。
既存2 mモデル、最終test、既存lock/processを保全。AWSIMホスト操作・pushなし。
実行条件と再現コマンドは `docs/spatial_long_v4_diagnostic_20260910.md`。

## 教師確認と生成

全データ監査ではなく、事前固定のtrain 2,048 / validation 1,024候補を監査した。
Dataset内部identity `181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388`、
manifestファイル `d625f42ca05a18ea76952376c6392268191c4895d6e605e0c49ceaa66dcbe1de`、
splitファイル `7d0e433dbd032ad695227051573e7d8d17072fa4ea3b4e28f4c44f56fde27b4f`、
coverage台帳 `35781616e8faab5117b0d9da7c8560519c5f196383ea66ab1465d999f5645e35`
を照合。各futureはcanonical manifestのsize/hashを検証し、run/segment/epoch/splitへ対応付けた。
台帳の`val`表記は正式splitの`validation`へ正規化して照合した。

連続h30 prefixの時刻・欠損・停止・reverse・jumpを確認し、
5 mm処理後の折線と46点再サンプリングの全支持距離で角切り・双方向距離上界を算出。
原点は幾何補助のみ。欠損を越えた再開、別anchor連結、遠方外挿は行っていない。
0.15 m距離上界、max(0.1 m, 支持弧長2%)角切りという暫定診断閾値を学習前に固定した。
2 m区間の角切り判定を20 m品質判定に転用していない。

| 集合 | 監査候補 | 診断適格・生成 | 学習/評価へ選択 | 選択run数 | 選択20 m支持 |
|---|---:|---:|---:|---:|---:|
| train | 2,048 | 1,786 | 128 | 16 | 28（7 run） |
| validation | 1,024 | 897 | 64 | 5 | 12（2 run） |

診断適格**2,683件すべて**を `diagnostic_teachers.npz` へ保存。
選択192件の教師は `selected_teachers.npz`、全候補理由は `teacher_audit.json`。
train/validationとも長距離角切り超過1件を除外。
支持なし231/110件、長時間hold136/71件、reverse4/2件、方向反転8/3件などを記録した
（理由は重複するため加算して除外総数にはしない）。今回候補ではjump/時刻異常/nonfiniteによる除外0件。

選択trainはnormal101/recovery27、validationはnormal46/recovery18。
幾何形状はtrain直線55/右47/左26、validation直線31/右21/左12。
形状は観測幾何の分類でありroute intentではない。短い復帰prefixの遠方はmask false。
選択trainの未観測点2,391 / 全5,888点、validationは1,338 / 全2,944点。
これらはloss・誤差へ入らない。支持のない原点集中予測を正解として扱っていない。

全候補の実走行教師適格性は引き続きUNKNOWN。canonical bytesの照合はraw収集の物理正当性、
障害物clearance、教師経路の安全性の検証ではない。隣接anchorは独立場面ではない。

## 学習と検証

Camera + 2D LiDAR + ego + 過去外部command、履歴4/4/10/10を使用。
forwardにはfuture/teacher/maskを渡さない。既存表現の構造を再利用した**scratch初期化**で、
既存重みのロードは0。旧2 m学習コード・重みは変更していない。

近0–2 m（20点）、中>2–10 m（16点）、遠>10–20 m（10点）の観測点を距離帯内平均し、
存在する距離帯を等重み、その後anchor平均。SmoothL1 beta0.1 m、AdamW、
backbone LR1e-4/head1e-3、clip1、microbatch2×accumulation4、float32、seed42。

少数8件100 step: 観測点平均誤差 **5.685400 → 0.241943 m**。
最初のstepでshape/有限勾配を確認し、100 step後の重み・optimizerはmainへ継承していない。
その後のmainは500 step完了、best選択なしの最終stepを保存・評価。
validationを閾値調整・早期停止・checkpoint選択に使用していない。

学習・評価active179.718秒（上限900秒）、準備/保存を含むwall218.962秒。
RTX4080、PyTorch2.7.1+cu128、CUDA12.8。PyTorch peak allocated601,893,376 bytes
（約574 MiB、GPU全体占有量ではない）。NaN/OOM/予算停止なし。

WSL限定テスト25 passed。全体pytestは**1,814 passed / 4 skipped / 52 warnings、87.78秒**。
skipはOSQP、完全なDraft2020 validator、jsonschema、任意公式packageの不足。
新規mask勾配・距離帯重み・20 m形状異常・shape/例外条件を検証した。

## Run分離validation

各anchorの**観測された帯内点のEuclidean距離平均**をanchor間平均した値、単位m。
母数は帯ごとに異なる。支持maskは予測によって縮めていない。

| 距離帯 | train最終 | val初期 | val最終 | val直線基準 | val train平均基準 | val anchors / points / runs |
|---|---:|---:|---:|---:|---:|---|
| 0–2 m | 0.08174 | 0.86056 | **0.08640** | 0.04182 | 0.04008 | 64 / 980 / 5 |
| >2–10 m | 0.35061 | 5.48786 | **0.48951** | 1.64474 | 1.49631 | 33 / 457 / 2 |
| >10–20 m | 0.96146 | 12.89170 | **3.21146** | 9.01150 | 7.82360 | 21 / 169 / 2 |

20 m**地点のみ**の誤差はtrain2.15165 m（28件/7 run）、
validation **5.22622 m（12件/2 run）**。これは上の遠距離帯平均とは異なる。
validation 2/5/10 m地点は0.18099 / 0.34958 / 1.09570 m。

中・遠距離は両基準値より良いが、近距離は両基準値より悪い。
支持区間内の予測proper self-intersectionはtrain28/128、validation19/64。
この交差検査はcollinear/touchingを網羅しない。曲率・動的追従・障害物回避は未評価。
図の20 m支持例でも遠方の旋回を十分に再現しない例がある。
損失重み、学習量、入力情報、データ分布のどれが支配原因かは未切り分け。

`validation_paths.png` は20 m支持12件を観測点ADE順に並べた6分位例。
良い例だけの選択ではない。基準値、run/形状別、全予測はJSON/NPZを参照。
遠距離はvalidation全5 runのうち2 runだけなので、広い場面への汎化とは呼ばない。

## 保存後の自己検証と実行上の訂正

- 出力14ファイルのSHA256を照合し、選択192教師と生成2,683教師の一致、maskの連続prefix、
  支持上限、train/validation run重複0、最終予測shape/有限性を確認。
- 距離帯指標を保存予測から別スクリプトで再計算して一致。
- `final.pt`をstrict loadし、canonical入力からvalidation2件を再構成。
  学習と同じTF32無効設定で保存予測との差**0.0 m**。
- 学習終了時に既読future/sensor/metadataとmanifest/splitを再hashしてPASS。
- 最初の起動はPYTHONPATH不足でデータ読込前に失敗。元ログを保全し、`PYTHONPATH=src`で修正起動。
- 再読込検証の初回は検証側のcuDNN TF32既定設定により最大0.00091457 m差で不一致。
  許容誤差を緩めず学習時の設定へ揃え、再照合0差。学習のやり直しはしていない。
- 成功/失敗両ログと検証script `verify_precisionfixed.py` を専用logsディレクトリに保存。
  `verification.json`は実装者の自己点検で、外部独立レビューではない。

code/config/teacher/checkpoint/artifact hashesは `execution.json`、
source asset hashes・全選択履歴は `input_provenance.json`、
全optimizer stepは `training.jsonl`、少数例診断は `overfit.json` に保存した。

今回の有限診断は終了。全データ教師監査・本学習、近距離劣化と自己交差の原因切り分け、
遠方形状の改善、ROS/AWSIM/closed-loop検証は未完了。追加学習やruntime昇格は自動実施していない。
