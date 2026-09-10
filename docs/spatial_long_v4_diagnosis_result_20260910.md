# 20 m版：学習量と使用教師数の切り分け結果

## 結論

今回の遠方誤差には、**学習へ使った教師の少なさ・カバー範囲不足が影響していた**。
同じ128件を長く学習してもrun分離validationの遠方は改善せず、
同じcheckpoint/optimizer・追加更新数で256件へ増やすと遠距離誤差が**40.06%低下**した。
この条件では、同じ少数データの更新だけを増やすより、既存の適格教師を広く使うことが優先。

元データ全体の量が不足しているとは判定していない。監査済みtrain教師1,786件のうち
今回も256件を使用しただけで、未使用1,530件がある。新規収集の必要性は未検証。
単一seed・単一の追加集合による開発用比較であり、一般的な必要データ数の推定ではない。

## 実験条件

実行commit `512c2477a5a481741f3062be959fb0aac815fed7`。
Windows正本で編集・commitし、CheckOnly/通常sync後、native WSLの共有lock内で
教師照合・学習・評価を実施した。元2 mモデル、control/ROS/AWSIM、最終testを保全。
外部レビュー、push、runtime昇格、新規データ収集は実施していない。

事前条件・コマンド: `docs/spatial_long_v4_diagnosis_20260910.md`。
親run: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_20260910_run01`。
新run: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_diagnosis_20260910_run01`。
ログ: `/home/thistle/e2e_autonomous/runs/spatial_long_v4_diagnosis_20260910_logs01`。
Windowsの小成果物: `tmp/spatial_long_v4_diagnosis_20260910/`。

- 同じ8件: 親の保存済みランダム初期重みから1,000 step、fresh optimizer。
- 同じ128件: 親step500のmodel **およびoptimizer**から追加1,000 step。
- 256件: 同じ親step500から追加1,000 / 2,000 stepを固定評価。
- 後ろ2条件はmodel、loss、学習率、batch、前処理、開始optimizerを揃えた。
  RNGは両方43へ揃えて再開。旧実行のRNGを完全再現する継続ではない。
- 教師は既存の監査済みtrainから、run×距離支持binごとの件数を正確に2倍へ拡張。
  同run内0.5秒以上の間隔を保持。16 train runsは同じで、新runは追加していない。
- validation64件・5 runは前回と同一。既に観測済みの開発用validationであり最終testではない。
- 未観測点mask、teacher閾値、近中遠の等距離帯loss、float32、TF32無効、
  AdamW LR1e-4/1e-3、clip1、microbatch2×accumulation4は変更していない。

128件はnormal101/recovery27、左26/右47/直線55、20 m支持28件/7 run。
256件はnormal202/recovery54、左60/右91/直線105、20 m支持56件/7 run。
形状構成は完全一致でなく、量とrun内カバー範囲を合わせて増やした比較である。
隣接anchorの独立性や新しいscene多様性を保証しない。

## 同じ8件への適合

8件中7件が20 mまで観測され、左3/右5、7 run。直線のみの集合ではない。
以下は今回の同一実行内の固定snapshotで、100 stepと1,000 stepでseedを変えていない。

| 更新数 | 近0–2 m誤差 | 中>2–10 m誤差 | 遠>10–20 m誤差 | 20 m地点誤差 | 自己交差 |
|---|---:|---:|---:|---:|---:|
| 100 | 0.07826 | 0.16539 | 0.66319 | 1.53582 | 2/8 |
| 500 | 0.03914 | 0.07572 | 0.20569 | 0.27538 | 0/8 |
| 1,000 | 0.03610 | 0.06502 | 0.18633 | 0.30705 | 0/8 |

少数例でも追加更新が有効だった。現モデル/lossが全く適合できないという結果ではない。
ただし500→1,000の遠距離改善は小さく、20 m地点は少し悪化。
完全な記憶に近い低誤差まで到達したとは言えず、固定LR・最適化・model/loss要因は残る。

## 同じ計算量で教師を増やした比較

単位m。各anchorの観測された帯内点のEuclidean誤差を平均し、anchor間平均。
予測でmaskを縮めていない。表の自己交差は支持区間内のproper crossing検出数。

| 条件 | val近 | val中 | val遠 | val20 m地点 | val自己交差 |
|---|---:|---:|---:|---:|---:|
| 前回128件・step500 | 0.08640 | 0.48951 | 3.21146 | 5.22622 | 19/64 |
| 128件・追加1,000 | 0.06273 | 0.46515 | **3.24948** | **5.73552** | 3/64 |
| 256件・追加1,000 | 0.06852 | 0.38577 | **1.94768** | **2.87869** | 4/64 |
| 256件・追加2,000 | 0.05394 | 0.30177 | **1.91192** | **2.95686** | 0/64 |

固定validation母数: 近64 anchors/980点/5 run、中33/457/2、遠21/169/2、
20 m地点12 anchors/12点/2 run。遠方の結論は実質2 validation runsに基づく。

128件の追加1,000 stepで、**同じ学習128件**の遠距離誤差は0.96146→0.56026 mへ改善。
一方validationは3.21146→3.24948 mで停滞。学習集合への適合と汎化の差が残った。

追加教師128件を親重みで評価した遠距離誤差は3.14022 m（52 anchors）。
元128件の0.96146 m（56 anchors）より大きく、同じrun内の未使用観測にも誤差があった。
256件の開始時train全体誤差2.01049 mを、128件の開始時train誤差とそのまま比較しない。
全snapshotで元128件だけの共通評価も保存している。

同じ追加1,000 stepでは、256件のval遠距離誤差は128件より40.06%低い。
追加500 stepでも128件3.35228 mに対して256件1.72067 mだった。
ただし途中で良かったsnapshotへ選び直さず、事前に固定した全時点を報告している。

## 追加提示回数を揃えた比較

batch8なので、128件×追加1,000では8,000提示、平均62.5回/anchor。
256件×追加1,000では平均31.25回、追加2,000では平均62.5回。
一致するのは**追加分の平均提示回数**であり、個別回数と親学習の既使用回数は揃わない。
各anchorの実際の提示回数はexecution.jsonに保存した。

256件を追加1,000→2,000へ延長すると、近0.06852→0.05394、中0.38577→0.30177 m、
自己交差4→0へ改善。遠方は1.94768→1.91192 mと改善が小さく、20 m地点は少し悪化した。
更新数だけで全指標が単調改善するとは言えない。比較には計算量とoptimizer年齢の違いも含まれる。

## 残る問題と次の優先順位

1. 既存の監査済みtrain教師を広く使う有限学習を優先する。
   今回は同じrunから追加しただけで効果があったため、直ちに新規収集不足とは扱わない。
2. 近距離精度と遠方形状は継続評価が必要。最終val近0.05394 mは直線基準0.04182 mより悪い。
   最終20 m地点も約2.96 m残る。自己交差0は固定64件でのこの検査結果だけを意味する。
3. 原因未確定として残るのは、固定LR・loss配分・model表現・入力情報の限界、
   追加run多様性の必要性、教師の物理正当性。これらをデータ数だけの問題へまとめない。

今回の結果は提案モデルのoffline診断。安全な経路、障害物回避、停止、
ROS/AWSIM/closed-loop完走・追従能力を示さず、runtimeへ昇格していない。

## 再現性・検証

全4,000更新が完了。学習/評価active1,015.111秒（約16分55秒、上限2,400秒）、
wall1,039.243秒。RTX4080、PyTorch2.7.1+cu128、CUDA12.8、
PyTorch peak allocated701,231,616 bytes（約669 MiB、GPU全体占有量ではない）。
NaN/OOM/時間停止なし。元future/教師/入力/親重みを再照合してPASS。

最終実行版のpytest: **1,826 passed / 4 skipped / 52 warnings、65.70秒**。
skipは既存のOSQP、完全なDraft2020 validator、jsonschema、任意公式package不足。
教師拡張のsplit/重複/不足/間隔/比率、mask母数、optimizer状態独立性を追加検証した。
optimizerのCPU step tensorを実験間で共有しないようdeepcopyしてからloadしている。

保存後に別scriptで32成果物のhash、11 prediction snapshotsの指標を再計算してPASS。
保存optimizerの更新回数はtiny1,000、128件条件1,500、256件条件1,500/2,500を確認。
同じ追加1,000の2条件はoptimizer年齢も一致する。各phase最終checkpointのstrict reload後、
代表2件の予測差はすべて0.0 m。train/validation run重複0。
実装者の自己点検であり独立外部レビューではない。

| WSL内checkpoint | SHA256 |
|---|---|
| tiny8_1000.pt | `3aeaf71f54fcb576ef1427ff5ced4518bc1d28ce49b779f52e3ae99d86169fd1` |
| same128_1000.pt | `8560b06ed2f561b5ca81002dae66ee2c62ca0e1ffb534e6134f50647fcbcaea4` |
| expanded256_1000.pt | `3472f321164ef28eef585b8cda11b34f1d82da70d99f70459b59c835cca50758` |
| expanded256_2000.pt | `dc350ed3e892ecdb265795328aef0c2f6940f03a0f257e45e585d13c13362bf9` |

execution.jsonにsource/親/data/teacher/checkpoint hashes、各phase/実提示回数を保存。
input_provenance.jsonにfutureと入力履歴のasset hashes、selection.jsonに固定集合を保存。
比較値はcomparison_summary.json、図はcomparison_validation.png。
logs内verify_and_summarize.pyとverify_stdout.logで自己点検を再現できる。
重み・データはWSL内に保持し、Gitへ追加していない。追加実験は自動実施していない。
