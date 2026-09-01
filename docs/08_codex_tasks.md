# 08. Codex向け実装タスク

以下は1タスクずつ実行する。大きな依頼にまとめない。

## Task 01: Dataset audit

```text
このリポジトリのschemas/index_columns.mdを契約として、index.csvを検査する
 tools/dataset_audit.py を実装・改善してください。
欠損ファイル、必須列、NaN、数値範囲、run/scenario分布、stop比率、
操舵・速度・加速度の統計をreport.jsonへ出してください。
Done when:
- pytestが通る
- 不正データに非0終了コードを返す
- READMEに実行例を追加する
```

## Task 02: ROS bag extractor

```text
使用するbagのtopic/type一覧を前提に、Camera、LaserScan、velocity、steering、controlを
canonical datasetへ抽出するツールを実装してください。
同期方式と許容差をconfig化し、各sampleのstream deltaをindexへ保存してください。
Done when:
- 10秒bagからindex.csvとimage/npyが生成される
- 欠損streamを明示する
- future情報をinputへ混ぜない
```

## Task 03: LiDAR preprocessing

```text
src/aic_transfuser_lite/data/lidar_preprocess.pyを契約として、
LaserScanのNaN/inf/0/range外処理、0-1正規化、valid mask、任意のBEV変換を実装してください。
Done when:
- shapeと有限値をpytestで確認
- 角度原点・正方向を引数化
- 境界条件をテスト
```

## Task 04: PyTorch Dataset

```text
index.csvからimage、lidar、ego、waypoints、speed、stop、modeを読むDatasetを実装してください。
relative pathはindex.csvの親を基準に解決してください。
Done when:
- 1 sampleとbatch shapeのtestが通る
- 欠損列に分かりやすい例外を出す
- optional labelをconfigで無効化できる
```

## Task 05: LiDAR-only baseline

```text
LiDAR 1080点 + ego stateからwaypoints、target speed、stop logitを出す軽量baselineを実装してください。
Done when:
- forward shape testが通る
- synthetic datasetで1 epoch学習できる
- checkpoint save/load testが通る
```

## Task 06: Safety Supervisor

```text
停止距離、前方sector、sensor timeout、NaN command、command clamp/rate limitを実装してください。
ROS依存をcore logicから分離してください。
Done when:
- obstacle/timeout/nan/normalのunit testが通る
- 介入理由をenum/stringで返す
- しきい値にhysteresisを追加できる設計
```

## Task 07: Camera-only and late fusion

```text
ResNet18 Camera encoderとLiDAR encoderを使い、concat+MLPのlate fusion baselineを実装してください。
Transformerはまだ使わないでください。
Done when:
- camera-only/lidar-only/late-fusionを同じtrain scriptで切替可能
- parameter countを表示
- smoke testが通る
```

## Task 08: Transformer fusion

```text
camera tokens、lidar tokens、ego tokenにmodality/position embeddingを加え、
TransformerEncoderで融合するAICTransFuserLiteV0を実装してください。
Done when:
- configs/transfuser_lite_v0.yamlから構築できる
- forward、loss、train smoke testが通る
- late fusionと同じoutput contractを保つ
```

## Task 09: ROS inference node

```text
公式環境の実topic/typeを確認した上で、同期済みsensorをmodelへ渡し、
debug outputとnominal commandをpublishするROS 2 nodeを実装してください。
Done when:
- model load失敗を明示
- p50/p95 inference latencyを記録
- stale sensor時にcommandを出さない
```

## Task 10: Closed-loop evaluator

```text
scenarioごとに完走、衝突、offtrack、stop判定、safety介入、latencyを集計するツールを実装してください。
Done when:
- run単位JSONと全run summary CSVを出す
- baselineとの差分を表示
- failure timestampを抽出する
```
