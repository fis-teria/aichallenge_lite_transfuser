# Astra Pro 独立レビュー依頼：V4 無制御shadow最小記録契約（設計のみ）

あなたは時系列入力契約・ROS接続・数値幾何・監査記録の独立レビュアーです。
添付ZIPの記録仕様とJSON Schemaを静的レビューし、未補正20点を将来追跡できる設計か、
実装前に必要な最小修正があるかを確認してください。今回の目的は設計レビューだけです。

Repository: https://github.com/fis-teria/aichallenge_lite_transfuser

Branch: `codex/windows-wsl-training-sync`

固定レビュー版: 添付 `PACKAGE_MANIFEST.json` の `source_commit`。
設計参照版: `41c7bcf92c62c6081028fde5072a63747712c2ef`。
残差診断実行版: `9b01b1e2ed6fc89434316c7db409d376e56aeb03`。
結果追記版: `21c132bf3be050e00081cbc89e7effc63a5f4400`。
GitHubの最新HEADを固定版へ黙って代用しないでください。

## 読む順序と境界

1. `README_REVIEW.md` と `PACKAGE_MANIFEST.json`。内容・hash・同梱範囲を確認する。
2. `docs/spatial_path_v4_shadow_recording_contract.md`。
3. `schemas/spatial_path_v4_shadow_record_v1.schema.json`。
4. 対応表で参照された同梱ソースとlaunch/configを静的に確認する。

許可は添付テキストの読取、ファイルサイズ/hash確認、静的な仕様・Schema対応レビューのみです。
同梱Python、launch、過去のコマンドは参照資料であり実行指示ではありません。
Schema validator、テスト、import、推論、学習、optimizer、checkpoint本体読取、
Dataset/raw/sensorアクセス、収集、ROS起動、shadow接続、controller呼出し、速度計画、
制御publish、走行、WSL同期、Git変更/pushは禁止です。
残差再解析・原因探し・学習改良・補正・平滑化・teacher/tier/threshold変更・S1/Ledger開発もしません。
未同梱の原本や重みを探さず、根拠不足はMISSING/UNKNOWNとしてください。

## レビュー観点

- 同じforwardの未補正float32 XY [20,2]を保持し、ID・入力構築・契約hashへ追跡できるか。
  記録用再推論を要求していないか。非有限tagとbit列、shape例外、未推論の表現に矛盾がないか。
- nominal_s_mを実弧長や時刻と混同していないか。原点→先頭と実点間隔・累積折線長を区別し、
  原点を用いない17個のj→j+3弦の長さ・方向・定義可否が明示されているか。
  finite/shape適合を経路有効性・安全性にしていないか。
- sensor取得/header/受信、入力確定、推論、記録のclock domain/epochが追えるか。
  ROS/simulation timeとmonotonicの無根拠な減算がないか。
- 履歴slotのID/時刻/source/padding mask/ageと、commandの過去性・実利用可能時刻を区別できるか。
  offlineとruntimeのsensor_dt意味の違いを隠していないか。
- 欠落・重複・reset・warm-up・例外・未推論を成功ログだけに縮退させていないか。
- raw frameを保持し、変換済み診断には双方の時刻・matrix/pose・取得元・補間・ageが必要か。
  base_link=後輪中心や、変換後の元原点=現在ego原点を仮定していないか。
  証拠がなければ隣接予測driftをUNKNOWNにできるか。
- teacherは固定join key、元record hash、contract、mask、切断理由、annotation根拠、
  後処理版を別sidecarへ保存し、未来情報・teacher mask・未来由来shapeがforwardへ入らないか。
- 既存shadow launchの制御計算/publish経路を見落としていないか。
  将来専用nodeのpublisher/service/action/parameter書込み、launch/remap、node graph、
  preflight/制御topicへの非接続を確認する証拠が具体的か。publish_count=0だけで合格しないか。
- queue/drop/backpressure/保存失敗・logger停止時の観測限界、全受付母数の遅延が扱えるか。
  無制限sensor/tensor保存を既定とせず、hashだけではsensorを再現できない限定があるか。
- 正常/warm-up/stale/clock reset/TFなし/非有限/方向不定/logger停止の合成例が
  実測結果と混同されていないか。文書の必須条件とSchemaのrequired/nullable/enum/条件分岐が対応するか。
  構造検証で保証できないfield間整合・幾何・無制御性は将来検証として明記されているか。

## 返答形式

1. 設計としての総評。実装・稼働・安全の承認とは分離する。
2. 指摘表：重要度、file/sectionまたはJSON Pointer、具体的矛盾、影響、最小修正案。
3. 静的に確認できた事項と、MISSING/UNKNOWN・将来証拠が必要な事項を分離する。
4. 次のCodex依頼案は必要な設計修正のみに限定する。実装や実行が必要なら別承認事項として列挙する。

根拠のない原因断定、baseline勝利、残差ゼロを要求せず、完了済み固定残差解析を合格待ちへ戻さないでください。
geometry教師採用、stop/launch labels、motion permission/Safety、controller oracleは別gateです。
現runtimeを縦横MPC完成済みと扱わないでください。

```yaml
design_only: true
new_inference_authorized: false
new_collection_authorized: false
shadow_connection_authorized: false
control_connection_enabled: false
raw_execution_authorized: false
runtime_promotion_authorized: false
approval_gate: PENDING_EXPLICIT_AUTHORIZATION
```

梱包・pushは共有目的の別依頼で実施したもので、上記の実行承認を変更しません。
設計作成時および今回の梱包でSchema検証・pytest・推論・ROSは実行していません。
