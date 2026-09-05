# Spatial Path V4: 最小追加pose証拠取得計画 (PLAN ONLY)

## 1. 版と許可範囲

正本: `E:\workspace\e2e_lite_transfuser`。
origin: `https://github.com/fis-teria/aichallenge_lite_transfuser.git`。
branch: `codex/windows-wsl-training-sync`。
開始HEAD/結果追記版: `abec800e031a07f2898612af5d8177651354fd84` (docのみ)。
以下のobjectはローカルでcommitとして存在し、開始HEADの祖先 (merge-base exit 0)。

| 用途 | commit |
|---|---|
| pose競合監査実行 | befc97dd98434843245b888fb6d1d7110acc8a93 |
| raw監査実行 | 7ae8298b71aa7bdbacf1d1757798bf0de66bcfc4 |
| raw結果追記 | 428f30aca0418bc6362cbe8eb78b9aa6a07e1e4f |
| coverage | 2a2558749706e7554362d3d49613d92fbe3030f6 |
| pose競合開始 | ec255bce2d2d838698a6c964f880ba2d09a28303 |

今回の依頼は外部添付 `9374b255-4e03-498b-9e22-f3669c3501f1/pasted-text.txt`。
開始HEADまでの履歴に今回の依頼文だけの独立commitはなく、今回も添付自体のcommitは作っていない。
初期実装: `1efb7a4b206f51d71d83fa37dcb022f243366a90`。
最終コード・計画実行版: `17ead237de8f1a6b8e52fdc18bcf7c01e75e5403`。
本書の結果追記commitはその後のdoc-only commitであり、実行版とは区別する。
Webの過去422はローカルobject不存在の証拠ではなく、古い公開版への代替なし。

変更は新規plan module、CLI、合成テスト、本書の4ファイルのみ。
固定9 JSONの読み取り、一次仕様のWeb読取、関連コードの静的確認のみ。
bag/MCAP/zstdのmetadata/indexを含めrawは開かない。JSON内path/commandは実行しない。
Dataset本体、センサ、重み、学習、推論、optimizer、ROS、走行、pushは対象外。

## 2. 引継ぎ事実と照合方法

固定4 conflict JSONと5 evidence JSONを、依頼記載SHA-256と前後hashで結合する。
record IDは旧実装の `SHA256(canonical_JSON([raw_JSON_hash, run, original_array_index]))` と照合。
候補集合hash、run/topic/stamp、payload hash、anchor ID/tier、左右endpoint対応、件数を検査する。
これは旧分類の再計算でも原本の再測定でもない。
report/selectionの期待hashは後日記録であり、元実行時の独立結合ではない。
Dataset identityは旧/新manifestの文字列一致のみで、本体の検証ではない。

旧報告: 6,600 records、29 anchors、172重複pose群 (171非ゼロ投影差、1投影一致)。
recovery12件のpayload未取得を無競合としない。既知0、UNKNOWN、NOT_INSPECTEDを分けて保持する。
今回のJSON結合・保存診断集計で6,600 records / 29 anchors / pose685群 / 重複172群を確認。
172群は全て2候補・異なるpayload hash。非ゼロ171、投影一致1、正のXY差<=1e-8mは44、
yaw非ゼロ170、XY差>20µmは73。保存最大XY差0.1311721647999626m、最大yaw差0.015095404171680205rad。
anchor姿勢endpointに候補差10件、strict prefixへの候補差依存はh15/h30各15件。
旧summaryの29/29 status/flags一致、旧再現PASS17/UNKNOWN12、支持13/既知0が14/先頭欠損2、
原点のみNOT_COMPARABLE16件は引継ぎ報告値として保持し、新たな独立再現とはしない。

静的コード照合で、旧converter `_deduplicate_sorted` と保存抽出側 `interpolate_records` は
それぞれ渡された列のsemantic stampごとの最後をdictで残すと確認。
ただし、その列の由来が同一である証明はない。
旧 `observed_boundaries` はbag順の隣接poseの同stamp XY差>1e-8を検査し、
全候補組・yaw・物理一意性を証明するものではない。
`compare_horizons` はstrict_polylineへboundary_eventsを渡していない。
forward readerの早期停止にはコード上もlog_time単調性仮定の表記がある。
これらは静的確認のみで実行・変更なし。

## 3. Claimと十分条件

| Claim | 必要証拠 | 必須ではないfield / 限界 |
|---|---|---|
| A 保存候補差 | hashで結合された保存診断 | 新raw不要。物理誤差・安全性ではない |
| B 限定記録stream再現 | source/schema/domain、候補完全性、指定policy | 順序非依存policyなら物理順序不要。旧AnyReader版・列挙・tie証拠なしに旧converter再現とはしない |
| C 投影不変性/感度 | 完全候補集合、frame/projection/domain | 全関連投影同値なら順序不要。非ゼロ差の自動PASSなし。教師XY生成なし |
| D 物理正確性 | 推定器仕様・校正・独立検証 | channel/順序/publisher/covariance単独では不足。原本から解決不能もある |
| E 採用/発進/Safety/MPC | 教師policy、意図/許可、clearance、環境/車両/制御検証 | BやReference実在から導出しない。全て別承認 |

`claim_requirements.json` にpredicate、代替十分条件、解消範囲を保存する。
現在のAnyReaderインストール版を歴史的実行版と認定しない。

仕様上Messageにchannel_id/sequence/log_time/publish_time、Channelにschema_id/metadataがある。
sequenceは0や収録側カウンタでもよく、publish_timeはlog_timeと同じ場合がある。
channel IDをpublisher IDと同一視しない。ファイル内chunk開始offsetと展開chunk内record offsetを分離する。
indexはlog_time基準でありheader/epochの完全性を保証しない。明示domainが保存されているとは仮定しない。
これは実ファイル内容の確認ではなく形式仕様の確認。
[MCAP一次仕様](https://mcap.dev/spec)

AnyReader APIはconnection/timestamp/rawdataを返し、startは包含、stopは非包含。
このAPI説明だけでは旧版の同時刻tie順や各file/connection列挙を証明できない。
[AnyReader一次資料](https://ternaris.gitlab.io/rosbags/api/rosbags.highlevel.html)

## 4. Seedとclosure

normal `20260902-131505` / `20260902-132822` の保存群のみ。
anchor endpoint非ゼロ最大、strict prefix依存最大、旧1e-8mに最も近い非ゼロ、投影同値対照 (なければ保存singleton) の役割別先頭を選ぶ。
同一群は役割を統合し、同順位はstable group ID。最大4seed。ABSENTを捏造しない。
全体最大群がprefix非依存ならその事実を保持し、prefix最大誤差と呼ばない。
診断用の偏った選択でありDataset不良率推定には使わない。

各seedから独立したsingle-target partial probeを1件設定する。
anchor役割はその群をanchor姿勢に使うanchorを優先し、次にstrict内依存・stable ID・先頭step。
先頭欠損anchorのstep1は診断対象であり、適格なprefixとしては扱わない。
関連anchorのh15/h30既存strict prefix全stepは別claimとして全endpointを列挙する。
anchor姿勢は全futureへ依存し、targetより後の右endpointも省かない。
source/schema/topic、候補全ID/hash、旧endpoint hash、clock観測窓を保存する。
観測closure上限: 32群/64候補/256clock/1秒bag窓、probe合算64群/128候補。
±0.25秒marginは提案値でありepoch証明の閾値ではない。
上限超過は `CLAIM_CLOSURE_BLOCKED`、列挙を切り捨てない。
domainで限定された完全候補scopeが未確定なら、観測closure内に収まってもsource closureはBLOCKED。
局所clock整合からrun全体のepoch alias不存在は主張しない。whole-run走査へ拡大しない。

## 5. 最小追加項目と取得不能分岐

優先順: source/schema binding → 記録位置とpolicy順序 → bounded domain。
対象ID/window/必要field/不足理由/metadata-indexまたはchunk段階/旧hash/更新範囲を各項目へ保存。
yaw投影の仮説検査だけ原quaternionを同一承認chunkで確認する案。
具体的仮説のないcovariance/publisher追加収集は計画しない。
候補欠落・追加・schema差は対応を無効化し、旧成果物を維持したまま依存claimをBLOCKED。
意味のないsequence/publish_time、未収録publisher/domainは代替証拠を評価し、なければ
`NOT_RECORDED` / `UNRESOLVABLE_FROM_THIS_SOURCE`。再試行や探索の自動拡大なし。

## 6. 予算と明示承認

以下は独立した将来停止上限案であって、必要量見積りでも実行承認でもない。
source 64MiB、expanded 128MiB、messages 5,000、time 60秒、temporary disk 0、single record 16MiB、chunks 8。
必要コストestimateは全てnull。JSONサイズをsourceコストに使わない。
metadata/indexすら今回は読まない。将来もmetadataのコスト判断後にpayload段階を別途承認する。
旧残量/欠測counterは修復済みとせず、流用しない。

将来readerの承認前条件: 時間例外でも消費bytesを保存、partial/error manifestとprogress、
schema定義hash結合、早期停止仮定とcoverage分離、stat/hash変化伝播、
topic/file/window未取得はPASSにしない、log_timeとheader範囲を分離、全raw不変性未検証の限定を残す。
今回reader変更なし。

## 7. 独立gate

geometry-only設計は仕様検討のみ可能。実データ採用はprovenance/適格性/教師policyの別gate。
停止教師は意図・発進許可不足、controller/MPC oracleは環境・車両・policy検証の別gate。
現行選択runtimeを縦横MPC実装済みとしない。新tier/PASS、教師生成、実行許可なし。

## 8. コマンドと実行結果

Windowsで今回4ファイルだけcommitし、次の順で同期・検証する。全pytestは実行しない。

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

WSL repo内:

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_pose_evidence_plan_v4.py tests/test_synchronization_v3.py
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/plan_spatial_pose_evidence_v4.py \
  --conflict-root /home/thistle/e2e_autonomous/runs/spatial_v4_pose_conflicts_final_20260905 \
  --evidence-root /home/thistle/e2e_autonomous/runs/spatial_v4_evidence_run_v3_20260906 \
  --output /home/thistle/e2e_autonomous/runs/spatial_v4_pose_evidence_plan_v2_20260905
```

入力上限: 各16MiB/合計32MiB/10,000 records/2,000群/64 anchors/30 steps/60秒。
allowlist外探索なし、symlink/reparse拒否、入出力包含拒否、既存出力拒否。
schema/hash/不足入力でBLOCKED、上限でPARTIALと空seedを出す。
logical identityは入力hash+code hash+policy+設定+論理計画に結合し、時刻/環境pathを除外する。

成果物6件: input_manifest.json、claim_requirements.json、minimal_read_proposal.json、
unresolved_and_unrecoverable.json、execution_manifest.json、report_ja.md。

### 実行結果 (2026-09-05)

Windows/WSLとも開始時clean、Windowsの対象学習processなし。
各commit後のsync preflightがWSL clean/process/lock/commitを検査し、CHECK_OK → SYNC_OK。
同期scriptは既存Datasetディレクトリの存在を確認するが、Datasetファイルは読んでいない。
Windows側の無関係なPythonプロセスは停止・変更していない。

最終コード版 `17ead237de8f1a6b8e52fdc18bcf7c01e75e5403` にて:

- lock付きfocused実行: **43 passed in 0.43s** (新規33 + 非学習synchronization10)。
- WSL plan: **COMPLETE_PLAN_ONLY**, exit 0。
- Windows Python 3.10でも同じ入力からplan生成、**COMPLETE_PLAN_ONLY**。
- 論理identityは双方一致: `394292c7c268715a5efc4cd408f0e9634835d5d4cf016729a01c9f4786dde71d`。
- 9入力、計11,768,262 bytes (初回読取の合計)。期待hash・前後hashすべて一致。
- 入力内Dataset identity `181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388` を両manifestで照合。本体未確認。
- 旧pose監査logical identity `ccd1f4e830d4726fcb8fbef45ac83bc60a9f99ec9ca4233dce6f4590f6f4ba44` は入力manifestの記録値。

WSL成果物root:
`/home/thistle/e2e_autonomous/runs/spatial_v4_pose_evidence_plan_v2_20260905`。
Windows独立生成root:
`E:\workspace\e2e_lite_transfuser\tmp\spatial_v4_pose_evidence_plan_windows_v2_20260905`。
旧v1計画は上書きせず保持し、今回の採用計画はv2。入力成果物・旧tierの上書きなし。
既存output拒否のため再実行するときは、新しい未存在versioned rootを指定する。

選択4群は役割別順位の結果すべてnormal131505となった。normal132822も候補母集団には含むが、
run均等化を目的としていない。recovery/test/未選択sessionへの一般化はしない。

| 役割 / stable group ID | bag=header stamp (ns) | 保存XY差 (m) | 部分probeの観測closure |
|---|---:|---:|---|
| anchor endpoint / `8fbd120c37e872d2bc51ab58bc95813636caa7de04a335b560e9337c6993f12b` | 6189999861 | 0.00006032426614470164 | 6群/9候補/122clock、0.610秒、上限内・未承認 |
| strict prefix大差 / `208cfcac87744ded9ef39f3c85ac2ae6d3f545255e81239e33e8819e1947b20c` | 256259994272 | 0.060719386103918874 | 6群/8候補/587clock、2.925秒、上限超過BLOCKED |
| 旧閾値近傍 / `353301a96a1f493e253091d78bef0c43c2b90818097aa892e392a5756250811d` | 5449999878 | 0.000000009857472638787348 | 6群/7候補/128clock、0.635秒、上限内・未承認 |
| XY/yaw同値対照 / `ffe514be1a0f7756c75006e4066566bcaf025610e726962ed1ecb9c93fca65f7` | 939999978 | 0 | 1群/2候補/101clock、0.500秒、prefix依存なし |

全候補record ID/payload hash・関連anchor全件・endpoint/stepは `minimal_read_proposal.json` に保存。
部分probeの対象anchorは順に `20260902-131505__epoch0000__6192918933` step1 (strict外の欠損診断)、
`20260902-131505__epoch0000__253892918933` step24、
`20260902-131505__epoch0000__5292918933` step1、対照はanchorなし。

全体最大群 `8fb56a44449a41e5f3f294004247a9d3bd4bb891089e9348f65ac5ead395447e`
(保存XY差0.1311721647999626m) は選択済みanchor群のstrict prefix非依存。
この値を対象prefixの最大誤差や経路誤差とはしない。

4 partial probesの観測unionは**19群/26候補**で合算上限内だが、1 probeの個別窓上限超過を解除しない。
関連5 anchors × h15/h30 = **10 full-prefix claims**を別列挙。
6件は観測closure上限内、2件は超過、2件は正の既存prefixなし。
後半走行anchorのh30は122群/146候補/705clock/約3.520秒が必要であり、seedだけで代用しない。
**14 claim全て、独立domain partitionと原本候補scopeの完全性は未確定**。
上限内の計画も追加取得可能性や幾何適格性のPASSではない。

全pytest・raw fixture・Datasetテスト・optimizerは実行せず、過去52/680 passedを流用していない。
raw読取 (metadata/index含む)、Dataset本体読取、学習、推論、制御変更、走行、pushは0。

追加取得は未実行・未承認。対象source/stage/window/claim・独立予算・reader修正と試験・原本不変性と新出力先の明示承認後に、別タスクとして判断する。
