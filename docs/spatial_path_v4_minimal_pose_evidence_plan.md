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
依頼文だけの独立commitは存在しない。実装commitは出力execution_manifestに記録する。
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
検証後の実数・選択ID・closure件数は下の実行結果節に追記する。

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
  --output /home/thistle/e2e_autonomous/runs/spatial_v4_pose_evidence_plan_v1_20260905
```

入力上限: 各16MiB/合計32MiB/10,000 records/2,000群/64 anchors/30 steps/60秒。
allowlist外探索なし、symlink/reparse拒否、入出力包含拒否、既存出力拒否。
schema/hash/不足入力でBLOCKED、上限でPARTIALと空seedを出す。
logical identityは入力hash+code hash+policy+設定+論理計画に結合し、時刻/環境pathを除外する。

成果物6件: input_manifest.json、claim_requirements.json、minimal_read_proposal.json、
unresolved_and_unrecoverable.json、execution_manifest.json、report_ja.md。

実行結果は検証後に追記。過去52/680 passedは今回の結果に流用しない。

追加取得は未実行・未承認。対象source/stage/window/claim・独立予算・reader修正と試験・原本不変性と新出力先の明示承認後に、別タスクとして判断する。
