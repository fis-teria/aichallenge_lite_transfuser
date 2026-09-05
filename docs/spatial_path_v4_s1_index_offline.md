# Spatial Path V4 S1 index core：非実行readiness report

## 目的と判定境界

S1のmetadata/index構造を合成メモリだけで検証する。実取得、取得承認、record結合成功、教師採用を意味しない。
本番File/Path adapter、raw CLI、S2 reader、decompressor、Message decoderは実装していない。
既存planner/CLI/reader/converter/runtimeは変更しない。現行runtimeを縦横MPC実装済みとは扱わない。

```yaml
raw_execution_authorized: false
approval_gate: PENDING_EXPLICIT_AUTHORIZATION
deployment_or_training_approved: false
actual_raw_reads_performed: 0
actual_dataset_body_reads_performed: 0
mode: OFFLINE_SYNTHETIC_ONLY
```

## 版・入力同一性

- repository: https://github.com/fis-teria/aichallenge_lite_transfuser
- branch: `codex/windows-wsl-training-sync`
- 作業前HEAD／前計画結果文書: `48cb39d1b371ae6920c88a76134a996c67ad8976`
- 計画コード・テスト実行版: `29c11b1c04e5cb40b237e4bfc4a747fc0b0658d1`
- 旧計画コード: `17ead237de8f1a6b8e52fdc18bcf7c01e75e5403`
- 旧結果文書: `69d21d376ac95dde881fa75d5773827fb68fd04e`
- 上記objectは全てローカルcommitとして存在し、作業前HEADの祖先だった。開始時working treeはclean。
- 本依頼文は外部attachment `57303244-be82-44b8-878d-0545a4b69c5f`。独立した依頼文commitなし。
- 今回reader版: `spatial_s1_index_v4_offline_v1`。実装commit／実測結果は末尾に別記する。

計画rootは `/home/thistle/e2e_autonomous/runs/spatial_v4_record_binding_plan_v2_20260905`。
指定された7成果物だけを読み、既存plannerと同じcanonical JSON
（UTF-8、ensure_ascii=False、sort_keys=True、区切り`,`/`:`、NaN不可）のSHA-256を再計算した。
新identityは `937cbbd09efa8e7fe729a0e58687a39157ca7ec1582c49b231f1a6fe82411597` に一致。
旧identity `394292c7c268715a5efc4cd408f0e9634835d5d4cf016729a01c9f4786dde71d` は新計画の束縛値。
元9 JSON／旧6成果物には降りず、再分類していない。型付き契約へ4probe/8recordを結合できた。
実record IDとpayload hashは計画mappingから取得し、手入力・fixtureへの転記をしていない。

新identityの入力はpolicy/limits、manifest内のoriginal/prior input hash mapping、旧identity、code_hashes、
proposal/approval/facts、claim requirements、statusである。日時等の非identity fieldのbyte差を期待値の変更で吸収していない。
`input_manifest.files`もmanifestの一覧と照合する。report/unresolvedはcanonical identityの対象外であり、下表のbyte hashで別途記録する。

| 指定成果物 | SHA-256（読取前後一致） |
|---|---|
| execution_manifest.json | `fef330caf13e31026f7bab88767d07e8b14ac7e6d9062e539758e91ac75a3f42` |
| minimal_read_proposal.json | `29846dd965815cfed7d61111c03f695b1f82a2d28d6e205f1dae2b50ec1ee499` |
| approval_request.json | `357d66dc403239d828a421b1ccac9c90ae6678eb9308a2020e4facec1a0c60d9` |
| claim_requirements.json | `d00dbb1544d4095b904a78ebfc53892050d38f4bc56bd8819663c9105b8b9db6` |
| input_manifest.json | `cea84f0308aabf32e0b47f0e0d1a452458fed458fc738e0cec55d58fab4054d1` |
| unresolved_and_unrecoverable.json | `6f5afd399f499844fe539ae17bb3f4e490dd0b09e683b30b7d6173b90246a8ad` |
| report_ja.md | `b838b3087a93b31d1dd4b4da4f1dabcfc7eda9a56d9a7535769da3a4a720047c` |

保存source locatorは不透明な文字列のまま。rawのopen/stat/exists/resolveは実施していない。
metadata報告hash `087f45eb23b6e8aa846822b155d603bf7f40157d999a3396a55a9383751c4b05` は
原本MCAP全体hash、publisher ID、現在の実ファイル確認ではない。

## API・構造対応

- `validate_s1_contract(plan_mapping) -> Contract`: mappingのみを検査。固定identity、不変scope、未承認flag、envelopeを照合。不一致はBLOCKED_PLAN。
- `inspect_s1_index(byte_source, metadata_input, contract, ledger) -> dict`: synthetic=Trueの明示protocolのみ。
  `read_at(offset_bytes, length_bytes)`、uint64の`size`、変更検知用`revision`が必要。
- `MemorySource(bytes)`はBytesIO実装。path/stream引数を受けない。metadata hash検証も合成bytesだけ。
- offsets/lengthはbytes、log_timeはuint64 ns、active時間は秒。inclusive→exclusiveはconsumer側上限も確認し、overflowを拒否。

[MCAP一次仕様](https://mcap.dev/spec)に基づく小さな部分実装。
magic/Header/Footer、Summary/任意Summary Offset、Schema、Channel、Chunk Index、対象topicのMessage Indexを扱う。
Summaryはopcode単位のgroup境界を検証し、Summary Offsetがあれば実境界と照合する。
同ID異定義／channel→schema欠落は停止。同名異schema IDは検出・報告してIDで区別する（それ自体は形式違反ではない）。
schema definitionはhash/長さ、encodingは文字列として保持し、schemaからコードを生成しない。
索引時刻順を仮定せず、bounded Summary全体を列挙して全4窓との交差を検査する。
対象topicの索引entryがない窓はPARTIAL_SCOPE。entryの存在は候補record/payloadの存在証明ではない。

Chunk本文はヘッダを含む全Chunk領域を読取禁止区間として扱う。Message Indexの場所はそのChunk直後の
宣言index領域内であること、次のindex境界までに一致することを確認する。Message.dataは扱わない。
declared compressed/uncompressed sizeは報告値であり、実展開量・必要費用・payload正当性の保証ではない。
その他topicのMessage Index本文は取得せず、clock/anchor/velocity endpointを追加しない。

## CRC・未対応・安全性限界

- Summary CRCはSummary＋Summary Offset＋Footerのsummary_offset_startまでを検証。
  0はNOT_AVAILABLE、非0はMATCH/MISMATCH。未提供をPASSとしない。
- Chunk/DataEnd CRCはpayloadを含むためNOT_INSPECTED_PAYLOAD_OUT_OF_SCOPE。CRC全体検証済みとはしない。
- file-zstd、索引なし、必要summary定義なしは停止。scan/decode fallbackなし。
- Statistics/AttachmentIndex/MetadataIndex等を含むSummary、record末尾の拡張field、index間のgap等は現状未対応。
  形式上正当でもUNSUPPORTEDとして扱い、壊れたMCAPとは断定しない。
- footer宣言のSummary位置を入口とする。悪意ある偽装footerがpayload領域をSummaryと称する場合まで
  非payload性を証明する仕組みではない。今回の手組みfixtureと明示source契約でのみ境界を検証している。
  将来実adapterには信頼境界／許可read-range設計の独立審査が必要。
- revision/sizeによる前後確認はfakeの変更検知。ABA、偽のrevision、不正なadapter、OS TOCTOUや原本全byte不変性の証明ではない。
- full header候補完全性はUNKNOWN、original occurrence一意性、B_recordbinding、C、replay、D、4つのEはNOT_EXECUTED。
  geometry教師採用、stop/launch label、motion permission/Safety、controller oracleは別gate。

## 共通予算と例外

共通envelope候補はsource_bytes=67108864、expanded_bytes=134217728、messages=5000、seconds=60、
temporary_disk_bytes=0、single_record_bytes=16777216、chunks=8。
`Ledger`を同じsource/contractの全attemptで共有する。上限の引上げは拒否、過去raw残量は流用しない。
S2は未実装であり、将来S2にこの台帳を共有する実装・永続化は別途必要。stageごとの新台帳作成は禁止条件。

`synthetic_io`でreturned source bytes（再読取含む）、metadata hash bytes、metadata/index record解析数、index entry数、
union unique chunk数、read call数、attempts、active秒を分離。payload取得/展開attempt、decoded messages、expanded bytes、temporary diskは0。
8chunksは今回「交差した物理offsetのunique union」の判定でありpayloadを8回読んだ意味ではない。
個別probe cap／union cap／累積physical capを別predicateにする。9以上でもdescriptorを切り捨てずCAP_BLOCKEDとし、後段候補を採用可能にしない。
S2 retryのchunk数え方はPENDING_EXPLICIT_POLICY_AND_REVIEW。

要求長をread前に検査し、返却bytesを先に記帳してからtimeout/部分read/source changeを検査する。
例外で実消費不明ならUNKNOWN_ACCOUNTINGをstickyにし、同台帳で再試行しない。確定済み消費・診断は保持する。
active時間はattempt内だけを累積し、人の承認待ちを含まない。deadlineは協調的でOS I/O／JSON parseのhard realtime中断を保証しない。
Summaryを最大single_record_bytesの一括readに制限（より大きい有効SummaryもPARTIAL_BUDGET）。
追加の解析上限は累積metadata/index records 10000、index entries 100000。
messages=5000はdecoded message予算でありindex entry数ではない。

ファイル出力writer/CLIはない。戻り値はメモリだけ、この文書が非実行readiness report。
pending/final manifest、renameを行わず、出力commitの原子性を実装・検証したとは主張しない。
将来writerを追加する場合は必須成果物後のfinal manifest、書込/rename失敗時の非COMPLETE、stderr/非zeroの検証が残る。

## 実行手順と合成テスト

Windowsで今回の新規3ファイルだけをcommit後に実行する。sync既定処理がWSLのprocess、dirty、lockと
環境ディレクトリの事前条件を確認する。rawファイルやDataset本体は開かない。

```powershell
Get-CimInstance Win32_Process -Filter "Name = 'python.exe'" |
  Where-Object { $_.CommandLine -match 'e2e_lite_transfuser|training\.train|torchrun' } |
  Select-Object ProcessId,Name,CommandLine
git status --short
git rev-parse HEAD
./tools/sync_to_wsl.ps1 -CheckOnly
./tools/sync_to_wsl.ps1
wsl -d Ubuntu-22.04-Recovered -- bash -lc 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_s1_index_v4.py tests/test_synchronization_v3.py'
```

新規testsは手組みの小さなbytesのみ。正常、順不同4窓、共有Chunk、schema/ID衝突、欠落、対応外形式、
Chunk sentinel、bounds/UTF-8/length/truncation、整数上限、3種cap、timeout、partial/unknown accounting/retry、
再読取、source change、CRC3状態、非昇格、改変plan、外部I/Oなしを検査する。
synthetic planテストの一時identity置換はpytest monkeypatchの範囲内のみ。本番期待値は書き換えない。
既存回帰は純粋数学の`test_synchronization_v3.py`のみ。全pytest、Dataset/raw fixture tests、合成学習/optimizer testsは依頼に従い未実行。

## 実測結果・残条件

初回実装時点: WSL合成テストはまだ未実行。実測後にcommit SHAと件数を追記する。

実sourceへ進む前には、(1)このcoreと範囲制約の独立レビュー、(2)source binding/immutabilityと実File adapterの別設計、
(3)共通台帳の永続化・retry chunk計数・partial出力の設計とテスト、(4)4窓/8recordのみのS1明示承認が必要。
その後のS1実結果をレビューし、S2は別のpayload承認が必要。自動移行しない。
実Chunk位置・数・費用は未取得。合成数値を実計画に転載していない。pushは本依頼の範囲外につき行わない。
