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
- 初回reader版: `spatial_s1_index_v4_offline_v1`。v2は `spatial_s1_index_v4_offline_v2_read_layout`。
  今回修正版は `spatial_s1_index_v4_offline_v3_ledger_lifecycle`。

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
- `inspect_s1_index(byte_source, metadata_input, contract, ledger, layout=None) -> dict`: synthetic=Trueの明示protocolのみ。
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

v2では後述の独立SyntheticReadLayoutを全readの信頼の起点とする。Chunk本文はヘッダを含む全Chunk領域を補助的な読取禁止区間として扱う。Message Indexの場所はそのChunk直後の
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
- v1ではfooter宣言のSummary位置を入口とし、独立した読取許可がなかった。v2では候補範囲を独立layoutと
  read_at発行前に照合する。任意の不正sourceや不正なlayoutに対して非payload性を証明したわけではない。
  将来実adapterには独立した範囲根拠・信頼境界設計の審査が必要。根拠がなければ本番adapterへ進めない。
- revision/sizeによる前後確認はfakeの変更検知。ABA、偽のrevision、不正なadapter、OS TOCTOUや原本全byte不変性の証明ではない。
- full header候補完全性はUNKNOWN、original occurrence一意性、B_recordbinding、C、replay、D、4つのEはNOT_EXECUTED。
  geometry教師採用、stop/launch label、motion permission/Safety、controller oracleは別gate。

## 共通予算と例外

共通envelope候補はsource_bytes=67108864、expanded_bytes=134217728、messages=5000、seconds=60、
temporary_disk_bytes=0、single_record_bytes=16777216、chunks=8。
`Ledger`を同じsource/contractの全attemptで共有する。ENVELOPE超過は拒否、過去raw残量は流用しない。
v2までは一度縮小したlimitsのENVELOPE以内への再拡大を防いでいなかった。v3で下記の単調縮小契約に修正。
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

2026-09-05の実測:

- reader初回実装commit: `e208444b7a0fd77ffe776a998faa4153685bf2f6`。限定テスト72 passed in 0.19s。
- 例外途中の診断保持・破損索引境界テスト追加版: `e4078f74c62567e24065f90169ecc1fb04d5b68c`。
  このcommitでWindows/WSL同一SHA、CheckOnly→通常syncの成功を確認後、worktree lock付きで
  **82 passed in 0.20s**（新規S1 72件＋既存同期数学10件）。失敗0。
- この結果追記は上記実行版の後続文書commit。実装・テスト実行SHAと結果文書SHAを同一と偽装しない。
- Windowsでの計画7成果物のcanonical identity照合は成功。各成果物の読取前後byte一致も確認。
- 実raw/metadata/index、Dataset本体、学習・推論、ROS環境は未実行。旧14claim、4seed/8record/4窓は変更なし。
- readiness判定: **合成入力の最小S1 coreを検証済み**。本番readerとしての適格性やS1実取得承認ではない。

実sourceへ進む前には、(1)このcoreと範囲制約の独立レビュー、(2)source binding/immutabilityと実File adapterの別設計、
(3)共通台帳の永続化・retry chunk計数・partial出力の設計とテスト、(4)4窓/8recordのみのS1明示承認が必要。
その後のS1実結果をレビューし、S2は別のpayload承認が必要。自動移行しない。
実Chunk位置・数・費用は未取得。合成数値を実計画に転載していない。pushは本依頼の範囲外につき行わない。

## 2026-09-06：v2 読取前範囲契約の修正

依頼attachment: `2399173d-4054-43b6-8877-f5e23d64ce75`（外部依頼、独立commitなし）。
開始HEAD／前回梱包・結果文書版: `01d484272dffc391de9683c9bdc787d778451467`。
指定5objectのcommit存在・祖先関係、origin/branch、clean working tree、前計画結果版からの3ファイル差分を確認。
前回82 passedは上記e4078f7の記録であり、今回結果とは別扱い。

### 信頼の起点とAPI

`SyntheticReadLayout`はfrozen dataclass。synthetic_only、Contract全体（plan identity/probes含む）、
sourceオブジェクト参照、size、revision、許可範囲tuple、provenanceを外部から明示的に渡す。
各rangeはuint64 bytesの半開区間[start, stop)。非空・昇順・非重複・source size以内とし、bool/負値/逆転を拒否。
接するrangeのunionは許可できるが、1byteでも隙間を横切るreadは拒否する。

fixture組立側がHeader、Message Index、Summary/Offset/Footer/magicの出力位置を保持し、
coreが読むbyte列と別の契約として渡す。FooterやIndexを解析してlayoutを生成するfallbackはない。
Footer/Header/Indexの数値は候補にすぎず、Summary CRC一致やDataEndらしいbytesも許可根拠ではない。
全source.read_at呼出しはReader.readの共通経路でlayout全範囲包含を検査してから行う。
Header/record header/body、magic、Footer、Summary、Message Indexを例外にしない。

layoutなし/不正はBLOCKED_READ_LAYOUT、別Contract/source/size/revisionはBLOCKED_READ_LAYOUT_BINDINGで
source.read_atを発行せず停止。範囲未許可はBLOCKED_READ_RANGE。既存の予算・時間上限も維持する。
source参照の`is`照合は合成プロセス内のlayout取り違え防止であり、原本の強いsource bindingではない。
provenanceは呼出側の申告。coreがその真実性や悪意あるfixture作者を検証できるわけではない。

`synthetic_io.read_calls`は実際のsource発行数、returned_source_bytesは返却bytes、
core_pre_read_range_rejectionsは発行前拒否数、rejected_rangesはoffset/length診断。
拒否だけでbytes/read_callsを増やさない。GuardedSourceも発行を先に記録し、sentinel拒否を別countする。
core拒否試験ではsource側sentinel_rejections=0も検査する。固定payload counter=0だけでは境界成功の根拠にしない。
read_layout_verifiedとread_boundary_basisは合成契約の確認であり、実rawアクセス0の不変fieldとは別。
S1_SYNTHETIC_INDEX_INSPECTEDは独立に渡された合成layoutの下での結果に限定する。

### v2時点で修正に混ぜなかった残課題（履歴）

- Ledger.limits縮小後のENVELOPE以内への再拡大。
- Ledger bindingに個別/union cap、source_run/source_idが含まれない点（layout照合は台帳修正ではない）。
- finishで初めて時間超過が判明する場合、およびclockがfinishで例外になる場合の最終化。
- 直接構築Contractと固定plan検証済みContractの区別。
- 同size/revisionを持つ別source等に対する強い原本binding。新layoutの参照照合はこの問題全体を解消しない。
- Message Indexの最低Message領域長等のdescriptor内部整合性。

対応形式拡大、永続台帳、writer、S2、本番adapterは追加していない。
4probe/8record/4窓、旧14claim、計画identity、B/C/D/E非昇格と全未承認flagを維持。
今回は計画成果物を再生成・再分類していない。実raw/Datasetアクセス、学習/走行、pushはいずれも0。

### 今回の検証

実行対象とWindows commit→CheckOnly→sync→WSL lock手順は上記と同じ2テスト限定。
新規反例は元layoutを固定し、FooterをChunk先頭/records内部に向ける、CRC一致の偽装、
Header長増大、未許可Message Index、range欠落/異常値/隙間、別source/contract/size/revisionを検証する。
実装・今回テスト実行commit: `06f2b9381ee2057bf75b1ec4b9b75e6f0063442d`。
Windowsの対象processなし、既定CheckOnlyによるWSL clean/process/lock確認後、通常syncはSYNC_OK。
Windows/WSL同一SHAを確認してlock付きで実行し、**115 passed in 0.26s**。
内訳はS1 105件（従来72＋新規33）＋同期数学回帰10件。過去82 passedと区別する。
偽装Footer（先頭/内部、CRCなし/一致）、Header、Indexの新規反例はBLOCKED_READ_RANGE、
source側sentinel拒否0で成功。未layout/不正layoutはsource発行0で停止。
全pytest、実raw fixture、Dataset tests、合成学習/optimizer testsは未実行。
この実測追記は上記実装・実行版の後続文書commitであり、実行SHAへ遡及的に含めない。

## 2026-09-06：v3 Ledger lifecycle

依頼attachment: `cce496bc-16fb-41ef-9413-6d2eeb44041e`。開始HEADはローカル履歴で
`666c14c27272f66601cddc2ca523ffd4414530d1`と確認。これはv2実測追記commitで、実行commit
`06f2b9381ee2057bf75b1ec4b9b75e6f0063442d`とは別。差分は結果文書9行だけだった。
指定6objectの存在・祖先関係、origin/branch/clean状態を確認。v2 layoutの独立入力、全read前照合、
core拒否とsentinel拒否の分離をコード/testsで確認した。提供報告115 passedは今回結果として流用しない。

### 採用上限・縮小・binding

ENVELOPEは従来通り。Ledger初期化でlimits mappingをコピーし、initial_limitsと実効limitsを分離。
両propertyは読取専用の独立snapshotであり、呼出側の辞書変更が反映されない。
`tighten({dimension: new_value})`だけが実効上限を変更する。attempt外のみ、全変更値が現在値以下であることを
原子的に検証。bool、NaN/Infinity、負数、整数以外、未知keyを拒否。同値はno-op、正当な0は保持。
private属性やcounterを故意に改変する呼出側に対するsecurity sandboxではない。

累積予算はsource_bytes（返却bytes）、seconds（active秒）、chunks（unique交差offset数）。
expanded_bytes、messages、temporary_disk_bytesはS1では消費0。metadata hash入力は既存メモリで別count。
single_record_bytesは単一read/body等の要求上限で、累積source bytesとは比較しない。
縮小が既消費未満ならtightening_historyへlate_tighteningとその時点の消費・前後上限を記録する。
過去のattemptを違反だったと書き換えず、次attempt/readをPARTIAL_BUDGETで止める。

初回begin時に`identity(asdict(contract))`で全dataclass fieldの論理内容を束縛。
plan/probe/record/hash/window、source_locator/run/id/metadata hash、個別/union capも含む。
後の不一致はsource読取前にBLOCKED_CONTRACT。layout側のContract照合も独立に維持する。
同じLedgerは同じ論理Contractを意味し、size/revisionの既存照合やlayoutのsource参照照合を併用するが、
publisher・全原本hash・物理source一意性を証明しない。

### attempt状態・時計・結果確定

`begin(contract) -> token`は検証と開始clock成功後だけ所有権を返し、attempt countを増やす。
ACTIVE中の再入はBLOCKED_ATTEMPT。`finish(token)`は現在ownerだけが呼べ、二重finishや別tokenを拒否。
拒否で所有権や消費を巻き戻さない。終了時のclockを計上してからFINALIZEDへ移り、その後で予算を判定する。
最終check=59.9秒、finish=60.1秒ならPARTIAL_BUDGETでありS1成功ではない。

clockは数値・有限・単調非減少を検査。例外/NaN/Infinity/逆行はUNKNOWN_TIME_ACCOUNTINGをstickyにする。
最終sampleが失敗した場合は最後に確定できたactive秒の下限を残す。既知byte会計は別flag accounting_knownのまま。
time_accounting_known=falseのLedgerで追加readは行わない。source例外によるUNKNOWN_ACCOUNTINGも解除しない。
snapshotは時計を呼ばず、消費・状態変更なしで履歴のdeep copyを返す。attempt間の人の待機時間はactiveに含めない。

inspectは処理完了を仮記録し、finishが成功してからS1_SYNTHETIC_INDEX_INSPECTEDと
s2_candidates_within_caps=trueを設定する（S2承認は依然false）。失敗時に成功用trueを残さない。
優先順位はUNKNOWN_TIME_ACCOUNTING > UNKNOWN_ACCOUNTING > 元の処理失敗 > 終了時予算失敗。
diagnostics.processing_error/finalization_errorの両方を保持するので、元の境界拒否やsource例外を消さない。
chunk_capsの個別predicateは観測時の診断であり、最終成功や後段採用を表さない。
attempt_historyはstart/endの既知bytes/read calls・limits・時間・finalization statusを記録する。
協調deadline、プロセス内の会計のみであり、電源断・OS I/O強制中断・永続耐障害性の保証はない。

### 維持境界と残gate

v2 layoutの取得方法・範囲検査は再設計していない。4probe/8record/4窓、旧14claim、計画identityを変更しない。
全raw/Dataset読取0、未承認flag、B/C/D/E/S2非昇格を維持。学習/推論/optimizer/ROS/走行なし。未push。
残るものは、検証済みContractと直接構築ContractのAPI区別、強い実source binding/immutabilityと独立範囲根拠、
Message Index内部整合性、未対応Summary形式、永続台帳・別Ledger生成統制・S2 retry chunk policy、
本番adapter、writer/CLI/partial-final manifest、S2 decoder。今回完了で本番adapter/S1取得へ進めるとは判定しない。

### 今回の限定検証と資料

前記Windows commit→既定CheckOnly→通常sync→WSL lockで、同じ2テストファイルだけを実行する。
今回は`-vv`で試験名を含むpytest生ログを保存する。全pytest/実bag/Dataset/合成学習testsは実行しない。
新規別directoryへ3対象ファイルとログ、再実行に必要な同期数学・package init・conftestだけをコピーしhash/sizeを記録する。
実行commit・環境・今回の件数/時間・資料位置は実測後に追記する。
