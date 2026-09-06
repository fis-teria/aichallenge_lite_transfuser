# TinyLidarNet Ready後short・lap実行結果（2026-09-07）

```text
READY_PATCH_LIVE_TESTED: CONFIRMED
READY_POST_RECEIPT_SCAN_USED: CONFIRMED
TINY_UPDATED_STEERING_SENT_WHILE_MOVING: CONFIRMED
JUDGE_ORDERED_ONE_LAP_CONFIRMED: NOT_CONFIRMED
BRAKING_STOP_AFTER_MOTION_CONFIRMED: CONFIRMED
HOST_FREEZE_OR_KILL_USED: YES
COLLISION_FREE / DEPARTURE_FREE: UNKNOWN / UNKNOWN
DEADLINE_AND_BUDGET_COMPLIANCE: CONFIRMED
```

**Ready後shortは成立。一周試験は実施したが未完走。**
lapの最初の終了要因はruntimeの`RuntimeError: HOST_MONITOR_STALE`。
独立supervisorによる制動停止を観測後、所有AWSIMをfreeze/KILLした。
駆動枠は累計3/3に達したため追加試行なし。モデル・速度・監視上限を変えて突破していない。
これは提供ログに基づく自己点検であり、独立監査済み・実車安全性・競技採用の保証ではない。

## 版と実行環境

- origin: `https://github.com/fis-teria/aichallenge_lite_transfuser.git`
- branch: `codex/windows-wsl-training-sync`
- 開始HEAD: `a87e17a089a75c7ab6ed3c9251fb6b8fefc7f13d`、clean。
- 局所実装・WSL test・今回short/lapの実行SHAはすべて
  `219eb2f00b8e4277275072d0b5e9f344f365edc7`。
- 結果文書・梱包版はZIPの`versions.json`と`PACKAGE_MANIFEST.json`の
  `result_document_commit` / `package_source_commit`。実走SHAと別に記録する。
- Windowsを編集・commit正本、WSLのnative checkoutをlock付き限定検証に使用。
  実行hostは`graneple@192.168.3.10` (`graneple-local`)。
- source archive SHA256:
  `24997bdea58c92fec7d90844765d778256d05b8a3588c707b4b20508ed329d57`。
  新規専有directoryに展開。元dirty checkoutを上書きしていない。
- 元sim repo HEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`、151 dirty entries。
  終了後もGit状態SHA256
  `0b9af671098d0d60fd59457c6aba36e75c88b17ff6a04c1045622ea5617eb9a3`は不変。
  自動push、reset/stash、他者process停止、lock削除なし。

## 今回の2 attempt（旧実績とは別）

| 項目 | short_219eb2f_01 | lap_219eb2f_02 |
|---|---:|---:|
| 開始→終了 JST 2026-09-07 | 05:50:16.705→05:50:53.022 | 05:51:38.535→05:52:06.821 |
| runtime wall / host wall予約 s | 120 / 230 | 600 / 710 |
| sim / Tiny forward予約 | 20 / 300 | 240 / 5200 |
| 実host wall s（起動・終了込み） | 36.317126763 | 28.285359454 |
| Tiny forward | 165 | 37 |
| 全操作送信 / Tiny操舵送信 | 309 / 153 | 170 / 31 |
| Tiny distinct scan送信 | 149 | 30 |
| 観測速度≥0.1m/sに結合したdistinct scan | 133 | 14 |
| 正加速度要求数 | 153 | 31 |
| Ready fence違反 / worker→送信join失敗 | 0 / 0 | 0 / 0 |
| 最高観測速度 m/s | 1.395915985 | 0.272549868 |
| 速度sample台形積分 m（位置軌跡ではない） | 6.277439266 | 0.231533800 |
| 駆動episode / 制動込み駆動sim s | 1 / 9.854999779 | 1 / 2.514999944 |
| 負加速度制動送信 | 25 | 7 |
| 最後の停止帯 新規sample数 / sim s | 16 / 0.524999989 | 16 / 0.524999988 |
| 元Unity section / lap event | 0 / 0 | 0 / 0 |
| runtime / make exit code | 0 / 0 | 1 / 2 |
| 終了理由 | SHORT_MOTION_COMPLETE | RuntimeError: HOST_MONITOR_STALE |

`COMMAND_SENT_NOT_APPLIED_ACK`は送信記録であり、適用ACKではない。
速度・舵角reportは別観測。新scan数と再送数、要求と送信と物理応答を区別する。
速度積分はGNSS/位置軌跡や周回証拠ではない。2.0m/s定常、本格カーブ通過、全周適合は未確認。
shortの最後の正加速度要求は初回から7.954999822sim秒、停止判定は8.004999821sim秒。
制御周期による判定差を含み、厳密な物理適用時刻や8秒ぴったりの停止を主張しない。

過去のTiny3 attempt（1、8、164forward、合計173）も`history/attempts/`に全ログを保持。
旧shortの最高0.297166m/s・155 Tiny送信・153distinct scanは今回の数字ではない。
以前のV4を含む全attemptの消費は同じbudget履歴に保持し、V4を再実行していない。

## Ready・新scan・制動の結合例

short最初の正加速度要求`command:121`：

- Ready受信mono `597866918797581`、scan sequence fence `126`、sim `6344999858`ns。
- 使用scan `scan:6349999858`、元受付mono `597866956243445`、sequence `127`、forward `5`。
- 要求mono `597866996959513`、sim `6394999857`ns、操舵 `-0.2109568864107132`rad、加速度 `+0.6m/s²`。

lap最初の正加速度要求`command:122`：

- Ready受信mono `597948959208060`、fence `127`、sim `6354999857`ns。
- 使用scan `scan:6399999856`、元受付mono `597948994579546`、sequence `128`、forward `5`。
- 要求mono `597949042769043`、sim `6409999856`ns、操舵 `-0.2070089876651764`rad、加速度 `+0.6m/s²`。

両者とも元受付時刻・sequenceがReadyより後。同stampの旧scanやworker終了時刻への置換ではない。
worker JSONL→supervisorのforward番号/input ID/元受付/sequence/操舵値を照合した。
集計の方法と全精度の値は`tools/summarize_tiny_ready_trials.ps1`と`evidence/saved_log_check.json`。

shortの最初の制動は`-1.0m/s²`、最後の停止帯はsim `15714999648..16239999637`ns。
lapの最初の制動は`-0.7609335780143738m/s²`、最後の停止帯はsim `8399999812..8924999800`ns。
それぞれ負加速度送信→別観測速度低下→新規sampleで|v|≤0.03m/sを0.5sim秒以上確認。
その後host freeze/KILL、unpauseなし。初めから静止、自然減速、host強制停止だけの結果ではない。
pause/KILL使用自体は別判定として残す。

## 一周を最初に止めた要因と未確定部分

lapは初回正加速度要求から約1.600sim秒で`HOST_MONITOR_STALE`によるSTOP_BEGINへ進んだ。
worker例外・forward上限到達は記録されていない。host側の`error`はnull。
新Unityログにsection/lap eventは0件。一周成立を示す証拠はない。

該当経路は`tools/run_tiny_lidar_dev.py:host_armed`とそのmain loop。
`host_armed(now)`はtoken/armed/停止証拠に加え`0 <= now-arm_stamp < 750ms`を確認する。
`now`はファイル読取り前に採時されるため、hostがその間に新ARMを書けばageが負になる静的反例がある。
実際の更新遅延が750ms以上だった可能性も残る。
**失敗時に読んだARM内容・age・失敗分岐は保存されておらず、直接の内部原因はUNKNOWN。**
最後の`host_armed.json`から失敗時刻の内容を復元したとは扱わない。
モデル精度、LiDAR契約、実host停止のせいだとは断定しない。

次の最小修正候補は、ARMを読んでから比較時刻を採り、判定分岐/読んだtimestamp/ageを記録し、
「事前now→並行ARM更新→読取」の合成反例を確認すること。750ms制限や独立停止は緩めない。
**これは提案のみで未実装・未再試験。** 再駆動にはpowered枠を含む別の明示許可が必要。
本タスクでは4回目、V4/S1への回帰、新モデル、再学習を行わない。

## 予算・期限・全attemptの連続性

適用は実host時刻2026-09-06T20:50:16.702125Z（09-07 05:50:16.702125 JST）。
依頼SHA256 `3210c97ce30a8af18abb9633378a0c9993f55c01f2231dda262142f17a646bcd`、承認者名null。
Tiny instance用limitsだけ変更し、共通`AttemptBudget.limits`は不変。
変更前/変更記録/予約前/予約中/精算後を同梱。過去6attemptは削除せず今回2attemptを追加。

| 予算 | 開始used | 終了used | 上限 | 残量 |
|---|---:|---:|---:|---:|
| host wall s | 1189.191213333 | 1253.793699550 | 3600 | 2346.206300450 |
| V4 forward | 124 | 124 | Tinyと共通 | — |
| Tiny forward | 173 | 375 | V4と共通 | — |
| shared forward | 297 | 499 | 6000 | 5501 |
| powered episode | 1 | 3 | 3 | **0** |
| 駆動sim s | 8.889999801 | 21.259999524 | 300 | 278.740000476 |
| log bytes（実台帳基準） | 58002335 | 61961373 | 536870912 | 474909539 |
| snapshot | 7 | 7 | 16 | 9 |
| MPC | 1 | 1 | 6000 | 5999（今回呼出し0） |

最終active=null。shortを実測精算後にlapを予約した。失敗lapも消費を残した。
新しいraw attemptログは各96MiB予約内。レビュー複製・ZIP等を含むlocal保存総量は
外部ZIP receiptへ別記し、台帳の実行ログ消費とは混ぜない。
両試験は09:50駆動cutoff・10:00期限より前に終了。予約も残wall/cutoff内。
残sim秒やforwardがあってもpowered0のため走行しない。

## 固定モデル・入力・consumer・隔離

公式commit `1f54dff995d02625566341f9e1be1c39369224f2`、重みSHA256
`7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963`。
18 tensors / 150286 elements全key/shape/finite照合、hash確認後ロード、部分loadなし。
公式core/modelは固定。公式sourceの小さなコピーとinventoryを同梱するが重み本体は含めない。
公式preprocessのNaN→0、±inf→30m、clip[0,30]等を独自変更していない。
実scan750点・角度順を維持。負Infinity処理の既知の差を今回変更していない。

Tiny操舵のみを使用。縦制御は既存速度feedback/制動、目標2.0m/s、上限2.4m/s、周期0.05s、
操舵上限pi/6rad。consumerが使わないspeed fieldだけで速度制限したとは扱わない。
gripSteerFactor=0.6は未補償。V4/MPC/classic/route由来操舵なし。
各`resolved_config.json`はphase/予算以外の同じ制御設定を保持。

各attemptの`compose.json`/`instance_inspect.json`/graph eventsを現instanceの根拠とする。
sim network=none、Tinyは同一network namespace、cap_drop ALL、GPU以外の物理制御deviceなし。
source/AWSIMはread-only。選択consumer以外への制御接続なし。元racing-kart Makefileは未実行。
`consumer_static/`は以前固定DLLから抽出した参照で、今回の再ビルド・AWSIM改修ではない。
両試験前後にbinary/DLL/scene/assets/vehicle/起動scriptの固定8hash一致。
終了後docker psは空。host全体の安全性や未知deviceへの普遍的な証明は主張しない。
衝突通知0件だけでは無接触・無逸脱は証明できず両方UNKNOWN。
利用可能な既存動画手段がなかったため実画面動画なし。生成図による代替なし。

## 限定検証・アクセス境界・再現資料

今回の提供test結果は**72 passed in 5.44s、exit0、skip0**。
`tests/test_tiny_lidar_sim.py`のみ。同SHAをWSL lock下で実行し、stdout記録とJUnitを保存。
旧112実行とは別。全pytest、学習test、追加モデルforwardは行っていない。
別stderrファイルはなく、提供console捕捉とexit、JUnitから判断する。空stderrを捏造しない。
今回のtestは公式重み全parameter確認を含むがforwardなし。simの202forwardとは別。
実行環境・command・exitの提供記録は`evidence/execution_receipts.json`。

- **既定同期によるDatasetルートの存在確認を実施**（CheckOnly/通常sync）。同期script変更なし。
- **Dataset内容・raw・学習sensor・V4 checkpointの読取りは未実施**。
- 今回明示許可された公式Tiny重みと専有sim scanは使用した。全sensor保存なし。

固定source/差分/versions、元依頼全文、独立レビュー依頼全文、全Tiny5attempt、予算履歴、
test/JUnit、実Unityログを小さなZIPに保存する。manifestは自身以外のsize/SHA256。
ZIP hashは外部receiptへ分離し、ZIPを再読取りして全entryを照合する。

実走commandは`docs/tiny_ready_lap_20260907.md`とpacketの`evidence/execution_commands.txt`。
保存済み資料の集計・梱包だけを行うPowerShell command：

```powershell
pwsh -NoProfile -File tools/summarize_tiny_ready_trials.ps1
pwsh -NoProfile -File tools/build_tiny_ready_review_packet.ps1
```

これらは再走行commandではない。梱包版の追加差分は結果文書と保存資料用scriptだけで、
実走runtimeは219eb2fのまま。配布資料を追加実行の承認とは扱わない。
