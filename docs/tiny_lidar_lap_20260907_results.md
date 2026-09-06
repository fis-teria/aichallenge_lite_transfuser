# TinyLidarNet 初回実装・有限試験結果（2026-09-07 JST）

## 結論

**1周は未達成・未試験。予算追加承認待ち。**
公式配布重みを全parameter照合して実scanから推論し、単独sim consumerへ
Tiny操舵＋速度制限加速度を送る接続は実測できた。短い前進と制動停止を確認したが、
最高速度0.297m/sであり、定常2m/s・コースのカーブ通過・完走を確認したとは言わない。
最後の実走版は `4d6ddd284323fa73f38d6106b22cbd5a0eeb37ed`。
Ready待ち修正版 `dcf3f9c248609ecd4e8a49e916f1ac510fff6e96` は39合成/contract tests pass、実走未実施。

## 全attempt（旧失敗を保持）

| attempt | 実行版 | host wall秒 | Tiny forward | 駆動要求回/秒 | 結果 |
|---|---|---:|---:|---:|---|
| stationary_7b88981_01 | 7b88981 | 21.049610 | 1 | 0 / 0 | 操舵reportがclockより5ms先に届きINPUT_STALEで終了。モデル出力自体は有限 |
| stationary_4d6ddd2_02 | 4d6ddd2 | 21.763227 | 8 | 0 / 0 | 停止中に新scan8件のTiny推論・更新確認 |
| short_4d6ddd2_03 | 4d6ddd2 | 27.087848 | 164 | 1 / 8.889999801 | 8sim秒要求後に独立制動、移動後停止を観測。lapなし |

host壁時計の開始/終了、正加速度要求の開始、最後の速度stampは各生ログに保存。
全3attemptとも所有containerのみfreeze→KILLで終了、unpauseなし、残る実行containerなし。
旧試行のV4 forward124、MPC1等は別に保持している。今回V4 forward0、MPC0。

## 接続と停止の証拠

- 使用公式版 `1f54dff995d02625566341f9e1be1c39369224f2`、重みSHA256
  `7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963`。
- 18 parameter tensor / 150286 elements、全key/shape/float32/finiteと公式loader後の一致を確認。
- 実scan750点、frame `lidar`、50ms scan_time、公式max_range30m、実sensor最大25m。
  -infの処理は公式runtimeと抽出コードに差があるがruntimeを改変していない。実観測では±infの内訳を保存していないため、その影響は未評価。
- 初回forwardは約1.278ms。短走行164 forwardは0.321〜2.058ms、Tiny操舵は-0.240889〜-0.188897rad。
  これはこのCPU・実scanの処理時間であり、一般的な性能保証ではない。
- 短走行は163出力をsupervisorが受信。最後の1 forwardは終了と競合して送信に使われなかった。
  Tiny操舵送信155回、153種類のscan ID。要求加速度の範囲[-0.891499,+0.6]m/s²。
- 速度最大0.297166m/s。STOP開始8.214999816sim秒、停止持続確認の速度stamp9.099999796sim秒。
  新規速度報告の0.03m/s以下・0.5sim秒継続を確認。制動後のfreeze/KILLとは別の証拠。
- consumerは現在の専有namespaceの`awsim_d1`。制御/mode/gearのGID/QoSを生ログへ保存。
  他controllerのpublisherは検出されず、classic/MPCの操舵は使用していない。
- 接触messageは0件。ただしそのsubscriberが存在することだけで、sceneの接触publisherが全接触を網羅していた証明にはならない。
  無接触・無逸脱は未確認。実画面動画なし。LapCountのsection/lap実ログはいずれも0件。

入力・推論・送信・状態の参照は`tiny_supervisor.jsonl`の`input_id`、
`tiny_forward_index`、`operation_id`、各source/receipt/monotonic時刻と
`tiny_worker.jsonl`を使う。送信はapplied ackとは呼ばず、別の速度/操舵報告を観測として残す。
全sensorは保存しておらず、hashだけでsensor内容を再現できるとはしない。

## 最初に進行を止めた要因・最後の修正

最初は実reportとclock callbackの受信順序をstaleと誤認した。
`4d6ddd2`でclock watermark待ちへ変更し、過去入力の有効期限は緩めていない。

次の短試験では約6.26sim秒の`Ready`通知まで駆動できず、8秒枠の大半を消費した。
`dcf3f9c`は`Ready`＋その後の新scan出力まで正加速度を送らない。
未Ready推論は3回で待つ。モデル、前処理、速度政策、AWSIMは変更していない。
本修正の39 testsは全passだが、その後の駆動は行っていない。
今回の限定test実行は初版36＋clock修正37＋Ready修正39＝112件。全pytestや学習は未実施。
原因を学習不足やTinyの走行能力に断定する段階ではない。

## 追加承認が必要な具体的差分

既存sourceの`state_lattice_overtake_planner/data/course_centerline.csv`は
最終s=367.0261964m、閉じ区間を加えると約368m。
これは**予算見積もり専用**であり、実instanceで一周した証拠やTinyの操舵入力ではない。
元referenceの全座標や速度をTinyへ渡していない。実走距離との一致は未確定。
目標2m/sでも約184秒、実際には加速・速度偏差・制動の余裕が必要。

| 項目 | 現承認 | 今回消費後 | 続行のために提案する上限（未承認） |
|---|---:|---:|---:|
| 1駆動episode | 60 sim秒 | — | 240 sim秒（周回用、停止余裕含む） |
| 駆動累計 | 180 sim秒 | 8.889999801、残171.110000199 | 300 sim秒（+120） |
| forward合計の保守的共通枠 | 3000 | V4 124 + Tiny173 = 297 | 6000（+3000）。V4/Tinyは別counterのまま |
| 駆動episode数 | 3 | 1、残2 | 3のまま：Ready後短走行＋一周試験 |
| sim host wall | 3600秒 | 1189.191213、残2410.808787 | 変更なし |
| log上限 | 536870912 bytes | host台帳58002335 bytes | 変更なし、配布物の複製容量はmanifestに別記 |
| snapshot上限 | 16 | 7（今回0） | 変更なし |

20Hz観測で240sim秒の枠は約4800forward。残2episodeに短試験と1周を収める予約案であり、
無制限retryやモデル変更を許可したものではない。現在のcode/config/台帳の上限は変更していない。
追加承認後、Ready後短走行と停止を確認し、成立した同一構成で一周を試験する。
09:30以降は新規モデル/architecture変更なし、09:50 cutoffは維持。

## 保全・再現

起点、コマンド、公式出典、実装契約は `tiny_lidar_lap_20260907.md`。
各実行はWindows commit→既定CheckOnly/同期→WSL lock付き限定pytest後に専有archive展開。
既定同期によるDatasetルートの存在確認を実施。
Dataset内容・raw・既存学習sensor・V4 checkpointの読取りは未実施。
Tiny公式重みと今回のsim live scanだけは今回の明示許可内で使用している。
同期scriptは未変更。自動pushなし。Windowsは実装ごとにcommit済み。

WSLの今回作成した公式packageコピーが非ignored tmpとして同期保護に検出されたため、
そのコピーだけをlock下で`/home/thistle/e2e_autonomous/tiny_lidar_lap_20260907/official_package`へ移動・保全した。
既存dirtyの破棄や同期保護の無効化はしていない。
remote元repo HEAD/dirty151件/hashは前後一致。既存AWSIMの指定8ファイルも全attempt前後一致。
これは今回の変更がないという確認であり、未知の過去改変まで否定するものではない。

レビューの主題は公式配布重みの正しい利用と実走証拠。V4/S1再監査・独立レビュー待ちを追加しない。
この報告は独立監査済み、実車安全性、競技採用保証を意味しない。
