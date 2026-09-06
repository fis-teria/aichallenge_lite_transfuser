# Tiny GUI: 短距離走行・制動停止と終了処理の結果

## 結論

**公式Tinyの更新操舵による短距離走行と、その後の制動停止を確認した。**
ただし停止後のRViz終了が間に合わず、試験全体の終了結果は
`RuntimeError: FINALIZE_TIMEOUT_WHILE_PAUSED` / make exit 2。
短距離の走行・停止成立と、GUI終了処理の未解決を分ける。一周は未実施。

## 版・環境

| 対象 | 固定値 |
|---|---|
| origin | `https://github.com/fis-teria/aichallenge_lite_transfuser.git` |
| branch | `codex/windows-wsl-training-sync` |
| 開始Windows HEAD | `d672a82c46c4af355076cd0adab5699d812230c1` / clean |
| 実装・test・ROS build・実行版 | `31f4e5a4e93673d60d8032d26701183c8bdbebaf` |
| host | `graneple@192.168.3.10` / `graneple-local` |
| 元環境 | `/home/graneple/git/autononous_ai/aichallenge-racingkart` |
| 元環境HEAD | `4af395eee10f928c7fc7225760adfa04c4c07ff4` |
| 新package / method | `aic_tiny_sim_test` / `tiny_lidar_net_guarded` |
| attempt | `gui_retry2_31f4e5a_01` |
| 出力root | `/home/graneple/e2e_autonomous/tiny_gui_retry2_20260907` |
| source tar SHA256 | `b58615e3f1f167bd467f8c1f9c9816a9d97625b5ed27fe90452da51c2e25f3f1` |
| 保存attempt tar SHA256 | `48bdcd11e61f72aa0921ebc99ad7e1d078fc32c546e293dc708c739cfbcb782d` |
| 固定image | `sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7` |
| 公式Tiny出典commit | `1f54dff995d02625566341f9e1be1c39369224f2` |
| 公式重みSHA256 | `7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963` |

Windows commit→未変更CheckOnly/同期→同版WSL lock付き検証→専有archive/別install。
元racing-kartのdev recipeを追加includeから実行。通常のAutoware走行stackではなく、
新packageから公式Tiny coreを利用する独立supervisorとRVizを起動した。
V4経路・MPC・classic操舵・正解routeは使用していない。学習済みでない加速度headも使っていない。

## 限定検証・実行

- WSL限定pytest: **103 passed / 3.13 s / exit 0**。Tiny関連2fileのみ、全pytestではない。
- 07:10:51–07:10:53 JST: 新ROS packageだけbuild、1 package / 1.13 s / exit 0。
  byte-compilation無効の警告あり。既存workspace/install未変更。
- 07:11:07.710–07:11:53.496 JST: host試行、wall **45.785750242 s**。
- 前回のhostname/network競合・子make target不在は発生しなかった。
- 実行中のinstance inspectに対するnetwork/mount/device/image判定を通過。
  `CURRENT_CONSUMER_VERIFIED`にconsumer `awsim_d1`、制御topicに競合publisherなし、interface `lo`のみ、physical deviceなしを記録。
  過去instanceの判定だけを現在の隔離証拠に流用していない。
- AWSIMとRVizのX windowを今回containerのnamespace PIDへ照合し、IsViewableを確認。
  ただしRVizは設定directory/DRI警告あり。window存在だけでscan正常表示を証明したとは扱わない。

実行commandは計画文書と`evidence/gui_retry2_31f4e5a_01_console.txt`、
具体的な子make/docker/終了commandは`gui_retry2_31f4e5a_01/host.jsonl`。

## 走行・停止の保存ログ照合

`tools/summarize_tiny_gui_retry2.py`は今回の保存済みJSONL/summaryだけを読み、
追加推論・ROS・制御なしで以下を再照合した。生ログは変更していない。

| 項目 | 結果 |
|---|---:|
| Tiny forward | 165 |
| Tiny操舵送信 | 154 |
| 送信元のdistinct scan | 150 |
| 直近速度sampleが移動帯のTiny送信 | 136 (distinct scan 132) |
| 要求→送信の値不一致 | 0 |
| worker→送信のID/時刻/操舵不一致 | 0 |
| Ready時刻・受信時刻・scan sequence条件違反 | 0 |
| 最高絶対車速 | 1.380666494 m/s |
| 制動sourceの送信 | 34 (うち負加速度25) |
| 制動後の停止帯 | 16 fresh samples / 0.524999988 sim s |
| 最終停止帯sample車速 | 0.000192476 m/s |
| 駆動開始から制動停止確認まで | 9.899999779 sim s |

停止理由は`SHORT_MOTION_COMPLETE`、supervisor/worker例外なし。
停止は「最初から静止」ではなく、運動観測後の負加速度送信と、新しい速度stampによる
|v|≤0.03m/s・0.5sim秒以上の持続で確認した。**この停止観測はhost pauseより前**。
topic送信は適用ACKそのものではないため、`COMMAND_SENT_NOT_APPLIED_ACK`という元field名を保持する。

正加速度の最後の送信は開始から7.999999822sim秒、STOP_BEGINは8.039999821sim秒。
設定8秒に対し停止処理開始が約40ms遅れる離散周期の差があり、厳密に8秒以内で制動へ切り替わったとは言わない。
制動込み20sim秒の上限内で停止した。速度2.4m/s上限超過なし。

## 終了処理・GUIの限定

- supervisorは停止結果を保存し、hostへfreezeを要求。hostのpause確認も成功。
- supervisor processはclean exit。一方RViz終了ログに`malloc(): smallbin double linked list corrupted`があり、
  SIGINT/SIGTERM後も終了待ちが続いた。hostのpause後5秒の終了待ちがtimeoutとなった。
- `//.rviz2`への設定作成/保存失敗、DRI警告も記録。これらとheap破損との根本因果は**UNKNOWN**。
  timeout閾値を増やして正常終了扱いにする変更はしていない。
- host cleanupで所有AWSIMをfreeze状態からKILL（exit137）。所有runtimeも停止し最終exit0。
  RViz自身は最終SIGKILL/exit -9。container exit0だけで試験成功とはしない。
- unpauseなし、cleanup errorsなし。07:12:17 JST確認時に実行中containerなし。
- snapshot要求2回のうちAWSIM XWDは有効、RViz XWDは0 bytesで取得失敗。失敗も削除せず保存。
  AWSIM画像は停止後freeze時の実画面。保存XWDをPNGへ無加工変換して閲覧しただけで、追加撮影ではない。
  画面の0.0 km/h表示だけで制動を証明せず、上記時系列と合わせて扱う。
- 動画なし、lap/section証拠なし。無接触・無逸脱はUNKNOWN。短試験を一周完走とは呼ばない。

## 予算・保全

今回承認 `TINY_GUI_RETRY2_20260907`、hash
`599eee1622043222bef50de1288de693ec192055809d039f175d2eb26f9bef8b`。
ユーザーの明示依頼によりpowered上限5→6のみ追加。旧10attempt/失敗/保守計上は保持。
今回は最終summaryがあり、165forward/1駆動/9.899999779sim秒を実測精算。旧不確定消費を減額していない。

| 共通台帳 | 最終used | 残量 |
|---|---:|---:|
| wall s | 1310.883059019 | 2289.116940981 |
| shared forward | 1864 (V4 124 / Tiny 1740) | 4136 |
| powered attempts | 6 / 6 | **0** |
| powered sim s | 71.159999303 | 228.840000697 |
| log bytes | 68661039 | 468209873 |
| snapshots | 9 | 7 |
| MPC（過去分のみ） | 1 | 5999 |

今回log4388490 bytes（reserve含む）、snapshot要求2、active=null。
ビルド・単体test・保存ログ照合時間はtrial wallとは別。09:50駆動cutoff/10:00期限は不変。
追加試験は行っていない。次の走行やlapには別の有限枠の承認が必要。

AWSIM固定8fileの前後hash一致。元checkoutのdirty状態も前後
`0b9af671098d0d60fd59457c6aba36e75c88b17ff6a04c1045622ea5617eb9a3`で一致。
既定同期によるDatasetルート存在確認を実施。Dataset内容/raw/学習sensor/V4checkpoint読取は未実施。
今回実scanはTinyの推論に使用したが全sensor保存はしていない。hashだけではsensorを再現できない。
公式配布重みの読取・推論は今回の許可範囲。学習・収集データセット作成・実車接続・自動pushなし。

## 保存と次の確認事項

```powershell
C:/Python310/python.exe tools/summarize_tiny_gui_retry2.py --input tmp/tiny_gui_retry2_20260907/gui_retry2_31f4e5a_01 --output tmp/tiny_gui_retry2_20260907/saved_check
pwsh -NoProfile -File tools/build_tiny_gui_retry2_review_packet.ps1
```

既存の解析出力やZIPは上書きしない。Windows Python3.10/Pillow10.2.0で保存済み画像を変換。
レビューZIPには実行source tar、限定tests、全生ログ、実画面PNG/XWD、manifestを含める。
次の修正候補はRViz設定保存先・正常終了と、実scanが表示されるwindowの確認。
今回のheap破損原因や終了改善は未確定・未実装であり、再走行での解消を主張しない。

### Astra Pro向けプロンプト

添付ZIPのREADME_REVIEW、versions、saved_check/saved_log_check.jsonを起点にレビューしてください。
対象はfis-teria/aichallenge_lite_transfuser、branch codex/windows-wsl-training-sync、実行31f4e5a。
未pushなので添付sourceを正本にしてください。今回は短距離のTiny操舵走行・制動停止まで成立しましたが、
停止後RViz終了待ちでFINALIZE_TIMEOUT_WHILE_PAUSEDとなり、全体clean exitは未達です。
worker→要求→送信と速度観測、停止のfresh stamp持続、freeze前後、GUI所有照合、予算を確認してください。
RViz window存在をscan正常描画としないこと、0byte snapshotを成功扱いしないこと、
正加速度最終送信7.999999822秒とSTOP_BEGIN8.039999821秒を区別することが重要です。
根本因果不明はUNKNOWNにし、次はRViz終了・正常表示の問題だけに絞って提案してください。
V4/S1/学習再監査へ広げず、レビューを追加試行・台帳減額・実車制御の承認にしないでください。
