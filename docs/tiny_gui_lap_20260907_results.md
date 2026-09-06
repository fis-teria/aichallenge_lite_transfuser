# Tiny GUI 1周試験結果：未完走、区間4後に壁際で停滞

## 結論

2026-09-07 07:25:17–07:29:04 JST、追加承認された1回を実施した。
未改変AWSIMとRVizを実デスクトップ表示し、新規ROS packageの
`CONTROL_METHOD=tiny_lidar_net_guarded` / Lite側 `make dev TINY_GUI_LAP=1` を使用。
judge区間は0→1→2→3→4、Lap completedログは0件。**1周未達**。
保存AWSIM画面では車両が壁際へ突き当たったような姿勢で静止している。
接触イベントの定量証拠は未取得のため、衝突時刻・無接触性はUNKNOWN。
画面の「Lap 1」は周回中表示であり完走証拠として使わない。

最初の進行阻害は区間4通過後の壁際停滞。原因となった操舵/scan/速度の寄与は未断定。
公式Tinyの使用と制御接続は成立したが、この構成での完走は成立していない。
推論・要求・送信一致は実consumerでの全操作適用ACKではない。

## 固定版と環境

- 実行/限定テストcommit: `bf0498aea8761dac74e04d764907519d30843056`。
- 開始HEAD: `a40abef37047dab7b4184d250552456fe8105c65`、clean。
- Windows正本origin: https://github.com/fis-teria/aichallenge_lite_transfuser.git
- branch: `codex/windows-wsl-training-sync`。
- 実行ホスト: `graneple@192.168.3.10`。
- 既存repo: `/home/graneple/git/autononous_ai/aichallenge-racingkart`、HEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`。
- 既存dirty状態のSHA256は前後一致: `0b9af671098d0d60fd59457c6aba36e75c88b17ff6a04c1045622ea5617eb9a3`。
- 実行専有root: `/home/graneple/e2e_autonomous/tiny_gui_lap_20260907`。
- source: `source_bf0498a`、install: `install_bf0498a`、attempt: `gui_lap_bf0498a_01`。
- 公式Tiny commit: `1f54dff995d02625566341f9e1be1c39369224f2`。
- 配布重みSHA256: `7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963`。
- 全18 tensor / 150286 parameterを完全load、partial loadなし。公式coreは不変。
- target2.0m/s、上限2.4m/s。学習加速度は不使用。V4/MPCは今回0。

source tar SHA256: `faf52fc8513e72dbeef3b82cf6f4ee26fbeaabbe1d860eb667d08638a941ea80`。
raw attempt tar SHA256: `09d436f81118466e29a18c50290c2e39a3cd4a62a3539844836ccfdef6918941`。
原本AWSIMの固定8ファイルhashは前後一致。実行物・scene・sensor設定は変更していない。
実行引数全文はpacketのevidence/execution.logおよびmake_invocation.jsonに保存。

## 実測と停止の限定

| 項目 | 結果 |
|---|---:|
| wall | 227.387秒 |
| 駆動開始から停止までのsim | 198.830秒（停滞時間を含む） |
| Tiny forward | 3971 |
| Tiny操舵送信 | 3793 |
| 送信に対応する異なるscan | 3727 |
| 移動速度観測付きTiny送信 / 異なるscan | 2351 / 2306 |
| 最大速度 | 2.018901m/s |
| request→sent / worker→sent不一致 | 0 / 0 |
| Ready過去scan違反 | 0 |
| 停止系送信 / 負加速度の停止送信 | 10 / 0 |
| 最終速度 | 0.0m/s |

区間4以降に進行せず、低速観測を受けて操作担当が途中停止を要求した。
停止直前速度0.001523m/s、最後の0.1m/s以上の観測から74.545sim秒が経過。
現行監視は低速停滞を停止理由にしておらず、この間も正加速度要求を継続した。
壁際での詰まりを自動停止できたとはしない。

**操作担当側の失敗**: stop_request.jsonを最終名へ直接scp転送し、作成途中の読取と競合。
supervisorは `JSONDecodeError: Expecting value: line 1 column 1 (char 0)` で停止処理へ入った。
意図したreasonは `PROGRESS_STALLED_OPERATOR_STOP`、実際の終了reasonは上記例外。
正しい正常停止要求として成功扱いしない。次回は専有一時名へ転送してから同一directory内のatomic renameで公開する。

runtimeの `observed_braking_stop=true` は保存値のまま保持するが、今回は負加速度送信0件であり、
実際には停止要求前からほぼ静止している。これは**制動因果の証拠ではない**。
freeze前の新しい静止観測16件は確認。hostはその後AWSIMをpauseし、unpauseせず所有instanceをKILL終了。
AWSIM exit137、supervisor exit1、ROS launch/container exit0。
親container exit0だけで全体正常終了としない。
cleanupエラー0、最終docker ps空。

## GUI・検証

RVizの専有設定保存先とruntimeのみのGLX vendor指定を追加。
AWSIMとtiny_scan.rviz付きRVizメインwindowのPID/可視性を確認。
今回RVizはSIGINT後に正常終了。前回の保存エラー・heap corruption・FINALIZE_TIMEOUTは再現しなかった。
原因全体の確定やあらゆる終了状況の解決を意味しない。

AWSIM実画面はsaved_check_final/awsim_saved_frame.png（既存XWDの無加工format変換）。
RViz XWDは保存したがheaderのbits-per-pixel24とstrideが本変換器の対応外のためPNG化しない。
診断図や生成画像を実画面としていない。動画はなし。judge生ログを保存。

Windows commit→既定CheckOnly/通常同期→WSL lock付き限定pytest **104 passed**。
追加ROS packageの固定image内colcon build成功。
全体pytestはDataset等の禁止範囲を避けるため未実施。
**既定同期によるDatasetルートの存在確認を実施。**
**Dataset内容・raw・学習sensor・V4 checkpointの読取りは未実施。**
公式Tiny重みと実sim scanの使用は今回の許可範囲内で実施。

保存ログ照合:

```powershell
C:\Python310\python.exe tools/summarize_tiny_gui_lap.py --input tmp/tiny_gui_lap_20260907/gui_lap_bf0498a_01 --output tmp/tiny_gui_lap_20260907/saved_check_final
pwsh -NoProfile -File tools/build_tiny_gui_lap_review_packet.ps1
```

出力が既存なら上書きしない。初回保存画像変換はRViz layout assertionで停止し、二回目から未対応として明示。
試験再実行ではない。生ログと初回失敗ログを保持する。

## 累計・残枠・次の一手

累計powered **7/7**、sim269.989995/320秒、shared forward5835/7100（V4 124 + Tiny5711）。
wall1538.270059/3600秒、snapshot11/16、log88578313/536870912byte、MPC1/6000。
残りpowered0、sim50.010005秒、forward1265。過去失敗の保守課金を変更していない。
active予約なし。**追加走行は未承認、自動再試行・pushなし**。

次は保存済みログの区間4直前の操舵/速度遷移と画面を対象に、詰まりを起こした一要因を絞る。
まず停止要求のatomic公開と停滞監視を整え、停止因果のラベルを区別する必要がある。
操舵・scan契約・速度を一度に変更せず、原因に直接関係する一項目を選ぶ。
次の1周試験にはpowered枠だけでなく、残sim/forwardに対する具体的な追加承認が必要。
AWSIM改修、新規学習、V4再開、実車安全性保証へ広げない。
