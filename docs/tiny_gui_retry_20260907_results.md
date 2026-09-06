# Tiny GUI追加短試験の結果（2026-09-07）

## 結論

**実走は未達。** 前回のDocker hostname競合は解消し、今回はAWSIM/control containerの作成と
ROS launch配下のsupervisor・RViz process起動まで進んだ。しかし既存Makefileの子makeが
別Makefileを読んで失敗し、入力/推論/制御の開始前に所有instanceを終了した。
今回の不具合も実装側の起動接続漏れであり、Tinyモデルの性能による失敗ではない。

子makeに同じincludeを渡す修正を追加し、限定101testは成功。
**修正版のROS再起動・GUI表示・走行は未実施**。一周・無接触・制動成功を主張しない。

## 版

| 対象 | 版 |
|---|---|
| origin / branch | `https://github.com/fis-teria/aichallenge_lite_transfuser.git` / `codex/windows-wsl-training-sync` |
| 開始Windows HEAD | `0e15a675b433e74f2efadff0d7635af5a7d55800` / clean |
| 承認追加・build・実行 | `3bddc665efdec7d9ab4f4cc5e89821a2cc4b424e` |
| 再帰make修正・最新test | `af80183146e84f65f4fa8d56aa77e1ef9e8dd167` |
| 実行host | `graneple@192.168.3.10` / `graneple-local` |
| 元環境 | `/home/graneple/git/autononous_ai/aichallenge-racingkart` |
| 元環境HEAD | `4af395eee10f928c7fc7225760adfa04c4c07ff4` |
| 専有出力root | `/home/graneple/e2e_autonomous/tiny_gui_retry_20260907` |
| attempt | `gui_retry_3bddc66_01` |
| source archive SHA256 | `7991b31f05fb412655dc4b9cc39229b4d070508adb85598698cc88f640b87efc` |
| 実行image | `sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7` |
| 公式Tiny出典commit | `1f54dff995d02625566341f9e1be1c39369224f2` |
| 公式重みSHA256 | `7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963` |

Windows編集・commit→未変更同期CheckOnly/通常同期→WSL lock付き検証→専有archive/build。
元remote checkoutは変更していない。公式重みはload-only試験とhost固定hash確認対象で、
今回のruntime workerによるforwardは開始していない。

## 観測経過

- 06:59:47–06:59:49 JST: 新ROS packageだけをビルド。1 package / 1.11 s / exit 0。
  byte-compilation無効の警告あり。既存installには触れていない。
- 07:00:01.932–07:00:11.874 JST: attempt、host wall **9.941168785 s**、make exit 2。
- 両containerのcreate/start成功。従来の`hostname and the network mode`エラーなし。
- 元Makefile `autoware-simulator` 内の `$(MAKE) autoware-command-mode-run` が
  `-f integrations/tiny_gui/Makefile`を引き継がずsource側のMakefileを参照し、
  **`No rule to make target autoware-command-mode-run`相当の日本語エラー**で停止。
- `host.jsonl`の子make stderrに原文。旧Makefileには当該targetが実在し、そのrecipeはno-op。
- ROS launchログにsupervisor PID74、RViz PID76の開始あり。
  RVizには設定directory/DRIの警告があるが、OpenGL 4.5初期化の記録もあり、今回のmake失敗とは分ける。
- hostはmake失敗のため`instance_inspect.json`を発行する前にcleanupへ進んだ。
  supervisorはこのfile待機がROS初期化・worker開始より前にある。worker/supervisor推論・制御ログなし。
- GUIの両window所有PID/IsViewable確認未到達。RViz process起動を両GUI表示成功と扱わない。
- 所有AWSIMをpause→KILL（exit137）、所有control containerをstop（exit137）。cleanup errorsなし、unpauseなし。
  自然制動・運動後の制動停止を観測した結果ではない。
- 07:01:57 JST最終確認: 実行中containerなし。AWSIM固定8 fileの前後hash一致。
  元checkout porcelain hashも前後`0b9af671098d0d60fd59457c6aba36e75c88b17ff6a04c1045622ea5617eb9a3`で一致。

## 修正・限定検証

新methodの`MAKE`変数へ同じincludeの`-f`を渡す変更のみを追加。
実fixtureを模した子make・孫makeを合成Makefileで実行し、command variableも引き継ぐことを確認。
既存remote MakefileやAWSIM、制御・推論方針を変更していない。

| test実行版 | 結果 |
|---|---|
| 3bddc66（承認追加、試験前） | 100 passed / 2.86 s / exit 0 |
| af80183（再帰make修正後） | 101 passed / 3.05 s / exit 0 |

旧100件と最新101件を重複なし201件と数えない。全pytestではなくTinyの2file限定。
ビルド・単体test時間は試行台帳wallとは別記録。修正後のlive試験を合成testで代用したとは扱わない。
同期では既定Datasetルートの存在確認を実施。Dataset内容/raw/学習sensor/V4checkpoint読取は未実施。
公式Tiny重みの許可されたload-only検証を「重み読取ゼロ」とは表現しない。

## 追加承認と最終台帳

ユーザーの「もう一回やりましょう」に基づきprofile `TINY_GUI_RETRY_20260907`を別記録。
承認hash `90b60a4c99c6e17e490454a9410755b33b1f5fa36e5b3c29f27d46d69795156d`。
適用unix ns `1788732001927474902`、記録ISO `2026-09-07T07:00:01.927479+09:00`。
保存欄名は`applied_utc`だが実文字列には+09:00が付いているため、UTC文字列とは扱わない。
旧9 attemptと消費を保持し、powered上限4→5以外の共通上限は不変。

runtime最終summaryがないため、従来規則で今回もTiny600forward・駆動1回・20sim秒を保守計上。
**これは実測の推論600回/走行20秒ではない。** `tiny_forward_exact=false` / `exact=false`。
観測ログを理由に台帳の減額や枠の自動復活はしていない。

| 項目 | 最終used | 残量 |
|---|---:|---:|
| wall s | 1265.097308777 | 2334.902691223 |
| shared forward | 1699 (V4 124 / Tiny 1575) | 4301 |
| powered attempts | 5 / 5 | **0** |
| powered sim s | 61.259999524 | 238.740000476 |
| log bytes | 64272549 | 472598363 |
| snapshots | 7 | 9 |
| MPC（過去分のみ） | 1 | 5999 |

今回log1190063 bytes（reserve含む）、snapshot0、active=null。
09:50 JST駆動終了/10:00期限は不変。追加再試行・lapへの拡張はしていない。
次に必要なのは、別途承認された有限枠で修正版のGUI/scan/駆動/停止を確認すること。

## レビュー資料

`pwsh -NoProfile -File tools/build_tiny_gui_retry_review_packet.ps1`で保存済み資料のみを梱包する。
実行source tar、実行後差分、今回全生ログ、test/buildログ、版情報、manifestを同梱。
重み・sensor・Dataset・実画面動画は含めない。旧試行は共通台帳上の履歴のみ。
ZIP外部receiptと各file hashを照合。自動pushなし。

### Astra Pro用プロンプト

添付ZIPのREADME_REVIEWとversions.jsonを起点に、Tiny GUI短試験の起動失敗と修正を確認してください。
GitHub fis-teria/aichallenge_lite_transfuser、branch codex/windows-wsl-training-syncですが未pushなので添付が正本です。
実行版3bddc66、修正版af80183。前回のDocker hostname競合は解消しましたが、今回は再帰makeが追加includeを
引き継がず起動処理が終了しました。修正後は合成101testのみでlive再試験はしていません。
source tarとpost_execution.patch、host/launchログを参照し、子makeのtarget/変数引継ぎ、
部分起動cleanup、未推論と保守予算計上の区別を確認してください。
RViz process開始を両window表示成功とせず、UNKNOWNを明示してください。
レビューは再実行・追加駆動・台帳減額・期限延長の承認ではありません。V4/学習再監査へ広げないでください。
