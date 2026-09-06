# Tiny新package / CONTROL_METHOD: 実装・GUI短試験結果

## 結論

`aic_tiny_sim_test` と `CONTROL_METHOD=tiny_lidar_net_guarded` を追加した。
限定pytestは最新 **98 passed**、専用ROS packageのビルドも成功。
ただし今回のGUI短試験はDockerの制御container作成で失敗したため、
AWSIMとRVizの同時表示、Tiny実scan推論、走行、制動停止、一周はいずれも未確認。
Docker設定は修正し、同じnetwork共有条件のX11接続だけを確認した。再走行はしていない。

## 版と環境

| 対象 | 固定版・状態 |
|---|---|
| origin / branch | `https://github.com/fis-teria/aichallenge_lite_transfuser.git` / `codex/windows-wsl-training-sync` |
| 開始Windows HEAD | `5c8079dd3e0592d403c4be7992cdb6cc3e6bc4a9`、clean |
| 実装・ROS build・GUI attempt | `52376121d52a44f678e13a20a87127e1532e203a` |
| Docker修正・最新限定test | `d0b86e3cc89bbcecabfd841c7c9a772352043566` |
| simulator host | `graneple@192.168.3.10` (`graneple-local`) |
| 既存環境の実パス | `/home/graneple/git/autononous_ai/aichallenge-racingkart` (`autonomous_ai`ではない) |
| 既存環境HEAD | `4af395eee10f928c7fc7225760adfa04c4c07ff4`、151 dirty entriesを維持 |
| 専有出力 | `/home/graneple/e2e_autonomous/tiny_gui_control_method_20260907` |
| 固定image | `sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b4c6b96d1ba7` |
| 公式Tiny出典 | `AutomotiveAIChallenge/aichallenge-racingkart` commit `1f54dff995d02625566341f9e1be1c39369224f2` |
| 公式重みSHA256 | `7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963` |
| 配布source_5237612.tar SHA256 | `d1032fa907de427fb0452b7a3b5d94cdf775ab2a06ad7ea0dd2085451c255228` |

Windowsが編集・Git正本、WSLは同commitのlock付き検証、SSH先は専有archiveで実行。
既存dirty checkoutは変更していない。既存Makefileのdev recipeを新method専用includeから使用し、
通常のAutoware制御stackではなく新ROS packageのsupervisor/RVizを起動する構成。
公式Tinyの推論coreを利用するが、公式ROS nodeを無変更で起動した結果ではない。
通常の無指定`make dev`と同じ構成だったとは主張しない。

## 限定検証

| 確認 | 結果 | 限定 |
|---|---|---|
| 5237612 WSL pytest | 98 passed / 5.76 s / exit 0 | 2 fileの限定test、全pytestではない |
| d0b86e3 WSL pytest | 98 passed / 2.94 s / exit 0 | 同じ98件の再実行、196件の異なるtestではない |
| 公式重みload-only | 全18 tensors / 150286 elementsのkey・shape確認 | model forwardなし |
| 5237612 ROS build | 1 package finished / 1.48 s / exit 0 | 追加packageのみ、元workspace未変更 |
| package executable / help / show-args | exit 0 | ROS node起動・推論なし |
| d0b86e3共有network + Xauthority probe | `X11_OPEN=True` / exit 0 | XOpenDisplay/XCloseDisplayのみ。ROS・model・制御なし |

ROS buildは06:40:03–06:40:05 JST、X11 probeは06:43:21–06:43:22 JST。
これらは試行台帳wallとは別のビルド・診断時間であり、sim/forward消費ではない。
初回Windows PowerShell同期でBOM由来の`set: command not found`が出たため、
未変更scriptを`pwsh -NoProfile`でCheckOnly/通常同期とも再実行し成功した。両方のログを保存。
ROS buildのbyte-compilation無効警告も保存し、無警告だったとは扱わない。
実行commandの記録は配布物の`evidence/execution_commands.md`と各attemptの`host.jsonl`。

`HOST_MONITOR_STALE`対策は、ARM JSON読取後の時刻取得と失敗時stamp/age/reason記録を追加。
750 msの閾値は変更していない。静的反例を修正したもので、前回lapの実原因確定や今回live解消ではない。

## GUI短試験: gui_short_5237612_01

- 実行版5237612、2026-09-07 **06:41:15.743–06:41:17.105 JST**、host wall 1.362440442 s。
- 既存Makefileのdev recipeからAWSIM containerは起動した。
- autoware serviceの作成が `conflicting options: hostname and the network mode` で失敗。make exit 2。
- `runtime_container_id=null`、supervisor/worker未起動。今回のTiny推論・制御送信は開始していない。
- 原因はnetwork namespace共有serviceにhostnameを併用したcompose設定。d0b86e3でそのserviceのhostnameを削除し、Xauthority向け`XAUTHLOCALHOSTNAME=graneple-local`を明示。
- 修正版は限定testと非ROS X11接続のみ確認。修正版ROS起動、両windowの所有PID照合、駆動・制動は未実施。
- 部分起動した所有AWSIMをcleanupでpause後KILL、exit 137。unpauseなし、cleanup errorsなし。
- `host_pause_verified=false`は通常watch loopの証拠欄。cleanupの`sim_pause`/`sim_kill_frozen`とは区別する。
- 実画面動画・snapshotなし、judge section/lapなし。接触・逸脱なしを証明する結果ではない。

Makefileの`capture-run-fingerprint`置換警告は新methodの明示file allowlist分岐に伴うもの。
旧source/build/install全探索やPilotNet重み読取は行わず、全tree整合性証明とも扱わない。

## 予算: 観測と保守計上を区別

ユーザーの追加短試験許可を06:41:15.737 JSTに適用。累積駆動上限3→4のみ変更。
承認記録は`configs/control/tiny_gui_authorization_20260907.json`、SHA256
`b7d722c01979958ed92aa2e2d4cf21b8098d6edb56ed27b086eea106ef6283a7`。
旧8 attemptと消費を保持した。GUI profileは1 attempt限定で、未駆動失敗でも自動再試行しない。

今回、runtime最終summaryがない既存の保守精算規則によりTiny600forward・駆動1回・20sim秒を計上。
**600回推論した、20秒走行したという実測値ではない。** `tiny_forward_exact=false` / `exact=false`。
実際の起動失敗の証拠を根拠として、勝手に台帳を減額・復活させることもしていない。

| 共通台帳 | 最終used | 上限 | 残量 |
|---|---:|---:|---:|
| wall s | 1255.156139992 | 3600 | 2344.843860008 |
| shared forward | 1099 (V4 124 / Tiny 975) | 6000 | 4901 |
| powered attempts | 4 | 4 | **0** |
| powered sim s | 41.259999524 | 300 | 258.740000476 |
| log bytes | 63082486 | 536870912 | 473788426 |
| snapshots | 7 | 16 | 9 |
| MPC | 1 (過去分のみ) | 6000 | 5999 |

今回log計上1121113 bytes（保守reserve含む）、snapshot 0、active=null。
期限は09:50 JST駆動終了/10:00 JST提出のまま。残りwall/forwardがあってもpowered残量0。
再走行には、追加1回の明示許可と有限上限、または証拠に基づく台帳照合・再試行方針の承認が必要。
実装完了・パッケージ配布は走行枠の追加許可ではない。

## 保全と未確認

- 既定同期によるDatasetルートの存在確認を実施。
- Dataset内容・raw・学習sensor・V4 checkpointの読取りは未実施。公式Tiny配布重みは明示許可されたload-only検証対象であり、全重み読取ゼロとは言わない。
- AWSIM固定8 fileの前後hash一致。実行物/scene/DLL/vehicle/sensor設定の変更なし。
- 元remote checkoutのporcelain SHA256は前後`0b9af671098d0d60fd59457c6aba36e75c88b17ff6a04c1045622ea5617eb9a3`。実値は`evidence/final_host_state.txt`。
- 06:43:23 JST最終確認時に今回の実行中containerなし。既存processの停止なし。
- 学習・V4/MPC改良・追加収集・実車接続なし。自動pushなし。

最初に進行を止めた要因はDocker hostname/network競合。修正後に残る具体的作業は、
承認された新しい有限枠で同構成を配布し、両GUIの所有・表示確認→実scan推論→短距離駆動→制動停止を検証すること。
一周完走や独立監査済み、実車安全性、競技採用を主張しない。

## 配布物の再作成

cleanなWindows checkoutで、保存済み今回logだけを読み梱包する（推論・test・ROS起動なし）。
既存出力は上書きしないため、再梱包は新しいOutputNameを指定する。

```powershell
pwsh -NoProfile -File tools/build_tiny_gui_review_packet.ps1 -OutputName review_tiny_gui_d0b86e3_v1
```

`tmp/tiny_gui_control_method_20260907/`にZIPと外部receiptを作る。
README_REVIEW内にAstra Pro向けプロンプト、ZIP内manifestに各fileのhashを含む。
ZIPから全対象entryの長さとhashを再照合する。
