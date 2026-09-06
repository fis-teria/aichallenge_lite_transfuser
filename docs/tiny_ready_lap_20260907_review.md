# README_REVIEW — Tiny Ready後の実試験（旧packetを更新する今回資料）

Ready後shortは成立。**lapは実行したがHOST_MONITOR_STALEで早期停止し、未完走。**
両試験ともReady後の新scanで発進し、移動中のTiny操舵更新、制動停止、host freeze/KILLを記録した。
累計powered3/3なので追加走行なし。衝突/逸脱はUNKNOWN。独立レビューは未実施。

今回の実装・test・両実走SHA: `219eb2f00b8e4277275072d0b5e9f344f365edc7`。
文書/梱包SHAは`versions.json`および`PACKAGE_MANIFEST.json`。
未pushの添付固定版を主資料にし、公開branchを別の最新版へ置換しない。

## 読む順序

1. `PACKAGE_MANIFEST.json`、`versions.json`。
2. `request/implementation_request.txt`（今回許可全文）、shortの`budget_authorization_change.json`。
3. `report/tiny_ready_lap_20260907_results.md`、`diff/`。
4. `attempts/*/resolved_config.json`とbudget前後、`official_identity.json`、`instance_inspect.json`、`consumer_static/`。
5. `evidence/tests_219eb2f.txt`/XML、`execution_receipts.json`/`execution_commands.txt`。
6. 今回2attemptのworker/supervisor/host/元Unityログ。
7. `history/attempts/`の旧3attemptと`history/evidence/`。旧結果・旧未承認はその時点の履歴。

数値の補助集計は`evidence/saved_log_check.json`。説明と矛盾すれば生ログを優先する。
全Tiny5attempt、旧V4を含む共通budget履歴は保持。今回V4/MPC実行なし。
公式sourceの小さな固定コピーを含めるがweight/sensor/Dataset/仮想環境は含めない。
重み非同梱なので独立確認の限界を残す。実画面動画なし、生成図による代替なし。

## Astra Proへ渡すもの

このZIPを添付し、**`request/independent_review_request.md`全文**をプロンプトとして渡す。
元依頼の第8節を省略せず、今回の実値と結果を記入した文書である。
静的レビューのみ。コード/test/command実行、SSH/ROS、修正、学習、追加走行、pushは禁止。
「Ready修正の実走」「短試験」「一周」「制動」「host強制停止」「無接触/無逸脱」を別判定する。

一周を止めた直接条件はHOST_MONITOR_STALE。ARM-at-faultが未記録なので内部原因はUNKNOWN。
読取前nowと新ARMの競合は静的反例であり、今回実際に起きたと断定しない。
新規監査基盤・V4/S1/学習法へ広げず、直接関係する最小修正だけを指摘してもらう。
レビューは4回目の駆動枠や再走行を許可しない。
