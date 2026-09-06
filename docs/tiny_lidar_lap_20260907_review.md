# README_REVIEW — Tiny配布重み・未改変AWSIMの実行証拠

## 先に結論

**一周していません。一周試験へ進む前に予算追加承認待ちです。**
配布重みの全parameter照合、停止中8scanの推論、短い前進・操舵反映・制動停止は実測済み。
Ready後に走行開始する修正は39tests passですが、修正後の実走は未実施です。
最高速度は0.297m/s。一周・本格カーブ通過・定常目標速度・無接触・無逸脱の成功は主張しません。

## 読む順序

1. `source/docs/tiny_lidar_lap_20260907_results.md`：全attempt、数値、未検証、必要予算。
2. `source/docs/tiny_lidar_lap_20260907.md`：出典、入力/出力、実consumer、隔離・停止、実行方法。
3. `attempts/*/official_identity.json`：固定配布重み18tensor/150286elementsの全照合。
4. `attempts/*/tiny_worker.jsonl` と `tiny_supervisor.jsonl`：scan→推論→要求/送信→状態。
5. `attempts/*/tiny_supervisor_summary.json` と `host_summary.json`：移動/制動停止/host freezeを区別。
6. `attempts/*/awsim_unity.log`：実AWSIMログ。今回section/lap完了記録なし。
7. `evidence/tests_ready_fix.txt` と同期ログ、`consumer_static/`、`manifest.json`。

実行コマンドは`evidence/*console.txt`と`attempts/*/host.jsonl`。
READMEの説明より生ログを優先し、矛盾は指摘してください。
`consumer_static/`は同SHAの既存DLLを以前静的展開したもので、今回AWSIMを書き換えたものではありません。
sourceの`awsim_dev_v4/simulator.sh`とhost helperは既存隔離・終了処理の再利用です。
Tiny実行ではV4モデル、参照fit、MPCを呼んでいません。

## 版・含有物の境界

- 使用公式Tiny commit `1f54dff995d02625566341f9e1be1c39369224f2`。
- 重みSHA256 `7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963`。
- 最新実走版 `4d6ddd284323fa73f38d6106b22cbd5a0eeb37ed`。
- Ready修正runtime版 `dcf3f9c248609ecd4e8a49e916f1ac510fff6e96`。梱包HEADはmanifestに記録。
- `official_source/`は公式core/modelの小さな固定sourceコピー。重み・Dataset・raw・V4 checkpointは含めない。
- 失敗attemptを含む全3attemptを保存。動画なし、診断図も実画面として添付しない。
- `manifest.json`は自分自身を除く全fileのSHA256。ZIP再読取で全一致を検証。

## Astra Proへ渡すレビュー依頼

あなたは、公式TinyLidarNet配布重みの利用とシミュレータ実走証拠を検査するML/制御エンジニアです。
このZIPのみをまず読み、次の点を根拠file/field付きで評価してください。

- 公式package/重み/全key・shape、scan点数・角度順・前処理・出力単位は整合しているか。
- Tiny操舵と速度制限/停止が分離され、正しいsim consumerへ単独で届いた証拠はあるか。
- 停止中推論、正加速度要求、実前進、実制動停止、host pause/KILL、完走を混同していないか。
- 最新実走版とReady待ち修正版の未実走境界は明確か。
- 約368mという既存referenceに対して、1回240sim秒/累計300sim秒/forward共通枠6000の
  提案は有限か。これらは未承認であり、実行を許可するものではない。
- 次のReady後短試験→一周試験に直接必要な残る問題だけを挙げてください。

結論は「確認済み」「未確認」「修正必須」を分け、証拠のない完走や安全性を認定しないでください。
V4/S1/参照fit/学習法の再監査、新モデルやMPCの設計へ広げないでください。
これは独立レビュー待ちを新たな実行gateにする依頼ではありません。
実行gateはユーザーからの明示的な予算追加承認です。
