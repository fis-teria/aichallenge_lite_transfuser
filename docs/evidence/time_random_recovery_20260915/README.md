# 条件付き反復外乱の収集結果

[実施記録](../../time_random_recovery_collection_20260915.md)の小容量evidence。実行・全pytestのsource commitは `404ea7cf51097c74cb0ba0a26753fa5b8d219666`。モデルの学習・走行評価ではなく、教師PPによる実測収集とnative WSLでのデータ品質検証である。

`collection_index.json` が確定集計。独立2run・6イベント、train r64に273件、validation r65に243件、合計516件。両run完走・正常停止、原本と生成入力・教師の照合PASS。`evidence_manifest_v2.json` の23ファイルは、転送元とのSHA256一致または実行した解析scriptとの一致を確認したもの。原本bag・学習配列・重みは含まない。

| 内容 | ファイル |
| --- | --- |
| 計画・seed・split | `collection_plan.json` |
| 実行sourceと参照のhash | `preparation.json`, `deployment.json` |
| WSL全pytest 2464 passed / 4 skipped | `test_gate.json`, `pytest_full_404ea7c.log` |
| 公式イメージの構文/import/有限スケジューラsmoke | `official_image_smoke.json` |
| rawの圧縮・全hash/構造/SQLite・承認済み整理 | `random_pair01_20260915_{shipping,verified,cleanup}.json` |
| イベントごとの実測状態・採用数 | `r64_event_summary.json`, `r65_event_summary.json` |
| 全生成ファイルの再hash・教師shape・代表6anchorのraw入力再生一致 | `post_collection_verification.json` |
| 562候補→516採用、46除外の排他的内訳 | `selection_and_visual_details.json` |
| 横ずれ・向きの実測復帰曲線 | `recovery_events.png` |
| 実カメラと未来30点の教師（目視確認済み） | `representative_inputs_and_teachers_v2.png` |
| 元checkout/コンテナ/compose/sourceの終了照合 | `host_final_v2.json` |

`host_final.json` は初回の順序依存チェックの失敗記録。Docker psのMounts文字列表現の順序だけが変わるため、トークンと重複数を保つ比較で再確認した。他のチェックは厳密比較のまま。`representative_inputs_and_teachers.png` は狭い軸幅の旧図で保全用。採用図v2は共通のXY範囲・左正の向きで再描画した。元ラベル・split・最初の検証結果は変更していない。

初回r62は収集スケジューラの確認保持不具合の診断用で、1周完走したが反復ゲート不成立。`r62_diagnostic_verified.json` はその保全用archiveの照合記録。r62は516件の集計や新しいsplitへ含めない。初回r63と固定位置r52〜r61は未実行。従来test4runとr48/r49も本タスクで読んでいない。

## 実行コマンドの記録

正本Windows checkoutでcommit後に `tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1` を実施し、WSLの同一commitで全pytestを通してから専用runtimeへ配置した。[本記録](../../time_random_recovery_collection_20260915.md)にprepare/setup/collectのコマンドを掲載。次のcollectは一度だけ実行済みで、計画はsealed。消費済みIDや既存出力へ再実行しない。

```powershell
python -X utf8 -u tools/collect_time_random_recovery.py collect --plan configs/time_path_p1/random_recovery_confirmed_20260915.json
```

collect内のnative WSL監査は以下のコマンドで実施された。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u tools/collect_time_random_recovery.py audit \
  --plan configs/time_path_p1/random_recovery_confirmed_20260915.json
```

追加確認は以下のWindows driverから実行した。各driverの中でnative WSLのworktree lockを取得する。ホスト確認のみAWSIM実行先へ読み取り接続する。追跡版 `inspect_prepared.py` / `verify_host.py` / `render_representatives.py` は実行時のdriverと同一bytes。結果ファイルを保護するため、既存出力への再実行はしない。

```powershell
python -X utf8 -u tmp/inspect_random_recovery_confirmed_20260915.py
python -X utf8 -u tmp/verify_random_recovery_host_final_20260915.py
python -X utf8 -u tmp/finalize_random_recovery_visuals_20260915.py
```

生データは `/home/thistle/e2e_autonomous/raw/time_recovery_random_confirmed_20260915`、解析と `materialized` / `prepared` は `/home/thistle/e2e_autonomous/runs/time_recovery_random_confirmed_20260915`。各anchorの `recovery_event_id` とrun単位splitを保持している。既存学習データへの統合と再学習は未実施。
