# 実行証拠

実走は graneple@192.168.3.10 の2 AWSIM、検証・教師生成はnative WSL。
生bag、画像キャッシュ、教師配列、モデル重みは本ディレクトリに含めない。

- `collection_index.json`: 検証済み保存先、run split、採用数、教師ハッシュ。
- `reference_manifest.json`: 大きな経路配列はnative WSLに保持し、設定・地図判定・保存先・hashだけを記録。
- `coverage_final.json` / `coverage_final.png`: 厳格な入口状態の取得状況。未取得を含む。
- `critical_state_final.json`: 過去の失敗状態との同一座標基準・同一許容幅による比較。
- `skipped_window_diagnosis.json`: 取得窓の見送りを実測制御ログから確認。
- `corners_pair*_verified.json`: ファイル照合とSQLite検査。対応するcleanupは検証後だけ。
- `host_final_checks.json`: AWSIM全ファイルと元repoの不変、所有プロセスの終了。
- `parallel_capacity_pair02.json`: 2台同時走行中のシミュレーション時間の実測。
- `synthetic_smoke_result.json`: 公式ROSの配線試験。学習サンプルには含めない。
- `full_8cf58f1.log`: 最終実装の全pytest結果。
- `final_verification.json`: manifest対象とnative教師の最終照合記録。自己参照を避けmanifestには含めない。

`operators`は実行時ソースの保存で、ここから再起動するlauncherではない。
実行時配置は `tmp/time_corner_recovery_20260916`。依存ファイルは
`operator_dependencies`以下の元の相対パスを使用した。既存run ID・保存先を再使用しない。
再現コマンドと結果の解釈は `docs/time_corner_recovery_collection_20260916.md` を参照。
