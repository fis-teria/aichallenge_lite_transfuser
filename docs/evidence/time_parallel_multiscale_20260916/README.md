# 実行証拠

学習・比較・入力教師検証はnative WSL、AWSIM実走はgraneple@192.168.3.10で実施。
本ディレクトリは報告・ハッシュ・operatorソースのみで、重み・raw・配列を含まない。

- `parallel_result.json` / `live_isolation.json`: 教師PPでの2環境実走と通信分離。
- `host_final_checks.json`: AWSIM全ファイル不変、元repo不変、container終了。
- `training/`: 学習完了・初期重み一致・再読み込み予測一致。
- `comparison/`: 同一validationでの旧新比較と実測55〜65cm帯。
- `collection/`: 新規並列収集分の転送照合・因果入力・未来教師の検証。
- `operators/`, `operator_dependencies/`: 実行時ソースの記録。直接ここから再実行するためのlauncherではない。

operatorの実行時配置は`tmp/time_parallel_multiscale_20260916`。依存ファイルは
`operator_dependencies`以下の元の相対パスを参照した。既存run IDと保存先は再使用しない。
実行コマンド・条件・解釈は`docs/time_parallel_multiscale_20260916.md`を参照。
実装の全pytest確認はcommit `5e4cea7f030994c2ee4f58fb6fd5af4b170bb1e5`、
2,696 passed / 4 skipped。以降の変更はレポート・証拠保存のみ。
