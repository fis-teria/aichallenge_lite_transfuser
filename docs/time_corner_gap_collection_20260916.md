# 未取得のコーナー入口状態を狙う追加収集

2026-09-16の追加指示。既存の20/40/60cm収集を保全し、新しい有限campaignで
未取得条件を狙う。実行先はgraneple@192.168.3.10、ROS_DOMAIN_ID 1/2の2環境。
AWSIM本体を変更しない。目標5km/h、`record_actual_v1`、復帰観測15秒を使い、
収集用速度上限超過だけでは除外しない。停止領域、センサ、時計、制御締切は維持する。

最初の診断はnative WSLの `runs/time_corner_gap_20260916/initial_diagnosis.json`。
前回の復帰サンプルは有効だが、原計画の入口横ずれ±5cm・向き±1度に合う画像が
0〜2枚のイベントが多く、3枚必要な入口カバレッジを満たしていなかった。
準備経路のoffset/headingだけを小さく補正し、実測の目標値と教師定義は維持する。
準備経路を教師へ混ぜず、実際の復帰開始後の観測と実測将来3秒を使用する。

事前の半径1.4m円形検査は、C03/C06の正常走行軌跡にも不適合を出していた。
明示した `map_screen_policy=oriented_body_v1` は、標準車体をbase_link基準
前方1.985m・後方0.509m・半幅0.85mの矩形として、地図上の姿勢・回転を検査する。
半幅は実行時の標準監視と同じ。地図セル半対角とサンプル間移動の上界を加え、
未知セル・地図外・車体内部の障害物も不適合とする。
これは静的な準備経路と候補復帰経路の検査であり、実際の復帰成功を保証しない。
実走で確認してから教師へ採用する。旧記録の既定は円形判定のまま維持する。

各siteの `preparation_offset_bias_m` は±0.1m以内、
`preparation_heading_bias_rad` は±2度以内。これらは人工的な準備経路だけを変え、
`target_offset_m` / `target_heading_rad` の実測到達条件は変更しない。
別位置・別方向・別振幅の代替は元の条件取得に数えない。

Windowsでコミット後、公式同期を行い、生成・検証・教師化はnative WSL lock内で実施する。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py check
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
```

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

新しいcampaignは最大12 run（6 pair）、1周最大3イベント、各run最大30分。
各pairの閉じたrawをWSLへ転送してhashとSQLiteを検証し、確認済み対象だけ実行側から整理する。
同条件の失敗を繰り返さず、pairごとの実測到達状態と採用結果で次の計画を更新する。
全コーナー取得・学習モデルの改善・再学習の完了は、対応する実測結果がない限り主張しない。
