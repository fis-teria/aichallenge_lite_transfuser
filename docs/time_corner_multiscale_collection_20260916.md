# コーナー復帰 20 / 40 / 60 cm の追加収集

2026-09-16ユーザー指示: 20 cmの残件を先に試し、その後40 cm、60 cmを収集する。
収集用の速度上限超過だけを理由に収集や教師採用を中断しない。

`LargeRecoveryConfig.speed_policy=record_actual_v1` を新しい収集計画へ明示する。
目標速度は5 km/hのまま。実測速度はraw、入力、教師、イベント監査へ残す。
旧収集帯1.15〜1.4 m/sの上限と旧nominal速度監視6 km/hは、この明示モードでは
超過のみを理由に中断しない。前進条件、非有限値・逆走、指令値、センサ・時計、
実測速度を使った停止領域監視は維持する。旧記録は既定の旧条件で再現する。

実行先は `graneple@192.168.3.10`。AWSIMを変更せず、別ネットワーク名前空間の
ROS_DOMAIN_ID 1 / 2でtrain / validationを独立したrunとして収集する。
各振幅最大12 run、1周最大3イベント、各run最大30分。pairごとにrawをWSLへ
転送・hash/SQLite検証後、確認済み転送対象だけを実行側から整理する。
地図不適合地点は未取得として記録し、安全に実現できる候補を別途調べる。
同一条件の失敗を繰り返す代わりにログで原因を確認して次の候補へ進める。

Windowsでコミット後、公式同期でnative WSLへ反映する。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py check
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
```

native WSLでの検証:

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

経路生成は `tools/generate_time_large_recovery_reference.py` に渡すplan JSONへ
`"speed_policy": "record_actual_v1"` を指定する。生成・監査も同じWSL lock内で行う。
準備経路は教師に含めず、通常PPへ切り替えた後の観測と実際の将来3秒を採用する。
単なる完走数、採用復帰イベント数、指定入口状態の取得数は区別して報告する。
