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

40cmの最初の実走では、C07の進行投影値が一時的に固定された箇所で向き差が
1.076度になり、開始前の1秒安定確認がリセットされた。横ずれは約1.2cm、
停止領域の余裕は約2mで、速度超過による中断ではなかった。
以後はplanへ `"entry_heading_tolerance_rad": 0.03490658503988659` を指定して、
既存の復帰安定条件と同じ2度を開始にも使う。既定値1度と旧記録の意味は維持する。
横ずれ5cm以内、1秒連続安定、現在のクリアランス、センサ・停止監視は維持する。

## 実装の検証

- 収集速度条件の変更: `78e6f89d5a97aa380d9e3d8281b6368788d7c72a`。
  native WSLで `2730 passed, 4 skipped`。
- 開始時の向き許容値を明示する変更: `21ccfef00a415c85dd033d447cd8ed25a5ad974f`。
  native WSLで `2731 passed, 4 skipped`。既定の1度、明示した2度、範囲外、
  横ずれ過大、現在のクリアランス不成立を回帰テストで確認した。
- 20cmの全runと40cmのpair01は前者、40cmのpair02以降と60cmのpair01/02は後者を使用する。
  各runの `source_sha` と参照設定を記録する。

60cmでは、最新LiDARの時刻が最新poseより約1.3ms先で、1つ前のLiDARもposeの
70ms欠落区間に当たる事例を確認した。20ms待つ再取得1回ではまだ実測poseが揃わず、
計算自体は約2msでも停止した。時刻関連の理由に限り再取得を最大3回とし、再取得・待機・
再計算を含めた100msの総締切、150msのcapture鮮度、poseの実測補間条件は維持する。
補間用poseの捏造・外挿、停止領域・source異常・clock resetの再試行は行わない。
CPU割り当てを3Pコア/環境に増やす試験ではこの問題を解消できなかったため、元の
2Pコア/環境と、AWSIM/Autoware用1P+4Eコア/環境の構成へ戻す。

独立した停止領域計算の物理モデルが扱える速度上限6km/hは変更していない。
収集用の上限超過を理由に捨てる条件と、実測速度で車体停止領域を計算できる条件は
別である。今回の実走速度と上限超過による収集停止数は、最終集計に記録する。

## データと再現手順

native WSLの保存先は各振幅について次のとおり。

- raw: `/home/thistle/e2e_autonomous/raw/time_corner_multiscale{20,40,60}_20260916`
- 教師・入力キャッシュ・監査:
  `/home/thistle/e2e_autonomous/runs/time_corner_multiscale{20,40,60}_20260916`
- 教師は復帰開始後の実測将来3秒、30点のxy座標 `[N,30,2]`。
  準備用に生成した経路は教師に含めない。
- domain 1のrunをtrain、domain 2のrunをvalidationとする。
  同じコーナーの独立走行であり、未見コーナーへの汎化を測るsplitではない。

実行したコマンド形式:

```powershell
python -u tmp/time_corner_multiscale40_20260916/collect_pair.py --pair 2 --left train_lap02_h2_e2 --right validation_lap02_h2_e2
python -u tmp/time_corner_multiscale40_20260916/audit_pair.py --pair 2
python -u tmp/time_corner_multiscale40_20260916/audit_finish.py --pair 4
python -u tmp/time_corner_multiscale40_20260916/finish_remote.py
```

`collect_pair.py` が独立した2環境の起動、周回、停止、転送、照合、確認済みrawの
実行側整理を行う。`audit_pair.py` はWSL lock内で教師生成・入力検証・時計監査を行う。
走行中に前pairの監査を進めるが、同期・試験・転送照合とWSL lockを同時に取得しない。
40cm pair01は同期とのlock競合で照合の起動だけが拒否されたため、保存済みarchiveから
`resume_transfer01.py` で照合を再開した。再走行や未照合rawの削除は行っていない。

実際の計画JSON、operator、検証receipt、全体テストログを
`docs/evidence/time_corner_multiscale_20260916` に保存する。
operatorは実行履歴であり、終了済みcampaignへ同じrun名で再実行しない。
新しい収集では保存先・run ID・有限の試行上限を新設する。

これらは教師走行の収集・監査結果であり、再学習後のE2Eモデルの完走や復帰改善を
示す結果ではない。再学習・学習モデルによるAWSIM試験は別途必要である。
