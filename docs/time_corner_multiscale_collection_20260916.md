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

## 今回の収集結果

全28 runを終了し、raw 34,857,919,261 bytesをnative WSLへ保存した。
全ファイルのhash・ディレクトリ構造・SQLite quick_checkを照合済み。
採用した教師は55復帰イベント、5,213サンプル。1サンプルは1時刻の観測入力と
実測将来3秒の30点であり、連続サンプルを独立した復帰イベントとして数えない。

| 指定ずれ | 完走 / 実行数 | 採用復帰イベント | train | validation | 合計サンプル |
| --- | ---: | ---: | ---: | ---: | ---: |
| 20cm | 8 / 8 | 18 | 769 | 795 | 1,564 |
| 40cm | 7 / 8 | 19 | 940 | 675 | 1,615 |
| 60cm | 8 / 12 | 18 | 1,010 | 1,024 | 2,034 |
| 合計 | 23 / 28 | 55 | 2,719 | 2,494 | 5,213 |

収集用速度上限による中断は0件。実測最高速度は20cmで5.176km/h、40cmで
5.178km/h、60cmで5.185km/hだった。旧上限1.4m/s（5.04km/h）を超えた教師も
40cmで8、60cmで8サンプル採用しており、速度超過だけでは除外していない。
物理モデルの速度範囲を超えた走行の検証結果ではない。

両側が通常に完走したpairでは、同一ホストのmonotonic時刻に基づく同時走行時間は
約223〜228秒。2並列を実測で確認した。60cm pair01は途中停止まで約89秒、
pair02はd2の起動不成立により同時走行0秒で、成功した並列収集とは数えない。

既存の20cm分（17イベント・1,475サンプル）を合わせると、20cmは35イベント・
3,039サンプルになる。元の入口状態カバレッジで残る不足は、trainが
C01/C03/C06/C07、validationがC03/C04/C06/C07。カバレッジは集計したが、
既存データとの学習用統合や再学習は今回実行していない。

60cmの最後の2 pairでは、C05_LATE・C09・C11、およびC04・C08・C11を各2走行で
収集した。4 runすべて完走し、12イベント・1,526サンプルを採用した。
15秒設定でC04/C08も両splitの独立監査を通過した。採用観測の最大横ずれは
20cm設定で約25.6cm、40cmで約47.1cm、60cmで約69.6cmである。
指定ずれへの到達後に一時的にずれが増える挙動も、実測値として教師に残している。

未採用のrun・イベントもrawと診断を保全した。

- 40cm pair04 d2: C01/C07の復帰後、選択されたscanの受信鮮度で停止。
  独立監査ではrun全体を採用しない。速度超過による停止ではない。
- 60cm pair01 d1/d2: 時刻整合scanの欠落、100ms制御締切超過で停止。
- 60cm pair02 d1: 実測poseとの時刻整合不成立で停止。
  d2は通常RVizの起動確認に失敗し、走行許可前の静止状態で終了。
- 60cm pair03のC04は10秒の境界で独立した連続安定1秒を満たさず、
  pair04のC08は10秒以内に復帰確認できなかった。後続の15秒設定へ読み替えず除外した。

これらの5 runはrawを保持し、今回の採用数から除外している。
原計画で定義した全コーナーの入口状態は未充足。40cmのC02/C03/C06、60cmの
C01/C02/C03/C06は既存の地図クリアランス条件に適合しない。20cmの近傍代替
C03A/C06Aも元のC03/C06を取得したとは数えない。
収集計画の向き許容2度と、原計画の入口状態カバレッジに使う1度は区別する。

以前の停止走行と同じ基準経路・固定許容幅で比較した結果、狭い近傍の一致は
今回も0サンプル。60cm追加では160秒時点（横ずれ約82cm）の広い近傍に
train 5 / validation 6サンプルが入ったが、停止時点（約169cm）の一致はない。
復帰データの追加はできたが、過去の失敗状態をすべて網羅した結果ではない。
比較の定義と件数は各振幅の `critical_state_final.json` に保存した。

## 実装の検証

- 収集速度条件の変更: `78e6f89d5a97aa380d9e3d8281b6368788d7c72a`。
  native WSLで `2730 passed, 4 skipped`。
- 開始時の向き許容値を明示する変更: `21ccfef00a415c85dd033d447cd8ed25a5ad974f`。
  native WSLで `2731 passed, 4 skipped`。既定の1度、明示した2度、範囲外、
  横ずれ過大、現在のクリアランス不成立を回帰テストで確認した。
- 同じ100msの締切内で時刻整合を再取得する変更:
  `dcec9ea47ab833e4e260e99048efa0fb69ece84d`。
  native WSLで `2732 passed, 4 skipped`。
- 復帰観測時間を明示する変更: `a488ab68ab7fac5f5db140fdb514bf5d13afd03c`。
  native WSLで `2733 passed, 4 skipped`。10秒では未確認・15秒では確認できる
  合成復帰、末尾の不安定、記録設定の欠落、非有限値・範囲外を検証した。
- 20cmの全runと40cmのpair01は `78e6f89`、40cmのpair02以降と60cmのpair01/02は
  `21ccfef` を使用する。
  60cmのpair03/04は `dcec9ea`、pair05/06は `a488ab6` を使用する。
  各runの `source_sha` と参照設定を記録する。

60cmでは、最新LiDARの時刻が最新poseより約1.3ms先で、1つ前のLiDARもposeの
70ms欠落区間に当たる事例を確認した。20ms待つ再取得1回ではまだ実測poseが揃わず、
計算自体は約2msでも停止した。時刻関連の理由に限り再取得を最大3回とし、再取得・待機・
再計算を含めた100msの総締切、150msのcapture鮮度、poseの実測補間条件は維持する。
補間用poseの捏造・外挿、停止領域・source異常・clock resetの再試行は行わない。
CPU割り当てを3Pコア/環境に増やす試験ではこの問題を解消できなかったため、元の
2Pコア/環境と、AWSIM/Autoware用1P+4Eコア/環境の構成へ戻す。
60cmの全runで、再取得1回後に通常の正速度指令を送信できた記録は7件。
2回以上の再取得成功例は今回の実走にはなく、3回化の分岐は回帰テストで検証した。
改善後の正常周回だけから、初期の遅延停止が今後すべて解消すると判断しない。

60cmのC04では観測時刻と指令送信時刻の差が10秒の境界で監査結果に影響し、C08では
10秒以内に安定1秒を満たせなかった。該当記録は保全し、既存の採用条件では未採用とする。
60cmの追加計画だけ `recovery_duration_s=15.0` を明示し、復帰を観測する時間を延ばす。
既定値は10秒、設定範囲は10〜15秒。横ずれ10cm以内・向き2度以内・1秒連続安定、
最後の安定状態、将来3秒の確保、センサ鮮度と100msの制御締切を維持する。
既に記録済みの10秒イベントを15秒条件へ読み替えない。

独立した停止領域計算の物理モデルが扱える速度上限6km/hは変更していない。
収集用の上限超過を理由に捨てる条件と、実測速度で車体停止領域を計算できる条件は
別である。採用イベント内の制御sim時刻の最大間隔は20/40/60cmの順に
約85/90/95msだった。各最終indexとclock監査へ記録した。

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
python -u tmp/time_corner_multiscale60_20260916/collect_pair.py --pair 6 --left train_lap06_f15 --right validation_lap06_f15
python -u tmp/time_corner_multiscale60_20260916/audit_finish.py --pair 6
python -u tmp/time_corner_multiscale60_20260916/finish_remote.py
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

3振幅の終了時にAWSIMの1,089ファイルを照合し、開始前から変更がないことを
確認した。実行先の元repoも不変。収集用containerとsupervisorは終了し、
今回のrawはWSL照合後の対象だけ実行先から整理した。最終の実行先空きは約14.8GiB。

集計は `operators/cm20/aggregate.py`、証跡パックは `operators/cm20/pack_evidence.py`、
同期後のhash・source・教師shape・split検証は `operators/cm20/verify_final.py` に保存する。
これらのnative処理も `tools/with_wsl_training_lock.sh` 内で実行する。
証跡の `manifest.json` は各ファイルのbyte数とSHA-256を記録する。
最終検証の `final_verification.json` はmanifest作成後に追加するsidecarであり、
自己参照を避けるためmanifestの対象外とする。

これらは教師走行の収集・監査結果であり、再学習後のE2Eモデルの完走や復帰改善を
示す結果ではない。再学習・学習モデルによるAWSIM試験は別途必要である。
