# 未取得のコーナー入口状態を狙う追加収集

完了結果: 2環境で12 runを実施し、10 runが完走。独立監査に合格した
19復帰イベント・2,432画像anchorを教師化した。trainは1,032、validationは1,400。
原計画の入口条件の不足は41から28へ減少し、13条件を解消した。
全条件の取得完了ではない。詳細と未取得理由は後半に記載する。

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

最初の試走では、C01の20cmは準備heading補正+1度が強く、横ずれと向きの到達が
揃わなかった。C01の60cmは準備中に実行時停止領域が占有となり停止した。
静的地図合格を実走成功と混同せず、heading補正を小さくする。

追加の `failed_site_policy=continue_after_target_miss_v1` は、入口目標を作れなかった
`TARGET_NOT_REACHED` に限り、そのイベントを不採用のまま次の地点へ進める。
通常PPへ戻して15秒と将来3秒分を空け、次地点の現在のクリアランスと1秒連続安定を
再び必要とする。横ずれ・向き限界、センサ、時計、計算締切、停止領域の異常は対象外。
旧記録の既定は後続イベントを行わない条件のまま。
独立監査は、イベントの通し番号と成功イベント数を分けて完了判定する。

第3pairのC03では、開始位置の直前まで連続安定していたが、開始境界で現在の
クリアランスが0.4mを下回り、外乱を実行できなかった。明示する
`entry_window_lead_m=0.5` は、既に地図検査する開始前1mの範囲内で、受付だけを
最大0.5m早める。準備CSV、目標位置・横ずれ・向き、現在クリアランス0.4m、
連続安定1秒、実行時間上限、前イベントの待機時間は変更しない。既定は0m。
C06は開始窓で安定1秒に届かなかったため、地図検査済みの短いapproachを使い、
安定を確認できた後方の開始位置を狙う。

## 最終結果

| 要求横ずれ | trainイベント / anchor | validationイベント / anchor | 合計anchor |
|---|---:|---:|---:|
| 20cm | 2 / 272 | 3 / 392 | 664 |
| 40cm | 3 / 384 | 6 / 744 | 1,128 |
| 60cm | 3 / 376 | 2 / 264 | 640 |
| 合計 | 8 / 1,032 | 11 / 1,400 | 2,432 |

採用したrunはtrain 5本・validation 4本で、run単位の分割は交差しない。
anchorは有効な画像・LiDAR・ego履歴と実測将来3秒を持つ観測時点。
同じイベント内の連続画像を、独立した復帰試行数には数えない。
全教師の形状は`[N,30,2]`、単位m、将来30点のmaskは全て有効。
全体で20イベントの復帰完了を観測したが、途中停止したrunの1イベントは不採用。

raw計15,101,579,084 bytes（約15.10GB）をnative WSLへ移送した。
全12 runでファイル・ディレクトリ構造・SHA-256・SQLite quick_checkを確認済み。
採用イベントの制御publish間隔の最大は約90ms。これは推論時間の測定ではない。
各pairで2台の同時進行をホスト記録から確認でき、正常に完走したpairの同時進行観測は
約225秒。ROS_DOMAIN_ID 1/2と個別network namespaceを使用した。
終了時の実行機空きは約14.44GiB。稼働containerと本campaignの実行supervisorは0。
元のrepository状態とAWSIM本体1,089ファイルの一致を再確認した。

収集用上限超過を理由とする打切りは0。rawの最大実測速度は約5.177km/hで、
旧上限1.4m/sを超える有効anchorも2件採用した。
一方、既存の収集用**下限1.15m/s**は維持されており、C06の1試行に影響した。
物理監視の6km/h適用範囲、停止領域、NaN、センサ・時計の監視も維持している。

## 入口条件の取得状況

1条件は「振幅×コーナー×train/validation」。同じ確認済みイベントに有効anchorが
60枚以上あり、そのうち3枚以上が元の入口位置区間・横ずれ±5cm・向き±1度に
入ることを要求する。復帰イベント全体の採用と、この入口条件の完了は別に数える。
既存のsealed収集分も含めて、条件を変えず再計算した。

| 振幅 | 今回解消したtrain条件 | 今回解消したvalidation条件 |
|---|---|---|
| 20cm | C01, C03 | C03, C04, C07 |
| 40cm | C03, C04 | C03, C04, C11 |
| 60cm | C11 | C08, C11 |

| 振幅 | 残るtrain条件 | 残るvalidation条件 |
|---|---|---|
| 20cm | C06, C07 | C06 |
| 40cm | C02, C05, C06, C07 | C02, C05, C06, C07 |
| 60cm | C01, C02, C03, C04, C05, C06, C07, C08, C09, C10 | C01, C02, C03, C04, C05, C06, C07 |

不足リストには、復帰データは採用できても入口画像が1〜2枚に留まる条件も含む。
例えば40cm C02は復帰直後の一部画像で`CURRENT_SENSOR_MISSING`となった。
150msの切替後余裕や、現在入力の有効性判定を緩めて枚数を増やしてはいない。
有限12 run内で未試行の条件、開始条件が揃わなかった条件も残る。

## 失敗と未解決事項

- Pair01 validation: C01 60cm準備中の`STOPPING_SWEEP_OCCUPIED`で停止。
  このrunは不採用。同じ設定で繰り返さなかった。
- C06 40/60cm: 元の目標周辺で検査した各36姿勢は、標準車体の地図検査に不適合。
  検査した姿勢の結果であり、連続する全姿勢に対する不可能性の証明ではない。
- C06 20cm: 最初は開始窓内の連続安定が不足。開始を後ろへ移したmeasured-normal案では
  横ずれ20cm時の向きが約7.6度となり、4度目標と一致しなかった。
  その後1.1494598m/sとなり下限1.15m/sで外乱を終了、通常PPへ戻して完走した。
  後続C09の外乱は実施されなかった。最後のnominal-path案は下限で終了せず継続できたが、
  横ずれが最大約14cmで目標未到達。C06イベントはどちらも教師に採用しなかった。
- C03 60cm: 開始余裕を確保する長い準備経路の追加72案は静的地図検査に不適合。
  短い準備経路の開始区間では、通常走行の現在クリアランスが0.4m未満となる記録がある。
- Pair05 validation: C06より前の通常走行、基準進行186.77mで`SWEEP_VEHICLE_STATE`。
  bag内の操舵角を確認すると、capture `187739995803ns`の1サンプルがNaNだった。
  前後約35msの操舵角は有限。停止確認後にrawを保全し、run全体を教師から除外した。
- C07/C11の目標未到達には、横ずれが揃う時点で向きが5度付近の境界を跨ぐ事例がある。
  準備headingを個別に調整し、最後のC07 40cmとC11 60cmでは復帰を確認した。

C06は到達可能な位置・姿勢と準備経路の再検討が必要。
収集用の速度下限と物理監視の役割も、次回の改善項目として残る。
今回の失敗rawを別の復帰条件として利用するには、独立した教師監査が必要となる。

## 保存先・検証・実行記録

native WSLのrootは`/home/thistle/e2e_autonomous`。

- raw: `raw/time_corner_gap_20260916/`
- index・診断・教師: `runs/time_corner_gap_20260916/`
- 実測将来教師: 同ディレクトリの`materialized/<run_id>/teachers.npz`
- causal入力cache: 同ディレクトリの`prepared/train/`と`prepared/validation/`
- 軽量証跡: `docs/evidence/time_corner_gap_20260916/`

`collection_index.json`に採用数・分割・元run・教師hashを保存した。
`coverage_final.json`は既存データを含む入口条件比較、`parallel_observation.json`は同時走行の記録。
`fault_diagnosis_pair*.json`と`pair*_entry_diagnosis.json`に未到達・NaNの根拠を保存した。
初期計画と未実施の調整案も保全し、実際に使った条件は各runのreference hashで識別する。

Windowsでコミットした実行sourceは`cfd9366`、`62dbe8b`、最終`d591c8f`。
最終sourceのWSLテストはfocused 68件、全体`2,752 passed, 4 skipped`。
教師生成と入力検証もnative WSLの同一worktree lock内で実施した。
走行したのは教師PP。学習用データへの統合、再学習、E2Eモデルの走行再評価は今回未実施。

以下は実行済みコマンドの例。campaignはsealedなので、再収集では新しいrootと有限予算を用意する。
保存したoperatorsを使う場合は、証跡READMEに示した元のtmp配置と依存transportを復元する。

```powershell
python tmp/time_corner_gap_20260916/run_native_script.py adaptive_plan_v3.py --pair 6 --train 40:C03 20:C06 60:C11 --validation 40:C04 40:C07 60:C11 --heading-bias-deg 0.3 --label revised --overrides entry_calibration_after05.json
python tmp/time_corner_gap_20260916/collect_pair.py --pair 6 --left train_p06_bias03_revised --right validation_p06_bias03_revised
python tmp/time_corner_gap_20260916/audit_pair.py --pair 6
python tmp/time_corner_gap_20260916/run_native_script.py finalize_native.py
python tmp/time_corner_gap_20260916/finish_remote.py
```
