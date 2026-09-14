# 問題のコーナーに限定した復帰教師の収集

2026-09-14: ユーザーの収集指示に基づく小規模pilot。実行先は
`graneple@192.168.3.10`、参照生成・データ検証はnative WSL。

## 目的と事前条件

`codex-time-segment01` の停止監視地点はmap
`(89649.00400737983, 43143.3497366721)`、yaw `1.5675418939256511 rad`。
固定基準CSVへの同方向投影は `s=93.57092578152393 m`。
この付近で元の走行ラインへ戻る、実測camera/LiDAR/egoと未来3秒の教師を採る。
参照生成器に元コースのapproach開始距離の範囲指定を追加し、範囲外への自動代替を禁止する。
未指定の従来の選択、曲率・地図clearance検査、収集runtimeを維持する。

最初は左右20cm、各1周。最大4試行（主要2、原因が判明した確認用2）、
1試行2GiB・30分sim/31分wall/33分outer、合計8GiB、最低空き10GiB。
2本ごとに停止・bag close後にWSLへ転送し、全ファイルを照合する。
同じ物理失敗を条件変更なしで繰り返さない。既存run、台帳、source、参照を保全する。

従来どおり目標5km/h、自車1台、公式PPと外部監視を使う。
通常RVizで基準・収集参照・実走のPathを表示する。終了は1周+4秒未来、正常制動・停止確認。
時刻、センサ、単一command発行元、操舵・停止領域の監視は維持する。
最初から1mの横ずれや独立のyaw介入は与えず、まず実測の横ずれ・向きずれを評価する。

## 教師の採否

寄せるapproachとholdは学習anchorに採用しない。実測の復帰だけを候補とし、
過去入力履歴は保持する。CSV座標を未来教師として代用しない。
以前の固定CSVからの判定（holdが指定量の半分以上、復帰後5mの最大誤差10cm以下、
誤差低減5cm以上）を併記し、不成立を成功扱いしない。
この角では固定CSVと通常の実走教師自体にも差があり得るため、既存の同ホスト通常走行との
位置・向きの比較も必要。採否基準の事後的な緩和で主教師に昇格させない。
最終採用には元センサからの入力再現と全30点の未来pose確認が必要。
独立試行数と重複するanchor数を分け、run単位splitを維持する。学習は本収集の範囲外。

## 再現コマンド

Windowsでcommitし `tools/sync_to_wsl.ps1 -CheckOnly`、同スクリプトの通常sync後、
WSL checkout内で実行する。

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/generate_time_recovery_collection.py \
  --inputs /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs \
  --output /home/thistle/e2e_autonomous/runs/time_corner_recovery_20260914/references_corner020 \
  --offset-m 0.2 --geometry left_curve \
  --maximum-base-curvature-inv-m 0.25 --preferred-base-curvature-inv-m 0.15 \
  --base-start-range-m 76 82
```

上の範囲は事前候補。実際の採用区間・地図適合・通常教師との対応を確認してから走行する。
最大基準曲率0.25/mは収集場所の選択条件で、車体や安全監視の制限変更ではない。
生成物、実行コマンド、取得結果、未成立条件は検証後に追記する。

## 走行前に確定した局所条件

生成source `99d79d90f417b56f0b7a629f70a54ed9c2b27c11` のWSL pytestは
**2286 passed, 4 skipped, 64 warnings (93.98s)**。
左右ともapproach開始 `79.88994279228969 m`、hold `83.88994279228969〜89.88994279228969 m`、
recovery `89.88994279228969〜95.88994279228969 m`。停止地点s=93.57mを復帰区間に含む。
接続部分を含む0.1m刻みの地図center clearance 1.4m検査は両側通過。
実車体全体の無接触や走行成功を保証する検査ではない。

同ホストの既存r22/r23の当該区間は両方baseline。固定CSVに対してhold相当位置の
実走中央値は右27.6〜27.9cm、recovery後5mの最大誤差は19.1〜19.2cm。
このため旧「CSVから復帰後10cm以内」は通常走行自体が不成立であり、結果に必ず併記する。
**新しい走行を見る前に**、今回の局所的復帰確認はr23の実測baseline線を基準とし、
r22でも整合を確認する。同じ元コース進捗で対応づけて、指定側へのhold中央値10cm以上、
復帰後5mの差の最大値10cm以下、holdから5cm以上の誤差低減を条件とする。
これは実測教師への復帰のpilot判定であり、旧CSV判定の通過や無接触証明とは区別する。
曲線上の元コース投影差による比較と、実測線への直接投影を区別して報告する。

起動helperは [start_pilot.py](evidence/time_corner_recovery_20260914/start_pilot.py)。
主要2試行のみを許可し、追加診断2枠は自動使用しない。
専用runtimeのsrc/tools/configs 400ファイルが既存の実走commit `0484dc3` と一致した。
実行時にも全件と基準CSV/地図/C++入力のhashを再確認する。

## 最初の試行と限定した環境調整

`cornerleft020-r27` はs=17.35m、復帰場所に到着する前に
`COLLECTION_COMPUTATION_TIMEOUT`。最初のscanが古く再選択に入り、
判断全体108.01msで100ms期限を超過。最後の計算はwall28.24ms/thread CPU19.79ms、
GC pauseなし。正常停止の3秒確認は成立せず、監督がAWSIMを停止した。bag closeは成立。
この記録は復帰教師ではない。

実行ホストはP/E混在20 logical CPUs。他のdesktop処理にも負荷があるため、
次の1回は専用nodes containerをP coresのlogical CPU2-5へ、今回所有するAWSIM/Autowareを
CPU0-1,6-19へ割り当てる。全containerの設定を公式Start前に確認する。
他のアプリの停止やCPU設定変更は行わない。これはCPU競合を減らす試験であり、
スケジューリングが全遅延の原因と確定したわけではない。
入力期限・100ms判断期限・retry回数・速度・操舵・安全監視は変更しない。
追加診断枠の1本を`cornerleft020-r29`に使用し、同じ失敗が再発したらそこで中断する。
成功時の右`cornerright020-r28`も同じCPU割当を使用する。総試行上限は3本/6GiB。
