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
