# 2 m継続試験と20 m候補：準備実装

実行版 `f48f5c861f4c4e1225a601013f016299037087e5`。
Windows正本branch codex/windows-wsl-training-sync。既存2 mコードの既定設定・重みは保全。
本報告は両モデルの走行・学習完了報告ではない。

## 2 m側

`shadow_resample_v4.py` とAdapter/接続に任意の `shadow_spacing_m` を追加。
値は0.2/0.3 m、既定None。fixture_modeを持つshadowのみで有効。
ROS configに指定すれば使用できるが、今回既定configは変更していない。

元20点は保持。NaN・重複・大きな点間不連続は変換前に拒否。
変換後の形状検査は既存のままで、さらに元折線との双方向距離の上界3 cmを要求。
2 mmサンプリングの誤差上界1 mmと、二段目再サンプリングによる差も加算する。
3 cmはshadow比較用の明示方針であり、障害物clearanceや安全許容値ではない。

保存済み168件を時系列順にAdapterへ投入し、幾何・時刻・plan jumpまで検査:

|間隔|参照準備|変換量超過|曲率超過|折返し|plan jump|
|---|---:|---:|---:|---:|---:|
|既定raw|0|0|121|47|0|
|0.2 m|78|48|42|0|0|
|0.3 m|96|48|16|0|8|

nowは各source stamp+0.01秒のoffline比較。車両現在状態・commandは未評価。
元の108件という0.3 m比較と異なり、変換量上界と時系列plan jumpも判定している。
新規AWSIM走行・新規実モデル推論は未実施。

## 20 m側

既存h30監査台帳SHA256:
`35781616e8faab5117b0d9da7c8560519c5f196383ea66ab1465d999f5645e35`。
暫定ノイズ除去済みprefix長を集計。数値は教師適格数でなく距離支持の候補数。

|通常走行|既知長の件数|5 m支持|10 m支持|20 m支持|
|---|---:|---:|---:|---:|
|train|27,829|17,918|12,698|5,862|
|val|8,014|5,263|3,846|1,883|

train通常の未検査/長さ不明は3,108件、valは881件。UNKNOWNを0 mと扱わない。
testは全13,866件が長さ未検査で、支持数0を「支持なし」と解釈しない。
復帰はtrain最大1.327 m/val最大1.330 m、5/10/20 m支持なし。
件数には隣接観測が含まれ、独立シーン数ではない。
台帳のteacher eligibilityはunknownのまま。距離があるだけで採用しない。

追加した候補:

- `spatial_long_view_v4.py`: h30連続prefixから46点教師/maskを生成する純粋関数。
  0.1〜2 mは0.1 m刻み、2.5〜10 mは0.5 m刻み、11〜20 mは1 m刻み。
  未観測距離はmask false。停止・復帰から20 mまで外挿しない。
- `spatial_path_long_v4.py`: 入力backboneを再利用した別クラス、出力[B,46,2]。
  元の固定[B,20,2]モデルは変更しない。**候補は未学習・実データ未評価**。
- 近距離・遠距離の点密度が異なるため、損失の距離帯バランスは学習設定で別途確定が必要。

実データの教師生成、適格性確認、学習コードの46点/mask対応、学習・評価・checkpoint保存は未完了。
既存学習コードは固定20点・固定診断実験用なので、そのまま20 m版として実行しない。

## 検証・再現

Windows commit → 既定CheckOnly/同期 → WSL lock検証。
同期によるDatasetルート存在確認あり。今回の監査は保存済み台帳を読み、
Dataset内容/sensor/raw/checkpointは読んでいない。
限定35 passed、全体1804 passed/4 skipped/52 warnings（70.70秒）。
モデルshape試験は人工入力・ランダム初期化で、学習済みモデル性能の証明ではない。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python tools/audit_spatial_long_horizon.py --ledger /home/thistle/e2e_autonomous/runs/spatial_v4_coverage_full_v2_20260905/anchor_audit_ledger.csv --output /home/thistle/e2e_autonomous/runs/v4_dual_horizon_20260910/coverage.json
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python /mnt/e/workspace/e2e_lite_transfuser/tmp/compare_bounded_shadow.py
```

audit出力先は既存の場合上書きせず失敗するため、再試行には新規出力先を使用。
coverage.jsonのWindowsコピーは `tmp/v4_dual_horizon_20260910/coverage.json`。
比較script初回はPYTHONPATH不足でimport失敗。上記明示コマンドで再実行成功。

次は2 m版の変換参照を実入力shadowで検証し、20 m版はtrainの実futureに対して
prefix品質・長距離再サンプリング差を確認してから教師生成と有限診断学習へ進む。
