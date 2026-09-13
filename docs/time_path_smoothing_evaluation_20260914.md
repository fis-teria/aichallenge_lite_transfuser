# 保存した時間軌道の平滑化評価

ユーザー依頼: 平滑化前後の操舵余裕と元の経路からのずれを評価する。
Windows正本で実装し、同期後のnative WSLで保存済みデータを評価する。
実行前の比較条件をこの文書とコードで固定し、WSLで全条件の評価を完了した。
今回の5種類の平滑化では候補の発進時PP成立はすべて0/103件。走行用への適用は行っていない。

続く[先読み条件の整合性調査](time_lookahead_alignment_audit_20260914.md)では、
形状を変えずに元の線分内を調べると103/103件でPP成立位置が見つかった。
離散点選択による取りこぼしと速度・操舵余裕の課題を切り分けた結果で、走行用の修正は未適用。

## 固定比較条件

- raw、mean3、mean5、d2_lambda1、d2_lambda10、d2_lambda100の6条件。
- mean3/5: 予測時刻方向の対称移動平均。端付近は対称に窓を縮める。
- d2: `sum ||q-p||² + lambda * sum ||D2(q)||²` の制約付き最小二乗。
  lambdaは0.1秒間隔の点列に対する無次元重み。物理的な加速度上限ではない。
- 既知の観測原点(0,0)と3秒先の終点を固定し、30点の0.1秒時刻対応を保持する。
  したがって終点誤差は不変。途中の教師誤差、点列の二階差分、同時刻の位置変化を評価する。
- 平滑化は各予測の内部だけで行い、他時刻のセンサ、他予測、教師を使わない。
  曲率・先読み点の採用上限・安全監視の設定は変更しない。

## 評価母集団と範囲

1. 保存AWSIM旧モデル `codex-time-recovery-model-base01` と候補 `codex-time-recovery-model-candidate01`。
   元の制御計算を既存replayで再確認し、その同じpose/速度で平滑化した予測をPPへ通す。
   全記録可能区間と、走行許可後の最初の5.2秒を分ける。pose等が記録されていない指令は分母を明示する。
2. 各試験で制御に参照されたunique予測について、全30点の同時刻位置変化と暗黙の速度変化を測る。
   「位置変化」は教師誤差・車線偏差・障害物余裕ではない。
3. 既存の候補epoch3 validation予測12,346件と元の6run教師を比較する。
   input欠損・教師maskを維持し、cache/order/hashと元のADE・件数の再現を確認する。test splitは使用しない。

同じ予測の平滑化前後は対応する比較だが、新旧モデル同士は別AWSIM試験である。
PP成立は制御計算だけの結果。変えた指令による車両状態の推移、操舵応答、scan衝突判定は再現しない。
結果から発進・完走・安全な走行の成功を主張しない。比較条件の事後調整も行わない。

## 実行コマンド

Windowsでcommit後、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`。
native WSL `/home/thistle/e2e_autonomous/e2e_lite_transfuser` で以下を実行する。
出力directoryは新規のみ。既存出力は上書きしない。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python tools/evaluate_time_smoothing.py \
  --records ../runs/time_recovery_finetune_evidence_20260914 \
  --cache ../datasets/cache/time_recovery_20260914 \
  --training ../runs/time_recovery_finetune_20260914 \
  --output ../runs/time_path_smoothing_evaluation_20260914
```

## 実行状態

完了。source commit `8044aea4b5a655c9986bfd35d9dc6182c823e980`。
WSL全体pytestは **2,239 passed / 4 skipped / 64 warnings**（76.35秒）。
skipは以前と同じOSQP、jsonschema関連2件、任意の公式LiDAR package。
平滑化の停止/定速直線・始点終点保持・非破壊性・既知の振動抑制・不正入力・PP操舵上限をテストした。
評価コマンドは16.40秒でexit0。元の旧モデル1,958件・候補103件の制御replayがPASS。
runtime/controller/ROS設定やモデル重みは変更していない。AWSIMの再起動・走行試験は実施していない。

## 評価結果

以下のPP成立は、記録されたpose/速度における純粋な制御計算の成立件数である。
1件を独立した走行試験として数えない。候補は記録全区間が発進待機であり、全103件が拒否のまま。

| 条件 | 候補の発進時PP成立 | 候補の元予測からの最大変化 | 旧モデル発進時PP成立 | 旧モデル全区間PP成立 | 候補validation全点ADE |
| --- | ---: | ---: | ---: | ---: | ---: |
| raw | 0/103 | 0cm | 104/104 | 1,956/1,958 | 1.6225cm |
| mean3（3点平均） | 0/103 | 0.82cm | 104/104 | 1,956/1,958 | 1.6112cm |
| mean5（5点平均） | 0/103 | 1.20cm | 104/104 | 1,956/1,958 | 1.6094cm |
| d2_lambda1（弱） | 0/103 | 0.84cm | 104/104 | 1,956/1,958 | 1.6104cm |
| d2_lambda10（中） | 0/103 | 1.69cm | 104/104 | 1,956/1,958 | 1.5959cm |
| d2_lambda100（強） | 0/103 | 4.88cm | 103/104 | 1,957/1,958 | 1.6220cm |

最大変化は各予測の同時刻のXY差の最大ノルム。候補の制御に使用されたunique予測49個が母集団。
観測時刻が走行許可以前でも、発進直後の制御に使われた予測は含めた。
旧モデルはunique予測873個、全2016指令のうちpose等を持つ1958件を比較し、58件は記録不足として除外。
旧モデルの強い平滑化は元のgeometry拒否2件を解消する一方、元は通っていた発進指令1件を新たに拒否した。
したがって合計成立数の増加だけを改善とは判定しない。

### 操舵余裕

`余裕 = 0.3rad - 先読み距離内の候補点の最小必要操舵角の絶対値`。
正なら角度条件内、負なら超過。geometryや衝突の成立を保証する量ではない。

| 条件 | 候補103件の操舵余裕（min〜max） |
| --- | ---: |
| raw | -0.001672〜-0.000530rad |
| mean3 | -0.002022〜-0.000880rad |
| mean5 | -0.002723〜-0.001580rad |
| d2_lambda1 | -0.001664〜-0.000523rad |
| d2_lambda10 | -0.001902〜-0.000762rad |
| d2_lambda100 | -0.004163〜-0.003043rad |

弱いd2で余裕がわずかに改善しても、全件が負のまま。
候補の点列二階差分RMSの予測平均はraw 0.645m/s²から強いd2で0.329m/s²に減ったが、PP成立には結びつかない。
3点/5点平均や強いd2では、先読み距離内の操舵余裕が悪化した。
この結果は「見た目や点列を滑らかにするだけでは今回の発進条件を満たせない」ことを示す。
あらゆる平滑化や制約付き最適化が無効だという評価ではない。

### 教師との誤差

validationは6run/12,346アンカー、入力が有効な予測12,301件。
教師も有効な11,889アンカーの356,670点で、全点ADEを比較した。
入力が無効な45件、教師の支持がない区間は元のmaskのまま扱い、分母を変えていない。
6runのlabels/inputs/anchor台帳18ファイルを元manifestと照合し、checkpointのcache/order identityも一致した。
raw ADEと支持点数を保存済み評価と再現した。

最小ADEは中程度d2の1.5959cmで、rawの1.6225cmから約0.266mm（1.64%）減少。
通常run等重みADEは1.8426→1.8152cm、復帰run等重みADEは1.3879→1.3836cm。
これは保存validationの観測値であり、別コースでの効果や統計的な優位性を証明したものではない。
終点を固定したため、3秒先誤差は全条件で同一。平滑化による3秒先精度改善として数えない。

## 判断と残る課題

今回の平滑化単独では発進を回復できず、走行用には採用しない。
特に強い平滑化には最大4.88cm（候補）/6.07cm（旧モデル）の変更と新たなPP拒否があり、無条件で追加しない。
次の優先課題は、先読み点の選び方と予測経路の曲がり方を、操舵条件まで含めて整合させることである。
その検証では元の予測からの許容変化、時刻対応、障害物余裕も評価する必要がある。
今回未評価なのは補正後のscan衝突判定、操舵応答、車両状態の閉ループ推移、複数予測間の時間平滑化。

## 保存先

- native WSL: `/home/thistle/e2e_autonomous/runs/time_path_smoothing_evaluation_20260914`。
  全指令の平滑化前後比較は同directoryの`*_commands.jsonl`。予測原本・重みは保全。
- テストと実行ログ: `/home/thistle/e2e_autonomous/runs/time_path_smoothing_evidence_20260914`。
- Windowsには小さい結果・図・ログ12ファイル345,411bytesをコピーし、SHA-256一致を確認。
  [結果JSON](evidence/time_path_smoothing_20260914/summary.json)、
  [比較図](evidence/time_path_smoothing_20260914/comparison.png)、
  [実行記録](evidence/time_path_smoothing_20260914/execution.json)、
  [転送照合](evidence/time_path_smoothing_20260914/transfer_verification.json)。
