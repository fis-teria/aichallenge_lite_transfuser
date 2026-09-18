# 目標10km/hの教師収集と選別

**教師MPPI V45・追加のearly-entry探索OFFで6走行を収集し、104窓を採用した。104窓すべてで保存センサから履歴・教師の再現を確認した。再学習はまだ実施していない。**

採用内訳はコーン周辺の観測軌道・速度74窓、通常走行30窓。窓は相互に重複する1秒履歴＋3秒将来であり、104回の独立した回避イベントではない。独立した収録は6本、配置グループは2種類。

## 10km/hでの実走比較

PC10 `graneple@192.168.3.10` で、5km/h試験と同じnative cone配置・同じ開始位置を使用。目標速度を10km/h（2.7778m/s）へ変更し、MPPIによる減速・停止判断は維持した。

| 開始条件 | early-entry OFF | early-entry ON |
|---|---|---|
| 近接開始 | passed、記録最高9.508km/h | passed、記録最高9.509km/h |
| 通常接近 | passed、記録最高9.513km/h | passed、記録最高9.526km/h |

いずれも障害物通過後約25mまでの有限試験。全周完走やE2Eモデルの成功を意味しない。4試行すべて公式crash/wall/overは0。各条件1本の結果であり、10km/hなら常に回避できるとは主張しない。近接OFFでも通過したため、収集は目標10km/h・early-entry OFFに統一した。ONの比較2本は今回の学習候補に混ぜていない。

速度条件で成否が変わったことは確認できたが、5km/hで棄却された原因の内訳（操舵応答・経路形状・物体不確実性等）は未確定。

## 収集母集団と採用数

すべてnative cone 1個、目標10km/h、early-entry OFF、通常RVizでMPPI経路を表示。全6本がscenario `passed`、公式crash/wall/overは0。Aは以前のE2E失敗位置、BはMPPI基準線上の別位置。

| run IDの末尾（共通接頭辞 `lidar-v45-pc10-front-`） | 条件 | 時間条件を満たす候補 | 採用 | 保留 | 除外 | 間引き |
|---|---|---:|---:|---:|---:|---:|
| close10-control-a01 | B・6m手前開始 | 104 | 2 | 100 | 1 | 1 |
| collect10-b8-a01 | B・8m手前開始 | 112 | 16 | 82 | 0 | 14 |
| normal10-control-a01 | B・通常接近 | 261 | 30 | 203 | 0 | 28 |
| collect10-a6-a01 | A・6m手前開始 | 82 | 14 | 54 | 0 | 14 |
| collect10-a8-a01 | A・8m手前開始 | 86 | 17 | 52 | 0 | 17 |
| collect10-an-a01 | A・通常接近 | 125 | 25 | 77 | 0 | 23 |
| 合計 | 6走行 | 770 | **104** | **568** | **1** | **97** |

元のカメラ観測アンカーは1,544件。770件は起動・終端の時間支持、1秒履歴、3秒将来、姿勢監査による切り出し後の候補。品質条件に通った201件から0.2秒以上の間隔で104件を採用し、97件は重複を減らすためだけに間引いた。

6m/8mはコーンをMPPI基準線へ投影した点からの経路上距離で、実測車体座標の前方距離ではない。AのコーンはMPPI基準線から横に約2.75m離れているため、Aの近接開始を真正面の近接条件とは扱わない。

## 選別基準と保留理由

- 時刻欠測、教師の欠損・非有限値、後退、停止・極低速、異常な教師指令を確認。将来速度は全点0.2m/sより大きいことを要求。
- IMUと推定姿勢の相対的な向きの不整合が継続する時刻より前までを使用。B通常接近では末尾の`HEADING_EVIDENCE_GAP`により41.55秒以前へ切り詰め、それをまたぐ将来窓を採用していない。他5本はこの相対監査で`NO_DRIFT_DETECTED`。絶対位置が正しいことの証明ではない。
- 壁とLiDARの整合、コーンの投影距離、観測点と車体の距離を既存条件のまま確認。0.30mの余裕と追加投影マージン0.30mを維持し、採用数を増やすための閾値緩和は行っていない。
- 保留568窓のうち547窓に`SCAN_MAP_ALIGNMENT_OR_SUPPORT`がある。内訳は照合距離中央値が0.15m超のもの539窓、0.25m以内の一致点率が70%未満のもの419窓（重複あり）。**点数80未満・角度範囲不足は0窓**。今回の主因は点数不足ではなく照合の不一致。ただし、これだけでオドメトリ単独の不具合とは断定しない。
- 他にコーン周辺条件がないのに非通常モードの窓86件、監査姿勢の時間支持が不明な窓3件が保留理由に含まれる（重複あり）。除外1窓は不完全または非有限の教師。

**前方0～6mの窓は47件あるが、そのうちコーン横位置の絶対値が1m以下の窓は0件。** 従って、今回の104窓で「近接正面の不足」を埋めたとは言えない。位置は記録EKF姿勢と静止コーン配置から求めた診断値で、モデルへの追加入力ではない。

採用は観測された軌道XYと速度の品質選別。停止意図・行動モードの教師は無効マスクのまま。物理的な全周囲の0.30m余裕や3D接触の保証ではなく、元の厳格な回避成功マスクを上書きしていない。

## 保存先・再現

WSL root: `/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918`

- `collected/<run ID>/`: 元rosbagとシナリオ・実行来歴。6本合計508,180,516 bytes、2,448ファイルを転送時SHA256で照合。
- `collect10_audit_v1/`, `collect10_pose_prefix_v1/`, `collect10_prefix_clearance_v1/`: 時間・姿勢・物体・壁の監査。
- **`collect10_curated_v1/selection_manifest.json`**: 採否、理由、ファイルハッシュ。各runに`selected_anchors.jsonl`と`selected_teachers.npz`。
- **`collect10_validation_v1/*.pt`**: 採用104窓を保存Camera/LiDAR/ego履歴から再構成した6つのTimeSample shard。教師XY/速度と履歴を全件照合。`summary.json`に検証結果と各shardのSHA256。
- `collect10_selection_report.json`, `collect10_hold_breakdown.json`: 集計。

既存の2配置グループ `failed_e2e_cone_location` / `straight_b_center` のtrain区分を維持。新しい検証・testセットをframe単位で作っていない。旧データ・旧モデルは維持し、今回の104窓は既存学習にまだ組み込んでいない。

実行コマンド（出力は新規専用。既存の結果を上書きする再実行はしない）:

```bash
# PC10: 10km/h, early-entry OFF
python3 /home/graneple/e2e_autonomous/mppi_close_adjust_20260918/source/tools/collect_mppi_v45.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_close_adjust_20260918/runtime \
  --scenario /absolute/path/to/scenario.yaml --run-id lidar-v45-pc10-new-run \
  --speed-cap-kmh 10 --wall-timeout-s 480 --run-budget-gib 0.75 --free-reserve-gib 2 --rviz --execute

# WSL worktree: /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u \
  /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/collect10_operators/audit_one.py RUN_ID
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src bash \
  /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/collect10_operators/finish_selection.bash
```

評価ソースはWindowsでcommit済みの`dfe2cfa89417d84c87343522f3694bee4246cd3d`をWSLへ同期。今回production codeの変更なし。前回検証済みruntimeをハッシュ確認して再使用し、今回の確認は実走とデータ全件再現を実施した。AWSIM本体・Unity資産・既存起動ファイルはハッシュ一致、専用コンテナは終了済み。大きな原本・教師配列・shardはGitへ追加していない。

[選別集計](evidence/mppi_teacher10_selection_20260918/collect10_selection_report.json)、[保留内訳](evidence/mppi_teacher10_selection_20260918/collect10_hold_breakdown.json)、[全件再現](evidence/mppi_teacher10_selection_20260918/replay_summary.json)、[10km/h比較](evidence/mppi_teacher10_selection_20260918/close10_comparison.json)。
