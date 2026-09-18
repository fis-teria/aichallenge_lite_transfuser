# 2026-09-19 回避教師の追加収集と地図照合方針

## 許容するズレ

ユーザー指示「そのズレは許容します」に従い、今回の追加選別では
LiDAR点群と記録EKF姿勢で投影した物理壁地図の不一致を診断値として残し、
距離中央値15cm・25cm以内一致率70%の二条件だけを採否から外す。
`--allow-scan-map-misalignment`を明示した新しい選別束を作成する。
既存の選別束・原本・教師座標・既定の厳格選別は変更しない。

NaN、時刻の欠損、観測点数・角度支持、姿勢prefix、教師の不成立、停止・後退、
観測された近接や壁重なり等の他条件は残る。物理接触ゼロや地図との一致の証明ではない。
この許容は地図上の絶対整合についてであり、モデルの入力と未来軌道を別々にずらす操作はしない。

```bash
cd /home/thistle/e2e_autonomous/avoidance_collection_validation_20260919
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u \
  tools/curate_native_teacher_data.py \
  --root /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918 \
  --output /home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/expand_alignment_tolerant_v1 \
  --run-pattern 'lidar-v45-pc10-expand-*' \
  --prefix-directory expand_pose_prefix_v1 \
  --clearance-directory expand_prefix_clearance_v1 \
  --allow-scan-map-misalignment
```

出力は新規ディレクトリのみ。configと`scan_map_alignment_policy`、窓ごとの
実測残差、原本・教師・選別のSHA256を記録する。検証用配置をtrainへ移さない。

## 容量確保と2並列

- PC10の終了済み11実験、14,758項目、5,302,786,149ファイルbytesをWSLへ保管。
  移動先で全ファイルSHA256を確認し、削除直前に元のファイルも再照合した。
- 保存先: `/home/thistle/e2e_autonomous/runs/pc10_past_backup_20260919_r2`。
  symlinkはリンク先文字列をmanifestに保全し、過去PCのリンクを有効化していない。
- PC10空きは削除直後7,682,428,928 bytes。別に、以前の6収集原本の重複も
  WSL照合済みで削除した。WSLの原本・checkpointは削除していない。
- PC10でROS_DOMAIN_ID 1/2に加え、互いに独立したDocker network namespaceを使用。
  両者を意図的に同じdomain 0とした公式ROS2送受信試験でも他方のtopicが混入しなかった。
- 最大2AWSIM、各実験480秒、0.75GiB、空き下限2GiB。目標10km/h、通常RVizあり。
- AWSIMバイナリ・assetsは改変しない。生成した外部起動設定で既存CLIのbase domainを指定。

実行済みは左右開始コーン4本（2配置）、未学習配置コーン／箱2本、単独環境対照1本。
7本ともシナリオの終了条件に到達し、公式crash/wall/overは0。
全原本はWSL `runs/mppi_v45_pc10_20260918/collected/` にSHA256照合して保存。
三つの並列pairでAWSIM 1,089ファイルの前後ハッシュ一致を確認した。
全収集後にも最初のpair前のハッシュと一致を確認し、専用holder 2個を削除した。
収集終了時の空きは7,079,858,176 bytes（約6.59GiB）。
終了条件到達は、すべてのrunで障害物通過後25mの観測が検証できた意味ではない。

## 厳格選別時の結果

train 3runのカメラanchorは634、prefix候補181窓。地図照合を必須にした
`expand_curated_v2`は全181窓保留。並列左右2本は81窓、単独対照も100窓が
地図照合で保留となったため、2並列負荷だけが原因とは説明できない。
右開始には教師不成立の理由がある14窓も含まれる。

左右2本の各窓の最悪スキャン中央値は約18～48cmと19～43cm。
点数201以上、角度支持約179度で、点数不足ではない。
0.5秒間引きの診断80スキャン中56が不一致で、54は位置・向きを最適化すると
既存閾値内に入る。ただし地図への後付けfitは自己位置の真値ではなく、
教師を修正する根拠にはしない。元の閾値は経験的な品質条件であり、
この不一致だけで車体相対の未来軌道が壊れているとは断定しない。

## 学習比較との分離

既に再現確認済みの338+104+259=701教師窓を用いた4条件比較は、WSLの
`runs/native701_retention_20260919`で別途実行する。
各条件512更新、batch40=旧32+native8、既存モデルを共通の初期値とする。
旧データ16,384提示とnative4,096提示は同条件間で揃えた有限の診断予算であり、
旧60,608提示全体を一巡するepochではない。新収集の窓は途中追加しない。
回避の教師適合と通常・発進・復帰保持を確認し、自動で実行モデルへ昇格しない。

## 検証

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

許容オプションの回帰テストを含め、WSL source `7a3245d63011b112998c34dc25a2b1959b4a3f9a`
で **3,282 passed / 4 skipped / 84 warnings**、146.54秒。
skipは既存の任意OSQP・jsonschema・公式パッケージ不足によるもの。

## 許容後の追加教師

`expand_alignment_tolerant_v1`は最初のtrain 3runで88窓。
追加の2並列収集も含む最終束`expand_alignment_tolerant_v2`は次のとおり。

| run suffix | prefix候補 | 適格（間引き前） | 採用 | 保留 |
|---|---:|---:|---:|---:|
| aleft-a01 | 61 | 56 | 32 | 5 |
| aright-a01 | 68 | 33 | 20 | 35 |
| left-a01 | 48 | 48 | 26 | 0 |
| left-a02（単独対照） | 100 | 99 | 50 | 1 |
| right-a01 | 33 | 19 | 12 | 14 |
| 合計 | 310 | 255 | **140** | 55 |

元のカメラanchorは1,078、適格255から時刻200ms間隔の間引きで115窓を外した。
採用140はコーン周辺98・通常42。残る保留は教師不成立、観測時刻支持の不足、
静止コーン文脈外でFREE_RUNではない教師など。地図照合を理由とする保留は0。

140窓は全て原本からcamera/LiDAR/ego履歴と未来30点XY・速度を再生成して照合した。
保存先は `/home/thistle/e2e_autonomous/runs/avoidance_expand_replay_20260919`。
元の絶対地図照合の許容と、原本から同じ教師を再現できることは別の確認である。

窓数は独立した回避イベント数ではない（新trainは5run）。
記録EKFに基づく配置診断では、前方コーン文脈42窓、前方0〜6m・横±1mは1窓。
この位置診断にも許容した地図ズレが含まれる。追加140窓で正面接近分布が十分に
埋まったとは判断しない。未学習配置のコーン／箱2runはvalidationとして別保存し、
このtrain束には含めない。

[選別manifest](evidence/avoidance_parallel_20260919/selection_manifest.json)、
[原本再現proof](evidence/avoidance_parallel_20260919/replay_proof.json)、
[配置診断](evidence/avoidance_parallel_20260919/coverage_summary.json)、
[容量確保の照合記録](evidence/avoidance_parallel_20260919/storage_cleanup.json)。

841窓の統合準備（学習の途中投入ではない）:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_native_replay.py prepare \
  --root /home/thistle/e2e_autonomous \
  --plan configs/time_path_p1/native841_prepare_20260919.json
```

このコマンドはprepareのみ。841窓を使った新しい学習の実行結果ではない。
source `3ab9ad3565857933750b43c1b66d586264bf52f4`で`PREPARATION_PASS`を確認。
22runの841窓を再現し、旧60,608提示の順序SHA256と既存validationを維持した。
準備済み総提示数は69,018（native 841窓×10回を追加）。
[統合準備proof](evidence/avoidance_parallel_20260919/native841_preparation_proof.json)。
701窓の4条件比較は有限予算の既存pipelineをWSLで継続する。
