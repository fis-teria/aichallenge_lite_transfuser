# 基本コース地図の静的占有認識

目的は保存地図の静的占有セルを認識すること。AWSIM/ROS起動、駆動、監視gate変更、
学習/checkpoint/Dataset読取は行わない。地図をV4入力/教師/正解操舵に使わない。

対象hostはgraneple@192.168.3.10。元repo
`/home/graneple/git/autononous_ai/aichallenge-racingkart`、HEAD
`4af395eee10f928c7fc7225760adfa04c4c07ff4`。
`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/official/occupancy_grid_map.yaml`
を基本候補とし、既存config/config.yamlが指すenv/final_ver3版も比較する。
ディレクトリ名officialは配布認証ではない。現在のV4 make devはこれらの地図をまだ読まない。
両版の明示したYAML/PGMだけをコピーし、原本を変更しない。

## 実装

- `control/static_course_map_v4.py`: uint8 PGMを占有100/空き0/不明-1へ分類。
  negate、occupied_thresh、free_thresh、解像度、origin yawを明示。
- 欠損/範囲外は不明。端へclipしない。境界threshold等号も不明。
  穴埋め、膨張/収縮、点除去、threshold調整をしない。
- queryは地図と**同じ座標系**の[N,2]mのみ。セル中心/画像上下/回転を合成検証。
- `tools/audit_static_course_map_v4.py`: hash、セル数、4近傍成分、格子npz、地図図版。
  成分数は壁や障害物の物体数ではなく連結した占有セルの数。
- 地図画素には物体ラベルがないため、壁/タイヤ/コーン等の種類はUNKNOWN。
  占有を静的障害物候補とし、実AWSIMの既定配置との一致は未証明。
- global map座標を現在V4のspawn-local UTM poseと同一視しない。
  runtime_frame_transform=MISSING、awsim_geometry_match=UNKNOWN、runtime_permission=false。

## 実行

Windows commit→既定CheckOnly/同期→WSL lock下。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_static_course_map_v4.py tests/test_spatial_live_pp_v4.py tests/test_spatial_run_continuation_v4.py
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/audit_static_course_map_v4.py \
 --map-yaml /home/thistle/e2e_autonomous/static_course_map_20260907/official/occupancy_grid_map.yaml \
 --output /home/thistle/e2e_autonomous/static_course_map_20260907/results_official \
 --source-commit 4af395eee10f928c7fc7225760adfa04c4c07ff4 \
 --source-path aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/env/official/occupancy_grid_map.yaml
# final_ver3にも同じ処理を実施。出力はresults_final_ver3へ。
```

既定同期scriptは変更せず、Dataset固定rootの存在確認のみを許可。
地図の空きセルは実時刻の空き領域を保証しない。動的/地図未掲載物体監視は別途必要。
