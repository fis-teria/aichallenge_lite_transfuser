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

## 結果

実装版76605bd82f850e19b0190c154997ae9fee15ed44で限定テスト43 passed / 6.63s。
初回CLIはpackage import解決で停止（地図読取/出力作成前）。
c6f1b50bf13f8818fc543a830a54d84ffb1a6129でsrc参照を明示し、両地図のCLI完了を確認。
固定元4ファイルのGit状態はclean（元checkout全体の既存dirtyは保全）。

| 候補 | 格子[H,W] | 解像度 | 占有セル | 空きセル | 不明セル | 占有4近傍成分 |
|---|---|---|---:|---:|---:|---:|
| official | 766,755 | 0.1m | 309887 | 268443 | 0 | 6 |
| final_ver3 | 759,751 | 0.1m | 320418 | 249591 | 0 | 5 |

両地図はサイズ・origin・画素が異なり、同じグリッドとして重ねない。
official origin=[89608.61776988552,43116.40095341299,0]、
final_ver3 origin=[89608.69387016428,43117.15165326744,0]。
official図版を目視し、主にコース通行領域/外側を分ける形状であることを確認。
個別物体の意味ラベルはなく、6/5成分を「障害物6/5個」と呼ばない。
既定のコーン/タイヤ/追加障害物が完全収録されている根拠もない。
**地図上の静的占有認識は実装済みだが、AWSIMの既定障害物との同定は未完了。**
地図内不明0も実環境の未知領域0という意味ではない。

| 原本 | SHA256 |
|---|---|
| official YAML | 2977e3b241ef4f1ce3527212fb0733679939d2d2cb87af40742ac76b002da212 |
| official PGM | 403ac8d681f5ab8ecba9df8892ff72bba13f7351016bf108f1145133f0d6a12a |
| final_ver3 YAML | 39d5aba44234c1e09fc57421d467d64c1769259cd3c3656032f008d2db4e6a79 |
| final_ver3 PGM | c24af2130a8df96047a49d5b7759af7a0b29645c023e7f3bc6d4ba436275725a |

Windows成果物は `tmp/static_course_map_20260907/`。
`results_official/` と `results_final_ver3/` にsummary.json、static_grid.npz、static_map.png。
validation.log、tests.xml、失敗を含むaudit.log/audit_retry.logを保存。
元地図はofficial/・final_ver3/へ保存、Gitには追加しない。
WSL原本/結果は `/home/thistle/e2e_autonomous/static_course_map_20260907/`。
summary SHA256 official=bbb7c9888f13ccbe109f4f60031c3e1a8fc05287952b693945e2f33945c8f924、
final_ver3=8bb325d861274851327191906bf44f26ff67430a781c7dd45ec0508a8243d579。

既定同期によるDataset固定ルート存在確認を実施。Dataset内容/raw/sensor/checkpoint読取は未実施。
AWSIM変更/起動、制御publish、駆動、学習、監視gate変更、pushはいずれも未実施。
次は選択AWSIMコースと地図の対応、座標変換、既定配置との突合が必要。
その後も占有地図単独で動的物体や地図差分を無視した駆動許可は出さない。
