# MPPI V45 近接開始の教師調整

対象は `front-cone-close6-a02` の停止。ログでは5候補が `execution_sweep` で衝突棄却され、速度0のretimeが採用された。物理衝突は記録されていない。近接開始時に前進回避可能かは別途実走で判定する。

`--early-entry-search` は収集専用の比較スイッチ（既定off）。AVOID、前進0～10km/h、非MERGE・非prepared front mergeの場合だけ、固定されていたBezierの横移動タイミングを near=[0,0.45]、far=[0.55,1] で探索する。nominalは(0.45,1)。移動距離下限4m、操舵遅延、壁・物体の衝突判定、速度計画は既存のまま。一般の追越し・復帰・高速走行には適用しない。

```bash
python3 tools/build_mppi_v45_overlay.py --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart --runtime /home/graneple/e2e_autonomous/mppi_close_adjust_20260918/runtime
python3 tools/collect_mppi_v45.py --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart --runtime /home/graneple/e2e_autonomous/mppi_close_adjust_20260918/runtime --scenario /absolute/path/to/scenario.yaml --run-id lidar-v45-pc10-front-close-entry-a01 --speed-cap-kmh 5 --early-entry-search --wall-timeout-s 480 --run-budget-gib 0.75 --free-reserve-gib 2 --rviz --execute
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

ビルドは公式Docker内のC++/ROS回帰試験を含む。AWSIM本体・元runtimeは変更せず、別runtimeを使用する。停止だけでは成功教師として採用しない。結果は確認後に追記する。
