# V4 PP shadowの曲率超過をログ判定へ変更

専用SHADOW_ONLY設定で `shadow_curvature_log_only: true` を指定する。
経路の頂点曲率に基づく `CURVATURE_INFEASIBLE` だけを拒否から診断へ変更する。
共通core/adapterの既定は従来の拒否であり、fixture_modeなしの有効化は例外にする。
context reset後も設定を保持する。raw点や算出曲率は変更・クランプしない。

`/shadow/v4/pp/status` の `curvature_diagnostics` に計算対象plan ID、
最大絶対曲率[1/m]、必要操舵角[rad]、操舵超過量[rad]、超過有無、判定方針を残す。
新規plan受入れ開始時に診断を消し、形状以前の拒否へ旧plan診断を混ぜない。
同じplanの後続状態拒否では当該plan ID付き診断を残す。

PP目標に必要な操舵角、操舵速度、横加速度、残り参照長による減速・停止、
NaN/Inf、折り返し、不連続、時刻等の検査は維持する。
曲率による減速も維持するので、ログ判定への変更は全経路の追従許可ではない。
車両command publisherは追加せず、AWSIM・既存基準PP・モデルは変更しない。

## 検証手順

Windows commit → 既定 `tools/sync_to_wsl.ps1 -CheckOnly` → 通常同期後、WSLで：

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_path_control_bridge.py tests/test_v4_pp_reference_adapter.py tests/test_v4_pp_connection.py tests/test_shadow_resample_v4.py
```

合成testでは通常拒否/ログ判定の比較、必要操舵角の拒否維持、非fixture禁止、
診断の世代分離、context resetでの方針維持を確認する。
実ROS/AWSIMへの反映・走行はこの変更だけでは実施しない。

## 実行結果

実行版 `ee4c7606f17eeb1b3548f64479d118d1dc4618ae`。
既定CheckOnly/通常同期が成功。既定同期によるDatasetルート存在確認を実施し、
Dataset内容の読取りは行っていない。WSL共有lock下で：

- 上記限定tests：50 passed in 0.79 s。
- 同じlockで `.venv/bin/python -m pytest -q`：1822 passed, 4 skipped,
  52 warnings in 81.35 s。
- skipはOSQP、JSON Schema validator、公式Tiny packageの任意依存不足。
- 前回の固定速度上限なしの追加testsも今回実行済み。
- 実ROS接続・AWSIM再試験：NOT_RUN。pushなし。
