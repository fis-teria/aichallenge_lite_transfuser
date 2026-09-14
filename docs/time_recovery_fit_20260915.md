# 既存復帰データの学習到達度の確認（2026-09-15）

目的は、新モデルが学習に使用した復帰場面の教師を再現できるか確認し、未学習runの誤差と区別すること。
新たな学習、checkpoint選び直し、AWSIM走行は行わず、保存済みの学習前・epoch1・epoch2・採用済みepoch3を同じ入力で比較する。

## 固定条件

- native WSL、RTX 4080、float32、batch32、workers4。Windows確定sourceを同期し、共有worktree lock下で実行する。
- `recovery_random_update_20260915.json` の確定cache。学習に使用した復帰1,410 unique anchors、未学習の復帰538 anchors。連続frameは独立した走行ではない。
- 収集時に分類した厳格な外向き状態は学習35、検証12 anchors。モデルの誤差から都合のよいsubsetを選ばない。
- trainはtrainのまま評価し、manifestを書き換えてholdoutと称さない。検証r22/r23は従来のcheckpoint選択に使用済み、r46/r47/r65は選択後診断用。封印testのraw/cacheは読まない。
- 全対象で実入力と3 s・30点の教師支持を要求する。欠損・不正出力・順序不一致は明示的に失敗し、難しいサンプルを黙って分母から除外しない。
- 教師と予測のXY誤差（0.5/1/2/3 s）、横方向誤差、runごとの誤差、教師PPとの物理タイヤ角の差を比較する。
- PPは固定目標5 km/h、`stopping_preview_extended_v1`、観測時age0 s、同じ実測速度・車両モデル。教師がPPを成立させる母数を固定し、予測拒否は既存0.6 radペナルティで残す。scan監視や実走成功は評価しない。
- 位置・角度誤差を完走の合否閾値に転用しない。trainとvalidationは状態分布が異なるため、全体の比だけで過学習・データ不足を断定しない。

## 実行

Windowsで本解析とテストをcommit後、`tools/sync_to_wsl.ps1 -CheckOnly` と通常同期を行う。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout 1200 .venv/bin/python tools/analyze_time_recovery_fit.py \
  --plan configs/time_path_p1/recovery_random_update_20260915.json --root .. \
  --output ../runs/time_recovery_fit_20260915
```

既存出力への再実行は拒否する。raw/cache/checkpointはWSLに保持する。結果はここへ追記し、少量のJSONと図だけをWindowsへ返す。
