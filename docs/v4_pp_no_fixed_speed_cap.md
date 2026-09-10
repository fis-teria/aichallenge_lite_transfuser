# V4 PP shadow：固定目標速度上限なし

2026-09-10のユーザー指示により、専用shadow configの
`limits.speed_cap_mps` を0.5からJSON nullへ変更した。
nullは固定試験速度上限の無効化であり、不明値や無限大の数値入力ではない。
既存の数値capを指定する他の利用箇所の挙動は維持する。

固定capなしの場合、同capを理由とする現在車速の拒否も行わない。
非有限値、負速度、操舵、横加速度、時刻、経路形状等の検査は維持する。
車両の実能力が無制限であるという意味ではなく、この設定はSHADOW_ONLYのまま。

V4は幾何経路のみなので、速度はモデル予測と呼ばない。
曲率による横加速度条件と残り参照長に対する終端停止条件から目標速度を作り、
速度sourceは `CONSTRAINT_DERIVED_TRIAL_POLICY_NOT_MODEL_SPEED` と記録する。
経路末端の速度0、加減速度、操舵角・速度、TTL等は維持する。
したがって短い2 m参照では低速の参照となり得る。点間距離から速度を推定しない。
raw経路、モデル、既存基準PP、実車出力、AWSIMは変更しない。

## 検証

追加tests：固定capなしの有限速度と終端停止、現在4 m/sからの減速要求、
明示速度0の維持、NaN/Inf/負/ゼロcapの拒否、専用connectionの既定設定。

Windowsで対象差分のみcommitし、既定CheckOnly/同期が通った後に実行する：

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
```

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_path_control_bridge.py tests/test_v4_pp_reference_adapter.py tests/test_v4_pp_connection.py tests/test_shadow_resample_v4.py
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

本変更時点では別学習タスクの `docs/spatial_long_v4_diagnostic_20260910.md` に
未コミット変更があるため、上書き・stash・無断commitをしない。
WSL検証は同期保護条件が成立するまでNOT_RUN。ROS/AWSIM再試験もNOT_RUN。
テスト追加や静的差分確認をPASSの実行証拠として扱わない。
