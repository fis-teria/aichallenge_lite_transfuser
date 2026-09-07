# V4参照長見直し：端点対応を残したoffline試験

目的：旧固定弧長fitで拒否された保存41件について、曲線長固定に由来する終点前方超過を見直す。
元20点・既存FIRST_CUSP prefix・その終点・10cm偏差上限を変更しない。
1自由度の長さ倍率を導入し、7舵角knotsと同時に1回だけSLSQPで解く。

## 新しい明示opt-in

`constrained_reference(..., length_policy='ENDPOINT_NORMALIZED_LENGTH_V1')`。
既定は引き続きFIXED_ARCLENGTH。runtime呼出し・設定は変更せず、新方式はこのoffline toolだけで選択する。
対応パラメータqは原本prefixの実折線長+初期接続。物理積分パラメータはq×scale。
正のscaleは開始と終端までの順序を保つ。rawの末端を未対応にしたり、nominal sを実弧長へ置換しない。
scaleの上限1、下限max(既存最小support,端点chord-10cm)/元長。
舵角速度は実物理knots間隔で計算。最大速度・加減速・舵角・横加速度制限は変更しない。
両折線のunion-breakpoint誤差証明、初期状態、正の長さ、原本hashを保持する。
証明は折線間幾何についてのみ。衝突・実車の可走性・teacher正当性ではない。

固定元replay SHA256 `8ca6e490f99cbc86ef57aa45246d4a1c3d4a890b2230d75aa7c4849591cbb927`。
旧不受理41件すべてへ1fitずつ。既存5件は再選択/再fitしない。
新経路のPure Pursuit/MPC追従、AWSIM起動、制御publish、学習、新推論は今回0。
追加fit41と合成unit testsを別計上する。成功までの設定探索はしない。

## 検証手順

Windows commit→未変更既定CheckOnly/同期→同SHA WSL lock下で限定test/replay。
Datasetルート存在確認だけを許可し、内容/raw/sensor/checkpointは読まない。
全pytestは学習系testを避けて未実行、対象純粋幾何と既存回帰のみ実行。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_reference_length_v4.py tests/test_spatial_blocker_fixes_v4.py tests/test_spatial_sim_e2e_v4.py
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/evaluate_spatial_reference_length_v4.py --input /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/replay_final/replay.json --configs /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/inputs --output /home/thistle/e2e_autonomous/spatial_length_review_20260907/results
```

既存資料・既存結果・学習重みは変更しない。新出力directoryは上書き禁止。自動pushなし。
