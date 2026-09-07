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

## 結果（2026-09-07）

実行/限定test版 `a4e70e208c78f712025aad48e0f4c0ad8137f843`。
origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`、branch `codex/windows-wsl-training-sync`。
ユーザー許可により `/prasentation/` と確認した4個の `.chart-data-*` directoryのみ
Windows `.git/info/exclude` へ追加。ファイル削除・共有gitignore変更・他の変更commitなし。
cleanを確認して既定CheckOnly/通常同期、WSL同SHA・worktree lockで実施した。

限定test **46 passed / 1.89秒**。固定41件のfitは各1回、計41回。設定探索・再試行0。

| 項目 | 旧固定長 | 新・終点対応付き可変長 |
|---|---:|---:|
| 41件中の幾何受理 | 0 | **41** |
| 最大偏差上界の範囲 | 11.65～17.41cm | **7.16～7.49cm** |
| 使用prefixの終点間偏差 | 11.65～17.41cm | 5.37～7.49cm |
| 積分長の範囲 | 1.681～2.084m | 1.623～1.955m |

長さ倍率は0.93772～0.96565、元の折線長に対して約3.44～6.23%短い。
これは**同じ使用区間の終点までを対応させる曲線の長さ**であり、
原本の末端を新たに削って達成したものではない。
旧FIRST_CUSPにより16点使用の38件と、20点使用の3件を同じまま維持。
全20点の元bytes/hash、使用indices/未使用tail、10cm上限を検査し不変。

同じ保存状態、同じ最大舵角/rate/横加速度制限で、長さ自由度を1つ追加した結果である。
従って**固定折線長の強制が、この41件に対する現方式の拒否要因だった**という
限定的な比較証拠は得られた。学習不足がない、任意経路が可走、最適解が一意、とまでは言えない。
endpoint equalityを強制したわけではなく、終点も10cm以内の偏差制約で検査した。

保存knotsと物理積分長から独立再構成し、union-breakpointの全区間証明値を1e-10以内で照合。
短くなった実knots間隔による最大操舵速度も再計算し、0.8rad/s+数値許容1e-8以内を確認。
fit wall実測は0.05975～0.09127秒/件、既定0.5秒の検査を維持。

保存先:

- Windows `tmp/spatial_length_review_20260907/results/summary.json`、`case_00.json`～`case_40.json`。
- 同root `evidence/validation.log`、`tests.xml`、同期ログ。
- WSL `/home/thistle/e2e_autonomous/spatial_length_review_20260907/results`。
- summary SHA256 `0b85d2cd459ce5c608feb2fe6e7a9a779682b055445c88ae2e5655abd43da659`。

既定同期によるDatasetルート存在確認を実施。Dataset内容/raw/sensor/checkpoint読取りは未実施。
新推論0、学習0、MPC0、PP試験0、AWSIM/ROS起動0、制御送信0。過去sim累計予算は変更なし。

## 完了範囲と次段階

今回の曲線長見直しと41件の幾何検証は完了。
新方式は**明示opt-inのみ**、既定runtimeのFIXED_ARCLENGTHは変更していない。
元の受理済み5件には今回の方式を適用/評価していない。
以前のPure Pursuit 5件成功は旧曲線に対する別結果であり、この新41件の追従成功には流用しない。
次は新41参照のPure Pursuit仮想追従・停止を別評価する段階。
周辺監視、更新するV4経路との接続、実sim挙動、走行予算は未成立/未承認のまま。
AWSIMやruntimeへ自動配置せず、pushも実施しない。
