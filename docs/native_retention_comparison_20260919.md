# 701窓・旧予測保持の4条件比較

ユーザー承認済みの比較。通常/発進/復帰の旧モデルと設定を保全し、4条件を独立保存する。
AWSIM試験・標準モデルへの自動昇格は含めない。

| arm | 旧提示/更新 | native提示/更新 | 旧出力保持 | native補助係数 |
|---|---:|---:|---:|---:|
| old_only | 32 | 0 | 0 | 0 |
| focused | 32 | 8 | 0 | 0 |
| retained | 32 | 8 | 1.0 | 0 |
| retained_weak_geometry | 32 | 8 | 1.0 | 0.1 |

共通: 初期checkpoint SHA256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`、
seed42、FP32、AdamW、LR1e-6、weight decay1e-4、勾配clip1.0、BatchNormは従来どおり更新。
各512更新、合計2048更新。128/256/512更新で12候補を保存し、初期モデル込み13回評価する。
時間上限2時間、重み保存見込み約1.8GB。完了・例外・timeoutでプロセス終了。
途中評価では乱数状態を復元し、再びtrainモードへ戻して続行する。

旧データは各条件16,384提示で順番を一致させる。混合条件はnative4,096提示を追加。
native8は前方4＋全体4、701窓/17runのtrain教師を使用する。841窓は混ぜない。
前回focusedのスケジュールhash一致を要求し、512更新後の701予測を前回と1e-5m以内で照合する。
旧データ入力欠損は従来どおり除外し、提示数・支持数は別に記録する。
これは全旧データを一巡する1epochではない。

旧出力保持は旧trainの支持窓だけを対象とする。凍結した初期モデルのeval予測を一度保存し、
anchor/run・split・checkpoint・予測hashを結び付ける。ターゲットは30点XY（m）。
Smooth L1 beta=0.02m、係数1.0、各窓内の有効点/XYを平均し、窓ごとの損失を合計。
既存L1＋既存復帰補助＋native補助＋旧出力保持を、混合バッチ全体の支持窓数で割る。
新native入力に旧モデルの直進予測を蒸留しない。validation/testを蒸留に使わない。

評価は通常11,505窓/4run、復帰8,962窓/42run、発進210ケース。
通常/復帰ADE・3秒先・復帰run別・発進操舵余裕の既存閾値は維持する。
native701窓/前方239窓は訓練内適合であり、未学習障害物や実走の成功を示さない。
旧のみの条件でも同じ評価を行い、回避を追加しない継続学習による変化を切り分ける。

## 実行

Windowsで対象ファイルだけをcommitし、既存差分を保全したclean cloneから同期する。
専用WSL checkout: `/home/thistle/e2e_autonomous/native_bn_validation_20260919`。

```powershell
git -C E:/workspace/e2e_native_bn_source_20260919 pull --ff-only
& E:/workspace/e2e_native_bn_source_20260919/tools/sync_to_wsl.ps1 `
  -WslRepository /home/thistle/e2e_autonomous/native_bn_validation_20260919 `
  -WindowsRepositoryInWsl /mnt/e/workspace/e2e_native_bn_source_20260919
```

```bash
cd /home/thistle/e2e_autonomous/native_bn_validation_20260919
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=30s 7200s .venv/bin/python -u \
  tools/compare_native_retention_losses.py --root /home/thistle/e2e_autonomous \
  --output runs/native701_retention_losses_20260919
```

出力先は排他的に新規作成。再試験は別outputを指定する。
学習ログ・checkpoint・全評価・selection・completion・progressをWSLの出力先へ保存する。
例外やtimeout時は完了扱いにせず、保存済み候補までと停止理由を報告する。

## 完了結果（2026-09-19）

**旧出力保持あり・native幾何補助なしの `retained_step0512` が、12候補中唯一、既存保持基準とnative適合改善基準をすべて通過した。**
通常・復帰・発進を既定の許容幅内に保ちながら、前方障害物239訓練窓のADEを44.09cmから39.15cmへ11.2%低減した。
これはオフラインの開発評価による次の検証候補であり、未学習障害物の回避成功・AWSIM完走・衝突回避の証明ではない。
運用中の初期checkpointは終了時にhash不変を確認し、標準設定の変更・自動昇格・AWSIM走行は行っていない。

### 512更新時点の比較

下表の誤差はすべてcm、低いほどよい。ADEは将来30点のXYユークリッド距離の平均、3秒先は末尾点の距離。
通常はvalidation 11,505窓/4run、復帰はvalidation 8,962窓/42runで、runごとの窓平均をrun間で等重みに平均する。
前方障害物は**train 239窓の窓・30点平均**であり、通常/復帰と母集団・集計が異なる。

| 条件 | 通常ADE | 通常3秒先 | 復帰ADE | 復帰3秒先 | 前方障害物train ADE | 総合基準 |
|---|---:|---:|---:|---:|---:|---|
| 初期モデル | 1.795 | 4.905 | 1.909 | 4.597 | 44.09 | 比較基準 |
| old_only：旧データのみ | 1.798 | 4.752 | 1.920 | 4.651 | 44.67 | 未達：発進1ケース、native改善なし |
| focused：回避混合、保持なし | 2.049 | 5.612 | 1.987 | 4.850 | 37.84 | 未達：通常、復帰3秒先、発進2ケース |
| **retained：回避混合、旧出力保持** | **1.885** | **5.094** | **1.945** | **4.748** | **39.15** | **通過** |
| retained_weak_geometry：保持＋幾何0.1 | 1.912 | 5.208 | 1.951 | 4.767 | 38.56 | 未達：通常ADE・3秒先 |

`retained` のnative全701窓/17runのrun平均ADEは41.70→37.76cm、静的コーン423窓/15runは44.09→40.38cm。
どちらも訓練内の適合である。発進210ケース（35観測×6遅延条件）は全件制御器が受理し、ケース別操舵余裕も基準を通過。
復帰42runもrun別基準を通過した。「通過」は誤差が一切増えていない意味ではなく、事前の許容幅内という意味である。

保持ゲートは初期値＋max(初期値の5%, ADE 0.001m / 3秒先0.002m)、発進操舵余裕の許容悪化0.001rad。
復帰run別は初期値＋max(初期値の20%, ADE 0.005m / 3秒先0.01m)。native全体・静的コーンのADE改善は各1%以上を要求する。
合格候補のうち静的コーンtrain ADEで順位付けする既存規則を変更していない。

### 途中保存と切り分け

| 条件 | 128更新：通常/復帰/前方train ADE cm | 256更新：通常/復帰/前方train ADE cm | 途中候補の判定 |
|---|---|---|---|
| old_only | 1.797 / 1.892 / 43.96 | 1.790 / 1.894 / 44.21 | 両方未達 |
| focused | 1.978 / 1.922 / 41.44 | 2.017 / 1.918 / 40.00 | 両方未達 |
| retained | 1.861 / 1.910 / 42.03 | 1.882 / 1.910 / 40.80 | 両方未達 |
| retained_weak_geometry | 1.872 / 1.913 / 41.80 | 1.901 / 1.914 / 40.44 | 両方未達 |

- 旧のみでも発進余裕が悪化する候補があり、発進の変化をすべて回避追加のせいにはできない。
- 同じ混合提示で旧出力保持を追加すると、512更新時の通常/復帰の悪化が抑えられ、合格候補が得られた。今回の有限予算では保持係数1.0・native幾何0を推奨する。
- 幾何0.1の追加は前方train ADEを③より0.59cm低減したが、通常保持を外した。今回の設定では③に対する優位性は確認できない。
- seed42の1試行であり、統計的な再現性や最適係数を確定した結果ではない。旧validationを各候補の選択に使っており、独立した最終test成績でもない。
- 選定候補でも、教師/予測の制御計算が成立し、教師操舵絶対値が0.02radを超える132窓中33窓が逆符号、54窓が教師の半分未満の操舵量。239窓中、教師制御不成立4窓、残り235窓中予測制御受理234窓。回避方向・操舵量の不足は残る。

### 実行証跡と次の検証

- 実行ソースcommit: `8a92537bdef856042bcae05bc7aad79d8ad2fe0f`。
- 終了コード0、`COMPARISON_COMPLETE`。合計2,048更新、保存12候補＋初期の13評価。比較ループ実測3,429.1秒（57.15分、事前キャッシュ準備を除く）、外側の上限7,200秒内で完了。
- 各条件512更新すべて成功。旧のみ16,384提示/16,352支持、混合各20,480提示/20,448支持。各条件とも同じ旧入力欠損32提示を除外し、教師非支持は0。旧データ提示順の一致を実行時assertで確認した。
- 旧trainの支持anchor 14,413窓の初期eval予測を保持ターゲットとして保存。参照anchor/split/hash、スケジュール、全予測、重みはWSL実験ディレクトリに保管。
- 初期モデルとfocused512の701窓予測は、前回比較の対応予測との差がともに**最大0m**。追加の途中評価を挟んでも対照条件を再現した。
- 各保存直後の再読込一致は固定した前方8窓で確認。その後、各候補を全評価母集団で評価した。
- WSLで `pytest -q`: **3,296 passed, 4 skipped, 84 warnings**、104.59秒。shape・単位・mask・referenceのdetach・旧train限定・不正入力/係数を新規8テストで検証。
- 全13評価JSON、選択理由、完了記録、設定、学習ログ、pytestログを [evidence](evidence/native_retention_comparison_20260919/) に保存。WSLからコピーした21ファイルのSHA256一致を検証し、`SHA256SUMS.json`に記録。`metrics_summary.json`はこれらの評価JSONから生成。

選定checkpoint:
`/home/thistle/e2e_autonomous/runs/native701_retention_losses_20260919/retained_step0512.pt`

SHA256: `e7afdab5d05873de0dbed454e1086b32f4d4f1b67f884a9417a349d7db726ad2`（完了後に再計算して一致）。

次はこの条件を基点に、逆符号33窓の位置・教師・観測を照合して方向違いを切り分け、別seedで保持の再現性を確認する。
未学習配置の評価と有限のAWSIM比較を経て、回避・停止・完走を別々に判断する。今回の比較だけで運用モデルへ置き換えない。
