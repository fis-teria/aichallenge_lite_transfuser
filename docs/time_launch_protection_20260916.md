# 発進教師の整理・発進配分の比較・段階別の採用判定

前回の[発進停止の切り分け](time_launch_regression_20260916.md)に基づき、モデル選択と学習配分を修正する。Windowsを編集正本とし、本学習・評価はnative WSL、AWSIM本体とPP／監視設定は変更しない。

## 実施する変更

1. 通常教師の`/awsim/state`で`Start → Ready`を確認する。Readyメッセージの受信時刻と次の`/clock`で保守的に開始を確定し、観測時点／入力確定時点でまだReadyでないフレームを今回の学習対象から外す。人が要求した終了時ブレーキを将来3秒＋補間50msの教師がまたぐ場合も対象外とする。元bag・cache・教師座標は保持する。
2. 通常走行の発進対象はReady後2秒以内、現在速度−0.03〜0.5m/s、入力と30点教師が完全なもの。予備監査ではtrain 78フレーム／12走行、validation 26フレーム／4走行。元の「静止中だが将来3秒に発進する152フレーム」とは対象の定義が異なる。
3. 両学習条件で同じ教師対象を使う。対象外になった通常枠は有効な通常走行の再提示で補充し、60,608提示/epochを維持する。発進重点条件だけは、復帰データの重複提示枠から発進枠へ配分し、全体の5%（3,030提示）を確保する。全ての採用済み復帰教師、対象内の通常教師を最低1回提示する。画像・LiDAR・教師を人工変形しない。
4. 初期モデルを比較候補に残し、発進のPP成立・操舵余裕、通常／復帰それぞれのrun均等ADE・3秒先誤差で採用判定する。学習内部の`best.pt`を、そのまま実走に昇格させない。

発進許可の追加モデル入力は作らない。今回の条件は、外部の開始待ちを含めず、走行可能になった後の経路を学習させるという教師対象の整理である。未知の待機／再発進一般を解決したことにはならない。

## 固定する実験条件

- 初期checkpoint: `time_recovery_multiscale_20260916/training/best.pt`、SHA256 `685f8b8f9936ab7e272be12616982374fd8a8d61fbe4a304b25475dc2a206ba8`。
- `protocol_control`と`launch_balanced`の2条件。両方3epoch、batch32、FP32、workers0、seed42、learning rate3e−5、最大5,682更新／181,824提示。1条件の外側上限は10,800秒。過去に失敗したworkers4は使用しない。
- モデル、損失、optimizer、scheduler、制御を固定する。復帰の幾何補助損失も既存の定義を保持する。2条件間の差は発進への提示枠の配分である。
- 通常教師の開始待ち／外部停止区間の除外は両条件に共通。以前の学習との比較では、この対象整理と追加学習の効果を分離した因果推定とは呼ばない。
- 全epochの重みを保存し、学習前の重みを含めて採用判定する。初期モデルも絶対条件を満たさなければ、新しい実走候補なしと明記する。

## 評価集合と採用条件

今回から既存validation全体を**開発用の採用判定**に使用する。以前の実験でcomparison-onlyだった追加validationも、この実験では開発用として明示的に位置付ける。過去の記録の役割を書き換えず、今回の設定に残す。run単位のtrain/validation割当は変更しない。封印testは開かない。

通常評価はReady後の有効な完全教師から発進対象を除いたもの、復帰評価は既存validationの復帰run。モデルごとに教師支持数を変えず、非有限の予測は失敗とし、分母から落とさない。

発進評価は上記のvalidation発進対象と、既存AWSIM校正baseline r30/r31の教師制御開始後0〜0.5秒を使う。校正baselineは学習へ入れない。観測から制御まで0〜0.5秒の各0.1秒について、同じ実測姿勢・速度でPPを計算する。将来の姿勢・教師は評価だけに使用する。曖昧な将来poseは全候補共通で欠測として記録する。

採用条件は学習結果を見る前に固定する。

- 発進: 教師PPが成立する全ケースで予測PPも成立。物理操舵限界±0.3radは維持する。初期モデルが成立した同じケースより選択操舵余裕を0.001radを超えて悪化させない。
- 通常・復帰: 各run均等ADEは初期値＋max(5%,1mm)、3秒先誤差は初期値＋max(5%,2mm)以内。
- 合格候補と初期モデルの中で、通常／復帰のADE・3秒先誤差を初期値で正規化した4指標の平均が最小のものを選ぶ。同点なら初期モデルを維持する。
- 全ての判断は`selection.json`に残す。`runtime_test_allowed=true`は次の有限AWSIM試験に進めることだけを意味し、完走・無衝突の証明ではない。

以前のepoch2を新しい判定へ通し、既知の発進不良を拒否できることも確認する。今回見た校正・validation記録を、未使用の最終評価として扱わない。

## 実行

Windowsでcommitし、`tools/sync_to_wsl.ps1`の公式手順で同一sourceを同期する。以下はnative WSL repo内。出力先は`/home/thistle/e2e_autonomous/runs/time_launch_protection_20260916`。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u tools/train_time_launch_protection.py prepare
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u tools/train_time_launch_protection.py evaluate --existing-only
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=30s 10800s .venv/bin/python -u tools/train_time_launch_protection.py train --arm protocol_control
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=30s 10800s .venv/bin/python -u tools/train_time_launch_protection.py train --arm launch_balanced
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -u tools/train_time_launch_protection.py evaluate
```

同じsource／計画で中断した学習だけは`train --arm <同じ条件> --resume`で再開できる。同期中と学習中は同じworktree lockで排他する。

## 実行結果

実装・検証・学習結果を確認後、この節に追記する。現時点の予備監査は収集済み通常走行16件のReady遷移確認までであり、新たな学習成功やAWSIM完走を示すものではない。
