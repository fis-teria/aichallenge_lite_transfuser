# 区間4のデータ分布と教師再現の切り分け

対象は12/20/40/60cm統合学習の採用epoch3。AWSIM走行で右カーブ外側へ膨らんだ理由を、
学習状態の不足と、既存教師への再現誤差に分ける。AWSIM・教師・学習設定・採用重みは変更しない。

比較条件は結果を見る前に固定する。

- 確定cacheのtrain 42,172、validation 15,399アンカーを監査し、元のsplitを維持する。封印testは開かない。
- 原bagのSHA256と、各アンカーのpose行ID・受信時刻から観測位置を再現する。
- 同一実行先で完走したr30の実測線を比較基準とし、基準経路の進行165〜210mを当該カーブとする。
- 発進後150/155/160秒と監視停止時の状態を比較する。狭い近傍は進行±3m・横位置±15cm・向き±5°・速度±0.25m/s、広い近傍は±5m・±25cm・±10°・±0.4m/s。これらは記述用の範囲であり、一般化可否の保証値ではない。
- フレーム数、run数、run内イベント数、同じデータの反復提示数を区別する。位置・向きはモデルへの追加入力にしない。
- 全復帰アンカーと当該カーブの通常走行アンカーのうち、入力と未来3秒30点が完全に支持されるものを評価する。採否理由を記録し、モデル誤差から対象を選ばない。
- 学習前・epoch1・epoch2・採用epoch3の同じ入力に対するXY誤差、横方向バイアス、教師PPとの操舵差を比較する。float32、batch32、workers4、optimizer更新なし。
- 教師PPが不成立の母数と、予測PPが拒否される母数を区別する。PP比較は観測時age0秒の幾何比較で、停止監視や実走成功を表さない。

「学習前」は通常走行を学習済みの共通初期重み（command-off epoch10）を指す。ランダム初期化のモデルではない。

Windowsでcommitし、公式同期後、native WSLで実行する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=20s 2400s .venv/bin/python -u \
  tools/analyze_time_corner_learning.py --root .. \
  --output ../runs/time_corner4_learning_diagnosis_20260916
```

出力先は新規作成のみ。raw・入力配列・全予測はWSLに保持し、Gitへ追加しない。
状態の不足を確認できても、データ追加だけで完走が保証されるとは解釈しない。

初回の状態監査は完了したが、凍結重みの予測照合で解析側のcuDNN TF32設定漏れが判明した。
最大差0.092mmを許容値の緩和で通さず、元の学習時どおりTF32を無効、cuDNN deterministicを有効にした。
64件の事前照合では保存済み予測との差0。以下で監査済み状態の3ファイルをSHA256固定で再利用し、
全4重みを再評価する。raw poseの再読込を省略したことと元監査commitを記録する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=20s 2400s .venv/bin/python -u \
  tools/analyze_time_corner_learning.py --root .. \
  --output ../runs/time_corner4_learning_diagnosis_20260916_fp32 \
  --reuse-coverage ../runs/time_corner4_learning_diagnosis_20260916 \
  --reuse-sha256 42cb339e79c0adab02f2319caacd07caf9bebe605fa6250a0a09d57036b37df7 \
    e553c88d4753da7e124a82e08a4edc39dfe351e35605ce80b91e3b304d88405a \
    43da38207799fdb6b7f5814671786b6d3057030d612c22d4921821c3e8d82414

tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  docs/evidence/time_corner4_learning_diagnosis_20260916/operators/report_native.py
```

## 状態分布の監査結果

確定cacheと採用された67runの元bagを照合し、入力・未来30点が支持されたアンカーの観測poseを再現した。
当該カーブ内の母数は以下のとおり。範囲外や入力・未来の不足は別集計に保持した。

|区間165〜210mの教師|train アンカー / run|validation アンカー / run|
|---|---:|---:|
|通常5km/h群|2,829 / 6|944 / 2|
|通常8km/h群|1,492 / 6|498 / 2|
|復帰|580 / 7（7イベント）|484 / 6（6イベント）|
|左へ20cm以上ずれた状態|42 / 2（2イベント）|42 / 2（2イベント）|
|左へ40cm以上ずれた状態|27 / 2（2イベント）|26 / 2（2イベント）|

左ずれは完走教師r30の実測線を基準にする。今回の右カーブでは外側に当たる。
最後の2行は上の復帰行に含まれ、別データではない。1イベント中の連続frameを独立した復帰経験と数えていない。
復帰580 unique anchorsは実際のsamplerでは864回/epoch提示されていた。反復しても独立イベント数は増えない。

|失敗走行の時刻|位置ずれ|向きずれ|狭い近傍のtrain|広い近傍のtrain|
|---|---:|---:|---:|---:|
|150.000秒|右47cm|左2.73°|0|0|
|155.005秒|左5cm|左6.66°|0|466（通常465、復帰1）|
|160.000秒|左82cm|左5.11°|0|0|
|164.740秒|左169cm|左5.31°|0|0|

狭い・広い近傍は上記の4条件をすべて満たすアンカー数。
155秒の小さな位置ずれでは範囲の取り方で母数が変わるので「近い通常走行さえ全く存在しない」とは解釈しない。
一方、82cm・169cmの状態は広い近傍でもtrain/validationとも0件だった。
モデルの未観測状態への一般化能力を、この件数だけで確定するものではない。

全cacheではtrain 42,172のうち入力・全未来支持不足1,368、当該カーブ外35,903、当該カーブ内4,901。
validation 15,399ではそれぞれ457、13,016、1,926。
推論比較には支持済みの全復帰と当該カーブ通常走行を使うため、train 9,767、validation 4,604を各保存重みで評価する。
