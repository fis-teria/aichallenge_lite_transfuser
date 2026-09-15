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
