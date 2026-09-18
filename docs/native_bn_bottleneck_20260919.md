# 回避追加学習の正規化統計診断

701窓比較は全層を`train()`にし、画像とLiDARのBatchNorm統計も更新する。
学習率を下げてもこの統計更新は小さくならないため、保持性能悪化の候補として
重みと統計を別々に交換して測定する。原因が確定したという主張ではない。

診断は既存モデル、focused、focused_geometryと、初期/更新後統計の交換を比較する。
画像だけ、LiDARだけの交換も含め8条件。学習更新0、実走0、checkpoint保存なし。
train教師701窓（うち前方239窓）と、既存validationを各runから時刻順に32窓ずつ
決定的に間引いた部分集合を使用する。testは使用しない。ADEはm、30点/3秒。
通常/復帰はrun等重み、native/frontは窓等重み。部分集合結果を採用判定には使わない。
元3モデルは保存済み701予測との最大絶対差1e-5m以内を要求する。

Windowsのcommitを隔離したclean checkoutから同期した後、native WSLで実行:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=15s 1800s .venv/bin/python -u tools/diagnose_native_bn.py \
  --root /home/thistle/e2e_autonomous \
  --output /home/thistle/e2e_autonomous/runs/native_bn_diagnostic_20260919
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

`--per-run 0`は既存validation全件での追試。出力は必ず新しいパスを指定する。
最大30分、8条件、学習更新なし。原本、旧モデル、運用設定を変更しない。

`freeze_batchnorm_statistics`は次の学習用の明示的な補助関数で、まだ既定学習へ
適用していない。`model.train()`の後に呼び、統計のみ固定してaffineの勾配は残す。
既存予測保持の蒸留や勾配干渉対策は、この診断結果を見て独立に比較する。

## 学習勾配の診断

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  timeout --signal=TERM --kill-after=15s 900s .venv/bin/python -u tools/diagnose_native_gradients.py \
  --root /home/thistle/e2e_autonomous \
  --output /home/thistle/e2e_autonomous/runs/native_gradient_diagnostic_20260919 --batches 8
```

focusedの既存スケジュール先頭8バッチ（旧32+新8）を使用。
各バッチ前に初期重みと統計を復元し、旧データの既存損失と、新データXY/XY+補助損失の
勾配のcosine、ノルムを測る。BatchNorm更新/固定の2条件で同じ入力・乱数を使う。
両損失はそれぞれ支持窓数で平均するため、実際の混合勾配では旧32/40、新8/40の
係数が掛かる。勾配ノルム比の解釈ではこの係数を掛ける必要がある。
負のcosineは局所的な勾配干渉を示し、未学習配置の性能や唯一の失敗原因を証明しない。
optimizer更新0、validation/test不使用、モデル保存なし。最大15分。
