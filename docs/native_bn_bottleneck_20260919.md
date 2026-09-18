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

## 今回の実測結果

2026-09-19、WSL CUDA。正規化診断source `cef5e3bc73673e86d2a00153888703d36f7abfae`、
勾配診断source `8b7ca46ea3d7e0f675f1ea1cae8d4592c1c2e860`。両診断とも正常終了。
8条件の正規化診断は推論部分計123.2秒（原本ハッシュ検査等は除外）。
元3モデルの701窓予測は保存済み予測と完全一致（最大絶対差0m）。

### 正規化統計の後付け交換だけでは解決しない

単位cm。通常124窓/4run、復帰1,344窓/42run、前方教師239訓練窓。
通常・復帰はrun等重み、前方は窓等重み。これらは保持判定用全件の部分集合。

| 重み / 統計 | 通常ADE | 復帰ADE | 前方教師ADE |
|---|---:|---:|---:|
| 初期 / 初期 | 1.813 | 1.984 | 44.091 |
| focused / 更新後 | 1.974 | 2.059 | 37.836 |
| focused / 初期 | 2.094 | 2.226 | 37.746 |
| focused_geometry / 更新後 | 2.420 | 2.163 | 35.322 |
| focused_geometry / 初期 | 2.536 | 2.641 | 35.733 |
| 初期 / focused_geometryの統計 | 2.525 | 2.574 | 43.454 |

統計のみ変えても悪化が起こるが、学習後重みに初期統計を戻しても保持性能は回復しない。
重みと統計の相互作用があり、「統計だけが主因」「統計固定学習は無効」のどちらも
この交換実験からは断定しない。初めから固定して学ぶ効果は別の学習比較が必要。
LiDAR統計のみ初期へ戻す条件は通常2.377cm/復帰2.157cmで、小幅変化に留まった。

### 局所的な学習勾配の干渉を確認

同じ初期モデル・8バッチ。旧データは通常/発進/復帰の既存samplerから32提示、
nativeは8提示。各群の平均損失を分けて微分し、勾配cosineを測定。
負値はそのバッチの勾配が逆向きであることを示す。新規独立イベント数ではない。

| BN条件 / native損失 | 負cosineのバッチ数 | 平均cosine | 混合係数込みnative/old勾配ノルム比の平均 |
|---|---:|---:|---:|
| 更新 / XY | 8/8 | -0.573 | 0.466 |
| 更新 / XY＋補助 | 7/8 | -0.620 | 1.126 |
| 固定 / XY | 6/8 | -0.243 | 0.831 |
| 固定 / XY＋補助 | 5/8 | -0.267 | 2.009 |

ノルム比にはnative8/old32=0.25を掛けている。平均勾配の比ではなく、各バッチの
比を平均した診断値。Adamの状態・全学習経過・汎化性能の代用ではない。
強い補助損失は今回の局所プローブでnative側更新を強くし、BN固定だけでも干渉は残る。
「追加教師を重くすればよい」より、既存出力の保持制約を先に比較する根拠になる。

## 次に推奨する学習比較（未実行）

まず701窓を固定し、841窓への変更は後段で比較する。
初期重みは運用モデル`1fe12ca0...`、seed42、AdamW、FP32、LR1e-6、最大512更新。
旧32提示の順番を全条件で同一にし、混合条件は追加native8（前方4＋全体4）を維持する。
128/256/512更新で候補を保存・評価する。これは旧データ全体の1epochではない。

1. **旧データのみ**：追加学習そのものによる変化を測る、これまで不足していた対照。
2. **focused再現**：旧32＋native8、nativeはXY損失のみ。旧復帰損失は従来どおり。
3. **focused＋旧出力保持**：旧train入力に限り、凍結した初期モデルの30点予測への
   Smooth L1（beta=0.02m、係数1.0）を追加。新回避入力には旧直進予測を強制しない。
4. **3＋弱い補助損失**：nativeの操舵・遠方横位置補助を既存実装の0.1倍で追加。

beta/係数は検証済み最適値ではなく、事前に固定する初回候補。
旧出力保持は学習データ上のみ。失敗したvalidation/発進ケースを学習へ移さない。
まず現在と同じBN更新条件で損失の効果を切り分け、統計固定は独立した追加比較とする。
旧出力保持でも干渉が残る場合に、実測した勾配の競合を抑える方式を検討する。
モデル大型化や全層凍結は現時点では原因を確認できていないため優先しない。

採用には通常11,505窓/4run、復帰8,962窓/42run、発進210ケースの従来基準を使用する。
ADEだけでなく復帰run別、3秒先、発進操舵余裕、前方教師の操舵方向と不足量を見る。
合格候補だけ別seedと未学習配置、AWSIMで確認。閾値緩和・自動配備は行わない。

## データ側の優先課題

701窓の既存束では正面0〜6m・横±1mの採用窓が0。
追加140窓でも同条件は1窓のみで、絶対位置には許容した地図照合ズレが含まれる。
窓数を増やすだけでは、接近・回避開始の分布不足を解決したと判断できない。
「回避開始前→回避開始→側方通過→復帰」を連続で再現できる教師を優先する。
近接停止を前進回避ラベルへ変更しない。箱試行の採用通常部分を箱回避教師と呼ばない。
未知配置のvalidationをtrainへ移さず、箱近傍の有効教師も別途確保する。

速度・停止headは今回未学習であり、回避不能時停止は今回のXY改善だけでは達成しない。

根拠・参考:

- [BN診断原本の控え](evidence/native_bottleneck_20260919/bn_summary.json)
- [勾配診断原本の控え](evidence/native_bottleneck_20260919/gradient_summary.json)
- [PyTorch BatchNorm公式](https://docs.pytorch.org/docs/2.14/generated/torch.nn.BatchNorm2d.html)
- [Learning without Forgetting](https://arxiv.org/abs/1606.09282): 旧出力保持の発想。
  本提案は旧データを使う軌道回帰用の応用案であり、論文方式そのものの再現ではない。
- [Gradient Surgery for Multi-Task Learning](https://arxiv.org/abs/2001.06782): 勾配干渉への対策候補。
  本プロジェクトでの有効性は未検証。

## コード検証と保全

`8b7ca46ea3d7e0f675f1ea1cae8d4592c1c2e860`でWSL全体pytestは
**3,288 passed / 4 skipped / 84 warnings、115.33秒**。
新規6テストは統計固定後もaffineが学習されること、選択統計だけの交換、
不正統計の拒否、勾配cosineの符号・ゼロノルム・shape異常を検証する。
初回全体実行の5失敗は隔離checkoutの既存互換性checkpoint参照不足。
原本へのsymlinkを設定した再実行で全件成功。重み本体は変更しない。

Windows/WSLは専用clean checkoutでcommitを照合し、`sync_to_wsl.ps1`と
`with_wsl_training_lock.sh`を使用。大きい成果物はWSLに置き、結果JSON2件のみ
SHA256一致を確認してWindowsに控えた。学習・推論の既定値変更なし、外部pushなし。
元からのstaged変更はM3/A1/D2185、unstaged14/untracked20を保全した。
未実施は上記4条件の再学習、全件による新候補採用判定、AWSIM回避・停止評価。
