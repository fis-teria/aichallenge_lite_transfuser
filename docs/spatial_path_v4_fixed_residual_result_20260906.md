# 固定出力だけによるSpatial Path V4の残差・形状診断

## 結論

保存済みtrain64・val main160・観察20の教師/予測だけで診断を完了した。
**遠方の主な増大成分はdy（横方向）。直線では右寄り残差と点間隔の非一様性が残る。**
これは観測された出力の特徴で、学習法・教師・入力・座標・初期化のどれが原因かは**UNKNOWN**。
補正、平滑化、再選択、追加推論、重み読取、Dataset/raw読取、学習、shadow接続はしていない。

実行版：`9b01b1e`（完全SHAと入力6ファイルのhashは成果物`provenance.json`）。
開始版：`7acc1effabb8d772a77557b97084a85595365385`。
Repo/branchは既存のfis-teria/aichallenge_lite_transfuser / codex/windows-wsl-training-sync。
WindowsのNumPy 1.26.3、標準unittest、matplotlibのみ。モデル/torch/ROSコードのimportなし。
元評価版・契約・元metrics/annotation_addendumは変更せず、固定IDで注釈を結合した。
結果追記は別commit、pushなし。

## 母数と軽量照合

| role | anchors | 支持あり | 有効点 | ADE m | 全20点支持の固定集合 |
|---|---:|---:|---:|---:|---:|
| train再現 | 64 | 64 | 977 | 0.013010 | 33 |
| val main | 160 | 160 | 2140 | 0.025181 | 58 |
| val観察 | 20 | 18 | 133 | 0.058042 | 2 |

全244件のper-anchor ADE/支持点数を保存metricsと照合して一致。
支持なし観察2件はゼロ誤差にせずUNKNOWN。有効点合計3250。
全roleを混ぜた性能指標は作っていない。

「variable_support」は元集合のmaskに従い、距離ごとに支持母数が変わる。
「fixed_2m_support」は同じ元集合内で元maskが20点すべてtrueのIDを固定した記述的部分集合。
新しい評価標本の選び直しではなく、元集合の記述的な分解である。
val固定58件はnormalの2 runsだけ（30/28件）。recoveryは2m支持0。
固定58件での全支持平均ADEは0.040203m。main160件の0.025181mと母数・距離支持が異なる。

## 成分・距離：同じ58件でも遠方ほどdyが増える

dx/dyはprediction minus teacher。X前方/Y左方、単位m。
全距離総合はanchorごとの点平均→anchor平均。
RMSEはanchorごとの平均二乗誤差をanchor平均した平方根。点数による重み付けではない。

| 固定58件の距離 | dx MAE | dy MAE | dx RMSE | dy RMSE | Euclidean点誤差 |
|---|---:|---:|---:|---:|---:|
| 0.5m | 0.005587 | 0.008910 | 0.006819 | 0.011286 | 0.011448 |
| 1.0m | 0.009521 | 0.025564 | 0.011596 | 0.033415 | 0.028954 |
| 1.5m | 0.007587 | 0.054452 | 0.009570 | 0.069495 | 0.055808 |
| 2.0m | 0.014833 | 0.100534 | 0.022473 | 0.126831 | 0.102357 |

集合の入れ替わりだけではなく、同じ58件の中でも遠方誤差が増えている。
ただしこの58件の結論をrecoveryや支持のないtailへ拡張しない。
float32保存値の集計順序による末尾の丸め差はあり、性能差として扱わない。

2mのdy符号付き平均は全58件では−0.007595mだが、shapeで分けると次の通り。

| 固定2m支持shape | N | 2m dx bias | 2m dy bias | 2m dy MAE |
|---|---:|---:|---:|---:|
| straight | 16 | −0.007664 | −0.020280 | 0.027012 |
| left | 20 | +0.002695 | −0.137871 | 0.137871 |
| right | 22 | +0.020039 | +0.120063 | 0.120063 |

leftの負dy、rightの正dyは、保存教師の曲がる側への横変位が予測で小さい特徴と整合する。
左右を混ぜると符号が相殺され、全体biasだけでは誤差を過小に見せる。
これを学習原因や車両のアンダーステアと同一視しない。

## 直線：横偏りだけでなく点間形状も非一様

元straight集合はtrain21件/val59件。anchor平均dy biasはtrain−0.006880m、val−0.009342m。
固定2mのstraight集合はtrain9件/val16件で、dy biasは−0.009829/−0.009758m。
train/val両方の保存出力に同じ符号の偏りがあるが、その原因は未同定。

固定val straight16件のdy biasは0.5mで+0.006023、1mで−0.013217、
1.5mで−0.016097、2mで−0.020280m。単一の一定横オフセットだけでは距離変化を表せない。
この観測を根拠にbias補正を加えてはいない。

点間隔は名目0.1mではなく、予測XYの隣接点間Euclidean距離を実測。
固定straightでは0.7→0.8m区間がtrain平均0.11446m / val平均0.11771m、
次の0.8→0.9m区間が0.08641 / 0.08766m。
この伸び縮みはtrain/val両方に見られ、単に経路全体を一様に短く出す形ではない。
val mainの教師支持内の全点間隔（原点→先頭を除く）はmin0.06940 / median0.09988 / max0.13816m。
各点の値と母数は`point_spacing.csv`とsummaryのspacingに保存。

固定2m支持集合の平均折線長（原点→先頭を含む）は以下。

| shape | 教師長m | 予測長m | 予測−教師m |
|---|---:|---:|---:|
| straight | 1.999994 | 2.001561 | +0.001566 |
| left | 1.999830 | 1.996570 | −0.003260 |
| right | 1.999823 | 2.019051 | +0.019228 |

長さがほぼ2mでも横位置や方向は合わない例がある。折線長一致を経路正当性にしない。

## 固定0.3m窓の接線方向proxy

grid j→j+3の弦方向を比較。名目窓は0.3mだが実弦長は別に保存した。
4点すべてに教師支持が必要。窓は0.1→0.4mから1.7→2mの17個、原点窓なし。
予測/教師の弦長<=1e-12mは数値的方向不定としてUNKNOWN。滑らかな曲線へのfitはしない。
これは有限窓の方向proxyで、微分接線や車体headingではない。

| 固定2m支持shape | N | 1.7→2m窓の符号付き方向差rad | 方向差MAE rad |
|---|---:|---:|---:|
| straight | 16 | −0.043511 | 0.049619 |
| left | 20 | −0.151511 | 0.151511 |
| right | 22 | +0.139454 | 0.139582 |

leftは同じ20件で0.7→1m窓のbias−0.066572から遠方−0.151511radへ増える。
rightは窓ごとに一様ではなく、1.2→1.5mのMAE0.046304から最終窓0.139582radへ増える。
straightにも最終窓の方向差が残るため、横平行移動のみの形状ではない。
窓は重複しており、17窓を独立標本17倍として扱わない。signed wrapped平均とcircular平均を別記録。

観察20件は別扱い。dx RMSE0.209317mと大きいが、主集合や直線固定16件と混ぜない。
停止・反転等の元の保留理由の原因説明を今回のXYだけで補作しない。

## 次の無制御shadowで記録すべき量（接続自体は未実施）

1. **固定版と生出力**：model/config/input contract hash、frame名、出力基準時刻、
   未補正の20点XY、名目s-grid、有限性、全点の隣接距離・累積折線長、0.3m弦方向/弦長。
   teacher maskはoffline支持情報で、runtimeの有効性やmotion permissionとして流用しない。
2. **時系列を結ぶ情報**：frame sequence、sensor取得時刻、推論開始/終了・出力時刻、
   欠落/重複/遅延、history IDs・padding mask・各入力age、command履歴の過去性/出典。
   名目0.1mを0.1秒と解釈しない。
3. **座標と変換の証拠**：基準frameのpose/時刻と実際に使用した変換、ego速度/yaw-rate、
   センサ外部パラメータの版。base_linkと後輪軸の一致を仮定せず、その定義と未確認を記録。
   連続予測を比較するときは同じframeへ変換した根拠を残し、異なるframeのXYを直接引かない。
4. **offlineで後から結合する量**：対応時刻を明示した観測future・teacher支持mask/切断理由、
   dx/dyのbias/MAE/RMSE、shape/run別、距離依存母数と固定支持集合別の指標。
   未来情報はモデル入力へ渡さず、記録・後処理専用とする。
5. **無制御の証拠**：shadow出力が操舵・速度・制御指令に接続されていない構成、
   publisher/subscriber・権限/経路の記録。既存Safety等を変更せず、shadow出力で動作許可しない。
   状態監視の記録と制御権限を分離し、現runtimeを縦横MPC完成済みとは扱わない。

今後これらを測るには別の承認と実装確認が必要。今回の作業ではshadow接続、controller/ROS変更、
走行、追加記録・新規収集を開始していない。shadowの無制御性だけで物理安全を保証しない。

## 成果物・検証・止めどころ

成果物root：`E:/workspace/e2e_lite_transfuser/tmp/spatial_fixed_residuals_9b01b1e`

- `summary.json`：role×変動/固定集合×distance/shape/run/normal-recovery/slice、成分/窓/点間隔。
- `per_point_residuals.csv`：全244×20点、mask外はnull。
- `per_anchor_residuals.csv`：244件の成分指標と実折線長。
- `tangent_windows.csv`：全244×17窓、定義不能理由・弦長・方向差。
- `point_spacing.csv`：全244×20区間、原点→先頭を明示。
- `ade_reconciliation.json`、`provenance.json`：照合結果、入力hash、版、禁止操作0。
- `source/`：実行時そのままのコード・純粋配列tests・方法文書。
- `figures/`：成分profileと4代表例。図選択は既存ADEのrole/shape内最悪例で、評価集合変更ではない。
- 生tests/stdout/stderrを同rootへ収録。標準unittest **10 tests OK（0.041s）**。

実行コマンドは`docs/spatial_path_v4_fixed_residual_method.md`。Windows環境にpytestがないため
追加installせず、標準unittestだけを使用。full pytest/モデルテストは実施していない。
入力は既存packetのmanifestでhash照合し、処理後も6ファイルのbytes不変を確認。
今回の照合は実装者の自己点検で、独立監査ではない。

原因判定は**UNKNOWN**で終了する。距離・成分・点間形状の特徴と、次のshadowに必要な
記録項目を明確にしたため、この範囲で停止。本学習、補正、平滑化、追加推論、接続、pushなし。
