# V4 blocker修正：固定出力offline検証

ユーザーの参照生成・診断学習・監視の問題修正依頼に対応する。
追加走行枠は0。AWSIM変更、追加sim、制御publish、自動pushは行わない。
既存データ全面の教師採用・学習予算は未確定であり、固定checkpointを自動置換しない。

## 今回の実装修正

1. 固定順序の折れ線偏差計算をunion-breakpoint証明へ修正。
   両折れ線の全折れ点の和集合で分割すると差ベクトルは区間内でアフィン。
   そのEuclidean normは凸なので最大値は端点にある。旧max-spacing加算は不要。
   10cm制限、元20点、prefix選択、7knots、舵角/rate/横加速度制限は不変。
   数値余裕1e-9mを保持。走行曲線そのもの、teacher精度、free-spaceの証明ではない。
2. 静的AABB監視の空footprint、NaN/逆転bounds、文字列等の動的監視flagを拒否。
   観測範囲外をfreeへ変更しない。実sceneの三角形分類/動的物体監視は未完成のまま。
3. 保存済みstable46出力だけを同じ初期状態・設定で一度ずつ再fit。
   元hash照合、独立折れ線証明、dx/dy診断、旧結果を別保存。
   推論・学習・MPC・sensor/Dataset/raw/checkpoint読取は行わない。

Windows commit後に既定CheckOnly/通常同期、WSL worktree lockで以下を実施。
同期scriptは固定Dataset rootのtest -dのみであることを静的確認し、既定動作を変更しない。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_blocker_fixes_v4.py tests/test_spatial_sim_e2e_v4.py tests/test_spatial_run_continuation_v4.py
bash tools/with_wsl_training_lock.sh .venv/bin/python tools/replay_spatial_reference_fixes_v4.py --input /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/inputs --output /home/thistle/e2e_autonomous/spatial_blocker_fixes_20260907/replay
```

inputはWindowsの既存attemptからworker.jsonlとresolved_config.yamlだけを専有folderへコピーする。
固定出力46件の追加fit（MPCではない）は旧実験のfit数へ混ぜず別計上する。
全pytestは学習/データaccess testを避け未実施。限定合成と固定出力replayを用いる。
受理増加を合格条件にしない。まだ10cm超なら拒否を保持する。
初回46fitでSLSQP成功だが最終boundが0.100000000000002mとなる境界丸め拒否を確認。
optimizer側の余裕をfeasibility tolerance 1e-8より大きい1e-7mにし、最終証明余裕1e-9mと分離。
これを修正後の追加46fitで一度確認する（合計92fit、学習/MPC/推論0）。10cm基準は不変。

## 実行結果

開始版 `fdcdaa3b6144694b6e2c2254f5e00c1eafc38a8b`、Windows clean。
origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`、branch `codex/windows-wsl-training-sync`。
最終実装・限定test・offline replay版 `25a2b388dcae65f625a2a4b9d7300a1db03a5609`。
初回CLIはsource import不足で停止（fit0）、standalone importを修正して実行。
旧出力・旧結果・実sim累計予算は上書き/減額していない。

| 項目 | 結果 |
|---|---|
| 最終限定pytest | 68 passed / 7.01秒 |
| 固定46件の旧受理数 | 0 |
| 同じ10cm上限での修正後幾何受理 | 5 |
| 修正後も拒否 | 41（BOUNDED_FIT_INFEASIBLE） |
| 修正後の証明偏差上界 | 0.099999901～0.174115316m |
| 各出力の最大abs dx範囲 | 0.084245518～0.167747136m |
| 各出力の最大abs dy範囲 | 0.052705802～0.087153191m |
| fit wall時間 | 0.007912～0.131060秒/件（WSL offline実測） |

全46件で最大abs dxが最大abs dyより大きい。
これは共通の固定順序パラメータ上の偏差であり、最短幾何距離やteacher誤差ではない。
予測点のジグザグによる折線長と車両が追える曲線の進み方の違いが候補だが、
学習不足・不正teacher・全ての参照曲線の不存在を断定しない。
受理5件も上限近傍であり、十分な余裕や実走行成功は主張しない。
最終solverの内部stepでbounds clippingのSciPy警告1件を記録。最終制約チェックは省略していない。

固定出力replayは2回×46=92 fit。最初の回は境界丸めにより全46拒否、最終回で5受理。
純粋合成test内のsolver実行は別であり、この92へ混ぜない。
新推論0、checkpoint読取0、学習0、MPC0、sim起動0、制御publish0。
修正コードはWindows commit済み、WSL同SHA検証済み。simulator hostへ未配置、pushなし。

保存: `tmp/spatial_blocker_fixes_20260907/replay.json`（初回）と`replay_final.json`（最終）、
`evidence/tests_final.xml`、`evidence/final_validation.log`、同期/初回失敗ログ。
replayは元worker/configのhashと全46出力hashを記録。独立関数で全折れ点の最大偏差を再照合。

**既定同期によるDatasetルート存在確認を実施。Dataset内容・raw・sensor・checkpoint読取は未実施。**
既存教師の情報は診断結果文書と小さなcontract/configだけを参照した。

## 残るblockerと必要な別判断

### 参照生成

41件は依然拒否。10cm制限や元20点を変えて受理を作っていない。
固定弧長対応そのものの見直しと学習出力形状の改善は異なる変更である。
新しい対応付け・prefix短縮・平滑化を無断で追加せず、どの契約を採るか別途設計する。

### 学習

`configs/spatial_diagnostic_v4.yaml`は意図的な64件/500step/scratchの診断設定。
64件制限を外すだけの変更は行わない。現在teacher tierは
`OBSERVED_DIAGNOSTIC_ONLY`、rear_axle_alignmentはUNKNOWN、物理採用は未承認。
現在データの適格範囲、run split、mask/遠方支持とruntimeの欠損履歴条件を確認してから
予算付き限定追加学習を設計する必要がある。
前段のDataset内容・checkpoint読取/学習禁止を解除する明示と、使用教師範囲・計算上限が必要。
今回の修正依頼を、正式teacher採用や無制限学習・runtime昇格の承認とは扱わない。

### 周辺監視

不正入力をfreeにする実装欠陥は修正したが、現在sceneの障害物三角形の路面/壁分類、
動的actorとの結合、側面/後方の現在空間と独立collision/departure監視は未成立。
1枚の前方scanだけから未観測領域をfreeと認める変更はしない。
未改変AWSIMの既存インターフェースで取得できる証拠と承認済み監視方式を確定する必要がある。

従って**V4全体のblock解除は未完了**。次の追加走行は今回実施せず、別の予算承認を要する。
