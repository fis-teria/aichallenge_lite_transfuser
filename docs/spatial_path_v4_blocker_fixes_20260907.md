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
