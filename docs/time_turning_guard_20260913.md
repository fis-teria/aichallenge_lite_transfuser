# TimePath B0 旋回に合わせた前方監視

ユーザー依頼「曲がり角でも旋回できるようにしてほしいです」。
前回 [lap06/lap07記録](time_lap_20260913.md) の診断を踏まえた新しい修正・試験判断。
目標5km/h、Pure Pursuit、生30点のTimePath B0、通常RViz表示、実行先
`graneple@192.168.3.10` を継続。成功条件はまず以前のコーナー入口を通過し、
引き続き同じ周回judgeと停車確認で1周を目指すこと。衝突/監視拒否なら終了して診断する。

編集前 `76b3ae7c4417c132cd7ef66c053ea70d8e7ad1ed`、Windows clean。
ホストの既存差分、停止済みcontainerと39個の履歴Composeは保全。
前回と同じ直線監視の再試行は行わない。ソース・テスト・WSL評価の正本はWindows。

## 変更が必要な理由と範囲

最初の阻害要因は、旋回する指令に対して直線の停止矩形を使用していたこと。
しきい値緩和では曲がる時の障害物位置を表現できないため、監視の幾何を変更する。
所有層はAWSIM試験用の前方近接監視と、その制御ノードへの接続。
推論・教師・PPの目標点・重み・目標速度・操舵/加速度制限は変更しない。
旧監視は `straight_v1` として保存し、新規設定の `steering_sweep_v1` のみで有効化する。

- 実操舵、前回送信操舵、今回のrate limit適用後の送信操舵を含む曲率区間を使う。
  その区間内で操舵が変化する場合の位置・姿勢誤差上界で、旋回する車体矩形を膨張する。
- 停止移動距離は従来の `0.4 + v*0.5 + v^2/(2*1.0)` mを維持。
  車体余裕は従来の左右±0.85m、vehicle root前端+1.5m（rear axle前端約1.984m）、
  後輪から後ろ0.510m。後方寸法は既存 `spatial_sim_e2e_v4.yaml` を参照。
- 25mm以下の移動刻みと補間移動量の膨張でsample間を覆う。scanの角度間隔も膨張へ入れる。
- LiDARの元capture姿勢から現在rear axleへ変換。base_link→LiDAR+1.65m、
  base_link→rear axle約+0.001m。時刻・frame不一致や変換不足は拒否。
- 各rayと旋回矩形の区間を比較し、矩形の奥まで観測が届かない場合も拒否する。
  NaN/-inf/範囲外は拒否、AWSIMの+infはrange_maxまでのno-returnとして従来同様に扱う。
- 既存のclock/timeout/source/authority/geometry/停止距離/overspeed監視を維持。
  停止時は前回操舵保持、target0・acceleration=-1。重大faultではホストがAWSIMをfreeze終了。

範囲の限界: これは**前方LiDARの観測rayに対する旋回近接監視**であり、
現在の側面・後方など未観測領域をfreeと認定する全周衝突保証ではない。
従来の前方監視と同じAWSIM限定の検証範囲を保ち、`full_body_free_space_verified=false`
を明示する。停止応答0.5s・減速度1m/s²と、操舵が区間内に収まる仮定は
実車に対する校正値ではない。モデルの未来headingを捏造して監視には使わない。

## 検証計画・コマンド

直進/左右旋回、曲がる側の障害物、外側の壁、実操舵が追いついていない場合、
前回指令が残る場合、NaN/時刻/frame/速度上限、曲率が時間変化する軌跡の包絡をunit test。
新旧設定は明示的に分けて切り戻せる。WSL full pytest後、実行物をHumbleでbuildし、
影響した純Python moduleとROS nodeのsource/install hashを確認する。
ROS shadowで実モデルPathと合成制御の負例を通してから、新規run IDを1回消費する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q \
  tests/test_turning_scan_guard.py tests/test_time_trial_v1.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

初回試験専有root `/home/graneple/e2e_autonomous/time_turning_20260913`、
run ID `codex-time-turn-08`、走行600sim/600wall秒・全体720wall秒以内。
通常RVizで `/visualization/time_path/raw_path` を表示する。

```bash
timeout --signal=TERM --kill-after=10s 710s python3 SOURCE/tools/run_time_path_awsim_trial.py \
  --deployment /home/graneple/e2e_autonomous/time_turning_20260913 \
  --run-id codex-time-turn-08 --display :1 \
  --config configs/control/time_path_turning_5kmh_20260913.json </dev/null
```

現在: 実装済み。`1c0cf319cd99c0dc0c40ce051bc64bfea5403fd4` をWindowsから同期し、
native WSLの限定テスト33passed (0.31s)。次は新設定を実際に使うHumble接続smokeと全体テスト。
まだホストへ適用・走行していない。
lap07には実操舵値が保存されていないため、新監視の完全な実測再計算とは称さない。
今回から実操舵と使用した監視区間・元scan姿勢を記録する。
