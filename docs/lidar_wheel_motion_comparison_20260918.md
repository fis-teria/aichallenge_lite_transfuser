# コーナー入口のLiDAR scan間移動と車輪オドメトリの比較

実行中の制御を変更しない保存データの診断。地図照合の成否と、scan間の相対移動を分離する。
LiDAR側は単位姿勢からの対称point-to-line ICP。地図、GNSS/IMU、記録済み自己位置、車輪の罰則項を使わない。
壁面の法線は近傍最大9点のPCAで推定する。片方のscanだけを正しい壁面と仮定せず、双方向の残差を同時に最小化する。
車輪を初期値にした再計算、逆方向の照合、幾何の弱方向も独立性・安定性の確認用に記録する。
LiDARを真値とは扱わず、条件の弱い区間も除外せず報告する。
`base_link -> lidar` の1.65 m前方の取付位置を考慮して両方を前時刻の車両座標へ変換する。
位置はm、角度はrad、時刻はシミュレーション秒。

## 再現

Windowsから通常手順で同一commitをnative WSLへ同期した後:

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q tests/test_lidar_wheel_motion_diagnostic.py
tools/with_wsl_training_lock.sh .venv/bin/python tools/diagnose_lidar_wheel_motion.py \
  --run /home/thistle/e2e_autonomous/runs/time_teacher_si26_20260911/laps03/5kmh_run01 \
  --records /home/thistle/e2e_autonomous/runs/lidar_map_runtime_20260918/replay05/records.jsonl \
  --start 35 --end 50 \
  --output /home/thistle/e2e_autonomous/runs/lidar_wheel_motion_20260918/new_compare
```

出力先は新規directoryとし、raw scan配列はWSLのruns内に保持する。
約0.2秒間隔に間引かれた連続scanの同一stampで比較する。元センサの全フレーム間隔ではない。
既存replayの車輪積分結果を使い、GNSS/IMU/既存poseを追加で読まない。

## 判定の限界

`diagnostic_consistent` は、対応点率70%以上、100点以上、正規化情報行列の最小/最大固有値比0.001超、
順逆の往復差および初期値変更の差が0.03 m/0.01 rad未満という診断上の条件。
安全性や統計的信頼区間を意味しない。5 mの角度スケールで固有値を比較する。
完全な直線は前後位置を拘束せず、残差が小さくても移動量の真値にはならない。
動的物体、遮蔽、実機のscan内運動歪みの一般的な除去は今回の診断の対象外。

## 結果

runtimeや制御設定を変更しない診断として実施。診断コードcommitは `d77b7696aa8c001771800a229c459f9f6791a7c4`。
対象は既存 `5kmh_run01` のシミュレーション時刻35〜50秒、73区間。
形状の弱さで1区間が条件外、72区間が診断上の整合条件を満たした。
全73区間の平面移動量差は中央値0.0302 m、95 percentile 0.0646 m、最大0.1141 m。
向きの変化量差の絶対値は中央値0.0707°、95 percentile 0.4678°、最大0.7583°。
これらは真値に対する精度ではなく、2推定値間の差である。

|比較対象|車輪|LiDAR|差・判定|
|---|---|---|---|
|45.589→45.789秒の前方移動|0.1884 m|0.1343 m|前方差−0.0541 m、左右も含む平面差0.0590 m|
|同区間の向き変化|−0.7426°|−0.2903°|差+0.4523°|
|45.589→46.590秒の直接照合での前方移動|0.9276 m|0.9203 m|平面差0.00731 m、向き変化の差0.1866°|
|同1秒を約0.2秒×5回のLiDAR推定で積算|—|—|車輪との平面差0.0334 m、向き差0.0969°|

1秒の直接照合の初期値は、LiDARだけの約0.2秒推定を5回積算した値。車輪の値で初期化していない。
順逆の差は小さいが、対称目的関数の性質も含むため独立した正解証明とは扱わない。
他の1秒区間（42.582→43.587、44.588→45.589、48.839→49.839秒）でも直接照合を追加し、
平面差はそれぞれ0.0240、0.00609、0.0322 mだった。

### 地図照合への予測差し替え

最後に採用された45.589秒の地図姿勢を共通とし、次の0.2秒の移動予測だけを変更した。
同じ地図・同じscan・同じ照合条件で再計算し、元の車輪側の数値も再現した。

|予測に使う移動|通常照合の補正要求|範囲拡大した再照合|結果|
|---|---|---|---|
|車輪|0.8682 m|1.0412 m|いずれも `CORRECTION_LIMIT`|
|LiDAR|0.7766 m|0.9548 m|いずれも `CORRECTION_LIMIT`|

通常上限0.45 m、再照合上限0.90 mをどちらも超えた。
**今回の失敗を、その0.2秒間の車輪移動誤差だけでは説明しにくい。地図との対応付け・形状不一致・既に存在する地図姿勢誤差を優先して調べる。**
道路境界と実測壁が違うことだけに原因を断定しない。車輪の長期ドリフトがないとも言えない。
35.092→45.789秒の単純積算ではLiDARと車輪の位置差が約0.182 m、向き差約0.406°あった。
これは形状条件の弱い1区間も含む診断値で、絶対位置の真値ではない。

### 計算の妥当性確認

- 既知の平面移動、取付位置の回転成分、直線の退化、NaN/inf、欠損、cm単位ノイズの単体試験6件成功。
- 全体 `pytest -q`: **3036 passed, 4 skipped, 84 warnings**（114.63秒）。skipは既存の任意依存不足。
- 初期の2ビーム法線版は0/73、片方向の平滑壁面版は3/73しか整合条件を満たさず、比較の根拠には採用していない。
  元結果はnative WSLの `compare01` / `compare02` に保全。
- 最終版は近傍壁面と対称残差を使用。LiDARと車輪を一致させる罰則項や、車輪との差による選別はない。

[比較図](evidence/lidar_wheel_motion_20260918/motion_comparison.png)、
[集計](evidence/lidar_wheel_motion_20260918/metrics.json)、
[1秒比較・予測差し替え](evidence/lidar_wheel_motion_20260918/crosscheck.json)を保存。
最終の実測出力はnative WSLの `lidar_wheel_motion_20260918/compare03`。
追加診断は同directoryの `crosscheck.py`、図の作成は `plot_comparison.py` に実行時のスクリプトを保存。
前者はWSLの `compare03` directory、後者はスクリプト隣のJSONとWindowsのMeiryo fontを使う。
raw scanはGitへ追加せず、元bag・抽出scanはnative WSLで保持する。
