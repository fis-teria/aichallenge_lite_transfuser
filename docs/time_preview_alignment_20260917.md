# 時間軌道とPure Pursuit先読みの整合

3秒先までの時間軌道に対し、操舵目標の距離にも停止距離の二乗項を要求していたため、高速になるほど予測範囲と制御要求が離れていた。操舵用の先読みと障害物監視の停止距離を分離する。モデル・教師・生の予測経路・AWSIM本体は変更しない。

**実装・WSL検証・AWSIM比較を完了。15km/h上限、20km/h上限とも1周完走し、先読み不足は0件だった。** 前回の同じ曲率速度計画・停止領域記録専用条件から、15km/h上限のラップは145.98秒から120.53秒へ短縮した。

| 条件 | 完走 | Judgeラップ | 実速度中央値 | 最高実速度 | 先読み拒否 | 停止領域侵入候補の指令数 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 修正前・上限15km/h | 1周 | 145.98秒 | 8.99km/h | 10.71km/h | 0 | 0 |
| 修正後・上限15km/h | 1周 | 120.53秒 | 10.28km/h | 14.84km/h | 0 | 0 |
| 修正後・上限20km/h | 1周 | 120.25秒 | 10.37km/h | 15.52km/h | 0 | 3 |

全て同じcheckpoint・同じシーンで各1回の比較。中央値は走行許可から完走判定までの実速度0.1m/s超の指令標本で計算した。15→20km/h上限によるラップ差は0.28秒にとどまり、反復試験で高速化を証明した値ではない。20km/h上限では実際の速度指令も最大16.40km/hであり、20km/h走行を達成した結果ではない。

## 実装

新しい`velocity_time_preview_v1`は、実車速v [m/s]に対し`max(1m, 0.4m + 1.5s × v)`をPPの最低先読み距離とする。1.5秒は今回比較する初期設定であり、最適値を同定したという意味ではない。元の年齢・姿勢補正済み経路上から、従来と同じ頂点優先・区間補間・操舵角制限で目標を選ぶ。探索幅0.5m、選択不能時の最大1m拡張も維持する。

終端による速度制限も同じ関数の逆算に統一する。`先読み(v) + 0.5m + 0.3s × v <= 補正後の予測終端距離`を満たす速度までとし、最低1mの分岐も計算する。車速を偽って先読みを短縮したり、予測を外挿したりしない。現在の実車速で必要な目標が取れなければ拒否する。

| 実車速 | 旧PP最低距離 | 新PP最低距離 | 新速度計画が求める予測距離 | 直進・等速3秒の距離 |
| --- | ---: | ---: | ---: | ---: |
| 10km/h | 5.65m | 4.57m | 5.90m | 8.33m |
| 15km/h | 11.16m | 6.65m | 8.40m | 12.50m |
| 20km/h | 18.61m | 8.73m | 10.90m | 16.67m |

カーブへの制動距離から決める速度制限、横加速度目安1m/s²、加速度指令−1〜+1m/s²、速度ゲイン4/sは維持する。障害物監視側の物理停止距離は従来の二乗式を保持する。今回のAWSIM条件は前回のユーザー指定を継承し、1m領域の侵入を記録のみとする。センサ鮮度・NaN・姿勢・運動・速度超過監視は継続する。

旧`curvature_preview_15kmh_v1`と固定速度条件の挙動は保持し、新しい`curvature_time_preview_15kmh_v1`と`curvature_time_preview_20kmh_v1`を明示的な設定で選ぶ。20km/hの車両応答係数は既存近似の外挿であり、高速域の実測校正ではない。指定ホスト・指定シーン・1周・有限時間のAWSIM試験に限定する。

## 検証・実行

解析解のある等速直線・左右円弧で先読み、速度上限、操舵制限を検証する。予測の経過時間0〜0.5秒、短い予測、NaN、速度超過、記録改ざん、独立した停止距離も検証する。全pytestと旧ログの再生はnative WSLの共有lock内で実行する。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_preview15_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
python tmp/time_preview15_20260917/manage.py package
python tmp/time_preview15_20260917/manage.py prepare
python tmp/time_preview15_20260917/manage.py start
python tmp/time_preview15_20260917/monitor.py
python tmp/time_preview15_20260917/finish_and_evaluate.py
python tmp/time_preview15_20260917/analyze_trial.py
python tmp/time_preview15_20260917/pack_evidence.py
```

保存済み入力による事前比較は同じ共有lock内で`tmp/time_preview15_20260917/audit_native.py`を実行した。15km/h完走後、上記operatorディレクトリを`tmp/time_preview20_20260917`に替えて20km/hの準備・配備・走行・評価を実行した。20側の`prepare_native.py`は15側の完走・停止確認・転送検証とsource一致を要求し、同じsourceで完了した全pytestと47入力一致を再利用する。20km/h専用のROS試験は別に実行した。停止領域候補の確認には、共有lock内で`tmp/time_preview20_20260917/proximity_native.py`も実行した。

これらは実行したコマンドの記録であり、operatorは既存出力への上書きを拒否する。再試験時は新しい出力ディレクトリとrun IDを割り当てる。operatorのコピーは各evidence配下に保存した。

## 検証結果

- 検証・配備したsource: `179cbe1e6134534943628939b363eb056a3d95b8`。
- checkpoint: `time_launch_protection_v2_20260916/launch_balanced/epoch_03.pt`。SHA-256: `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。再学習なし。
- WSL全pytest: **2908 passed, 4 skipped**。環境依存の既存スキップ4件。学習/runtimeの47入力が完全一致。
- 旧10km/h制御再生PASS。修正前の15km/h上限の3098指令も一致し、旧条件の再現を維持した。
- 同じ保存済み状態を新制御へ渡す比較では、3089件を受理、既存の初期方向拒否9件。目標速度差の中央値は+1.44km/h、操舵タイヤ角差の95パーセンタイルは0.00627rad。これは反実仮想の計算であり、実走結果とは区別する。
- 両速度域で公式環境の隔離ROS試験PASS。20km/hで最低先読み約8.733mを確認。合成障害物の侵入を記録しながら正の指令を継続し、古いplan、止まったclock、速度超過では制動した。
- 15km/h実走の制御再生は2570指令一致、planが古かった3指令は再生対象外。最大誤差1.60e-13。20km/hは2569指令一致、再生対象外0、最大誤差1.47e-13。これは経路・PP・速度計画・操舵応答等の再生で、全指令の生点群再生ではない。
- 20km/hでは追従2540指令中、速度制限の理由は先の曲率1384件、予測距離1154件、追従曲率2件。巡航上限20km/hによる制限は0件だった。
- 両試験で通常RVizに生のE2E予測を表示し、AWSIMとRVizを録画。全フレームのデコードと転送前後のSHA-256一致を確認した。
- 各試験の前後でAWSIM全1089ファイルのハッシュ一致。リモートの既存Git差分・RViz設定・過去コンテナを保全し、今回の試験コンテナは終了済み。

15km/hのrun IDは`codex-time-timepreview15-lap01`、20km/hは`codex-time-timepreview20-lap01`。実行先は`graneple@192.168.3.10`。各runは最大1周・600秒、正式なモデル昇格は行っていない。

## 20km/h上限試験の停止領域候補

走行許可から71.10〜71.21秒、右コーナーで3指令に侵入候補を記録した。実速度は7.74〜7.55km/hで、最初の監視余裕は−0.0284m、点群4点が余裕を含む監視領域に入った。最初の点群・姿勢・運動から同じ検出を再現した。前回203m付近で停止した地点の近傍である。

記録専用設定により正の指令を継続し、そのまま完走した。前後の抽出映像では壁との間に空間が見えている。ただし、監視余裕は実際の車体と壁の距離ではなく、物理接触センサによる無接触証明も行っていない。

[候補3指令と動画時刻](evidence/time_preview20_20260917/evaluation/proximity_observations.json)、[領域再生](evidence/time_preview20_20260917/stop_location/diagnosis.json)、[通過時の画像](evidence/time_preview20_20260917/evaluation/proximity_during.png)を保存した。

## 記録と残る点

[15km/h evidence](evidence/time_preview15_20260917/manifest.json)、[20km/h evidence](evidence/time_preview20_20260917/manifest.json)、[実測比較](evidence/time_preview20_20260917/evaluation/comparison.json)、[速度グラフ](evidence/time_preview20_20260917/evaluation/speed_comparison.png)、[時間と距離の関係](evidence/time_preview15_20260917/offline_preview/coverage_scaling.png)。

動画のWindowsコピーは`tmp/time_preview15_20260917/videos/`と`tmp/time_preview20_20260917/videos/`の`awsim.mp4`、`rviz.mp4`。原本はWSLの`/home/thistle/e2e_autonomous/runs/time_preview15_20260917`、`time_preview20_20260917`、およびリモートの`/home/graneple/e2e_autonomous/`直下の同名ディレクトリに保持する。重み・raw記録・動画はGitへ追加していない。

時間軌道とPPの二乗距離要求の不整合は、新しい明示設定では解消した。一方、20km/hの実走達成には至っておらず、予測範囲と曲率による速度制限が残る。学習データ不足とはまだ断定しない。1.5秒の先読み設定の反復検証、高速域の車両応答、余裕を含む監視の妥当性は今後の検討点とする。
