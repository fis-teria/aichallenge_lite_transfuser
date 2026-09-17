# 予測曲率と予測範囲に応じた速度制御

コーナー手前で減速し、出口で再加速する`curvature_preview_15kmh_v1`を追加する。15km/hは巡航上限であり、固定速度ではない。同一checkpoint、生の30点予測、PPの操舵・実速度での先読み条件、操舵応答補償、前回の最大1m停止監視を維持した比較とする。

## 制御の内容

- 年齢・姿勢を合わせた予測経路から、前後各0.5m以上離れた元の3点で円の曲率を測る。短い区間の向きや補間による曲率過大評価を避け、元の予測点・操舵経路は変更しない。
- 曲率ごとの速度上限は横加速度1.0m/s²を目安に設定する。各カーブへの距離から、減速度0.7m/s²でその速度へ落とせる現在速度の上限を求める。現在位置の経路上への投影、実速度で0.5秒進む距離、追加0.5mの余裕を差し引く。
- PPが今追う曲率にも同じ横加速度上限を適用する。
- 予測終端の距離からPPの必要先読み式を逆算し、追加0.5m・0.3秒の余裕を取った速度上限を設ける。先読み不足で拒否される前の減速を狙う。既存の実速度による先読み・拒否条件を短縮しない。
- 速度誤差に対する比例ゲインは2.0/s。加速度**指令**上限は発進時+1.0m/s²、実速度0.5〜1.0m/sで連続的に+0.8m/s²へ下げる。制動は従来どおり−1.0m/s²。出口では現在の速度上限へ加速する。急カーブでも強制的に速度を維持する下限は設けない。状態を持たず、各指令を記録入力だけから再現できる。

モデルの再学習は行わない。1mの停止監視は従来指定の診断用設定であり、実速度での停止距離や無接触を保証する設定ではない。速度計画だけで壁回避は保証できず、予測経路・操舵追従・監視余裕を含めてAWSIMで確認する。AWSIM本体は変更しない。

## 検証手順

1. Windowsでcommitし、native WSLへ同一commitを同期する。
2. 共有lock内で全pytest、既存10km/hログの制御再生、47入力の学習/runtime予測一致を確認する。新規テストは解析解のある円弧、先行カーブの制動距離、先読み不足前の減速、再加速、無効入力、PP条件保持、再生時の改ざん検出を含む。
3. 旧15km/hの記録状態に新しい速度計画を適用する。これは反実仮想の要求値比較であり、改善した実走の証拠にはしない。
4. `graneple@192.168.3.10`に独立deploymentを作り、隔離ROSで通常経路・減速指令・各監視の接続を確認する。
5. 最大1周・600秒のAWSIM比較。通常RVizに生のE2E経路を表示し、動画・指令・Judge判定を保存する。元のAWSIM全ファイルの前後ハッシュと既存環境の保全を確認する。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_curvature_launch_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
Get-Content -Raw tmp/time_curvature_launch_20260917/offline_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -'
python tmp/time_curvature_launch_20260917/manage.py package
python tmp/time_curvature_launch_20260917/manage.py prepare
python tmp/time_curvature_launch_20260917/manage.py start
python tmp/time_curvature_launch_20260917/monitor.py
python tmp/time_curvature_launch_20260917/finish_and_evaluate.py
```

初回試験（source `14edb2e`、`codex-time-curve15-lap01`）は加速度指令上限0.4m/s²で実施し、発進後ほぼ進まず停滞停止した。正の指令32件に対し最高実速度0.073km/h、途中から先読み拒否72件。速度目標は約1m/sあり、目標速度がゼロに制限されたことによる停止ではない。これは指令加速度と車両の実加速度を同一視できないことを示す。発進上限を実績のある1.0m/s²へ戻し、走行時0.8m/s²へ連続的に下げる修正を加えて、別deployment・runで再試験する。旧記録は保全する。

発進修正後（source `704af34`、`codex-time-curve15-lap02`）は約203.40m、section 4まで走行。先読み不足は0件に減ったが、前方1mの監視で点群2点が領域に約9.3mm入り停止した。実測速度は約7.49km/h、物理接触は未確認。横の固定余裕20cmを5cmに変更したオフライン比較では、この同じ点群の侵入は0点になった。

その後、ユーザーが「試験的に停止監視領域をなし」にするよう明示したため、横余裕変更の実装・実走は見送り、`scan_occupancy_policy=log_only_awsim_v1`による比較を行う。曲率速度計画と上限15km/hはそのまま。停止領域は計算・記録するが、点群侵入からの制動・停止ラッチを行わない。センサ鮮度、NaN、速度超過、姿勢、経路・PPの妥当性監視、1周後の停止、停滞・時間上限は維持する。新configは従来からこの1項目だけ変更し、AWSIMの指定ホスト・明示診断設定・1周に限定する。AWSIM本体を変更しない。

記録専用試験では`SCAN_GUARD_OBSERVED / LOG_ONLY_CONTINUED`に最初の点群と状態を保存し、その後も各指令へ通常なら停止するかを記録する。隔離ROSでは合成障害物があっても正の指令を継続し、古いplan・止まったclock・速度超過では制動することを確認する。

記録専用試験のコマンドは上記のoperatorディレクトリを`tmp/time_curvature_logonly_20260917`に置き換える。configは`configs/control/time_path_curvature_logonly_20260917.json`、run IDは`codex-time-curve15-logonly-lap03`。最終結果は取得後に追記する。
