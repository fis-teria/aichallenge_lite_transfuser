# 予測曲率と予測範囲に応じた速度制御

コーナー手前で減速し、出口で再加速する`curvature_preview_15kmh_v1`を追加した。15km/hは巡航上限であり、固定速度ではない。同一checkpoint、生の30点予測、PPの操舵・実速度での先読み条件、操舵応答補償を維持する。ユーザー指定により、最終AWSIM試験では点群の停止領域への侵入を記録するだけにし、侵入による制動・停止を無効にした。

**最終試験は1周完走。Judgeラップ145.98秒、実速度中央値8.99km/h・最高10.71km/h、先読み不足0件。** 完走後の停止も確認した。今回の「1周完走」という試験条件は満たしたが、同一場面の単発試験であり、反復成功率や物理接触センサによる無接触証明は得ていない。

| 条件 | Judge結果 | 活動中の記録距離 | 先読み拒否指令 |
| --- | --- | ---: | ---: |
| 従来の固定10km/h・停止領域あり | 完走134.74秒 | 370.11m | 0 |
| 従来の固定15km/h・停止領域あり | 停止領域検出で終了 | 315.20m | 327 |
| 曲率減速・弱めた加速応答・停止領域あり | 停止領域検出で終了 | 203.40m | 0 |
| 曲率減速・弱めた加速応答・停止領域は記録のみ | 速度低下後に停滞終了 | 215.14m | 103 |
| **曲率減速・従来の加速応答・停止領域は記録のみ** | **完走145.98秒** | **371.64m** | **0** |

速度中央値は走行許可から完走判定までの実速度0.1m/s超の指令標本。記録距離は同区間の位置列から計算し、Judgeの計時区間と厳密には一致しない。全行は同一checkpoint・同一場面で各1回の実測であり、固定10km/hより速くなった結果ではない。

最終試験では停止領域への侵入候補自体が0件だった。したがって、完走を停止監視無効化だけの効果とは切り分けられない。前回と同じ記録専用設定で加速応答を戻すと、215m地点を通過でき、速度低下→予測距離縮小→PP拒否の連鎖も今回は発生しなかった。

## 制御の内容

- 年齢・姿勢を合わせた予測経路から、前後各0.5m以上離れた元の3点で円の曲率を測る。短い区間の向きや補間による曲率過大評価を避け、元の予測点・操舵経路は変更しない。
- 曲率ごとの速度上限は横加速度1.0m/s²を目安に設定する。各カーブへの距離から、減速度0.7m/s²でその速度へ落とせる現在速度の上限を求める。現在位置の経路上への投影、実速度で0.5秒進む距離、追加0.5mの余裕を差し引く。
- PPが今追う曲率にも同じ横加速度上限を適用する。
- 予測終端の距離からPPの必要先読み式を逆算し、追加0.5m・0.3秒の余裕を取った速度上限を設ける。先読み不足で拒否される前の減速を狙う。既存の実速度による先読み・拒否条件を短縮しない。
- 速度誤差に対する比例ゲインは従来の4.0/s、加速度**指令**範囲も従来の−1.0〜+1.0m/s²を維持する。出口では現在の速度上限へ加速する。急カーブでも強制的に速度を維持する下限は設けない。状態を持たず、各指令を記録入力だけから再現できる。

モデルの再学習は行っていない。記録用に残した1mの領域は従来指定の診断用設定であり、実速度での停止距離や無接触を保証する指標ではない。AWSIM本体は変更していない。

## 検証手順

1. Windowsでcommitし、native WSLへ同一commitを同期する。
2. 共有lock内で全pytest、既存10km/hログの制御再生、47入力の学習/runtime予測一致を確認する。新規テストは解析解のある円弧、先行カーブの制動距離、先読み不足前の減速、再加速、無効入力、PP条件保持、再生時の改ざん検出を含む。
3. 旧15km/hの記録状態に新しい速度計画を適用する。これは反実仮想の要求値比較であり、改善した実走の証拠にはしない。
4. `graneple@192.168.3.10`に独立deploymentを作り、隔離ROSで通常経路・減速指令・各監視の接続を確認する。
5. 最大1周・600秒のAWSIM比較。通常RVizに生のE2E経路を表示し、動画・指令・Judge判定を保存する。元のAWSIM全ファイルの前後ハッシュと既存環境の保全を確認する。

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync
Get-Content -Raw tmp/time_curvature_response_20260917/prepare_native.py | wsl -d Ubuntu-22.04-Recovered -u thistle --exec bash -c 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 .venv/bin/python -'
python tmp/time_curvature_response_20260917/manage.py package
python tmp/time_curvature_response_20260917/manage.py prepare
python tmp/time_curvature_response_20260917/manage.py start
python tmp/time_curvature_response_20260917/monitor.py
python tmp/time_curvature_response_20260917/finish_and_evaluate.py
python tmp/time_curvature_response_20260917/analyze_completed.py
python tmp/time_curvature_response_20260917/pack_evidence.py
```

これは今回実行した一連のコマンド。operatorは既存記録への上書きを拒否するため、再実行時は独立した出力ディレクトリ・run IDを用意する。実行したoperatorのコピーは最終evidence配下に保存した。

初回試験（source `14edb2e`、`codex-time-curve15-lap01`）は加速度指令上限0.4m/s²で実施し、発進後ほぼ進まず停滞停止した。正の指令32件に対し最高実速度0.073km/h、途中から先読み拒否72件。速度目標は約1m/sあり、目標速度がゼロに制限されたことによる停止ではない。これは指令加速度と車両の実加速度を同一視できないことを示す。発進上限を実績のある1.0m/s²へ戻し、走行時0.8m/s²へ連続的に下げる修正を加えて、別deployment・runで再試験する。旧記録は保全する。

発進修正後（source `704af34`、`codex-time-curve15-lap02`）は約203.40m、section 4まで走行。先読み不足は0件に減ったが、前方1mの監視で点群2点が領域に約9.3mm入り停止した。実測速度は約7.49km/h、物理接触は未確認。横の固定余裕20cmを5cmに変更したオフライン比較では、この同じ点群の侵入は0点になった。

その後、ユーザーが「試験的に停止監視領域をなし」にするよう明示したため、横余裕変更の実装・実走は見送り、`scan_occupancy_policy=log_only_awsim_v1`による比較を行う。曲率速度計画と上限15km/hはそのまま。停止領域は計算・記録するが、点群侵入からの制動・停止ラッチを行わない。センサ鮮度、NaN、速度超過、姿勢、経路・PPの妥当性監視、1周後の停止、停滞・時間上限は維持する。新configは従来からこの1項目だけ変更し、AWSIMの指定ホスト・明示診断設定・1周に限定する。AWSIM本体を変更しない。

記録専用試験では`SCAN_GUARD_OBSERVED / LOG_ONLY_CONTINUED`に最初の点群と状態を保存し、その後も各指令へ通常なら停止するかを記録する。隔離ROSでは合成障害物があっても正の指令を継続し、古いplan・止まったclock・速度超過では制動することを確認する。

記録専用の初回（source `1d68712`、run ID `codex-time-curve15-logonly-lap03`）は215.14m、section 4で停滞終了。停止領域検出は0件。直前3秒は正の加速度指令0.11〜0.68m/s²を出していたが、速度が4.12→0.60km/h、予測距離も短くなり、104.29秒で年齢補正後の終端0.9969mがPP最低1mを下回った。その後先読み拒否103指令、停滞停止となった。停止画像ではコース内に壁との空間が見えており、壁接触は確認していない。この試験は停止監視だけを除けば完走する、という結果ではない。

2.0/sのゲインと0.8m/s²の加速上限では、速度低下と予測範囲縮小の連鎖に追従できない場面が観測された。この応答を調整するため、速度上限の計画は維持し、ゲイン4.0/s・指令上限1.0m/s²へ戻して再試験する。これは従来完走時の制御範囲内であり、先読みの最低距離や速度下限の強制緩和は行わない。

## 最終試験の検証と記録

- 実行先: `graneple@192.168.3.10`。run ID: `codex-time-curve15-response-lap04`。
- 検証したsource: `b0eeb56ddac87242bda375df262de58a66e004ae`。config: `configs/control/time_path_curvature_logonly_20260917.json`。
- checkpoint: `launch_balanced/epoch_03.pt`、SHA-256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`。再学習・重み変更なし。
- WSLの全pytest: **2882 passed, 4 skipped**。47入力の学習/runtime予測が完全一致。既存10km/hログの制御再生もPASS。
- 隔離ROS: 合成障害物が停止領域に入っても正の指令を17件継続し、記録専用設定の適用を確認。plan・clockの鮮度異常、速度超過では制動した。
- 実走: 全sectionを順番に通過し、Judgeが1周145.98秒を記録。走行許可から完走判定まで154.90秒、記録距離371.64m。完走後に停止を確認して終了。
- 制御ログの再生: **3098指令一致、再生不能0件、最大誤差3.38e-14**。経路・PP・操舵応答・速度計画の再現を検証した。全指令の生点群を用いた監視再実行ではない。
- 追従指令3081件中、予測距離による速度制限2624件、先の曲率による速度制限457件、計画に基づく負の加速度指令178件。15km/h上限まで加速できた試験ではない。
- PP先読み不足0件、停止領域侵入候補0件。その他の一時的な入力・運動妥当性の拒否は17指令（経路初期方向9、横方向速度6、yaw rate 2）で、その後復帰した。
- AWSIM・通常RVizの動画は全フレームをデコードして確認し、転送前後のSHA-256も一致。RVizで生のE2E経路を表示した。
- AWSIM全1089ファイルの試験前後ハッシュが一致。既存Git差分・RViz設定・過去コンテナを保全し、試験コンテナは終了済み。

最終evidenceは[こちら](evidence/time_curvature_response_20260917/manifest.json)。[評価集計](evidence/time_curvature_response_20260917/evaluation/summary.json)、[条件間比較](evidence/time_curvature_response_20260917/evaluation/comparison.json)、[速度グラフ](evidence/time_curvature_response_20260917/evaluation/speed_comparison.png)、[実走位置](evidence/time_curvature_response_20260917/evaluation/route_progress.png)を保存した。

動画のWindowsコピーは`tmp/time_curvature_response_20260917/videos/awsim.mp4`と`rviz.mp4`。raw記録・動画・checkpointはGitへ追加していない。WSLの原本は`/home/thistle/e2e_autonomous/runs/time_curvature_response_20260917`、リモート記録は`/home/graneple/e2e_autonomous/time_curvature_response_20260917`に保持している。

残る点は、同じ最終制御で停止領域を有効にした比較と反復試験、および高速化時の予測範囲との整合性。今回はユーザー指定の試験設定での1周完走を確認した段階で、一般的な障害物回避性能や正式なモデル昇格の判定は変更していない。
