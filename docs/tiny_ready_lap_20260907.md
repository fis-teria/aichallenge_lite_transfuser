# TinyLidarNet Ready後short → lap（2026-09-07追加許可）

今回の元依頼はattachment `f7325d22-7b3c-494e-bf56-ca4464746cef/pasted-text.txt`。
SHA256 `3210c97ce30a8af18abb9633378a0c9993f55c01f2231dda262142f17a646bcd`。
前回の未承認記録は履歴として保持し、この新許可を遡及適用しない。
開始HEAD `a87e17a089a75c7ab6ed3c9251fb6b8fefc7f13d`、branch
`codex/windows-wsl-training-sync`、origin `https://github.com/fis-teria/aichallenge_lite_transfuser.git`、clean。
実host開始確認は2026-09-06T20:44:08.459782228Z（05:44 JST）。
09:50 JST cutoff / 10:00期限を翌日へ読み替えない。

## 今回の局所差分

- `SimReadiness`はReady受信時のsim時刻に加え、supervisor monotonic受信時刻とscan受付連番を保持。
  正加速度にはReady後の元scan受付時刻・連番を要求。worker終了時刻・queue投入時刻では代用しない。
  既存clock watermark/expiry/単調source stamp、再初期化時終了を維持。
- Tiny専用profile `TINY_READY_LAP_20260907` にだけ共通forward6000、累計駆動300sim秒、
  lap1回240sim秒を許可。`AttemptBudget.limits`は変更せず、instanceごとにcopyする。
  旧used/attempt/activeを保持。未精算activeがあれば変更も予約も拒否する。
- config/runtime/hostで同じphase上限を確認。bool/NaN/inf/負値は拒否。
  shortは20sim秒/300forward/runtime120wall秒、lapは240sim秒/5200forward/runtime600wall秒。
  host予約はruntime+110秒以内、共通wall残量とcutoffからさらに縮小する。
- LapCountの実section/lap順序、lap番号増分、run/epoch、新ログのbyte offsetを記録。
  完了判定を距離・時間・文字列の出現だけに緩めない。

公式重み、750点前処理、速度目標2.0m/s・上限2.4m/s、操舵上限pi/6、周期0.05s、
速度feedback/制動式は変更しない。gripSteerFactor補償、V4/MPC、正解route入力なし。
既存AWSIM/起動script/sensor/vehicle/read-only compose条件は維持。
モデルと独立したsupervisor、送信前host ARM、失効時制動、host freeze/KILL、unpause禁止を維持。

## 予算の起点と実記録

実台帳は前回の値に一致：wall1189.1912133327914、V4 forward124、Tiny173、
powered1、powered_s8.889999801、log58002335bytes、snapshots7、MPC1、active=null。
新上限では残り駆動2回、291.110000199sim秒、共通forward5703回、wall2410.808786667秒。
変更前台帳・許可record・予約前/予約後/精算後を各attemptへ保存する。
承認recordの適用時刻は実行hostの実時計、承認者名はnull、根拠は上記依頼hash。
過去3attemptと旧112test実行を今回の成功実績へ混ぜない。

## 実行方法と順序

Windowsでcommit後、変更していない`tools/sync_to_wsl.ps1 -CheckOnly`→通常sync。
Dataset固定rootの存在確認だけ許可。Dataset内容/raw/学習sensor/V4 checkpointは読まない。
WSL同一SHA・worktree lock下で`tests/test_tiny_lidar_sim.py`だけ検証。
公式packageの全parameter確認を含めるがtestではforwardしない。

```bash
TINY_OFFICIAL_PACKAGE=/home/thistle/e2e_autonomous/tiny_lidar_lap_20260907/official_package \
  bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
  tests/test_tiny_lidar_sim.py --junitxml=/home/thistle/e2e_autonomous/tiny_lidar_ready_lap_20260907/tests.xml
```

固定commit archiveをremoteの新規専有directoryへ展開し、既存dirty repoは編集しない。
専用make dev（元racing-kart Makefileではない）：

```bash
make dev DEV_CONTROLLER=tiny TINY_PHASE=short TINY_WALL_SECONDS=120 \
  SIM_REPO=/home/graneple/git/autononous_ai/aichallenge-racingkart \
  TINY_PACKAGE=/home/graneple/e2e_autonomous/tiny_lidar_lap_20260907/official/tiny_lidar_net_controller \
  XVFB_ROOT=/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/xvfb-root \
  TINY_OUTPUT=UNIQUE_OWNED_ABSOLUTE_OUTPUT TINY_COMMIT=EXACT_WINDOWS_SHA \
  TINY_BUDGET=/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json
```

shortはReady後新scanで最大8sim秒の駆動後、0.03m/s以下を新規速度sampleで0.5sim秒確認。
実前進、更新操舵、制動停止、監視正常、残予算成立時は同じ設定のlapへ進む。
lapは上記のphaseを`lap`、wallを`600`へ変更し、別の新規attempt出力を使用する。
Judge一周通知か最初の上限で制動へ。234sim秒以内で正駆動を終え、6秒の停止余裕を残す。
今回の結果が失敗でも消費を精算し、episode累計3を越えない。

## 成果物

実行結果は別results文書、全attemptの生ログ、test stdout/stderr/exitとJUnit、
今回依頼・独立レビュー依頼、固定source/diff/versions、`PACKAGE_MANIFEST.json`付きZIPへ保存。
旧packetはそのまま保持。動画の既存取得手段がない場合は実Unity judge/logを使用する。
weight本体・全sensor・環境は同梱しない。自己点検を独立レビュー済みとしない。
