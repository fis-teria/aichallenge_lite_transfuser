# V4通常launch統合とStart/Ready調査の分離

2026-09-09。起点Windows `9ef8096`、remote racingkart
`4af395eee10f928c7fc7225760adfa04c4c07ff4`（dirtyを保全）。

## A: V4 shadowは入力準備で開始

`v4_shadow.example.json`は`inference_start_policy=INPUT_READY_SHADOW`、
`start_gate=null`へ変更。enabled=falseは維持。Start receipt、arm、Readyを
shadow推論の前提にしない。既存の明示的なgate付き設定は互換のため残す。
wall/forward/candidate/log上限、入力/graph鮮度、reset、外部command履歴は維持。
これは推論と走行権限の分離であり、車両Start gateの解除ではない。
停止中にも推論回数を消費する。枠を自動延長・再起動しない。

通常起動への変更所有者は`tools/normal_dev_v4_integration.py`の純粋変換関数。
remoteの現在の3ファイルを読み、無関係のdirty部分をそのまま残した小さなpatchを
生成する。新規runnerやCONTROL_METHODは作らず、既存launchの子としてV4を追加する。

|対象|追加内容|
|---|---|
|`aichallenge/run_autoware.bash`|enabled=trueの時だけ既存ビルド済みV4 overlayをsource。simulationモード・絶対パス・ファイル存在を確認|
|`aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/launch/aichallenge_system.launch.xml`|環境変数で既定OFFのgroup/includeを末尾追加|
|`docker-compose.yml`|既存autoware環境へV4用4変数を引渡し|

将来の実行例（今回未実行。既存の外側有限停止処理と個別試験許可は必要）:

```bash
V4_SHADOW_ENABLED=true \
V4_SHADOW_SETUP=/container/visible/install/setup.bash \
V4_SHADOW_LAUNCH=/container/visible/install/share/aic_e2e_runtime/launch/v4_shadow.launch.py \
V4_SHADOW_CONFIG=/container/visible/reviewed.json \
make dev CONTROL_METHOD=mpc DEV_AUTO_START=false
```

上のパスはplaceholder。実際のmountとビルド済みinstallを確認して置換する。
通常のsystem launch経由なので別の`docker exec ros2 run`やhelper成功ファイルは不要。
MPCは既存経路の制御担当、V4はshadow記録担当。V4経路がMPCへ渡る変更ではない。
既定OFFならV4 package/configを要求せず、現runtimeの挙動を維持する。
有効化時は既存system packageの**実ロード版**にincludeが入っていることを確認する。
sourceへのpatchだけをinstall反映済みと扱わない。

## B: Ready待ちは別の既存起動問題

確認済みの差分:

- remote `aichallenge/run_simulator.bash` のdev分岐は `start_mode=off`。
- その後 `AWSIM_START_MODE` があれば上書きする。
- 試験06の起動側は `AWSIM_START_MODE=sync` を指定した。
- 実ログ `tmp/v4_mpc_drive_06/evidence/v4_mpc_drive_06/awsim.log:3` は
  `--start-mode sync`。前回は通常make dev recipeだが既定と同じ起動条件ではなかった。
- 同試行でGrounded→Startがhelperログにあり、Readyは確認されず、
  既存autostartが `waiting_for_neutral=Ready` でrace armを拒否した。

sync上書きがReady欠落を引き起こしたかはUNKNOWN。offへ変更すれば成功する、
待機時間を延ばせば成功するとは断定しない。AWSIM本体・既存Start/Safetyは未変更。
今回のAの修正でBが直ったとも扱わない。

次のB検証はV4を既定OFFにした既存MPC環境で、通常devの開始条件・公式Start手順・
admin/vehicleの時刻付き状態遷移を限定確認する。起動modeが変わると車両動作も
変わり得るため、停止監視・有限予算を持つ別試行として実施する。
今回、実ROS起動・実入力収集・推論・Start・走行は行わない。

## 限定再現

Windows commit → `tools/sync_to_wsl.ps1 -CheckOnly` →通常sync → WSL:

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_normal_dev_v4_integration.py tests/test_v4_shadow_package.py \
 tests/test_shadow_start_gate_v4.py tests/test_publisherless_shadow_v4.py
```

同期時の固定Datasetルート存在確認のみ許可。内容・raw・checkpointは読まない。

## 今回の到達点

- 実装/試験版 `deaa5680ca605dea19469b897a070a72c252e067`。
  上記4ファイル、WSL lock付き **44 passed / 4.02s**。
- 既定CheckOnly/同期によるDatasetルート存在確認を実施。
  Dataset内容/raw/sensor/checkpoint読取は未実施。
- 現在dirtyの3ファイルから生成した配布差分:
  `integrations/normal_dev_v4/racingkart.patch`（`c722400`）。
  SSH先で`git apply --check`成功。**実適用は未実施**。
  手作業の変更破棄やsource/installの全置換は必要ない。
- SSH先の既存起動ファイル・install・AWSIMは変更していない。
  実ROS launch/build、推論、走行、外部レビュー、pushは今回NOT_RUN。
- 残作業はremoteへの限定差分適用と実ロードinstall反映/非走行launch検証。
  remoteの追加レビューポリシーもあるため、今回のローカル限定テストを
  remote適用や外部レビュー完了の証拠にはしない。
  Ready調査用の次回走行を、この統合の合格条件にはしない。
