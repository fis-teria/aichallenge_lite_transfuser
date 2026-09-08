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

## 実適用・install・追加試験07（上記の未実施状態と説明を更新）

ユーザーの実適用/install反映/追加走行依頼により、remoteの上記3ファイルへ
git apply --check成功後に限定差分を適用。既存dirtyは保持、remote commit/pushなし。
実system installのlaunchはsourceへのsymlinkであり、追加groupが実ロード先へ反映済み。
V4は`/home/graneple/e2e_autonomous/v4_normal_dev_07/install`へcolcon build、
1 package / 1.19s成功。コンテナ内`ros2 pkg executables`でv4_shadow_nodeを確認。

**訂正: 通常make devがoffというB節の結論は誤り。**
run_simulator.bash単体のdev既定はoffだが、現行Makefile:339は
`dev: AWSIM_START_MODE := sync`を指定している。試験07でコマンドライン指定を
外しても実ログは`--start-mode sync`だった。したがって、sync上書きが通常devとの
相違点・Ready欠落の原因という推測は撤回する。Ready原因は引き続きUNKNOWN。

今回試行は`codex-v4-normal-dev-07`。既存MPC、V4同一launch、DEV_AUTO_START=false、
AWSIM_START_MODEのコマンドライン指定なし。読み取り専用observerを先に起動。
新しい1試行枠を作り旧台帳は保持。1試行120秒/駆動10sim秒/forward40/MPC1500上限。
**結果は起動失敗、実走行未成立**。make exit0は車両動作成功ではない。

- launchパスをmerged install形式で指定したが、実際はisolated installだった。
  run_autowareのファイル存在判定で終了し、Autoware/V4の実動作へ到達しなかった。
- 別に、試験側の15秒開始タイマーをmake開始から数えていたため、
  START_WALL_LIMITで停止。全体18.0446秒、観測clock4件/最終0.019999999sim秒。
  ego/pose/command未取得、motion.jsonl空。走行・自然制動・V4経路更新は未確認。
- 今回所有container/projectは全て終了済み。observerの追加stopは既に消滅しており
  exit1になったが、終了後のinventoryで残存なしを確認。他の終了済み資源は保全。

パス問題はその後修正し実適用済み。sourceしたoverlayの
`ros2 pkg prefix aic_e2e_runtime`からlaunchパスを組み立てるため、
V4_SHADOW_LAUNCHの手入力は不要（旧指定があっても正しい解決結果で上書き）。
実解決値は`/v4/install/aic_e2e_runtime`。実ファイル存在と統合launch
`--show-args`のexit0をnetwork noneコンテナで再確認。推論/ROS graph起動ではない。
修正後の追加駆動再試行はしていない。

残課題は、試験側で起動待ち時間と駆動時間を混同しない有限監視を用意し、
修正済みinstall設定で実起動・走行を再検証すること。既存のReady/Safety条件は維持する。
AWSIM本体/scene/sensor、モデル、controllerは未変更。外部レビューはNOT_RUN。
証拠: Windows `tmp/v4_normal_dev_07/`、remote同名run。失敗の設定・run.pyは保全。
