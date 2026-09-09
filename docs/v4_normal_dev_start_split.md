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

## 既存MPCの通常自動Start試験08: 走行開始を確認

ユーザー「既存構成とAWSIMを弄らずに走行開始」に対し、今回はV4をOFFにし、
通常make devの既定`DEV_AUTO_START=true`を使用した。別途helperをdocker execで
手動起動する処理は使わない。既存のMakefile、Start helper、controller、Safety、
AWSIM本体/scene/sensorは今回一切変更していない。前段のV4追加hookはOFF。
変更はWindowsの記録文書と新規試験ディレクトリの起動/監視スクリプトのみ。

```bash
# 実行済み。既存GPU compose＋専有project、外側有限watchdog付き。
V4_SHADOW_ENABLED=false make dev CONTROL_METHOD=mpc CAPTURE=false ROSBAG=false \
 RUN_ID=mpc_normal_start_08 \
 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/mpc_normal_start_08/evidence
```

起動前から読み取り専用observerを有効化。全体95秒の終了開始閾値/120秒上限と、
helperのStart pulseログ観測から8sim秒/15wall秒の終了開始閾値を分離した。
後者をmake起動時やV4準備時から誤計測しない。Start後の10sim秒上限は維持。
旧履歴を保持し、新規1試行予算は当該runのbudget.jsonに記録。

**既存公式Start完了＋短距離移動を確認した。**

- make既定の`awsim-request-start`が`docker compose run --rm --no-deps autoware-command`
  を使用。ログにvehicle Start、その後Ready、official Start true: accepted、
  authoritative admin Start accepted、AWSIM race start confirmed。make exit0。
- 622件のposeとego速度記録。開始位置からの変位1.872412m、最大速度1.837929m/s。
  速度0.05m/s超の最初の観測10.649999761sim秒、最後12.634999717sim秒。
  0.05は解析用の区別であり、Safety閾値変更ではない。
- Start pulse観測4.529999898sim秒。DRIVE_TIME_LIMITで所有sim pause/KILL後、
  所有Autoware/observer/Composeを終了。全体39.1065秒、残存なし。
- 自然制動、無接触、一周完走は未確認。V4推論0、V4→MPC追従の成功ではない。

raw result.jsonのstatus=FAILEDは保全する。make完了後のinstance inspectが
watchdog終了と競合し空のcontainer IDを読んだためで、開始失敗という意味ではない。
同じ理由で終了済みobserverへの追加stopはexit1。終了後inventoryで残存なし確認。
試験スクリプト全体をPASSとは呼ばず、Startと移動の成立を独立した証拠で報告する。
末尾のINPUT_STALEはfreeze後の欠損であり最初の停止原因ではない。

この結果により、既存MPC/Start/AWSIMの改修が走行開始の必須条件ではないことを確認。
過去のReady未到達について、手動helper起動の差と時間窓のどちらが支配的だったかは
単独比較していないため断定しない。今後はこの既存自動Start経路を維持してV4を追加する。

証拠: Windows `tmp/mpc_normal_start_08/{evidence,evaluation.json,budget.json,run.py}`、
remote `/home/graneple/e2e_autonomous/mpc_normal_start_08/`。追加再試行なし、pushなし。

## MPC一周試験09: 未完走（2026-09-09）

ユーザー承認: 1試行、全体120秒、Start後90sim秒、MPC60000回、V4 OFF。
通常自動Start、CONTROL_METHOD=mpcは維持し、既存起動引数`AWSIM_LAPS=1`のみ追加。
AWSIM実行物/scene/sensor、controller、Start/Safetyには変更なし。
変更対象は新規試験ディレクトリの有限起動/読取監視と本記録のみ。

実行コマンドは試験08と同じmake devに`AWSIM_LAPS=1`、RUN_ID=mpc_normal_lap_09。
実ログで`--start-mode sync --laps 1`を確認。run参照Windows HEAD `1f21953`、
remote HEADは従前の`4af395eee10f928c7fc7225760adfa04c4c07ff4`（dirty保全）。
開始前の構文チェックで試験スクリプト生成時の引用符エラーを検出・修正した。
その時点ではROS起動/試行予約は未実施。実AWSIM試行は1回のみ。

**結果: 公式Startは成功したが、一周は完了しなかった。**

- make exit0、公式Start成立。最大速度6.9133m/s。
- 3694件のposeを取得。記録点を結ぶ折線長82.086m（測位の微小揺れを含むため
  厳密な車輪走行距離ではない）。開始地点へ戻ったことを完走とする判定は行わない。
- コーナー付近でほぼ停止。既存ログに`mpc_guard=1`、MPC infeasibleの反復、
  `reason=wall_footprint_margin`とspeed-only fallbackを確認。
  実際の壁接触、速度/参照曲率/初期偏差のどれが根本原因かは未確定。
- 一周のFinish通知なし。既存監視を解除せず、反復する実行不能を理由に
  agentが所有simulator `c76ba22f2a93`をpause→KILLし試験終了。
- 最終74.084998344sim秒、Start pulse観測6.079999864sim秒、全体101.5966秒。
  120秒/90sim秒以内。生ログ11,499,064byte。MPC60000枠は保守予約計上であり
  実際に60000回計算したという意味ではない。forward0。
- raw resultのINPUT_STALE/OBSERVER_EXITはfreeze後の結果として保全。
  最初の終了判断は上記MPC実行不能の反復。cleanup errorなし、所有container残存なし。
  外部停止であり自然制動の合格とはしない。無接触・一周・V4追従は未確認。

証拠: Windows `tmp/mpc_normal_lap_09/`、remote同名run。
`evaluation.json`はraw resultを書き換えず、判定と停止介入を分離して記録。
次は保存済みログから停止直前の参照曲率・速度・横偏差とMPC制約を照合する。
既存構成の変更や追加走行は自動で行わない。pushなし。

## MPC速度上限20km/h比較試験10: 実行準備

ユーザーが追加1回、全体120wall秒、Start後90sim秒、MPC60000回を明示承認。
V4 OFF、通常make dev、AWSIM_LAPS=1。仮説は速度上限低下によりコーナーの
追従崩れが改善するかであり、接触やsolver失敗の根本原因確定ではない。
`integrations/mpc_speed20/config.yaml`はremoteの現行設定のコピーから
mpc.v_maxのみ35→20km/hへ変更。元設定と比較可能に保管する。
実ロードinstallのconfigはsource configへのsymlinkと確認済み。
専有autowareコンテナだけにread-only bindし、SSH先dirty checkoutは上書きしない。
曲率/区間速度計画、操舵、制動、Start/Safety、AWSIM本体は維持。
撤回は専有コンテナ終了のみ。速度上限は目標の上限であり実速度保証ではない。
改善指標は問題コーナー通過、経路進捗、MPC失敗有無、一周Finish。
未完走時も枠を自動更新しない。監視は既存run09から継承し、反復MPC失敗と
Collision detectedログでの終了を明示する。外部freezeは自然制動と区別する。
再現: 設定/guard/runとtools/spatial_dev_host_v4.pyを専有runディレクトリへ配布し、
Windows実行commitをsource_commit.txtに保存。preflight後に
`timeout --signal=TERM --kill-after=3s 117s python3 run.py`を一度だけ実行。
生ログ・予算・結果は`tmp/mpc_speed20_lap_10/`へ回収する。

### 試験10結果: 未完走

実行版`3d707e0613d1c309f46fe98d2d2d4be931c2a9c4`。
Windows commit→既定CheckOnly/同期→WSL lock付き
`.venv/bin/python -m pytest -q tests/test_mpc_speed20_fixture.py`は2 passed / 0.06s。
全pytestはNOT_RUN。Datasetルートの既定存在確認を実施、内容/checkpoint読取なし。
network-noneコンテナでinstallの実ロード設定が20km/h版であることを確認し、
走行ログにも同じ値を確認。設定hashは
`273793ba675ca8608ff3d75c07ba3fd63056fba4324dd700e82f09840caf2252`。
SSH元configは前後とも
`1be5b0e0aa9a5753e93d6cec28c88d4bf52f978d2b9d584f949b7a5b0c589ba1`。

- 通常make dev exit0、official Start成立。V4 OFF、forward0。
- 目標速度最大5.555555m/s、実速度最大5.718049m/s（20.585km/h）。
  目標上限の反映は確認、実速度20km/h厳守ではない。
- 3722 pose、記録点の折線長102.645m（前回82.086m）。測位揺れを含む。
  overtake座標ego_s最大124.47、最終124.34（前回最終105.62）。
  前回停止地点より約18.7m先という比較であり、一周判定ではない。
- sim30.405→30.435の記録で速度4.660→1.315m/sへ急落。
  当時の要求加速度+3.0m/s²。接触・状態入力異常の切り分けはUNKNOWN。
  速度とposeは別callbackの最新値で、厳密な同時刻の加速度実測ではない。
- MPC失敗カウンタ最大6、最終0。前回の失敗継続とは異なる。
  最終reason=wall_footprint_margin、ego_d=2.41、速度約0.00033m/s。
- sim46.09でraw舵角が設定32degを超え、raw最大絶対値1.008829rad、
  final最大1.653471rad。finalは既存gain 1.639適用後。実タイヤ角の実測ではない。
  agentが停止状態と異常な指令増大を確認し、所有simulator
  `89a67b91de0a`をpause→KILL。以後unpauseなし。
  監視スクリプトには操舵絶対値の即時停止条件がなく、今回はagent介入。
  既存の操舵制約/出力処理の調査と停止監視確認が次の駆動前の課題。
- Finish通知なし、無接触UNKNOWN、外部停止を自然制動PASSとしない。
- 全体104.9106wall秒、最終74.680sim秒、Start pulse観測5.825sim秒。
  Start後約68.855sim秒。command2123件、MPC60000は保守予約計上で実計算回数ではない。
  生ログ13,189,632byte。新枠1回を使用、再試行なし。
- raw resultのFAILED / INPUT_STALE / OBSERVER_EXITはfreeze後の結果。
  start_basisの旧文字列は残存しているが、start_sim_sはStart pulseログ観測時。
  元結果は改変せず本節で解釈を分離。cleanup errorなし、所有container/project残存なし。

速度を下げて進捗は増えたが、原因解決や一周成功とは扱わない。
AWSIM/既存MPCソース/SSH設定は未変更、変更は専用設定コピーと試験スクリプトのみ。
実ROS/AWSIM試験は実施済み。接触telemetry、実舵角記録、solver正式終了状態、
独立レビューは未確認。生証拠はWindows `tmp/mpc_speed20_lap_10/evidence/`と
remote `/home/graneple/e2e_autonomous/mpc_speed20_lap_10/`。pushなし。

## 完走向けカーブ速度調整候補（未適用）

ユーザーのMPC単独パラメータ調整依頼。試験10の設定・結果は保全し、
`integrations/mpc_speed20/config.corner_candidate.yaml`を別名で作成。
v_max=20km/hを維持、ay_maxのみ9.5→3.0m/s²へ低下させる。
3.0は校正値・安全保証ではなく、カーブ進入速度を下げるための未検証の試験値。
既存reference_path.compute_speed_profileとMPCの曲率速度上限が対象。
操舵gain/rate/角度限界、車幅、壁margin、制動能力、Q/R、経路、AWSIMは変更しない。
適用はまだ行わず、run.pyも引き続き試験10設定を指す。追加走行枠なし。

調整前のコード確認で、MPC.get_controlはdec.info.statusを確認せずdec.xを使用し、
成功分岐では操舵rate clipだけ、例外分岐では過去予測操作を返すことを確認。
mpc_controllerの出力前フィルタも絶対角度のclipではない。
solver終了理由・許容残差・実操舵が未記録なので前回の超過原因の断定はしない。
パラメータ変更だけで操舵制約が確実に守られるとは言えず、実走行への昇格は保留。
次段階は既存MPCの解採用条件・操舵制約処理の修正範囲について承認を得て、
異常解を含む合成回帰試験で確認する。新規MPC開発やSafety緩和は不要。
その後に別承認枠で比較走行し、Finishと接触/逸脱/介入を評価する。

## MPC解採用・操舵制限修正（Windows実装、remote適用保留）

ユーザーが既存MPCの解採用と操舵制限の修正を承認。
`integrations/mpc_solution_guard/`に現在の2ソースの修正版とfocused patchを保存。
実装内容はREADME参照。OSQP solved以外、非有限/shape異常/組立制約逸脱を拒否、
旧操作再利用とゼロ曲率時のmargin緩和retryを削除。拒否時は予測を失効させ、
既存a_minの制動要求をfeedforward/filterより優先。raw絶対角/rateをpublish前にも制限。
既存gain変換は維持。未知の車体校正値は追加しない。カーブ候補設定とは別変更。

初回WSL試験はOSQP未導入でcollection error。その記録を合格扱いにしない。
依存追加せず、合成解用test doubleと実OSQP試験を明示分離し、
WSL lock付き25 passed / 1 skipped（OSQPなし）を確認。
remote既存MPC venvにもpytestがなくtest起動は失敗。
標準Python実行のsmoke_osqp.pyで既存OSQP 0.6.7.post1を使い、
正常解採用・primal infeasible拒否の合成QP 2件を確認した。
いずれもnetwork none、既存aichallenge mountはread-only、ROS/publish/走行なし。
追加のmetadata欠落とraw/final publisher境界試験を含む最終結果は下記に追記する。

最大主張はローカル実装と合成検証。完走、実車Safety、実停止成功は主張しない。
既存QPの隣接曲率差rate定式化と実タイヤ校正、実ROS全体の例外停止、
数値許容1e-5による実際の解採用率は未検証。
SSH先ソース/installへの適用は未実施、基準hashはREADMEに固定。
remoteのexternal_review方針を確認したが、今回は外部サービスへのソース送信を
明示承認されていないため未送信・REVIEW_PENDING。review完了と偽らない。
新しい走行枠もなし。AWSIM/V4/壁監視/既存remote dirtyは未変更、pushなし。

最終実装版`ff04c19bafb1cf95fc580002b50481542ae45450`。
WSL同版・worktree lock付き **27 passed / 1 skipped / 1.42s**。
スキップはOSQP依存1件だけであり、別途同版MPCを既存runtime venvの
OSQP 0.6.7.post1で試し **合成QP 2件成功**。ROSは起動していない。
stdout/JUnitは`tmp/mpc_solution_guard_results/`。
`git apply --check`は現行remoteに対してexit0。nodeの既存実行属性100755と
Windowsコピー100644の差についてwarningあり。実適用時は既存実行属性を維持する。
未適用・未レビューのまま完走済み/稼働修正済みとは報告しない。
全pytestと実ROS結合試験はNOT_RUN。同期の固定Datasetルート存在確認は実施、
Dataset内容/raw/sensor/checkpoint読取は未実施。

## レビュー・適用・AWSIM再試験依頼: キュー占有で保留

ユーザーがレビューからAWSIM試験までの継続を依頼。予定する新試行は1回、
全体120wall秒、Start後90sim秒、MPC60000上限、V4 OFF。まだ枠を予約/消費していない。
不変handoff/packetを`tmp/mpc_guard_review_20260909/`に作成してremoteへ配布。
execution=`mpc-guard-dc84956-20260909`、attempt=
`mpc-guard-dc84956-20260909-review01`をcanonical queueへ登録、integrity_valid=true。
最初の短いattempt名review01は既存名と衝突して登録失敗し、一意名で登録した。

専用transportのclaimは
`review_singleflight_busy:five-path-lease-canonical-ownership-review-01`
で拒否。他作業のlease/claimを変更せず、ブラウザ送信もdeferも未実施。
状態はREVIEW_PENDING、workflow_advance_allowed=false。
外部レビュー未完了のため、remote source/install適用・実ROS起動・AWSIMは未実施。
再開時は同一requestの状態と共有枠を確認し、同一packetのレビューを継続する。

並行して、有限observer向け純粋command_monitorを実装。実ロード設定由来の
raw角度上限と既存gain別domainでNaN/Inf/shape/角度超過を判定する。
この段階ではobserverへの接続前。Safety/controllerの緩和はしない。
Windows実装1518b4c→既定同期→WSL lock試験は34 passed / 1 skipped / 1.27s。
スキップは既報のOSQP依存。AWSIM、ROS、推論、駆動は追加していない。
旧結果と既存dirtyを保全。pushなし。

## 2026-09-09 外部レビュー必須条件の廃止（現行方針）

仕様変更に伴うユーザーの明示指示で、外部レビューを必須の進行条件から除外した。
上記のREVIEW_PENDINGによる実適用/試験保留は現在のブロッカーではない。
Windows AGENTS.mdとremote親リポジトリAGENTS.mdを更新し、レビュー送信・
キュー登録・搬送agent・待機は当該タスクでの明示依頼時のみとした。
remote docs/agent_guidelines/external_review.mdは冒頭で現行方針を定義し、
既存の本文は非適用の履歴として保全。過去M4の外部レビュー必須条件も置き換える。
キューCLIや記録そのものは削除/成功扱い/ロック奪取していない。
本件の未送信review requestも歴史的記録として保持し、自動送信しない。
ローカルテスト、source/install整合、Safety/権限/期限、有限試験予算は変えない。

remoteの既存external_review.mdの未コミット変更69行を含む読取snapshotに
追加差分だけ適用。最初のpatchは末尾context空行欠落でcheck段階にて拒否され、
修正後のgit apply --checkと適用、diff --checkは成功。内容SHA256は
Windows修正snapshotとremote実ファイルで一致した。
remote AGENTS: c27814e1280bde1c707719b9c0be3f6d1befddff376e233ceafd1d8c1499a4ce
remote external_review: 58372ef713b5b8917935b6bfd5cda1e0ba02aabd5d0645da75d88642235588e4
保存差分: integrations/review_policy/remote_optional_review.patch。
今回の変更は運用文書のみ。ROS/モデル/controller/AWSIMを起動・改変せず、
走行枠は未使用。外部レビュー待ちは解消したが、実適用/ROS結合確認は引き続き必要。

## 修正済みMPC・カーブ速度候補の試験11準備

ユーザーがAWSIM試験まで継続を明示依頼。以前許可された未使用枠を今回1回だけ使用予定。
上限は120wall秒、Start後90sim秒、MPC60000、V4 OFF。試験10までの使用量を継承。
remoteの2ソースhashが基準版と一致しgit apply --check成功後、限定patchを実適用。
既存node実行属性のwarningはあったが属性を変更しない。既存dirtyは保全。
制御設定は専有コンテナだけの20km/h/ay_max3.0候補。AWSIM本体・scene・sensorは不変。
installの実import元をpreflight.pyで確認し、修正版SHAと比較してから起動する。
run11のobserverは実設定のraw限界・gainを使うcommand_monitorを接続。
NaN/Inf/角度超過、既存authority/stale条件、MPC失敗反復、Finish、時間上限で終了。
外側timeout117秒+kill猶予3秒、watchdog105秒で終了開始。自動再試行なし。
実行は通常make dev CONTROL_METHOD=mpc CAPTURE=false ROSBAG=false AWSIM_LAPS=1。
記録先はtmp/mpc_guard_lap_11（remote同名）。過去試験との差はguard修正とay_maxであり、
単一パラメータの因果比較とは扱わない。動作未確認の候補設定を完走済みとは扱わない。

### 試験11結果: 起動後・Start前のsolver非収束で終了

実行Windows版8dac2675b9ddc7d9b571d57411109163e7134574。
既定CheckOnly/同期、WSL lock付き36 passed / 1 skipped / 1.22s。
Datasetルート存在確認のみ実施、内容/raw/checkpoint読取なし。
remote patch適用とinstall実import元を検証。2ファイルのSHAは以下で一致:
MPC.py 2810b8ba70b9d64398487b17a84c10cc3f077faaeada1a0f4dd6843ed5a71355
mpc_controller.py af080fb1169783114c533d60ddcc161d4a1b8cfdbe299481d9b50aa088f98328
installは既存sourceへの参照で、再buildによる無関係な置換はしていない。
preflightは最初のshell引用符ミス、次にsetup.bashのCOLCON_TRACE未定義で失敗。
ROS setup読み込み中にnounsetを使わないよう修正して成功。これらは駆動試行ではない。
実設定20km/h、ay_max3.0、hash:
ccb5cc4f1ff2aced753010411ab725ec97ae5db20cc4af705a2a819bedb1fa54。

通常make devの1試行を実施。AWSIM、ROS sensor/ego/pose、MPC raw/finalの接続を確認。
しかしMPC_REJECTED: SOLVER_STATUS:maximum iterations reachedが34回。
最初からこの終了状態のため、1e-5制約残差判定まで到達したという証拠はない。
この試験で判明した最初の失敗はsolver非収束であり、wall gateや外部レビューではない。
旧版にも同じ終了状態が出ていたか、ay_max変更が影響したかは未確定。

- observer raw/final各34件、要求は全て速度0・raw舵0・加速度-0.9m/s²。
- 56 pose、折線長0.000482m、最大実速度2.4e-7m/s。実走行は成立していない。
- stateはspawnedのみ、official Start pulse/Ready/Finishは未確認。
- REPEATED_MPC_FAILUREで所有sim freeze/終了。全体31.0145wall秒、最終1.255sim秒。
  追加1試行を使用。MPC60000/駆動90秒は保守予約計上で実際の回数/駆動時間ではない。
- raw resultはFAILED / BOUNDED_MONITOR_STOP_DURING_MAKE、make_exit=nullを保全。
  INPUT_STALEはfreeze後。制動要求確認はできたが、もともと静止中なので動的停止PASSではない。
- result生成時にhelper container 055a3caac60fの一時残存を記録。
  その後の独立inventoryで所有project/containerと全running containerが空と確認。
  元remaining_ownedを改変しない。postrun_inventory.logに最終状態を保存。
- AWSIM実行物/scene/sensor・V4は未変更。MPCソース修正は適用済み。
  ホストの元configは上書きせず、試験用速度候補は専有コンテナ内のみ。

証拠: tmp/mpc_guard_lap_11/とremote同名run。追加再試行・pushなし。
完走は未達。次は初期状態QPの非収束（尺度、重み、制約整合、反復状況）を
限定診断し、解採用条件を緩めずに収束するかを確かめる。未収束解を復活させない。

## 既存Pure Pursuitへ基準経路の接続先を変更

ユーザー方針変更によりMPC調整を止め、既存PPへ切替。
remote Makefile既定CONTROL_METHODをpure_pursuitへ変更（明示指定で旧方式は選択可能）。
reference.launchのPP分岐だけに既存overtake plannerを残し、PPのtracking_statusを接続。
MPC horizonはfalse、external固定速度はfalse、stale停止とoverride鮮度要求はtrue。
PP以外の分岐は変更せず、追従器/車両モデル/Safetyを新規実装しない。
Windows integrations/pp_reference/に適用ソース・差分・テスト・再現記録を保存。
git apply --check→remote限定適用成功、既存dirtyを保全、remote pushなし。

基準軌道生成の速度上限はPP選択時だけmin(既存指定,20/3.6)m/s。
実装調査により、既存buildExecutionProfileは全点速度の最小値を使う一様速度であり、
カーブ別速度計画ではないと確認。source速度0も停止点ではなくprofile拒否になる。
PPはその軌道速度を読み比例縦制御、stale等では既存-1.5m/s²停止要求。
MPCの制動/操舵gainを流用したとは扱わない。PP既存gain1.54を維持。
速度・制動能力の実走行検証、新しいカーブ速度計画は今回は行っていない。

Windows b086707→既定同期→WSL lock試験3 passed/0.03s。
install実ファイルhash一致、ROS launch --show-argsと上限式の実評価が成功。
PP executableの存在確認済み。実node起動・購読・publish・AWSIM・走行はNOT_RUN。
MPC用run11 observerはノード名/異常理由/gainが違うためそのまま再利用しない。
次はPP専有送信元、既存監視の入力充足、consumer制限と停止を確認する有限試験。
V4は未接続のまま。基準経路PPで成立後、V4経路接続を別段階とする。

## 2026-09-09 Pure Pursuit AWSIM有限試験 run12

実行commit: bf69a68f11828e079066833c3043f17d03064cbe。
Windows正本から既定CheckOnly/同期後、WSL lockで限定試験11 passed / 0.07s。
既定同期によるDatasetルートの存在確認を実施。
Dataset内容・raw・sensor・checkpointファイルの読取りは未実施。
実行時のROS sensor/ego/pose購読と小さな状態ログ保存は本AWSIM試験として実施。

実行環境: graneple@192.168.3.10、
/home/graneple/git/autononous_ai/aichallenge-racingkart。
専有project codex-pp-reference-lap-12、ホスト元checkoutの既存dirtyを保全。
AWSIM実行物/scene/sensorは変更せず、V4推論・MPCはOFF。
既存PPが既存CSV基準経路を追従する試験であり、V4経路追従ではない。

実行コマンド（同じrun IDの再実行指示ではない）:

```text
make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1 RUN_ID=pp_reference_lap_12 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/pp_reference_lap_12/evidence
```

ホスト専用run.pyの時間監視下で上記を実行。raw操舵はPP制限前の要求、
final操舵はPP制限後として監視し、MPCのraw gain契約を流用しない。
final送信元は/simple_pure_pursuit_nodeのみを要求。
追加試行は1回のみ。wall120秒、post-Start90sim秒、PP指令60000回以内。
旧予算はbudget.jsonのprior_usedに保持。駆動90秒は保守予約計上で実測値ではない。

結果:

- make終了0、公式Start helper完了。stateはspawned/grounded/start/ready/start。
  Finishはなし。RViz2起動ログはあるが画面動画は今回取得していない。
- 3966 pose標本の折線長91.9304m。これは走行距離の診断値で完走判定ではない。
  初めて速度0.1m/s超となった記録はsim12.815秒、最終sim79.530秒。
- 最大実速度1.389441m/s（約5.002km/h）。目標速度最大5.524161m/s（約19.887km/h）。
  終端も速度1.388028m/sに対して加速要求4.136114m/s²。
  約5km/hに留まる理由はUNKNOWN。速度fieldの設定だけで20km/h成立とは扱わない。
- raw9036件/final9037件。poseに対応付けて記録されたfinal操舵の最大絶対値0.530rad。
  これは全commandを保存した厳密な最大値ではない。監視時のfinal0.64rad超過検出なし。
- HOST_WALL_LIMITが終了理由。全体108.963wall秒。
  生resultはFAILED / OBSERVER_EXIT、最終guardはINPUT_STALEのまま保全。
  終了処理のobserver終了を主因のように上書きせず、時間上限停止と区別する。
- 所有AWSIMをpause→KILLして終了。最終制動要求-1.5m/s²は見えるが、
  最終速度は1.388m/sであり、動的制動停止成功ではない。
- 生resultのremaining_ownedは空。試験後の独立docker inventoryでも
  所有project残存・全running containerなし。過去の停止済みcontainerは保全。
- trajectoryのRELIABILITY QoS不一致警告が2件あり、対象購読者は未特定。
  PP実走行が成立したことだけで全subscriberの接続成功とはしない。

証拠: tmp/pp_reference_lap_12/ とremote同名runに生ログ・budget・resultを保存。
無接触、コース1周、実制動停止は未確認。追加再試行・pushなし。
次は目標約19.9km/hと実速度約5km/hの差をconsumer/制限設定から読み取り確認し、
必要な修正と次の有限試験条件を定める。AWSIMの変更や監視解除で解消しない。

## run13準備: 20km/h目標を維持し、過大な加速要求を修正

前回の単なる時間不足という説明を補足する。約5km/hへの制限に一致する
AWSIM penalty経路が保存済み逆コンパイルから判明。現在のAssembly-CSharp.dllと
保存DLLのSHA256は859e5560dbffd7d0833cd1fe5eb1f36b87fa8d22f6e45eb0e1203d04a1ea6d13で一致。
AccelInputAnomalyDetectorは加速度>3m/s²または64標本の到着頻度>=250Hzを異常とし、
VehiclePenaltyControllerは速度を1.388889m/sに制限する。
run12は加速4〜5.5m/s²を継続要求しており条件に該当。公式penalty event自体は未収録。

変更対象はPP分岐の既存speed_proportional_gainのみ1→0.5。
同一経路の目標約5.524m/sは維持し、静止からの要求を約2.762m/s²にする。
変更なしでは入力違反を反復する。加速gain単独比較で速度上昇と入力上限を確認する。
一般的なclampではなく、逆走や異常stateまで許容する設定ではない。
run13 observerはfinal加速>3で停止する。既存停止・舵角・鮮度監視は維持。
AWSIMや追従器C++、他controller分岐を変更しない。復旧はgain追記1行を戻すだけ。
元racingkart.patchの後にspeed20_gain.patchを適用する順序で、過去差分は保全。

予定検証: WSL lock下pytest tests/test_pp_run13.py tests/test_pp_run12.py
tests/test_pp_reference_switch.py tests/test_mpc_command_monitor.py。
実試験は専有pp_speed20_lap_13へ1回、全体120wall秒/Start後90sim秒以内、
PP60000指令以内、V4/MPC OFF。前run12予算を引継ぎ、AWSIMは未改変。
約20km/hで344mの単純移動時間は62秒だが、発進・旋回・停止等を保証しない。

### run13結果

実行commit d1bc0cb7544c7be841a45055b8881878cc2df0d1。
既定CheckOnly/同期成功、WSL lockの限定testsは13 passed / 0.10s。
既定同期によるDatasetルート存在確認を実施。Dataset内容/raw/checkpoint読取りは未実施。
実ROSの状態購読は本試験として実施。全pytestはNOT_RUN。
network-noneのinstall XML gain確認とROS launch --show-args成功。
Windows/remote source/container installのreference.launch.xml SHA256一致:
764a5bb9f340fdc7a4aaff236b4d08dd582e12929bcc78b6d36d07e3fc9396da。
ホストからのinstall絶対symlink参照は失敗したため、実container内で確認した。

```text
make dev CONTROL_METHOD=pure_pursuit CAPTURE=false ROSBAG=false AWSIM_LAPS=1 RUN_ID=pp_speed20_lap_13 OUTPUT_HOST_ROOT=/home/graneple/e2e_autonomous/pp_speed20_lap_13/evidence
```

専有wrapper run.pyと外側timeout117秒+3秒graceで1回実施。
make終了0、公式Start完了、V4/MPC OFF、AWSIM本体変更なし。
最高実速度4.831439m/s=17.393km/h、目標5.524161m/s=19.887km/h。
最大観測加速要求2.762114m/s²。5km/hで張り付く挙動は再現しなかった。
ただし20km/h維持の達成ではない。P制御の定常偏差が考えられるが未同定。
3944 poseの折線長277.5455m、最後の経路進行ego_s=293.66m、ego_d=0.24m。
距離は完走判定ではなく、Finishなし。全体109.1228wall秒、最終sim79.115秒。
HOST_WALL_LIMITで走行中freeze/KILL。raw result FAILED/OBSERVER_EXITを保全。
最終速度4.3931m/sで、制動要求-1.5は見えるが制動停止成功ではない。
remaining_owned空、独立postrun inventoryでも全running/所有project残存なし。
初期化を含むwall上限で1周に届かなかった。時間延長だけで完走保証とはしない。

証拠はtmp/pp_speed20_lap_13/とremote同名run。run12の履歴は保全。
次の20km/h精度改善は、単なるgain増加ではなく加速上限と縦制御の定常偏差を
両立する設計を先に確認する。走行再試行は未実施、pushなし。
