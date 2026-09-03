# d1log_0902 全11 run V3学習記録

> **2026-09-03 corrective status:** この記録の`48dee900...` checkpointは、
> 学習時の`command_history`にprediction anchorの教師commandを含め、runtimeでは
> 直前のmodel command 1件だけを入力していた旧契約で作成された。full-control用途では
> **再利用不可**。修正版は`causal_previous_only`の過去10件へ学習・runtimeを統一し、
> markerを持たないcontrol-sequence artifactをloaderが拒否する。下記の数値は不具合の
> 発見に至った履歴として保持し、修正版の再学習・AWSIM再試験結果ではない。

## 目的と範囲

`d1log_0902.zip`の全11 runをrun単位で分割し、公開AICデータで事前学習した
TransFuser重みを初期値として、V3 full-control modelを学習する。大容量dataset、
checkpoint、rosbagはGitへ追加せず、WSL native filesystemだけに保存する。

公開データはcommand由来のproxy targetで、実測pose/velocityを持つfull-control教師ではない。
したがって公開データ側の学習結果は初期化にだけ使い、制御性能の根拠にはしない。

## Datasetとsplit

- source archive: `/home/thistle/e2e_autonomous/d1log_0902.zip`
- canonical Dataset V3: `/home/thistle/e2e_autonomous/datasets/d1log_0902_all_v3`
- Dataset manifest SHA-256: `aa253e94dc4286f41590722bc204371999c9efe77f31bc7a83f477676e8d43ee`
- sample: 48,946、run: 11、停止frame: 1,099
- split config: `configs/data/split_d1log_0902_v3.yaml`、seed: 7
- train: 7 run / 760 stop frames
- validation: 2 run (`131505`, `132822`) / 168 stop frames
- test: 2 run (`111236`, `142944`) / 171 stop frames
- train/validation/test run overlap: 0
- behavior view: `/home/thistle/e2e_autonomous/datasets/d1log_0902_all_behavior_v1`
- valid behavior label: 44,164

canonical Datasetのraw commandは変更しない。学習viewでcurrent-control教師を絶対範囲へ
clipし、未来control教師系列を同じsteering-rate、jerk、絶対範囲へ投影する。
command履歴はanchor/current教師を除いた過去だけを使用し、epoch先頭はinvalid maskで
左paddingする。未来系列の初期加速度も直前の有効な過去command（無ければ0）から取る。

## 公開データ初期重み

- run: `/home/thistle/e2e_autonomous/runs/public_transfuser_pretrain_1050b84_rerun1`
- best epoch: 96 / 100
- validation loss: 0.12952150852120858
- checkpoint SHA-256: `e2513b1e7f8c11b29df926e94e2022bd331ca21ab5d872e27e8f6e928d446c34`
- V3へ移行できたparameter key: 170
- shape mismatch: 7、未対応source key: 24、新規V3 key: 49

## 実行コマンド

Windows正本をcommitした後、WSLへ同一commitを同期する。

```powershell
tools/sync_to_wsl.ps1
```

WSLでは学習lockを保持し、100 optimizer stepごとに原子的に`last.pt`を更新する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  -m aic_transfuser_lite.cli train \
  --config configs/models/full_control_lite_v3.yaml \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_0902_all_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_0902_all_v3/split_manifest.json \
  --view-config configs/data/view_temporal_v3.yaml \
  --behavior-view /home/thistle/e2e_autonomous/datasets/d1log_0902_all_behavior_v1 \
  --output /home/thistle/e2e_autonomous/runs/d1log_0902_all_full_control_v3_public_init_a731194 \
  --epochs 5 --batch-size 2 --device cuda \
  --init-checkpoint \
  /home/thistle/e2e_autonomous/runs/public_transfuser_pretrain_1050b84_rerun1/best.pt
```

中断時は同一commandから`--init-checkpoint ...`を外し、`--resume`を追加する。
Dataset、split、view、model contractのidentityが異なるcheckpointは拒否される。

## 検証結果

### 学習完了とcheckpoint選定

- output: `/home/thistle/e2e_autonomous/runs/d1log_0902_all_full_control_v3_public_init_a731194`
- 完了: 5 epoch / 77,330 optimizer step
- best checkpoint: epoch 5、step 77,330
- `best.pt` / `last.pt` SHA-256: `48dee90011dfb973c0d8cfa64eb1c86b0006015a825721aa4578d7415f958877`
- runtime artifact SHA-256: `ffa2f7ba0fc84b2920c8eea52edb94011fa428abb7837c634c7a2308bb558bc6`
- run manifest SHA-256: `afb3da468646ed2603ffac75a7074fe23e4e8d092e24d1e9e784646f7b3c5a3e`
- offline selection SHA-256: `1949f0f9eeaa361744a92545eeed4886f164bc9596943c88cfd61fb1356dd848`

validation trajectory ADEをprimary metricとして、各epoch末checkpointを比較した。

| step | validation trajectory ADE [m] | speed profile MAE [m/s] |
|---:|---:|---:|
| 15,500 | 0.408571 | 0.232353 |
| 31,000 | 0.324109 | 0.217737 |
| 46,400 | 0.234654 | 0.179328 |
| 61,900 | 0.285008 | 0.194976 |
| 77,330 | **0.227777** | **0.167048** |

選定に使わなかったtest 9,112 sampleをbest確定後に一度だけ評価した。

- trajectory ADE: `0.2311720061 m`
- speed profile MAE: `0.1925457987 m/s`
- future control MAE `[steering rad, speed m/s, acceleration m/s^2]`:
  `[0.0587548475, 0.6287313041, 0.1338182615]`
- current control MAE `[steering rad, speed m/s, acceleration m/s^2]`:
  `[0.0346859309, 0.1275281772, 0.0427123318]`

### 自動test

- WSL、commit `a731194`: `pytest -q` -> `452 passed, 33 warnings`
- Graneple公式container: focused unit / negative test -> `104 passed, 15 warnings`
- Graneple ROS workspace: `colcon build --packages-select aic_e2e_runtime` -> 1 package成功
- CUDA checkpoint load: `MODEL_LOAD_OK cuda:0`

### Graneple shadow / 限定full-control診断

`graneple@192.168.3.10`へ上記checkpointとruntime artifactを配置し、
`ROS_DOMAIN_ID=1`のAWSIM/Autoware環境で実行した。既存のmap-fixed RVizを再利用した。

shadow 20秒ではtrajectory / shadow-controlを各189 sample観測したが、車両停止中に
予測control speedが平均`4.151729 m/s`、trajectory endpoint xが平均
`-0.388621 m`だった。学習教師のmoving sampleでは1.5秒先xが99.99%以上正方向なので、
全体的なtrajectory符号反転ではなく、少数の停止例に対するtrajectory Headとcontrol Headの
整合不足と判断する。このshadow結果はfull-control昇格gateを満たさない。

ユーザー要求に基づき、昇格ではなく0.8 m/s上限・15秒の一回限りの診断trialも実行した。

- final command publisher: Safety Supervisor 1 publisher、終了後0 publisher
- model output: trajectory / full-control nominal command 142 sample
- Safety reason: `lidar_no_valid_front_beams` 300 sample
- Safety final command: speed `0.0 m/s`、acceleration `-4.0 m/s^2`、steering `0.0 rad`
- vehicle speed: min / mean / max / finalすべて`0.0 m/s`
- displacement: `0.00000461 m`
- probe SHA-256: `4afafc72d31814b25cc82fb05bd904489790189a2e83b6afc2a30625670710e1`
- trial後: V3 inference / Safety node停止、final command publisher 0、vehicle speedほぼ0
- cleanup: 公式`make awsim-request-reset`はSSH環境の空`DISPLAY/XAUTHORITY` mountで
  action前に失敗したため、実行中の公式simulator containerから同じ
  `/admin/awsim/reset`を一度publishし、admin `WaitStart`を確認した

この最初の試験中はSafety SupervisorがLiDARを無効と判定して全commandを停止へ置換したため、
モデルによる走行は成立しなかった。その後、AWSIMが`WaitStart`のままだったことを画面と
topicで再確認し、Startを明示して再試験した。

### AWSIM Start後の継続試験

V3 full-controlとSafetyを先に起動し、final command publisherが1であることを確認した。
公式`request_awsim_start.bash`はvehicle `grounded`、initialization ready `false`のため
45秒でfail-closeし、Startを送らなかった。ユーザー指定の手動StartとしてDomain 0から
`/admin/awsim/start=true`を一度だけpublishし、admin stateが`Start`へ遷移した。

最初の15秒で`0.229944 m`、次の15秒で`0.486998 m`移動した。その後、model brake speed
`0.800000 m/s`がcalibration上限`0.798115 m/s`をわずかに超えて推論がfail-closeしたため、
trial speed capとSafety max speedを安全側の`0.75 m/s`へ下げてV3ノードだけを再起動した。

0.75 m/s設定での15秒観測結果は次のとおり。

- displacement: `0.271389 m`
- speed mean / max / final: `0.018098 / 0.065256 / 0.028215 m/s`
- Safety: `normal` 300、`lidar_future_timestamp` 1
- runtime: model control 102、trajectory fallback 38、trajectory publish 140
- final steering absolute max: `0.118169 rad`
- probe SHA-256: `f17b08ed7417709250f1fe18b7005f5e849dbbd574d189ee5d5ecbca21acdfe9`
- runtime config SHA-256: `b28370e122b0598289e1770a37a69a85061b332beb22c7613bf2b18bd7e35f6f`

この観測時点ではSafety `normal`、full-control command publish継続、AWSIM `Start`である。
発進と短距離移動は確認したが、速度が極端に低く、完走または安定走行の合格とはしない。

実行に用いた主要commandは次のとおり。

```bash
ros2 launch aic_e2e_runtime transfuser_lite_v3_full_control_trial.launch.py \
  param_file:=/artifacts/runtime.v3.full_control_trial.48dee900.yaml \
  model_path:=/artifacts/best.pt \
  artifact_manifest_path:=/artifacts/runtime_artifact.json \
  calibration_artifact_path:=/artifacts/calibration_v3_stable_2c8388f_shadow.json \
  full_control_evidence_path:=/work/configs/runtime/v3_full_control_trial_authorization.yaml \
  launch_rviz:=false use_sim_time:=true
```

## 未解決事項

- `48dee900...`は旧history契約のためfull-controlへ再配備しない。修正版commitから
  新規output directoryへ再学習し、offline選定とGranepleでのshadow/限定走行を
  やり直す必要がある。

- `full_control_lite_v3.yaml`の`gradient_accumulation_steps: 8`を現行V3 trainerが
  消費しておらず、今回のeffective batch sizeは2だった。次回学習前にtrainer実装と
  resume identityへ追加し、unit / negative testで固定する必要がある。
- GranepleのLiDAR topic自体は受信できるが、Safety前方beam抽出が全sampleを無効とした。
  LaserScanの角度範囲、front angle、NaN/inf/range filterを実値で照合する必要がある。
- 停止例は全split合計1,099 frameに限られ、停止時のtrajectory / control Head整合が弱い。
  stop/start hard-caseのoversamplingまたは整合lossを、test splitを触らず検討する。

## 未確認境界

この記録はoffline学習、独立test評価、GranepleでのROS 2起動、shadow観測、Safetyの
fail-close、手動Start後の短距離移動までを証明する。閉ループ完走、障害物回避、目標速度での
vehicle dynamics整合、長時間安定性は証明しない。未実行のROS 2／AWSIM試験は成功として
扱わない。

## causal history修正版の再学習・限定走行（2026-09-03）

`causal_previous_only`契約を導入したcommit `1b7d298`から、旧runとは別の
`/home/thistle/e2e_autonomous/runs/d1log_0902_all_full_control_v3_causal_1b7d298`
へ全11 runを5 epoch再学習した。公開データ事前学習checkpointは初期値としてのみ使用した。

- 完了: `77,330 / 77,330` optimizer step
- checkpoint SHA-256:
  `6e8fc01b55ba438f299731a01fd1e35ef7f853c2399f5514454eefab30f93d0e`
- runtime artifact SHA-256:
  `25acef141e5292779cc585d0cd80d699c27d63164afdbd2d0bb736382b399fb0`
- run manifest SHA-256:
  `e4e329e4fc559ce68c43848b97a455619ab968d6fef0366d13e4d26037524f40`
- model contract hash:
  `33053caf8564e37f56079e9493193dfd2e2698f91731dd0b20159645132e90ef`

走行前preflightで、停止時の計測速度ノイズ`-7.4e-8 m/s`が負速度として拒否される
問題を実測した。commit `9295ba9`で`1e-4 m/s`以内の負値だけを0へ正規化し、
それを超える実後退はfail-closeするunit/negative testを追加した。
WSL全suiteは`465 passed, 34 warnings`、Graneple公式containerのfocused suiteは
`34 passed`、`colcon build --packages-select aic_e2e_runtime`は1 package成功した。
Graneple source archive SHA-256は
`30a53d662fb286069ae4cfc053b112de13743ddeb08778b07c7769e783e113a2`である。

修正後preflight 10秒はSafety `normal` 200/200、model command 93件、
`previous speed exceeds authoritative limit` 0件だった。AWSIMを`WaitStart`から
`Start`へ明示的に遷移させ、0.75 m/s上限で30秒の限定走行を実行した。

- Safety: `normal` 600/600
- model decision: 270件すべてsame-trajectory external fallback
- fallback理由: `max_position_error_m>0.75`かつ`endpoint_error_m>0.75`
- final speed command mean / max: `0.239232 / 0.245674 m/s`
- final acceleration command mean / max: `0.237130 / 0.243511 m/s^2`
- vehicle speed mean / max / final: `0.002102 / 0.002166 / 0.002059 m/s`
- global displacement: `0.029100 m`
- trajectory endpoint mean: `[1.001813, -0.233156] m`
- measurement JSON SHA-256:
  `69a7bda15b7d7f1612218d0c347047cc1c175938366115b6c5995f143f722ff6`

Safety wiringは正常だったが、rollout整合gateがmodel control sequenceを全件拒否し、
fallback加速度もAWSIMの発進deadzoneを越えなかった。この試験は完走・発進成功ではない。
試験後はfull-control graphを停止し、final command publisher 0、AWSIM `WaitStart`、
longitudinal velocity `0.0 m/s`を実測確認した。
