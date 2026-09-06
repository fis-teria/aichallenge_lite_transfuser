# V4保存経路 → 速度計画 → 縦横MPC → 仮想車両：限定統合結果

## 結論（独立判定）

|判定対象|今回の結果|
|---|---|
|コードの実装|実装済み。保存packet adapter・幾何検査・速度/時間参照・SLSQP NMPC・独立RK4 plant・offline CLI。|
|本物のMPCと再最適化|確認済み。主試験641回、smoke97回。毎周期の現在stateから解き、先頭操作だけをplantへ適用。|
|仮想車両での合成経路追従|8 sceneを実行。暫定目標5/8。初期横偏差2件は収束後maxの目標未達、短経路切替1件は停止距離不足。|
|固定V4保存経路での追従|未成立。指定12件は全て幾何検査で拒否（操舵幾何6、cusp/反転6）。保存pathのMPC/plant cycleは0。入力読取・固定選択・拒否までの統合を確認。|
|実時間deadline|offline受理上限0.5sは主試験641/641で満たしたが、0.1sは337/641。10Hz実時間性は未達。同期solverのhard cancellationなし。|
|teacher/inputの物理正当性、実clearance|今回未検証。savedはSAVED_PATH_OPEN_SPACE_ASSUMED、clearance_verified=null。|
|ROS・E2E perception closed loop・走行・Safety|未実行・未承認。CONTROL_CLOSED_LOOP_ON_FROZEN_PATHのみ。|

自己点検およびローカル実行結果であり、独立レビュー合格ではない。
全scene性能合格は実装完了条件にしないが、保存V4追従成功とは報告しない。
幾何拒否の原因を学習不足・データ不足・MPC限界と断定しない。原因はUNKNOWN。

## 版、既存実装、依存

origin https://github.com/fis-teria/aichallenge_lite_transfuser.git
branch codex/windows-wsl-training-sync
開始 c5bbf9a689ce1f96ee60c3df787fb5ab5e2797a7、clean。
基準bootstrap cfc419b、モデル学習f33b197、validation153a22a、annotation2386f0f、梱包7acc1efは全てローカルobject存在・開始HEADの祖先。
既存MPC関連の今回ローカル変更はなく、既存変更の破棄・stash・上書きなし。

実装/実行/test/trace最終版 af4176f7182e5a2bd9922720edf5a66c59a2759a。
途中版 f8ff96b00eccd4abeaa5e3d3af3f246c9e9664cc、f00ff5b5857f397db7c1ff89b991cbe426ee200a。
結果文書版はこの文書追加commit、正確なSHAはprovenance/package_versions.json。
梱包はlocal未commit spatial_mpc_integration_review_af4176f.zip。自動pushなし。
WSLは最終実行版を保持し、結果文書commitへ遡及して実行済みとしない。

既存delay_aware_controller.pyは先読み位置→atan曲率操舵とspeed_kpによる速度制御、
longitudinal_controller_v3.pyはPI。有限horizon最適化ではないため改名・置換せず、pure NMPCを別coreへ追加。
既存racingkart設定にはwheelbase=1.087m、steering=0.64rad、出典revision/shaがあるが実測校正済みとはしない。
今回は依頼のVIRTUAL_TEST_ONLY profile（wheelbase1.0m、幅0.6m、前後overhang0.2m、v_nom0.5/v_max0.75m/s、delta0.5rad等）を使用。
base_link_equals_virtual_rear_axleは合成比較の仮定であり実座標変換の確認ではない。

SciPyが既存venvに無かったため、許可された隔離venvへ scipy==1.15.3を追加した。
37.7MB wheel、numpy2.2.6維持、torch2.7.1+cu128維持。グローバルinstall/torch CUDA更新なし。
pyproject.tomlのoptional offline-mpc依存へ同じversionを追加。
Python3.10.12、Matplotlib3.10.9、pytest9.1.1。
公式API: https://docs.scipy.org/doc/scipy-1.15.3/reference/optimize.minimize-slsqp.html
現在版docsは1.18を表示するため1.15.3へ切り替えて確認。workers/multipliers等の新APIには依存しない。
SLSQPのmaxiter45/ftol1e-6、独立制約許容1e-5、wall受理上限0.5秒は試験前に固定。

## 実装と数値契約

- spatial_tracking_contracts_v4.py: Candidate/PreparedPath/TimedReference/MpcResultを分離。
- spatial_path_adapter_v4.py: 原float32を変更せずfloat64 copy。既知原点はindex=-1。
  actual_sは各segment長の累積。nominal_sや終端Xを弧長にしない。
  微小segment<1e-5mはindexを残してduplicate除外、非有限/穴は全拒否し橋渡ししない。
  segment>0.5m、cusp角>1.57rad、非隣接交差/接触、操舵幾何を理由別に拒否。
  局所曲率はheading差/(隣接2 segment長平均)。支持距離はturn_support_m配列が実値。
  config geometry_support_m=0.1は名目説明値であり、固定窓平滑化には使用しない。
  操舵相当上限 tan(0.5)/1.0=0.54630249 [1/m]。polylineの頂点角は有限支持の診断であり連続曲率の厳密値ではない。
- spatial_speed_profile_v4.py: mission/vehicle/curvature/steering-rate/終端/permission capを別保存。
  位置は元polyline内のみ補間。各spatial capの最小値を用いた保守的rest-to-rest quintic時間則を採用。
  v=0から発進でき、解析上のv/a/jerk最大からdurationを決定。終端は0.1m margin。
  horizon後半は同じ終端位置v=0を保持し、空間長を増やさない。
  定減速sqrt(2bd)はcapの概算。別の小刻みjerk制限braking積分で残距離不足を判定。
- spatial_mpc_v4.py: z=[x,y,yaw,v,delta]、u=[a,delta_rate]、Euler dt0.1s、15step direct shooting。
  位置/姿勢/速度/操作量/操作変化/終端位置の二次目的。SI unit scaleは全て1、重みはresolved config。
  boundsと非線形不等式でv非負/上限、a/制動、舵角/rate、前周期aを含むjerk、横加速度、既知footprintを制約。
  horizon末端へのhard停止は一律強制しない。終端位置はsoft objective、制約は別の不等式である。
  solver successに加えfinite/全制約残差/復帰後wall-timeを検査して受理。失敗時は別のbounded braking。
  warm startは操作初期値のみ。予測stateをplantへコピーしない。
- spatial_tracking_sim_v4.py: RK4 plantを0.02sで5 substep。
  要求/適用操作、速度0への片側接触、舵角/速度飽和を明示記録。rateを二重積分しない。
  pathへplant stateを強制投影しない。progress対応のみ、前回s・到達可能範囲・headingを使う局所検索。
  終端への接線方向overshootを別量に残す。別anchorを時系列に接続しない。
  拘束rollout/参照/plantのfootprintは別検査。参照は0.02m空間サンプル、rolloutはdt0.1s、plantは0.02s。
  離散点間の連続衝突ゼロを厳密証明したわけではない。
- tools/evaluate_spatial_mpc_v4.py: explicit packet/config/output/budget。モデル/ROS/live controllerはimportしない。
  8合成sceneと12保存pathの選択・configをsolver実行前にファイル保存。再選択・sweepなし。
  full raw path、PreparedPath、使用prefixのs/切断理由、全参照、全最適化state/control、要求/適用u、plant substepsを記録。

使用prefixは原polylineの0〜endpoint_s区間。原座標/actual_s/endpoint_sから再現できる。
拒否pathには使用可能prefixなし、PreparedPathは空。raw20点は別NPZに全て保存。
plotのstate/速度はcycleのnext_stateをcycle開始tに配置するため、厳密な状態時刻はログのvirtual_time_s+dtを参照する。
predictor/plantは独立関数・異なる離散化だが同じ運動学bicycle仮定を共有し、実車妥当性の独立検証ではない。

## 固定入力と選択

入力packet: spatial_v4_validation_review_20260906_153a22a.zip（明示パス）。
ZIPの単一root prefixを確認。entryの重複/危険path/symlinkを拒否し、依頼allowlistだけを読取。
各対象entryはmanifest size/hashと指定SHAを照合。JSON identityはidentity field除外canonical hashで再計算。
NPZはallow_pickle=False、keys=[sample_ids,xy,processed]、xy float32 [180,20,2]、objectなし。
元prediction hash d033997378bdcb9fa0cfc7efb038632ee840a0057561b43142ba75b53ee3d90f。
input identity 77fff3f9c5875b6f116cebe35fde88d42d459aa04c195676c44e5c6caca5edc7。
teacher identity cdb668834d4baf60c31fa5f934d01d8782f02544bc6fa29053d5a850432cc344。
selection identity 59393d98ad4e55a59da515ff324a147995889e98b48b3a7ac676660400e3851f。
元checkpoint期待hash0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f、現物未確認・未読取。

mainの5runからsample_id辞書順各2件、固定最悪例、残りstraight先頭1件、計12件を固定。
対応source NPZ rowは [0,20,41,58,78,100,125,123,162,156,26,2]。
最悪ID 20260902-131505__epoch0000__76292918933 はsaved_26。
run/sample/shapeは選択と記録のmetadataのみ、数値controller・参照生成には渡していない。
teacher XY/mask/未来poseは制御入力に未使用。annotationは元ID補足として同梱するが数値計算に使わない。
packet外Dataset/raw/sensorは未読。

config canonical SHA b1cc9bf1a8e78cee257bde31840b0ab26c9710681d48a221c12d32e0c225be8c。
全scene selection SHA ca8d5f5a338a4861fe3ff658db18192ccce7c73911e01e2304b63d517cd9db24。
保存選択SHA bec1e4d1ef1f28e4239a6c508ed83ad83470c18c38612517736c763222f844cd。
config/vehicle/solver identityは同一resolved configへ結合（分離slice hashはpackage provenanceにも記載）。

## scene結果

収束後は固定t>=2.0s。初期誤差を含む全期間のRMS/maxも各summaryに残す。
目標: progress>=90%、収束後RMS<=0.05m/max<=0.10m、終端距離<=0.10m、v<=0.03m/s、
制約<=1e-5、衝突/停止不足/fallbackなし。仮想診断基準であり実車安全基準ではない。

|scene|cycle / solver / fallback|収束後CTE RMS / max (m)|終端距離(m)|最終v(m/s)|結果|
|---|---|---|---|---|---|
|straight|97 / 97 / 0|3.57e-10 / 6.48e-10|0.000456|0.000832|目標達成|
|left|96 / 96 / 0|0.002696 / 0.004657|0.001994|0.000945|目標達成|
|right|96 / 96 / 0|0.002696 / 0.004657|0.001995|0.000945|目標達成|
|offset_left|97 / 97 / 0|0.037943 / 0.111405|0.000747|0.000832|max未達|
|offset_right|97 / 97 / 0|0.037943 / 0.111405|0.000747|0.000832|max未達|
|yaw_offset|97 / 97 / 0|0.005152 / 0.013155|0.000457|0.000832|目標達成|
|short_switch|31 / 25 / 6|5.10e-10 / 6.27e-10|0.050468|0.000405|停止不足・overshoot|
|obstacle|36 / 36 / 0|2.30e-10 / 2.65e-10|0.001910|0.003433|打切り終端で停止|

左右偏差は初期±0.12m。最終位置は収束したが2s以降最大0.1114mのため不合格のまま。
短経路はt=2.5sで同じworld pathを残0.08mへ切断、STOP_DISTANCE_INSUFFICIENTを検出。
6cycle brakingで停止したが約0.05047m先へ進んだ。path投影/clampで隠していない。
障害物はx=2.1〜2.3mの矩形。車体前端1.2mを含むpath footprintによりprefixを0.88mへtrim、
margin後目標s=0.78m。node中心だけで2.1mまで進めたわけではない。
主試験の記録されたplant/rollout制約残差は許容内、既知sceneのサンプルfootprint衝突なし。

保存12件は全て拒否、solver0、追従誤差/実時間測定はNOT_EXECUTED。
操舵幾何6件の最大局所曲率は2.29875〜4.60656 [1/m]で、仮想profile上限0.54630を超えた。
cusp6件の最大隣接方向変化は1.79255〜1.99201radで、固定基準1.57radを超えた。
最悪saved_26はraw index16付近で局所曲率4.31737、支持0.08729m、方向変化-0.37686rad。
これらは「指定の未補正折線＋今回の有限支持幾何gateでの拒否」であり、学習原因・実走行不可能の一般証明ではない。
正しい大回頭曲線のX減少自体を拒否しないtestを通した。曲線の形状を補正して合格へ変えていない。

## test / run / 予算

最終run /home/thistle/e2e_autonomous/runs/spatial_mpc_task_20260906/af4176f。
Windows commit→CheckOnly→sync→同一SHA WSL lockで実行。
最終限定pytest: tests/test_spatial_mpc_v4.py、tests/test_spatial_tracking_sim_v4.py、tests/test_spatial_bootstrap_v4.py。
197 passed / 1 skipped in 7.56s、exit0。
skipは未導入jsonschemaの完全Draft2020 validator。新規依存追加をこのvalidatorへ広げない。
unit実solverは4 calls/実行、3回の限定pytestで合計12 calls（fault injectionも実SLSQP復帰後に棄却）。
unitの短いplant testは主scene cycleと別。bootstrapのfake forwardは実V4推論ではない。
既存全pytest、Dataset tests、学習optimizerは実行していない。

過去試行を保存:
f8ff96b: 195 passed/1 skipped (11.16s)。その後CLI import失敗でsmoke開始前終了、主cycle0。
f00ff5b: 196 passed/1 failed/1 skipped (9.69s)。交差が非隣接segment端点上にある反例を追加し検出漏れを修正、主cycle0。
af4176f: 197 passed/1 skipped。smoke97cycle/97solver、その後main647cycle/641solver、main再実行なし。
設定/閾値変更なし。直線smoke重複も共通予算へ含む。
累積主cycle744/3000、solver738、main実行1/最大2。
CLI active時間81.4633秒 + 後処理0.4923秒。
CLI import失敗はBudget初期化前でduration未計測。0秒とは扱わず、
pytest全実行や未計測開始処理を含め保守的120秒を別にchargeし、予算確認は約201.956秒/3600秒とする。
これは実測値ではなく上限確認用の保守計上。追加scene/retryは実行しない。
各run directoryやsmoke/mainで既存cycle/active budgetをリセットしていない。
保存不能/中断時のpartial出力を成功へ修復していない。

主試験641 solverは全て0.5秒以内で受理。max0.334018秒、p95=0.174559秒。
0.1秒以内337/641 (約52.6%)。virtual dt=0.1sとwall実測は別。
fault unitではsolver failure / timeout / nonfiniteを棄却しbrakingへ切替。hard realtime中断は保証しない。
HOLD/staleは外部合成入力でありstop head/teacher代用なし。
brakingの停止可能な低速caseはplantのv=0接触を確認、停止不能caseはshort_switchでovershootを測定した。

## first_error、未実行境界、次工程

bootstrap finally入口で、既知writer.errorがありfirst_errorが空の場合だけ先に保存する2行修正。
receipt failure→cleanup-entry clock failureの追加testでfirst_error=OSError: FAKE_RECEIPT_FAILUREを維持。
後発時計はtiming_errors、saved={0}、dropped={}、exit5、node/context/writer cleanupを継続。
この付随修正のみを主成果物にせずMPC全結果と同梱した。

既定syncを3回、各CheckOnly/通常syncで固定Datasetルート存在判定を実施。
Dataset内容・raw・sensor・checkpointの読取り/探索は未実施。checkpoint stat/hashも未実施。
model新規推論、学習、ROS初期化、DDS、AWSIM、制御publish、active runtime/Safety置換、pushなし。
書いたfalse flagだけを証拠にせず、実行driver・依存・pure code path・生ログを同梱。

implementation_authorized: true
offline_mpc_optimization_authorized: true
virtual_vehicle_rollout_authorized: true
control_closed_loop_scope: FROZEN_PATH_AND_SYNTHETIC_REFERENCE
new_model_inference_authorized: false
training_authorized: false
real_ros_connection_authorized: false
live_sensor_subscription_authorized: false
real_control_publish_authorized: false
raw_execution_authorized: false
awsim_or_vehicle_driving_authorized: false
runtime_promotion_authorized: false
live_approval_gate: PENDING_EXPLICIT_AUTHORIZATION

次の実入力接続に必要なのはtopic/type/frame/clock、base_linkと後輪中心offset、車両・actuator profile、
外部RUN/HOLD・可走領域の出典、および明示承認。今回の仮想profileを実車へ昇格しない。
その前に、今回の保存経路の幾何拒否結果とMPC契約を独立レビューへ渡せる状態とした。
S1/Ledgerや無関係なraw監査の全面完了を追加条件にはしない。
