# Spatial Path V4：対象版・空間教師coverage監査

## 1. 対象版

編集正本 `E:\workspace\e2e_lite_transfuser`、origin
`https://github.com/fis-teria/aichallenge_lite_transfuser.git`、branch
`codex/windows-wsl-training-sync`。開始時HEADは
`8c6caac124e33fed1802ce4abb384d2cef4d6ffb`、working treeはclean。
保存版 `2989f9389415c121824c585754b8e10d7904a659` と開始時HEADの差は
`docs/astra_pro_spatial_path_v4_design_prompt.md` のみ。対象V3コードは保存版と同じ。
実験A `7a06d37e1a71740040079b42041fb41f5878020e`、判定名修正
`7ac377ec359083dcda1d2ebb854d7d65112b04a3` はローカルobjectとして存在し、HEADの祖先。
Webでの過去422はローカルobjectの不存在を意味しない。今回pushしない。
WSL開始版は保存版2989f93。監査実装をWindowsでcommit後、既定同期で同一版を検証する。

## 2. source・座標契約

canonical converterは30点、0.1秒間隔、3秒の観測futureを保存する。
モデル/loaderは先頭15点（1.5秒）を使用する。配列は `[H,8]`：
`time_sec,x_m,y_m,yaw_rad,longitudinal_mps,lateral_mps,yaw_rate_rps,valid`。
converterの世界→観測ego変換は `x=cos(yaw)*dx+sin(yaw)*dy`、
`y=-sin(yaw)*dx+cos(yaw)*dy`。yawは相対角rad。
schema上は `base_link@t_obs`。後輪中心との一致は未確認。
無効futureはmask=0、状態値NaN（時刻はconverterでは有限）。
canonical sample metadataには絶対poseの連続列は保存されない。
futureを別anchorからつないで長期poseに偽装しない。

manifest内部identityとmanifestファイルSHAは別物。
内部identity、split結合、samples/runsのSHA、処理対象futureのSHAを検証する。
Camera/LiDAR/rosbag payloadはhash/展開しない。
behaviorはmixed Datasetに結合、phaseはrecovery親Datasetに属するので、
親manifest identity・run/source subset・全trajectory inventory hash一致を確認した場合だけ結合する。
metadata inventoryはPRESENT/MISSING/UNREADABLE/UNSUPPORTED/NOT_INSPECTEDを区別する。
raw metadataのtopic存在はpayload使用済みを意味しない。

## 3. ledger・幾何・母数

sample metadata全anchorをledgerに残す。停止、ego無効、futureゼロ、filter除外、
test、処理上限到達後のanchorも消さない。非検査の距離・品質集合判定はnull。
primary exclusionはV3と同順（command欠損→futureゼロ→ego無効→矛盾停止）。
同時成立する複数flagも別列に保持する。censoredは追加除外しない。
V3 helperを直接使い、nominal優先・finite検査・float32 clip・final fallback、
`max(max(v_long,0))` を維持する。最大絶対速度に変更しない。

h15/h20/h30を別集計。raw arcは原点を含む連続prefixの折線長で、X変位ではない。
無効mask、時間逆行、0.2秒超gap、20m/s超位置飛びをつながない。
valid count/終端XY/最大変位はdisconnectedな有効点も含み、prefix countとは区別する。
ノイズ診断は最後の採用点から5mmの累積変位を待つ。各frameの小変位を捨てない。
速度<=0.01m/sかつ原点から10mm未満の停滞は暫定jitter扱い。
raw長と暫定処理長は両方保持し、0.5/1/1.5/2m到達数の分母を明記する。
停止時間を距離へ換算しない。曲率は空間支持0.2m・segment2cm以上のみ暫定計算、
それ以外はnull+理由。短経路の曲率0を「直線」と解釈しない。
10cm再標本化関数は診断専用：外挿・終端重複なし、角の切り落とし距離を返す。

閾値はセンサ校正済みではない。noise-filtered coverageは安全/学習採用coverageではない。
path-loss eligibility、continuation不可、発進許可、安全endpointはunknown/null。
time horizon終端・run終端・hold・観測欠損は負のcontinuation教師ではない。
collection caseはpreflight、phase/geometryはphase annotationを使用。
名前から直線や実測左右変位を補完しない。session/確定episodeはunknown。
stopped-commandedのepisodeはrun/clock epoch内0.5秒gapによる推定で、独立試行数ではない。
test幾何は既定off。`--detailed-test --splits test` の別出力のみで、閾値調整に使用しない。

## 4. 長期pose・Referenceの範囲

今回は保存済み30点までの観測geometryを診断する。これはoracle A/BやV4正式教師生成ではない。
raw odometryはmetadata上の存在を検査するが、圧縮bagの展開・全payload抽出は行わない。
任意の連続pose readerに接続できる境界validatorを合成試験する：
run/split/segment/reset/route変更、unknown intent、逆走、teleport、gap、長holdで打切り。
候補上限は1.5/2/3秒・2m。実rawへの接続とsensor時刻整合は未実施。
したがって長期pose教師coverageを0や安全な3秒教師と断定しない。

回復収集runの隣に `recovery_reference_v3.csv`、interval CSV、base Referenceが存在する。
metadata inventoryでhash・点数・列・幾何長・最大segmentを検査する。
絶対座標CSVにframe/timestampの保証はなく、anchorとの整列、route連続性、
車体輪郭/後輪基準/最大曲率、壁clearance、推論時route意図の取得は未確定。
Reference存在を安全な正解と扱わず、teacher-onlyとして保持する。

## 5. V3 runtime・評価の読取結果（変更なし）

呼出経路：ROS `inference_node_v3` → `build_executable_reference_v3` →
`control_from_executable_reference_v3` → delay-aware waypoint tracker +
`LongitudinalControllerV3`。横制御は `atan(wheelbase*2*y/(x²+y²))` と操舵rate制限、
縦はlaunch/PI/jerk/fault状態機械。選択profileは縦横MPCではない。
収集teacherの名称とは別であり、preflight上のteacher IDもinventoryに残す。
モデルXYを使い、speed headは学習/整合性lossに残すがruntimeでは外部0.75m/sへ置換。
曲率/ODD capと任意Safety cap、後段Safetyを適用する。
`stop_probability=None`、require=false、enable_model_stop=false。dummy stop未追加。
profileのSafety cap=0は「このoptional capなし」で、安全監視全体の無効化ではない。

先頭X<=1mmは、先頭ノイズ半径<=5cmかつ前方点が2点以上残る場合のみtrim。
短すぎるpathや非回復可能後方向は拒否。既存点をretimeし、pathを自動延長しない。
previewは0.5秒、minimum arc lookahead設定1mでも実path終端へclampするため、
設定1mは実際の1m支持の保証ではない。縦horizon0.5秒、wheelbase1.087mは設定値。
V4 MPCが必要とするpath horizon/後輪基準/車体制約は次の設計契約で確定する必要がある。
推論入力はCamera/LiDAR/ego/past commands。planned recovery route意図を入力する配線はない。
履歴4/4/10/10、past-only command、mask、gap/resetは実装/既存回帰試験で監査する。

readyはstopped abs(speed)<=0.05、selected command>=0.5の未filter cohortに対して、
Reference受理・終端X>=0.1m・controller要求速度>=0.2m/s・縦faultなし。
replayは毎anchor縦controllerをfreshにし、drive preflight=Trueと仮定する。
実停止理由/発進可否はunknownを含む。最低20anchors/2runs/推定3episodes、ready>=80%。
ADE非悪化は別gate。readyは走行・安全・M3の証明ではない。

比較CLIはper_sample.csv全行をbootstrapへ渡す。sample IDとrun一致を要求し、
非finiteは拒否、欠損ペアを勝手に落とさない。各frame差をrun内平均、runを等重みで
10,000回seed42再抽出し、同じpaired runをbaseline/candidateに使用する。
5 runsでsession相関は未補正。teacher-quality subsetのwaypoint重み付きADEとは
集合も重みも異なり、bootstrap CIを主ADEのCIと呼べない（別タスクの修正候補）。
提供値「worst-run FDE 0.353609」は保存版文書では全体FDE欄で、worst-runの値と断定できない。
trim96.79%は `trim_count>0` のanchor率（分母530）で、削除距離率ではない。
過去M3未達・今回ROS未実行・collision-clear未確認を維持する。

## 6. 実行方法・再現性

Windows側でこの監査の新規ファイルのみcommit、`tools/sync_to_wsl.ps1 -CheckOnly`、
通常syncの順。既存学習/lockを停止・削除しない。WSLで：

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_spatial_coverage_v4.py
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh .venv/bin/python tools/audit_spatial_coverage_v4.py \
  --dataset-root /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_v3 \
  --split-manifest /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_split_manifest.json \
  --expected-identity 181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388 \
  --behavior-view /home/thistle/e2e_autonomous/datasets/d1log_recovery_mixed_20260904_behavior_v1 \
  --phase-view /home/thistle/e2e_autonomous/datasets/recovery_20260904_phase_view_v1 \
  --phase-parent /home/thistle/e2e_autonomous/datasets/recovery_20260904_v3 \
  --max-anchors 100000 --max-seconds 600 \
  --output /home/thistle/e2e_autonomous/runs/spatial_v4_coverage_full_20260905
```

先に `--max-anchors 12` と別の新規outputでsmoke実行する。
outputはimmutable（既存path拒否）、sourceと重なる出力を拒否。
manifestにcommit/dirty/code hash/config/環境/コマンド/入力identity/上限/実件数を記録。
上限到達やasset読取エラーはPARTIAL、版・identity問題はBLOCKED。
COMPLETEは宣言した監査scopeの完了であり、V4 teacher採用条件の完了ではない。
全pytestは合成fixtureによる学習unit testを含むが、実データ学習は実行しない。

## 7. 次gate

versioned converterの正式教師生成・oracle A/Bは未実行。
次は同一train/val anchorsでA=15点、B=厳密境界付き長期poseを同一空間再標本化して比較する。
その前にpose基準・timestamp/reset/route意図・hold理由とcalibration・clearanceを確定する。
不足中に長pathを外挿したり、停止失敗をsafe endpointとするconverterは作らない。
監査のraw支持が長くても、安全かつ追従可能な教師の採用可否は別gate。

## 8. 今回の実測・検証結果

実行後にこの節へartifact identity・実件数・テスト結果を記録する。
