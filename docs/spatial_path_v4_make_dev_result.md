# V4 + MPC make dev: 実入力接続結果、走行は未達

## 結論と版

AWSIMの既存実行物を変更せず、Lite TransFuser repositoryに追加した専用`make dev`から
新Camera/LiDAR/ego → 固定step500 V4 → 制約付き参照 → HOLD用MPCまで実行した。
CUDA forwardは60回、MPC実solveは1回。MPC由来の送信は0回であり、閉ループ走行は未成立。
停止確認から発進・追従・制動成功へ昇格しない。AWSIM側の改修は不要と断定もしない。
現在の許可境界では、足りない走行証拠をtrueへ補完せず終了する。

- 今回継続開始HEAD: `d015a9c04e8aac1c2633e2518a5a3e6fec991bac`。
- 接続実装/主要限定test: `690dbb4705e22a5c1f128ebf697bebe27449986b`。
- コンテナ識別修正: `0afadefd5de1e02f2b08b7a372c13f9c5eded1c1`。
- 最終実行版: `c48f823023e67e091e7869b8ff7077e09109e544`。
- Branch: `codex/windows-wsl-training-sync`。自動push未実施。
- Host: `graneple@192.168.3.10`、AWSIM AIChallenge2026 / GoKart1、既存dev image。
- Run: `/home/graneple/e2e_autonomous/spatial_sim_e2e_20260906/dev_stationary_c48f823_03`。
- ローカル証拠: `tmp/spatial_sim_e2e_20260906/dev_evidence/`、集計: 同`dev_results/`。
- 結果文書/梱包版はpacketの`source/versions.json`で実行版から分離する。

従来のracing-kart Makefileそのものは実行していない。
専用targetが、未改変の`aichallenge/run_simulator.bash dev`と元AWSIMをread-only bindして起動する。
元targetに含まれるclassic controller、route由来heading、source/build/install全体のhash処理は起動しない。
詳細・実行commandは`docs/spatial_path_v4_make_dev.md`。

## 独立した到達判定

| 項目 | 今回の実値 |
|---|---|
| IMPLEMENTATION_COMPLETE | false: sensor→MPCコードは存在するが、走行用のpose/footprint/collision結合が未完 |
| SIM_ISOLATION_VERIFIED_FOR_SELECTED_INSTANCE | true: この専用instanceのinspect・network・実consumerを確認 |
| LIVE_V4_INFERENCE_EXECUTED_IN_SIM | true: 新観測で60回、CUDA、全model state前後一致 |
| MPC_CONTROL_PUBLISHED_TO_SIM | false: 0回 |
| E2E_SIM_CLOSED_LOOP_EXECUTED | false |
| LOW_SPEED_RUN_AND_STOP_PASSED | false |
| REALTIME_AT_1X | NOT_ESTABLISHED: time-scale設定実値未取得、閉ループなし |

隔離はROS_DOMAINだけでなく、simulator network none + runtime同一namespace、
loのみ・routeなし・非privileged・cap-drop ALL・物理device/bridge/Docker socketなしを確認した。
実control subscriberは`awsim_d1`。V4 publisher作成前に競合publisher 0を確認し、
作成後は1を監視した。mode/gearのconsumerも先に確認した。
これは選択instanceの結果であり、既存の通常dev/実車環境へ一般化しない。

## 全make dev attempt

| Attempt | 実行版 | 結果 | 課金したsim wall秒 |
|---|---|---|---:|
| dev_stationary_690dbb4_01 | 690dbb4 | hostnameをcontainer ID prefixと仮定した検査で停止、推論0 | 3.385854 |
| dev_stationary_0afadef_02 | 0afadef | ROS既定`//.ros/log`が書込不可、推論0 | 3.257373 |
| dev_stationary_c48f823_03 | c48f823 | 実入力V4 60、HOLD MPC 1、MPC送信0、正常終了 | 27.973301 |

失敗ログも保存。1回目のidentity仮定をDocker inspectの実Hostname/Cmd照合に修正。
2回目のROS_LOG_DIRを作業用`/evidence/ros_logs`へ固定。
AWSIMのファイルや既存checkoutは変更せず、各修正後はWindows commit→既定sync→限定test。
container hostnameは単独の隔離証明ではなく、他のnetwork/mount検査と併用する。

## 入力・参照・MPC

最終runはCamera114件、入力bundle102件。処理段階別に分けるとINPUT_NOT_READY40件、
CYCLE62件（うちforward前command不足2件）、実forward/raw保存60件。
40件のINPUT_NOT_READYには複数履歴slotの再試行があり、alignment拒否件数と母数は異なる。
Camera grid不適合50、LiDAR tolerance超過19、ego bracket不足1の**出現回数**を記録した。
cameraは約105ms間隔で100ms gridとのずれが蓄積する。40ms許容は緩めず、欠落を保持した。

V4は既存strict loaderで指定checkpoint1本を復元し、eval/inference_mode。
固定SHA: `0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`。
WSL→sim hostへ直接コピーし、WindowsやGit/ZIPに重みを保存していない。
load mapと全state前後inventoryあり。60件すべてのraw float32[20,2]/160bytes/hashを
保存ログから再照合した。入力は9 tensors、4/4/10/10、padding/maskと時刻を記録。
最初のsnapshotのみ保存したため、後続のsensor画像の再現はhashだけでは不可能。
最初の画像snapshotは黒画像相当であり、良好な撮像品質を証明しない。
後続image tensor hashは変化しているが、全frameの視覚品質は未検証。

commandは実送信receiptのみの`SIM_ONLY_POLICY_CHANGED`。
requested/sent/appliedを区別し、simulator内部適用確認はUNKNOWN。
AWSIMが無視するspeed fieldはdesired referenceで、実測速度や学習nominalとの完全parityとはしない。
同じforward後の操作やteacher mask/未来routeは入力へ渡していない。

| CYCLEの終了理由 | 件数 |
|---|---:|
| REFERENCE_INITIAL_STATE_LIMIT | 19 |
| REFERENCE_BOUNDED_FIT_INFEASIBLE | 37 |
| INTERPOLATION_missing_bracket | 3 |
| COMMAND_SOURCE_STALE_OR_MISSING（forward前） | 2 |
| CONTROL_DT_INVALID（MPC後） | 1 |

参照幾何受理4/60。受理参照の最大偏差証明上限は0.08637〜0.08828m（固定上限0.10m以下）。
FIRST_CUSPによる固定prefix/tail選択も保持し、名目2mを実使用可能長としない。
最初にV4候補を止めた直接理由はINITIAL_STATE_LIMIT。ゼロ近傍の負速度も補正せず、
既存の前進モデル初期状態条件で拒否した。停止付近の符号扱いは今後の入力/制御整合課題。
37件のfit不成立から、学習データ不足等を今回の確定原因とはしない。

MPCに到達した1件は**HOLDのみ**。SLSQP1回、6.318ms、最大制約違反0、u全0。
permission/safety/final速度capは0、usable end 1.361922m、stop target/remaining 1.261922m。
source pose/current poseを別にし、観測時参照を一度worldへ変換した。
GNSS+IMU時刻結合は診断扱いであり、完全な適格性は未証明。
現scanによる全車体検査は21560 cells中17945 cells UNKNOWN、obstructed0。
obstructed0はfreeの証拠ではない。前方LiDARの後方にある車体領域を既知freeにしていない。
rolloutもUNKNOWNとして拒否。さらに同じsim時計tick内では実送信間隔が0になり、
dispatchのCONTROL_DT_INVALIDで要求生成も拒否した。便宜的なdt=0.1への置換はしていない。
このため、MPC提案が送信されたとは報告できない。

## 実送信・移動・停止・時間

実送信532回 = supervisor HOLD208 + STOP324。MPC送信0、正加速度0、powered episode0。
停止loopはcallback処理ごとにpublishしているため20Hz保証ではなく、324を20Hzの証拠にしない。
mode1回、gear1回を専用sim consumerへ送信。
観測最大|v|=0.0001565702m/s、GNSS net displacement=0.0000796539mで、実走進行は未成立。
GNSS差分累積0.040949mはノイズを含み、走行距離に使わない。
終端で新しい速度観測が0.03m/s以下を1sim秒維持したが、元々停止中。
発進後の自然制動成功はNOT_EXECUTED。collision/departureはUNKNOWN。

推論wall: median7.869ms、p95 9.747ms、max337.332ms（初回を除外しない）。
Camera受信→CYCLE記録終了wall: median100.561ms、p95 139.441ms、max422.533ms。
200ms超過2/60。MPCに到達しなかった拒否cycleも含むが、未forward40件は別母数。
制御送信込みの10Hzやtime-scale=1動作の証拠ではない。
独立watchdogは合成試験済みだが、powered状態でのsolver停止/host pauseは実測していない。
host pause後の終了処理もpoweredで未検証であり、運用可能な停止系完成とは扱わない。
実sim画面の動画は取得していない。添付のraw/reference図は数値診断図で、実画面ではない。

## 残る直接gateと再開範囲

走行の直接blockerは**motion permissionを成立させる実環境証拠が不足していること**。
pose timing、body footprint profile、全車体のcurrent free space、live collision monitorを
未検証のままtrueにしない。コードもfalse固定なので`V4_PHASE=run`だけでは進まない。
collision/status binderは未実装で、通常runtimeを縦横MPC完成済みとはしない。
加えてcommand履歴/100ms grid/pose bracket/実dispatch dtの整合と、powered watchdogの検証が残る。
本件は限定scope内の停止時接続までで、E2E走行タスク全体の完了ではない。
AWSIM無改変の制約を維持してこれらを満たせる別の既存証拠源/入力構成が必要。
未知領域free化や衝突監視省略による突破は、この依頼から許可されていない。

## 検証・保全・予算

- 限定test: 初版59件 + receipt/policy/first_error回帰43件 + identity修正15件 + ROS_LOG修正15件、計132実行全pass。
  全pytest/学習testは未実行。JUnitと生stdoutを同梱。これらはsimulatorの走行実績ではない。
- 既定syncのCheckOnly/通常syncを各3回。**既定同期によるDatasetルートの存在確認を実施**。
- **Dataset内容・既存raw・既存sensorの読取りは未実施**。一方、今回許可された固定checkpointは読取/hash/copy/復元済み。
  新sim sensorは今回の試験証拠として読取済み。「Datasetアクセス完全0」「checkpoint未読」とはしない。
- simulator binary/DLL/vehicle.yamlの各attempt前後SHA一致。他のscene/assetはread-only mountで保護。
- SSH先既存checkoutはHEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`、dirty151件のまま。上書き・reset・stashなし。
- 今回所有コンテナはすべて停止済み。既存AWSIM/Autoware/RVizも勝手に再起動していない。pushなし。
- 全task累積sim wall 1039.900356/3600秒（今回34.616529秒、前段1005.283827秒）。
- 累積固定forward60/3000、nonunit MPC1/6000、reference fit65（前段24+今回41）、offline control cycles0。
- powered0/3、powered sim秒0/180、snapshot1/16。全task unit実行305（前段173+今回132）、unit内solver/forward数は未計測。
- 失敗attemptと旧固定12件を保持。新ログをDataset/教師へ昇格しない。実車・競技全般・Safety認証は未検証。
