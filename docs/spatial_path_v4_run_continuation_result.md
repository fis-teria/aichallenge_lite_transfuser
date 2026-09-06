# 未改変AWSIM: V4 RUN/dispatch継続修正・限定再試験結果

2026-09-07 JST。結論は **短距離走行・制動停止は未達**。
実装、限定合成検証、新観測V4推論、stable履歴、所有instanceの終了まで実施した。
最初の未成立箇所は、通常履歴46出力すべての **固定制約付き参照生成の拒否**。
学習不足/データ不足/MPC限界とは断定しない。原因はUNKNOWN。
同じHOLDの無目的な追加起動はせず、このscopeの結果を提出して終了する。

## 版・環境

- Repository: `https://github.com/fis-teria/aichallenge_lite_transfuser`。
- Branch: `codex/windows-wsl-training-sync`。
- Windows開始: `a04fa39c5b6dda014baab338ee5e7810d4fb1d46`、clean。
- 基準sim: `c48f823023e67e091e7869b8ff7077e09109e544`。旧結果は変更していない。
- 実装: `5d8c6850e7640859fb7e92207f8662770cf38c43`。
- test期待値修正/counter: `6a017ff4bfda220be318ea69337f1f525d288a63`。
- log予算: `d41022c7f7c70bab686e49fc769502e887006b06`。
- Queue終了: `b7a3faef236e5fa4cabddf8af53a3bdb216bf2ad`。
- 最終実装・限定test・sim版: `af1c7f12d99b1889fb8ea4ead395a4b6458c1b45`。
- 本結果は上記実行後の別文書commit。梱包はlocal artifact、版はpacketの`source/versions.json`。
- 指定host `graneple@192.168.3.10` / hostname `graneple-local`。
- 原racing-kart HEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`。
  通常Git statusのdirty entriesは前後151。既存checkoutへ書込/reset/stashしていない。
  終了確認で一度untracked全展開のGit statusも取得したが、内容検査ではない。
  その無関係なファイル名一覧はレビューpacketに含めず、通常status数/hashを同梱する。
- AIChallenge2026 / GoKart1、同じimage
  `sha256:8c650c13157ffabbc3a72bab08865ccff8338f9025b43a8f4405b4c6b96d1ba7`。
  実行CUDA、torch 2.3.1+cu121 / numpy 1.26.4 / scipy 1.15.3 / pyproj 3.7.1。
- 固定checkpoint: `0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f`。
  既存hostコピーを再利用。全attempt strict load/eval、前後state hash一致。

Lite TransFuser側の専用make devを使用。原racing-kart Makefileは実行/変更しない。
元 `aichallenge/run_simulator.bash dev` とAWSIM一式はread-only mount。
各instanceのcompose/inspectと実consumer `awsim_d1` のGID/QoSを保存。
network none + 専用namespace共有、非privileged、cap-drop ALL、GPUのみ、
hardware/CAN/serial/host Docker socket非mount。旧AWSIM/Autoware/RVizは再起動していない。
最終確認で起動中Docker、対象AWSIM/ROS/学習processなし。

## 全attemptと段階別判定

新run root: `/home/graneple/e2e_autonomous/spatial_run_continuation_20260907`。
Windows集約: `tmp/spatial_run_continuation_20260907`。既存結果を上書きしない。

| run / commit短縮 | 新V4 | stable / warm-up | fit solve / 参照受理 | MPC / MPC送信 | host wall秒 | host結果 / runtime最終exit |
|---|---:|---:|---:|---:|---:|---|
| run_d41022c_01 / d41022c | 40 | 36 / 4 | 36 / 0 | 0 / 0 | 34.929987 | HEARTBEAT_STALE / 137 |
| run_b7a3fae_02 / b7a3fae | 12 | 6 / 6 | 6 / 0 | 0 / 0 | 22.226929 | HEARTBEAT_STALE / 0 |
| run_af1c7f1_03 / af1c7f1 | 12 | 4 / 8 | 4 / 0 | 0 / 0 | 22.233258 | errorなし / 0 |
| 今回合計 | 64 | 46 / 18 | 46 / 0 | 0 / 0 | 79.390173 | 全所有instance終了 |

2回目は未消費Queue終了の修正確認、3回目はhostへの最終化引渡し確認。
参照上限・モデル・sceneを成功するまで変えた再試行ではない。
後2回はforward上限12を明示し、RUN参照失敗が続くまま試行を増やさない。

前段の **V4 60 / 参照受理4（全warm-up）/ HOLD MPC1 / MPC送信0 / powered0** は別実験。
今回に足すと固定forward124、non-unit MPC1。旧60を今回64に混ぜない。

| 判定項目 | 結果 |
|---|---|
| 時系列・dispatch・host終了の修正 | 実装＋関連合成検証済み。全RUN接続完成ではない |
| 選択instance隔離・実consumer | 各attempt実値を保存。実車一般への保証ではない |
| 新観測V4 / stable履歴 | 64 / 46。全raw20の160bytes/hash再照合済み |
| RUN参照 / 前進MPC | 0 / 0（参照生成段階で拒否）|
| 新V4由来MPC要求・送信 | 0 / 0。適用済み操作はUNKNOWN |
| 動いた後の観測→V4→操作更新 | 0。停止中入力の追加をE2E走行にしない |
| 2m進行 / 移動中10更新 | 未達 / 0 |
| 停止要求後の静止帯1秒 | 各attempt全30速度サンプル、約1.015 sim秒、最大間隔35msで成立 |
| 走行からの自然制動停止 | 未実施。STOPという名称の0加速度送信は制動成功ではない |
| pause/所有process終了 | 全attempt実施。unpauseなし。最終attemptは通常の終了引渡し |
| collision/逸脱 / time scale / 1x実時間性 | UNKNOWN / UNKNOWN / NOT_ESTABLISHED |

送信は今回266件、HOLD209/STOP57、mode3/gear3。MPC/正加速度送信は0。
実測最大絶対速度は各attempt 0.000239875 / 0.000000149809 / 0.000000185481 m/s。
GNSS終点の純変位は0.000094657 / 0.000324626 / 0.000054066 m。
静止ノイズの差分累積を走行距離として採用していない。

## 最初の未成立箇所と固定出力

stable46件は全件 `REFERENCE_BOUNDED_FIT_INFEASIBLE`。
実行した固定SLSQP fitの保存済み偏差証明上界は **0.112572～0.200759m** で、0.10mを超える。
他のfitや別の最適解が存在しないという証明ではない。現在方式/条件で受理できなかった実測である。
元20点を補正せず、対応順、FIRST_CUSP prefix/tail、接続区間、steering knots、
solver message/iterations、偏差サンプル、certificate gap、制約残差を保存した。
不受理時のreference.xy_mは空であり、走行参照として採用していない。

保存knotsから不受理候補を数値再構成し、保存偏差配列と1e-8以内で一致を確認。
これは **追加最適化なしの診断再構成** であり、新しい受理参照ではない。
`results/rejected_fit_from_saved_knots.npz`と`first_stable_paths.png`を参照。
図は数値診断であり、実sim画面ではない。

warm-up18件は `REFERENCE_INITIAL_STATE_LIMIT`。
raw signed速度と、controllerのみの停止近傍扱いを別保存した。
Drive gearの実送信＋連続1秒停止観測＋-0.001m/s以上の負速度だけ初期v=0とする固定policy。
今回適用25件（18/4/3）。現在ego/model入力の負符号は変更しない。
Driveの内部適用確認はUNKNOWN。真の後退をabs/0へ変換しない。

## 入力・時刻・dispatchの根拠と限定

保存した前段camera採用84 stampの再計算では、旧grid欠損由来reset5回、
実cameraの1秒超途絶0回。今回SIM_GRID_MISSING_V2は40ms許容を維持し、
100ms名目欠損slotを全mask=falseにする。欠損だけでcommand履歴を消さない。
実途絶>1秒・clock回帰・epoch変更は両履歴をreset。全新attemptでresetは初期EPOCH_CHANGE1回のみ。
4/4/10/10・9 tensorsを維持。stableは11名目枠が1秒を覆った意味で、**全slot有効という意味ではない**。
欠損のsource=null、内部の保管用stamp複製は無効mask。sensor_dtは有効slotだけ元の差、欠損は0。
これはsim限定の入力意味差であり、学習時完全parityとは言わない。

command選択386 slotを生送信receiptのsim/monotonic stamp・値に後結合し、
strict past、slotに対するage≤50ms、cutoff以前のavailability、epoch一致を再計算した。
operation ID/sourceはsupervisor receipt側、選択recordからstamp pairで結合できる。
未送信solver案/未来command/現在速度由来の架空commandは使っていない。
desired speedは実速度/適用確認ではない。常時command欠測を正常warm-upとして流さない。

poseの後結合は同じraw/forward/元cutoff/元200ms deadlineを保持する制御専用経路。
late cameraやcommandをinputへ入れず、現在stateはpose/v/steeringの共通source時刻へ補間する。
今回実simでは参照拒否が先行し、late join/RUN state更新/MPC dispatch枝は未到達。
合成の期限内/後bracket、epoch、停止割込を実sim検証済みへ昇格しない。

送信scheduleはmodel dt=0.1s、sim通常操作周期0.05s、次の操作区間、実送信間隔を分離。
senderだけが舵角rateを一度積分する。HOLD/STOPも実送信後にprevious aを更新。
新cameraのqueue受付だけでは古い期限内候補を拒否せず、epoch/元期限/採用順/
operation重複/最新state整合/jerkを送信直前に再検査する。緊急停止は別wall周期。
実MPC送信での成立は今回は未測定。

## 車体・領域・監視: fixed falseを根拠なしtrueにしない

binding SHA256 `0f5bed3cfd5b286f39d100094597c307d2ae3c4ef438be97625f0eb3731b5d4c`。
GoKart1 mesh collider1133はlevel1外部参照fileID2=`resources.assets` / mesh113。
既存資料の別asset扱いを修正し、body/wheel collider変換から保守的な幅1.535852m、
後overhang0.510m、前overhang0.526851mを使用（旧1.3m幅より厳しい）。
rear_x_in_base=0.0010000169m、wheelbase1.087m、LiDAR x=1.6499999762m。
GNSS local x=-0.26m、IMU local yaw=pi/2、設定delay0、20Hz bracket≤50ms+physics5msの
限定誤差仮定を記載。source acquisition時刻そのものはUNKNOWN。

現在footprint、参照、MPC rolloutについて、旧scan診断を保持してstatic AABB診断を別fieldにする。
障害物2個（mesh collider1157/mesh108、Foundation box1174）を同原本から確認。
AABBの外側除外は限定根拠だが、内側をfreeにしない。可動物監視未確認なら拒否する。
このsceneではAABBだけで現車体付近をfreeと認定できない。

追加で同じmesh108のspawn±5m XYにAABBが交差する881三角形を静的抽出した。
元117301三角形、全z保持、外部streamなし、出力hash
`5939ed24cc4d13374625ba36b3f82a8990366837110b5f27d911c651f75c540d`。
**幾何が取得不能なのではなく、路面支持と壁の分類・実稼働可動actorとの結合が未成立**。
この診断meshをruntimeのfree mapやV4のroute入力へ渡していない。
独立collision/departure monitorの適用範囲/heartbeat、停止までの別swept footprint検査も未完成。
これらは後続の未成立点であり、今回の最初の拒否である参照fitとは区別する。

## 独立停止・終了

host ARM→supervisor確認→最初のHOLD/mode/gearの順序を全attemptで確認。
powered=falseでもheartbeat停止を検知する。750msを広げて終了エラーを隠していない。
paused Docker helperを同hostでKILLできることを先に検証（非AWSIM、exit137）。
全cleanupで特定sim IDのpause→paused KILL、runtime停止、log取得、最終inspectを分離。
例外でも後続停止を試行し、first errorとcleanup errorsを別保存する。

1回目: worker0/supervisor summary正常の後もruntime終了せずhost HEARTBEAT_STALE。
未消費multi-MB Queueの終了待ちを合成processで再現し、cancel_join_thread/closeで修正。
実際のblock stackは採取していないので、1回目の唯一の原因と断定しない。
2回目: runtime最終exit0だがhost HEARTBEAT_STALE。通常finalizer中のheartbeat空白が残った。
3回目: HOST_FREEZE_REQUESTEDをfresh heartbeatとして受信し、hostが所有simのpauseを実確認。
凍結確認後だけ最大5秒runtime終了を待つ。host errorなし、runtime0、simは最後にKILL137。
一度もunpauseしない。freeze requestは停止済みの主張を信用して走行を続ける許可ではない。

各停止帯は停止開始以降の全30速度sample、最大gap35ms、約1.015sim秒で再確認した。
それでもpowered0なので「発進後に安全に制動できた」ことは未検証。

## テスト・性能・予算

Windows commit→既定CheckOnly→通常syncを5組実施し、同じSHAでWSL worktree lock下の限定test。
sync scriptは変更していない。全pytest/学習optimizer testは実行していない。

| 版 / 対象 | 結果 |
|---|---|
| 5d8c685: 新test＋dev_connection＋sim_e2e | 66 pass / 1 fail。最終窓に欠損が残るというtest期待の誤り |
| 6a017ff: 同上 | 67 pass。欠損がまだ窓内にある時刻をtestに明示、runtime閾値変更なし |
| 6a017ff: runtime/bootstrap receipt・policy・first_error限定 | 43 pass / 215 deselected |
| d41022c: 新module | 26 pass |
| b7a3fae: Queue終了回帰を追加 | 27 pass |
| af1c7f1: host最終化要求回帰を追加 | 28 pass |

今回258 test実行（再実行を含む257 pass / 初回1 fail）、累積563実行。
最終版の28件だけを全既存test通過と記載しない。独立レビュー合格ではない。
合成の順序/mask/採否は固定testのassertと生JUnitで確認した。全合成ケースの
内部eventを逐次吐く独立traceファイルは未保存であり、実sim traceとは区別する。
計測導入後のunit方法callはfake forward8、reference adapter13、scipy minimize6、
unit SpatialMPC.solve0。前段と最初の67件の方法別call数はUNKNOWN、0としない。

| 予算 | 今回増分 | 累積 / 上限 | 残量 |
|---|---:|---:|---:|
| sim関連wall秒（起動/cleanup含む）| 79.390173 | 1119.290529 / 3600 | 2480.709471 |
| 固定V4 | 64 | 124 / 3000 | 2876 |
| non-unit MPC | 0 | 1 / 6000 | 5999 |
| powered episode / sim秒 | 0 / 0 | 0 / 3、0 / 180 | 3 / 180 |
| sensor/tensor snapshot | 6 | 7 / 16 | 9 |
| reference fit（別counter）| 46 | 111 | 上限項目とは混合しない |
| offline control cycles | 0 | 0 / 1500 | 1500 |

累積runner log課金53,527,759bytes、上限536,870,912bytes。
前段の全local taskファイルを保守的に含め、各終了に追加1MiBを見込む上界。
unit/sync/静的証拠/梱包成果物は別のlocal byte集計をpacketへ付記する。
active予約は残っていない。`exact=true`は終了counterが取得できた意味で、
byte/time段階別計上まで厳密な実測という意味ではない。
paused終了確認用の非AWSIM Docker sleep helperは別probeとして起動/終了時刻を保存。
AWSIMは起動しておらず上のsim wall対象外。helperの稼働は約0.220秒、CLI全体wallは未計測。

推論wall中央値は各run8.977 / 8.596 / 9.433ms。
初回最大652.154 / 215.300 / 234.133ms、camera受信→cycle終了中央値122.365 / 130.328 / 104.138ms。
成功frameだけの遅延ではなく、全拒否64件を含む。MPC込み実時間性は未測定。
velocity受信のsource-time/wall-time比は0.99524 / 0.99227 / 0.99358。
これはtime-scale設定の取得ではなく、設定はUNKNOWN（変更なし）。

## 保存物とアクセスの境界

全attempt、生stdout/stderr/JUnit、strict load map、state hash、全raw160bytes/hash、
grid/mask/command/pose、全停止sample、compose/inspect、host pause/exit、累積budgetをpacket化。
6個のstable tensor snapshotは9入力と同forward rawのhashを照合。
画像tensor stdは約0.49～0.76で、一様な黒画像だけを採った結果ではない。
未正規化原sensor全保存はしていない。hashだけのframeの原sensor再現はできない。
実sim動画は未収録（専用headless Xvfb、動画収録経路未実装、追加起動せず）。
数値PNGを実画面に見せない。

- **既定同期によるDatasetルートの存在確認を実施**。
- **Dataset内容・既存raw・既存sensorデータの読取りは未実施**。
- 指定の固定checkpoint本体のhash/strict読取、新sim sensor購読は実施。
- 許可された原本scene/collider静的読取も実施。resources.assets約109MBを含む。
- AWSIM binary/DLL/vehicle/level1/sharedassets1/globalgamemanagers/resources/旧起動scriptの
  8hashは全attempt前後一致。AWSIM/physics/sensor/time scaleを変更していない。
- 原本重み、AWSIM実行物、Dataset/rosbag/venvはGit/ZIPへ入れない。自動pushなし。

次の進展を直接止める要因は **stable出力からの0.10m制約付き参照が受理できないこと**。
保存されたfitの反例をレビューし、修正するなら別途その範囲を明確にする。
scene支援の完成、走行の再試験、モデル変更/学習、一般runtime昇格は今回の実績から自動承認しない。
