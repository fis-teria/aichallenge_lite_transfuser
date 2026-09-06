# Spatial Path V4 専用runtime・限定offline検証結果（日本語）

## 結論と独立判定

固定step500の専用coreは実装し、代表2入力のCUDA batch1 replayで保存予測と一致した。
loggerだけの実装ではなく、pure入力adapter→core→未補正snapshot→private writer、および注入可能な入力専用wrapperがある。
ただし実センサで起動可能な完成runtime、安全性、走行可能性は主張しない。

| 判定 | 結果・限定 |
|---|---|
| RUNTIME_CORE_IMPLEMENTED | IMPLEMENTED / 固定重みoffline parity PASS |
| ROS_WRAPPER_AND_DISABLED_LAUNCH_IMPLEMENTED | wrapper・ROS node構築hook・entry point・default false launchあり。standalone mainは明示BLOCKEDで、live bootstrapは未完了 |
| INPUT_BINDING_SPECIFIED | 静的候補/型/field/QoSを記載。real_input_binding=BLOCKED、実graph未確認 |
| SYNTHETIC_INPUT_PARITY | PASS。in-memory offline build_inputsと9 tensorsの一致、warm-up～12 grid |
| OFFLINE_CHECKPOINT_PARITY | PASS。2件、batch1、最大差2.384185791015625e-7 m |
| RECORD_SCHEMA_AND_SEMANTICS_TESTED | PARTIAL。意味検証PASS、既存Draft7 validatorによる使用キーワード互換範囲14 records PASS。完全Draft2020-12はNOT_EXECUTED |
| NONACTUATION_STATIC_AND_MOCK_CHECKED | PASS in inspected code/mock only。実graph・実ROS証拠ではない |
| LIVE_INPUT_RUNTIME_TESTED | NOT_EXECUTED |
| CONTROL_CONNECTION | NOT_IMPLEMENTED |

独立レビューは未実施。自己検証を独立合格としない。
最終の限定pytestは26 passed / 1 skipped（draft2020 validator不在）。
ROS依存rclpyなし、実ROS import/buildはNOT_EXECUTED。専用node/launchはpy_compileのみ成功。

## 版と実行範囲

- 設計固定版: da8f5dc90e45f2bc80dce6651d84b509a9441042。
- 初回実装: 910e3d560b1f40375a7e015913a5338c35c03703。
- 最終core/adapter/記録のテスト・offline実行版: 19c6be967ef5a14dca7800aafcda75d1e1d685fb。
- synthetic証拠export版: e00ab95afe2e145474507695fadf601cf1408e13。19c6be9との差はexport tool1件のみ。
- 結果追記版・梱包時HEADはPACKAGE_MANIFEST.jsonに別記。repo/は19c6be9へ固定。
- 学習f33b197と設計da8f5dcの間でモデル/入力builder/ModelBatchV3の対象3ファイル差分はなし。
- design-v2原本を変更せず、新実装用SYNTHETIC/OFFLINE_TENSOR_REPLAY envelopeだけを追加。
  実行のforward_callsとdesign-only authorizationを混同しない。
- Windowsでcommit → CheckOnly → 通常sync → 同一WSL commitとlockで検証。
  過去run/weight/Datasetを上書きせず、pushはしていない。

初回910e3d5のpytestは10 failed / 13 passed（jsonschema import欠落）。
3cef5aeは23 passed / 1 skipped、52d24adと19c6be9は各26 passed / 1 skipped。
固定重みを使う前のchecker初回はPython module pathで停止（checkpoint読取/forward 0）。
fbe5303で同じ2入力に2 forward、19c6be9で最終確認2 forward、タスク合計4/上限6。
batch2追加forward不要。新学習/optimizer/勾配更新0。実センサ/ROS/走行0。
offline private writerの保存recordは各run6行（主frame2、health/end1、receipt3）、2 run合計12行。
別synthetic exporterはfake forward6回・8 event。固定checkpointは使わない。
pytest内fake呼出しの集計は未instrument（各coreの1回性をtestで確認）、実fixed forward countとは別。

## 固定重み・比較

checkpoint:
 /home/thistle/e2e_autonomous/runs/spatial_diagnostic_v4_20260906_f33b197/checkpoints/final.pt

期待/読取前/読取後SHA-256:
0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f

weights_only=True、key/shape/dtype/finite照合、全state strict restore、eval/inference_mode。
parameter/buffer inventory前後一致。AMP/TF32/量子化/別重み/補正なし。
環境: WSL、Python3.10.12、torch2.7.1+cu128、RTX4080、CUDA12.8。
rtol=1e-5、atol=1e-6mを事前固定。保存側microbatch2、今回batch1。

| 固定代表ID | 全20点の最大絶対差 m |
|---|---:|
| 20260902-131505__epoch0000__101192918933 | 2.384185791015625e-7 |
| 20260902-131505__epoch0000__276992918933 | 1.1920928955078125e-7 |

save_examples条件i<2→selection.selected順→validation_histories.anchor_id→保存prediction sample_ids/processedで結合。
input_0/1.npzは同じbytesで収録。全差分はevidence/offline_comparison.npz。
fixtureにないreceipt/availability/clock domainはUNKNOWN/MISSINGのままで、harness受付とは別。
historyは別join証拠に残す。fixtureだけから元cameraの厳密時刻を推定して埋めず、offline eventのt_obsはMISSING。
実sensor→前処理parity、実時間遅延、command producer、走行性能を検証したものではない。

## A～Dと未完了範囲

A: candidate/event識別、unique候補7段階、先行record ID/hash/writer/timeへのreceipt、
WRITE_COMPLETED_NOT_DURABLE、有限queue/file/record制限。失敗で新推論を停止（車両停止commandなし）。
B: timing8、9 descriptorの順/dtype/shape/little-endian/hash、commandの過去性とavailability。
C: 未推論/forward例外/shape-dtype違反/snapshot例外/正常/NONFINITE。
同じfloat32 snapshotの160bytesとJSON tagを保持。device/API戻り/copy完了を分離。
D: 4点全有限の17弦、原点区間、局所spacing、累積prefix非再開、float64派生。

Schemaの既存system jsonschemaは3.2.0でDraft2020未対応。依存追加なし。
使用するキーワードをallowlistしたDraft7互換検証（local refsのみ、ネットワーク取得不可）を別結果とした。
これは一般のDraft2020-12 validatorでの合格ではない。初回互換checkerのURN解決失敗と修正もログに保存。

重要な残り:
- standalone mainは例外で止まり、実ROSの起動/固定core生成を自動bootstrapしない。
  wrapper構築hookは実装済みだが、live bindingと記録envelope・起動bootstrapを次承認で閉じる必要がある。
- wrapperのgridはepoch0基準100ms nearest。学習収集時のgrid phaseとの実一致はUNKNOWN。
  egoはcamera時刻完全一致のみ。補間の仮定をせず、到着待ち/queue損失はboundedなrejectionsへ残す。
  wrapperの全sensor受信をcandidate母数へ数える完全なlive記録は未検証であり、現在の母数はcore受付単位。
- passive nominal producer、stampと実availability、実QoS/clock、750beam角度geometry、range min/maxはbinding未確認。
  final fallbackはdisabled。range validityはsensor min/max、正規化は0..25m。command最大ageは既存configの50ms。
- 非同期backpressure、保存失敗全分岐、実負荷適切性、epochを跨ぐ実transportの挙動は合成範囲外でUNKNOWN。
- field間意味検証は実装した検査に限定。Schema keyword検査やhash群だけで無制御/安全を証明しない。

次承認は具体的な入力source/clock/grid/ego/beam/QoS、bounded保存量・停止条件、
live envelope/bootstrapの実装と限定live shadow起動について。全raw監査・MPC完成を条件に追加しない。
速度参照・MPC/制御・Safety・teacher採用は別gate。基準点・現在時刻への変換・速度・必要長・validityの不足を本出力で補作しない。

