# V4地図非依存・局所センサ監視の設計見直し

## 1. 決定と範囲

2026-09-07。参照HEAD `d8dc22707009ae8f515acd2cb4b23abc8e18c13c`。
開始時の既存未追跡 `docs/static_map_boundary_distance_20260907.md` を保全。
今回は設計のみ。モデル、前処理、参照fit、Pure Pursuit、制御gate、設定、AWSIMは変更しない。
新規推論、収集、学習、ROS起動、同期、走行、pushを行わない。

決定:

- 外部コース地図、MGRS配置、一致率、scene形状との一致を局所走行許可の要件にしない。
- 地図診断の20/60cm区分は診断にのみ保持し、点群の削除、補正、車体余裕に転用しない。
- 地図非依存の監視は、観測・車体・時刻・候補操作から停止までの到達範囲を使う。
- 地図を外すことと、観測されていない領域をFREEにすることは別。
- 初期運用範囲は、専有シミュレータ・低速前進・静的障害物の限定試験とする提案。
  動的物体対応、後退、実車、全面的な安全保証を含まない。

`design_only: true` / `runtime_promotion_authorized: false`
`new_inference_authorized: false` / `new_collection_authorized: false`
`new_driving_authorized: false` / `approval_gate: PENDING_EXPLICIT_AUTHORIZATION`

## 2. 現状の訂正と実在コードの対応

以前の説明で「地図照合が直接の停止条件」と受け取れる説明をした点を訂正する。
実装の `compare_observation()` は既に `runtime_permission=False` の診断専用。
今回の問題は地図を使う経路生成ではなく、独立した監視条件の構成にある。

| 実在file/function/field | 現状 | 設計上の扱い |
|---|---|---|
| `src/aic_transfuser_lite/control/map_observation_v4.py: compare_observation` | 地図へ変換・集計、`geometry_binding_verified=False` | 任意のoffline診断を維持。制御入力にしない |
| `tools/run_spatial_sim_dev_v4.py: update_pose` | GNSS/IMUでpose、任意の地図診断 | 地図診断を制御経路の依存にしない。相対運動の時系列処理は必要 |
| `src/aic_transfuser_lite/control/spatial_sim_guard_v4.py: scan_coverage` | 全矩形内部を10cm格子で毎回検査。現在scanで全方向の証拠を要求 | 新方式では停止到達範囲の障害物・観測範囲判定へ置換。旧方式は比較用に保存 |
| 同file `scene_aabb_evidence` | scene全体を囲むAABB内はUNKNOWN、動的coverageも要求 | 局所走行の必須証拠から除外。任意のsim診断のみ |
| 同file `MotionEvidence.clearance / collision_monitor` | workerがscene検証と結合して作成 | 一括でTrueにせず、新しい局所監視契約へ段階的に置換 |
| `src/aic_transfuser_lite/runtime/spatial_sim_worker_v4.py` | `clearance=(coverage or reference_static) and current_static`、`collision_monitor=current_static` | scene必須のANDを廃止する実装を別段階で行う。新契約が未成立ならHOLD |
| 同worker PP分岐 | rolloutにもscan/scene判定を実施 | 単に前段だけ緩和せず、新契約で前段と送信前を整合させる |
| `src/aic_transfuser_lite/control/spatial_live_pp_v4.py: propose` | PP操作候補と一定操作の仮想rolloutを返す | 候補生成として維持。一定操作rolloutを制動完了の証拠としない |
| `src/aic_transfuser_lite/control/spatial_speed_profile_v4.py: stopping_distance` | jerk制限・delay付き数値制動距離 | 計算の参考として再利用候補。実制動性能の証明とは分ける |
| `src/aic_transfuser_lite/runtime/spatial_sim_adapter_v4.py: Sample/interpolate/await_control_join` | frame時刻・pose後結合 | 相対変換と履歴の基礎。timeoutは地図問題と独立に保持 |
| `src/aic_transfuser_lite/control/spatial_sim_guard_v4.py: OperationLease` | stale/異常/重複/過速度等で停止を要求 | 維持。新監視の失効を追加接続する設計 |
| `tools/run_spatial_sim_dev_v4.py` | host ARM、唯一のconsumer/publisher、操作送信、終了処理 | 隔離・停止経路を維持。監視workerが固まっても停止させる |

旧 `collision_monitor` は実接触イベントでなくscene AABB検証のboolだった。
新仕様では「予防的局所監視」「接触イベント取得」「host停止」を別fieldにする。
接触topicがあるとは仮定しない。静的参照だけでは未確認なので `MISSING`。

## 3. データの流れ（提案、未実装）

```text
センサ → 既存のV4入力 → 未補正20点 → 既存参照生成 → PP操作候補
  │                                                   │
  └→ 独立した監視用scan＋運動履歴 → 局所監視 ← 車体・停止到達範囲
                                        │
                             有期限の局所判定＋理由
                                        ↓
                      独立supervisor → 唯一のsim制御consumer

外部コース地図 → 任意の診断ログのみ（上記の許可判定へ接続しない）
```

候補を内部計算することと、RUN操作をpublishすることを分離する。
HOLD前提の候補だけを監視してRUN候補の証拠とする循環を避ける。
監視はモデル推論完了とは独立の周期で更新。制動中も新しい障害物を監視する。
モデル入力を監視用に書き換えない。観測更新が遅いモデルの入力scanだけで安全監視を続けない。

## 4. 観測契約と座標

1. 実scanのframe、点数、角度順、range範囲、単位、欠損・飽和値の意味を版付きで固定する。
   現行750点契約を無条件に別版へ転用しない。NaN/infは既定でUNKNOWN。
   no-returnを最大距離までFREEと解釈するには、当該publisher契約の別証拠が必要。
2. 反射点をLiDAR frameから観測時base frame、次いで現在base frameへ変換する。
   LiDAR外部パラメータとbase→後輪中心を別に保存する。両原点を同一視しない。
3. 絶対コース位置は不要だが、履歴を使う場合の相対並進・回転は必要。
   現行GNSS/IMU由来poseの差を当面利用しても地図照合にはならない。
   GNSS不要化まで完了したとは呼ばない。代替odometryは別実装・検証事項。
4. scan header、受信monotonic、現在state時刻、入力締切、判定時刻を記録。
   GPU版の配信時stampを取得時刻保証にしない。固定0.2秒補正は採用しない。
   点ごとの取得時刻が保証されなければ、time_incrementだけからdeskewしない。
5. age、相対pose不確かさ、取得時刻誤差を判定に含める。上限不明はUNKNOWN。
   yaw誤差による横方向不確かさは遠い点ほど増す。
6. clock reset、teleport、epoch変更、外部パラメータ変更で履歴を無効化。

## 5. 点群と停止までの範囲

監視対象は「全コース」「モデル出力20点全体」ではなく、今回送信し得る操作から
停止まで車体が到達し得る領域。候補曲線の中心線だけで検査しない。

- 前後の張り出し、車体幅、rear/baseの差を含む実車体footprintを使用する。
- 発行済み操作・次の操作の有効期間・監視遅延・送信遅延・制動応答を含める。
- 操舵保持、操舵速度制限、停止方策の操舵挙動、追従誤差を含む制動軌跡群から領域を作る。
- PPの一定加速度rolloutと、停止するまでのrolloutは別に保存する。
- 離散姿勢間の車体移動・回転による領域を埋め、離散化誤差を上乗せする。
- 反射点はレンジ・角度・時刻・相対姿勢の誤差を含む領域として判定する。
  不確かさを車体側と障害物側の両方へ二重加算しないよう内訳を保存する。
- 障害物領域と停止到達範囲が重なる場合、候補を拒否し既定の停止方策へ。
  停止も接触回避を保証できない場合は `STOPPING_CLEARANCE_UNVERIFIED` を記録する。
- 速度を下げれば常に解決するとはしない。静止からの正加速度も必ず到達範囲に含める。

余裕はセンサ誤差、時刻誤差、pose誤差、追従誤差、幾何離散化を根拠に設定する。
地図照合の20/60cmや過去の最大ずれをそのまま採用しない。
現行の速度上限・操作期限・承認予算は別承認なしに拡大しない。

## 6. 前方LiDARの死角と初期状態

「前方LiDARで現在車体の内部・後方まで毎回観測する」という旧要件は撤廃する設計。
ただし、車体の外へ新たに張り出す領域を無条件で許可してはならない。

- 現在footprint `F0` と、停止到達範囲 `S` を区別する。
- 静的で接触のない初期状態が根拠付きで成立した場合に限り、車体内部 `F0` を
  初期条件として扱う。LiDARでFREEを観測したという記録にはしない。
- `S \ F0` のうち新たに侵入する領域は、現在scanまたは有効な履歴の観測証拠が必要。
  コーナーで後端・側面が外へ張り出す領域も含む。必要な余裕の領域はF0と別に扱う。
- 静止、ゼロ速度、contactメッセージが来ないことだけでは初期無接触の証明にならない。
  初期状態の確認方法・主体・時刻・対象instanceを試験プロファイルに明示する。
  専有simの目視確認を採用する場合は、人が確認した限定試験条件であって自律センサ証明ではない。
- 履歴は観測済み領域の証拠を短時間保持する用途。経路を通っただけで周辺をFREEにしない。
  保存量、TTL、相対移動量上限を有限にし、古いFREEを新しいhitより優先しない。
- 光線間や反射点の向こう側を勝手にFREEで埋めない。光線間を扱うには角分解能・
  対象最小寸法・センサ検出性という適用範囲とその誤差モデルが必要。
- 動的物体が死角へ侵入し得る場合、静的履歴の再利用だけでは不十分。
  初期限定試験の静的条件を確認できなければUNKNOWN。一般動的環境の安全性は主張しない。
- 2Dscan高さより低い／高い障害物の検出限界を明記する。

初期状態の根拠や新規侵入領域の観測が足りなければ、UNKNOWNで駆動開始しない。
この場合の解決は追加センサ・観測可能な試験条件・別途承認された監督付き試験であり、
未知セルを削る、車体を細くする、地図FREEで補うことではない。
本設計だけで現ハードウェアの全コーナーを通過できるとは約束しない。

## 7. 判定と記録（以下のfield名は提案、未実装）

`LOCAL_CLEAR / OBSTRUCTED / UNKNOWN / STALE / FAULT` を分離する。
`LOCAL_CLEAR` は指定profile・有限horizon・誤差上限内の局所判定であり、全体安全の証明ではない。
UNKNOWN/STALE/FAULT/OBSTRUCTEDでは正加速を許可せず、停止処理と理由記録を行う。
停止処理が機能しなければ既存の所有sim freeze/終了へ。無条件unpauseはしない。

最小記録:

- schema/policy/config/code版、session/epoch、scan ID、pose/state ID、V4入力/forward/出力ID。
- 未補正20点・参照ID・PP候補ID・停止方策ID・実送信operation IDと時刻の結合。
- センサframe、TF/外部パラメータ出典、相対変換、全clock domain、age、時刻誤差上限。
- 有効hit・invalid・FOV外・occluded・履歴採用/失効の数と理由。
- 車体寸法、候補＋制動horizon、余裕内訳、最近接値、交差領域、未観測領域。
- 初期状態の根拠、静的限定profileの根拠、局所判定・失効時刻・拒否理由。
- `contact_telemetry_status`、`contact_event`、`host_watchdog_status` を別々に記録。
  未接続のcontactを `false` とせず `MISSING` にする。
- queue/drop/計算budget超過、停止要求・送信・実測速度、host freeze/終了の区別。

監視の有期限判定をsupervisorが送信直前に再確認する。
送信済みの古い操作を再利用せず、最新scanによる停止判断をモデルworker待ちにしない。
モデルが停止してもwatchdogは継続。隔離、競合publisher、logger、状態鮮度の既存条件は残す。

## 8. 別段階の実装順と検証仕様

1. ROS非依存の局所幾何・光線証拠・停止到達範囲・期限付き判定を実装。
2. 合成試験で現方式と新方式の理由を比較。実行profileの未知パラメータを埋める。
3. workerのscene依存と、前段/rolloutの二重判定を新契約へ一貫して接続する。
4. 明示承認後に、実scan・無駆動で新監視の記録を検証する。
5. さらに現時点の予算・期限・初期条件・隔離・停止を確認して限定低速試験へ。

| 合成／後続検証 | 期待する結果 |
|---|---|
| コース地図なし／違う地図／scene AABBを削除 | 同じセンサと候補なら同じ局所判定 |
| 直線の前方障害物／車体幅内の横障害物 | 中心線だけで見逃さず拒否 |
| 曲がると後端が死角の新領域へ進入 | 観測証拠がなければUNKNOWN |
| 物理的に許可された静的初期F0、前方新領域に証拠あり | 車体内部の不可視だけを理由に全拒否しない |
| 初期F0未確認／履歴なし | 初期領域を勝手にFREE化しない |
| NaN/inf/no-return/遮蔽の向こう／FOV外 | 契約に沿ったUNKNOWN、反射点優先 |
| 履歴FREEへ新しいhit／clock reset／teleport | hit優先、履歴失効 |
| 遅延・yaw誤差・age増大 | 対応する不確かさ拡大またはSTALE |
| scanの時間と現在egoの基準が異なる | 明示変換、二重補正なし |
| 静止から加速／次の推論停止／制動操舵の変化 | 発行操作から停止まで含めた領域を検査 |
| model/monitor/logger停止、queue飽和 | 独立停止、理由と実行結果を記録 |
| contact telemetry未接続 | MISSINGのまま。無接触成功とは報告しない |
| 元V4入力・未補正20点 | 監視方式を変えても値を変更しない |

未確定: 実取得遅延上限、相対pose誤差、適用可能な障害物寸法/高さ、履歴TTL、
有効制動性能、追従誤差上限、初期状態の確認手段、動的条件の除外方法、接触情報の利用可否。
これらを任意の数値・Trueで埋めない。現段階のstatusはUNKNOWN/MISSING。

## 9. 今回の検証と完了範囲

読み取り専用で上記関数・fieldと結合箇所を確認し、本文の対応表を作成。
設計Markdownのみ追加。合成test、pytest、推論、WSL/SSH接続、AWSIM試験は未実施。
この文書の完成は、新監視の実装完了・旧gate解除・走行承認を意味しない。
