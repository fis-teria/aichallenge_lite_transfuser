# Astra Proへ渡す設計依頼

以下を新しい会話へ貼り付け、対象GitHubリポジトリを選択してください。
Web側で参照できないローカルDatasetやcheckpointは、概要として本文に含めています。

---

あなたは自動運転の学習ベース経路計画、教師データ設計、MPC連携に詳しいML・制御エンジニアです。
既存V3実装と実験結果を監査し、次期「Spatial Path V4」の実装可能な設計書を作成してください。
ユーザーの呼称「Special Path V4」は、この空間経路生成モデルを指します。
今回は設計を依頼しています。コード変更、本学習、走行、データ収集の実行は行わないでください。

## 1. 対象とソース確認

ユーザー指定のリポジトリ名：`aichllege_lite_transfuser`。
現在のローカルcheckoutに設定されたorigin：
https://github.com/fis-teria/aichallenge_lite_transfuser

上記は綴りが異なります。接続されたリポジトリのowner/nameを最初に確認してください。
別リポジトリや別版を黙って参照せず、実際に閲覧できたURL・branch・commitを報告してください。
ユーザー指定名が新規・改名先であるかは未確認です。

対象branch：`codex/windows-wsl-training-sync`
V3現状保存commit：`2989f9389415c121824c585754b8e10d7904a659`
実験Aの学習commit：`7a06d37e1a71740040079b42041fb41f5878020e`
比較結果の判定名修正commit：`7ac377ec359083dcda1d2ebb854d7d65112b04a3`

ローカルcommitがGitHubへ公開済みとは限りません。指定commitを読めない場合、古いmainを現状として扱わず、コード由来の断定を控えて、以下の提供情報から暫定設計と不足ファイル一覧を作成してください。
リポジトリ本文や実データを読めていないのに「確認した」と報告しないでください。

優先して読むファイル：

- `AGENTS.md`、`README.md`
- `docs/v3_experiment_a_audit_20260905.md`
- `docs/v3_path_only_launch_recovery.md`
- `docs/v3_m3_limited_odd.md`
- `docs/v3_teacher_collection_pilot_20260904.md`
- `configs/models/trajectory_authoritative_finetune_v3.yaml`
- `src/aic_transfuser_lite/data/dataset_view_v3.py`と関連canonical converter/schema
- `src/aic_transfuser_lite/models/full_control_lite_v3.py`
- `src/aic_transfuser_lite/training/losses_v3.py`、`training/train_v3.py`、`cli.py`
- `src/aic_transfuser_lite/runtime/input_history_v3.py`
- `src/aic_transfuser_lite/control/executable_reference.py`と実際のcontroller
- `src/aic_transfuser_lite/evaluation/launch_replay_v3.py`、`evaluation/compare_v3.py`
- `tools/evaluate_path_only_v3.py`、`tools/compare_path_only_v3.py`
- ROS `inference_node_v3.py`、`runtime.v3.trajectory_authoritative.param.yaml`
- 関連のhistory/filter/parity/controller/promotionテスト

## 2. ユーザーの目的

縦・横の制御はMPCに任せ、モデルは追従可能な局所経路を安定して生成することに集中させたい。
Camera + 2D LiDAR + ego stateを中心に、停止状態、通常周回、左右壁際からの復帰、左右カーブでも経路が潰れず、走行可能領域内の経路を生成できる設計を求めます。

MPCが受け取る経路、速度計画、停止判断、Safety Supervisorの責務とインターフェースを明確にしてください。
MPCという呼称だけで現行controllerがMPC実装済みだと仮定せず、コード上の実装とユーザーの目標を区別してください。
制御をMPCに任せても、障害物を横切る経路や壁へ向かう経路を自動的に修復できるとは仮定しないでください。
経路選択にroute intentが必要なら、推論時に本当に利用可能な情報を特定してください。

## 3. 既存Datasetの提供情報

以下はローカルで取得した集計・レポートの情報です。あなたが実データで再計測した事実とは区別してください。

- Dataset：`d1log_recovery_mixed_20260904_v3`
- manifest identity：`181cf909b80589110574859990b0885005b7f9a0bb07cff1c24f38d6b090f388`
- 26 runs、72,697 samples、run単位train/val/test = 16/5/5
- normal lapとrecoveryを混合。連続frameが多く、容量やframe数は独立事例数を意味しない
- Camera/LiDAR各4 frames、ego/command各10 steps
- 15 future steps、約1.5秒
- trajectory教師は`future[:, 1:3]`、speed教師は`future[:, 4]`で、実際に観測された未来
- control教師はnominal優先、無効時finalへfallback
- canonical V3にplanned recovery Referenceは保存されていない
- raw bagや別sidecarにReferenceが残るか、1.5秒より長い連続poseを復元できるかは、実在確認が必要
- run/sourceのsplit跨ぎ検査は0件。ただし同一セッション・反復run間の相関は残る

| split | raw | 基礎除外 | 未フィルタ有効 | 完全futureの矛盾除外 | censored | 品質集合 |
|---|---:|---:|---:|---:|---:|---:|
| train | 45190 | 48 | 45142 | 1363 | 162 | 43779 |
| val | 13641 | 12 | 13629 | 450 | 37 | 13179 |
| test | 13866 | 22 | 13844 | 444 | 27 | 13400 |

基礎除外82件はego無効60件＋有効futureゼロ22件。
motion filterは停止中abs(speed)<=0.05 m/s、選択command>=0.5 m/s、全15点の最大speed<0.2 m/sかつ最大変位<0.1 mを除外。
partial futureは完全な停止未来と見なさずcensoredとして保持する。

valのstopped-commanded集合は530 anchors、5 runs、0.5秒gapから推定41 episodes。
内訳は矛盾停止450、観測motion43、censored37。停止理由や運用上の発進可否が全件で確定している集合ではない。
val recovery runはleft-far、left-near、right-far。right-nearが欠け、straight等のannotationにもunknownが多い。

## 4. 現行V3と実験A

V3の履歴は学習/runtimeで4/4/10/10に合わせ、padding/mask・past-only command・timestamp逆行/resetを修正済み。
記録区間を使うgolden parityではtensor/mask/model出力の一致テストを実施。
最終ローカルpytestは615 passed、40 warnings。公式ROS/AWSIMでの今回の実行検証は未実施。

runtimeはmodel XYを経路に使い、model speedを無視し、外部0.75 m/sに曲率・ODD・Safety制約を適用する。
stop probabilityは未接続。ダミー値で有効化したことにしない。
path-onlyへの変更で短いpathが自動的に延長されるわけではない。

実験A：既存V3 checkpointから全parameterをfine-tune。fresh AdamW、LR1e-4、weight decay0.01、float32、seed42、batch2、accumulation8、5 epochs、13685 optimizer steps。
lossはtrajectory2.0、speed1.0、plan consistency0.25、control/control sequence/behavior各0.02、behavior side0.01。

| 評価対象 | 品質waypoint ADE m | runtime互換ready / 530 |
|---|---:|---:|
| A0 | 0.133046 | 2 |
| epoch1 | 0.132324 | 4 |
| epoch2 | 0.141244 | 8 |
| epoch3 | 0.136326 | 506 |
| epoch4 | 0.135267 | 4 |
| epoch5 | 0.134472 | 4 |

ADE非悪化とready>=80%を同時に満たすepochはなく、合格候補なし、runtime artifactなし。
epoch3は診断用に詳細評価しただけで、採用モデルではない。

epoch3：平均path length0.267 m、endpoint forward0.198 m、lookahead0.221 m、trim96.79%、Reference拒否1.89%。
worst-run ADE0.197185 m、FDE0.353609 m。復帰3 runsは改善、normal2 runsは悪化。
高いready率は限定されたoffline controller要求の成立であり、経路の正しさ・実際の発進・安全性の証明ではない。
geometryには微小区間由来の大きな曲率値があり、推定方法とmaskの再検討余地がある。

比較ツールのbootstrapがどの母集団・加重を使うかも確認してください。品質集合のwaypoint ADEと、未フィルタのframe ADEなど、異なる指標・集合のCIを同一の改善証拠として扱わないでください。
過去M3は未達。今回closed-loopは実行しておらず、collision-clearも未確認。

## 5. 検討してほしい設計

### A. 空間経路の出力契約

固定弧長waypoint、可変有効長＋mask、spline/control pointsなどを比較し、最小の推奨案を1つ選んでください。
XY shape、座標軸、基準時刻、単位、原点、point間隔、経路長、validity、通信メッセージを具体化してください。
0.1 m刻み・1.5〜2 mは検討候補で、確定要件ではありません。車両寸法・旋回性能・MPC horizon・速度・遅延・観測範囲から根拠を示してください。
弧長と終端Xを混同しないでください。曲線で一律X>=1 mや各点X単調増加を要求すると正しい経路も棄却し得ます。
停止、行き止まり、障害物、観測不足でも長い経路を強制生成しない設計にしてください。

### B. 既存データから作れる教師

最初の実装はcoverage監査にしてください。元データを上書きせずversioned view/sidecarを追加する方針を検討してください。
固定時間futureを弧長で再サンプリングするだけでは、存在しない距離の教師は増えません。
連続poseから長い未来を得る場合、run/segment/reset/teleport/方向反転を跨がず、最大時間・最大gap・pose品質・最大距離を定義してください。
停止時に非常に遠い時刻の未来をつなぐと、anchor観測と障害物状態が一致しない可能性があります。動的環境、将来の意思変更、長時間holdを扱ってください。
低速poseノイズを弧長として累積しない方法、重複点処理、カーブを切り落とさない再サンプリング、打切りmaskを示してください。
成功実走とplanned Referenceの教師は品質・意味が違います。Referenceが壁を横切らない保証、egoからの接続、実行可否、収録時刻を検証し、無条件の優先順位を置かないでください。
失敗/停止例を経路lossからmaskしても、安全停止やfailure検出に必要な標本をDataset全体から消さないでください。
必要な監査集計をnormal/recovery、停止/走行、左右near/far、曲率、run、episodeごとに定義してください。

### C. モデル・入力・学習

既存encoder/fusionの再利用とpath headの変更範囲を設計してください。時間軌道headと距離経路headの意味は異なるため、同じshapeでも重みの互換性を自動認定しないでください。
command履歴は制御policy依存です。MPC変更時に利用可能か、入力として残す利益とshortcut/自己フィードバックのリスクを検討してください。
全補助head/lossを0にする案、補助lossを保持する案を根拠付きで比較してください。
speed lossだけ0にしても、幾何速度とspeedを結ぶplan-consistency lossが残るなら速度との結合は残ります。弧長pointを0.1秒刻みと解釈する旧lossをそのまま適用しないでください。
幾何lossを提案する場合、XY以外に必要なものだけを選び、過剰平滑化・コーナーカット・障害物侵入・停止例への前進強制を検証してください。
4/4/10/10の入力parityを維持するか、変更時は新契約・再テストを明示してください。

### D. MPC・速度計画・停止との接続

空間pathをMPCのtime horizonへ変換する責任、速度profile、曲率・停止距離・遅延補償、path不足時の減速/停止を具体化してください。
各cap、raw/trim後path、controller要求、実測速度を別々にログ化してください。
MPCへの連続性、経路更新時のindex対応、隣接予測drift、車体footprintと障害物clearanceを含めてください。
停止判断は今未接続のstop headに依存せず、最小構成でどう成立させるかを設計してください。

### E. 評価と合格条件

V3のtime-index ADEとV4のdistance-index ADEを直接比較しない共通評価集合・距離範囲・maskを定義してください。
teacher coverageの増減で難しい標本が消える場合は母数とunknownを明示してください。
経路誤差、cross-track、接線、曲率、self-intersection、clearance、実有効長、trim量、frame間drift、MPC受理率を整理してください。
長い直線や原点固定などの退化解が合格しないnegative controlを含めてください。
停止中commandがあるだけで「前進すべき」と決めず、発進可能性・意図的hold・Safety停止・unknownを区別してください。
run単位paired bootstrap、重要sliceの劣化、5 val runsの限界、test利用タイミングを明示してください。
数値gateは車両/controller契約から導くものと暫定値を区別し、前回答のtrim10%や長さ1 mを根拠なく既定化しないでください。

## 6. 制約・提出形式

データ/重みはローカルにあり、GitHubで実体を閲覧できるとは限りません。新規大容量取得や有料資源を前提にしないでください。
Windowsを編集/Git正本、WSLを学習環境とし、WSL/SSHからpushしない運用を維持してください。
現行AGENTS.mdの主出力にはspeed/stopも含まれます。V4で責務を変えるなら、必要な契約文書変更を設計に含めてください。
既存V3を壊さない段階移行、schema/checkpoint identity、rollbackを具体化してください。

日本語で以下の順に提出してください。

1. 閲覧できたコード版・事実・未確認情報・診断仮説
2. 推奨構成1案と簡潔な責務図
3. 入出力/MPCインターフェース仕様表
4. Dataset coverage監査とversioned教師生成仕様・擬似コード
5. モデル・初期化・loss設計と不採用案の理由
6. 評価母集団・metric・gate・negative control
7. ファイル/関数単位の実装計画と必要テスト
8. データ監査→converter→小規模overfit→offline評価→限定closed-loopの段階計画
9. 最初に実施すべき比較実験1本、必要データ、開始/中止条件
10. Codexにそのまま渡せる最初の実装タスクの依頼文

一般論だけで終わらず、観測事実から何を検証すべきかを示してください。
空間教師化だけで問題が解決する、データ量が足りないだけ、元TransFuser転移が必須、とは未検証で断定しないでください。
論文を参照する場合は一次資料のURLと今回への適用条件を示してください。
