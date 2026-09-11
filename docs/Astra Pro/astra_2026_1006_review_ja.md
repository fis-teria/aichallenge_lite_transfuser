# 時間基準E2E r01 — 批判的レビュー

## 結論：条件付き（学習性能・走行安全性の保証ではない）

**30点・0.1秒・観測時body座標の位置模倣というB0は採用できる。ただし、このZIPのまま学習開始・実車接続を承認するものではない。** 大規模モデルへの交換より、教師のepoch/frame境界、maskの意味、因果履歴、時間参照を制御へ渡す契約を先に修正する。

`time_path_v1.py`の30点decoder、教師をforwardから除去する処理、masked L1の通常契約に、学習が原理的に成立しない致命的な数式・shape矛盾は確認しなかった。一方、再利用される既存backboneおよびconverterには下記の再現可能な問題がある。checkpoint、trainer、time adapterの未実装を、存在しない実装のバグとして数えない。

**必須の順序変更：§7の時間adapter契約と合成oracle追従試験をP0へ前倒しし、command OFF/ONはP3ではなく最初のP1比較へ。** 学習済みE2Eを使うAWSIM閉ループ評価はP2のままでよい。

## 1. 範囲・証拠

`START_HERE.md`、`proposal.md`、core、関連converter/history/backbone/controller/tests/reportsを読んだ。元ソースの修正、optimizer step、実データ学習、ROS/AWSIM起動、actuator出力、添付資料の外部送信は行っていない。公開一次資料は公開ページだけで照合した。

ZIP manifestの484項目についてsize/hash照合不一致0。ZIP SHA256は `ca814128b50e565cde1c59055b31c43a5dc7f394274aa3f3455fd4d8a947b64c`。出所metadataのsource commitは `de8d5ba95a592be6d3dea8da7db18b0d22b12876`（Gitサーバーの履歴を独立検証したという意味ではない）。core SHA256は `9b5e4b9253df3b46e1762991380bf98a54f89cabf22a4abdfca84f887c85e7c7`。

| 今回実施した検証 | 結果 | 限界 |
|---|---|---|
| `tests/test_time_path_v1.py` | 5 passed / 2 warnings | 元テストの入力は1スロット、evalモード。4/10スロットのtrain-mode maskをカバーしない |
| temporal/full-control shape/Safety Supervisorの関連3ファイル | 30 passed / 5 warnings | 全repositoryの回帰試験ではない |
| model側追加probe | 7観測項目をJSON化 | 無学習の小型CPU fixture。実走行誤差ではない |
| converter側追加probe | 4観測項目をJSON化 | 合成レコード。20周の実データ中の発生率ではない |

Python 3.13.5 / PyTorch 2.10.0+cpu / torchvision 0.25.0+cpu。歴史的な元環境とは異なる。BatchNormのprobeは使い捨ての無学習モデルをtrainモードでforwardしたもので、重みの最適化は0回。警告はTransformerのnested tensorに関するもの。原資料の1858 passed等を今回再実行した結果とは扱わない。

証拠：`core_pytest.log`、`dependency_pytest.log`、`probe_results.json`、`data_probe_results.json`、`source_evidence.md`。

## 2. 指摘表

P0は意味・安全な接続を守るため次段階前に解消する事項、P1は最初の学習/評価/保存を信用する前に仕様化する事項、P2はbaselineで必要性が確認された場合の比較事項。実装phase名とseverityの役割は区別する。

ソースの短いファイル名はZIP内 `repo/src/aic_transfuser_lite/` の対応ファイルを指す。行番号は元ファイルの1始まりで、原文抜粋は `source_evidence.md`。

### 2.1 実装済みコードで再現した不具合

| ID | 重要度 | 分類 | ファイルと行／提案節 | 発生条件 | 影響 | 最小修正 | 確認テスト |
|---|---|---|---|---|---|---|---|
| B01 | P0 | 実装バグ（既存converter） | canonical_converter_v3.py:L280–307, L545–556 | epoch内の教師時刻の補間右端がepoch外にある。run全体のindexを使用。 | epoch外の変更が有効教師を変える。合成例では0.30m→20.14m。 | epoch別indexと補間両端検査。巡航教師では介入境界の前後混在も拒否。 | epoch外のposeだけを変えても内側教師が変わらない。境界を使う点はmask=False。 |
| B02 | P0 | 実装バグ（既存converter） | canonical_converter_v3.py:L191–205, L559–567 | 観測アンカーと未来poseのframe/childが異なる。未来側の補間両端同士は一致。 | 異なる座標系の数値を減算し、30点すべてvalidにできる。 | anchorとfutureの親・子frameとepochを照合。変換根拠がなければ拒否。 | anchor=map、future=other_mapのexact点列を拒否。既知90度旋回の符号も検査。 |
| B03 | P0 | 実装バグ（継承backboneのmask隔離） | full_control_lite_v3.py:L181–190; camera_encoder.py:L24–39; lidar_encoder.py:L43–53 | 欠落履歴を含むtrain-mode CNN。mask適用前にB×T全画像をBatchNormへ通す。 | 無効画像の値が有効特徴・予測・running statsを変える。camera経路で再現。 | TimePath側の新経路で有効フレームだけCNNへgatherし、特徴をscatter。旧V4の既定挙動は維持。 | 無効値・padding数を変更してもvalid特徴/BN統計が不変。有効入力の勾配は維持。 |


B03のcamera probeでは、無効画像だけを0→3へ変更するとeval-modeの予測差は0、train-modeの最大XY差は約0.8455、BN running mean差は約0.1218だった。これは任意の無学習fixtureの差であり、実車が0.85m逸脱すると推定した値ではない。LiDARも同種のBatchNorm構造を持つが、この数値probeはcamera側で行った。

BatchNormはforward時のmini-batch統計を用いるため[3]、後段GRUでmaskを掛けてもCNN内部の混入は消えない。gradient accumulationで有効batchを8にしても、1回のCNN forwardの統計は8アンカー分にならない。有効画像のみencodeする修正を先に行い、Frozen BN/GroupNorm等への変更は初期化条件を含め別比較とする。

### 2.2 設計・契約の不足（再現した既存挙動を含む）

| ID | 重要度 | 分類 | ファイルと行／提案節 | 発生条件 | 影響 | 最小修正 | 確認テスト |
|---|---|---|---|---|---|---|---|
| D01 | P0 | 設計欠落（挙動を再現） | temporal/gru.py:L62–85; proposal §2 L30,34 | 欠落位置が違うが、valid値の順序列は同じ履歴。 | GRUもTimePath全体も同一出力になり、経過時間を区別できない。 | valid特徴に固定スロットの相対時刻/位置を埋め込む、または欠落tokenで時間更新する。actual sensor dt追加とは分ける。 | A,欠落,B,C と 欠落,A,B,C の時刻表現を区別。offline/runtime一致も維持。 |
| D02 | P0 | 設計欠落（既存前処理との不整合） | mcap_converter_v2.py:L773–775, L878–885; proposal §2 L31–35 | 同じheader stampの早着レコードと、入力確定後の遅着レコードがある。 | 全体last-winsで、その時点では使えた早着レコードが失われる。後段maskだけでは回復不能。 | epoch・arrival・sequenceを保持。入力cutに対する適格性を先に判定し、その後重複を解決する。 | cutより後のイベントを追加しても過去tensor不変。早着のみが過去cutで選択される。 |
| D03 | P0 | 設計欠落（既存教師schemaとの不整合） | canonical_converter_v3.py:L548–567; proposal §1 L20–23 | 未来poseはあるがvelocityが欠ける。 | 位置教師まで全欠損になる。合成例は30位置あり/valid=0。 | time_viewでxy_mask、velocity_mask、command/stop maskを分離。旧combined validからXYの可否を決めない。 | velocity/commandだけを削除してもXY教師・XY lossは不変。速度区間maskは両端AND。 |
| D04 | P1 | 設計欠落（coreで再現、提案に既記載） | time_path_v1.py:L18–28, L50–54; proposal §3 L47–50, §5 L67 | dictの数値同値比較、またはuse_command=Falseのstate_dictをTrue構成へstrict load。 | bool/int違いを受理。weightsは完全一致loadでも推論の意味が変わる。 | 明示型schemaとmodel/preprocess/history設定をcheckpoint保存。構築前に検証。 | runtime_ready=0、time_sec内True、command設定不一致、距離checkpointを拒否。 |
| D05 | P1 | 設計欠落（設定API、提案に既記載） | time_path_v1.py:L53–59; full_control_lite_v3.py:L73–74; proposal §3 L49 | TimePathへtrajectory_steps=30をkwargs指定。 | 旧15点head制約で初期化失敗する。実際の新decoderが15点という意味ではない。 | 新decoder設定とbackbone設定を分離。旧headを新設定から公開しない。 | 新configで30点構築、旧distance/head key誤投入は説明付き拒否。 |
| D06 | P0 | 設計欠落（実装順序を含む） | proposal §7 L89–95, §8 L99–102; v4_pp_reference_adapter.py:L105–118 | 学習を先に行い、時間軌道のage処理・速度意味・低速lookaheadを後回しにする。 | 正しい予測でも追従不能、誤速度、毎周期減速、発進不能となり得る。 | 時間adapterの契約と合成oracle追従試験をP0へ。学習済み全体閉ループはP2のまま。 | fractional age、旋回中の自己運動補償、短い低速軌道、停止→発進、epoch reset。 |
| D07 | P1 | 設計欠落 | proposal §4 L54–59; safety_supervisor.py:L17–18, L178–194 | stationary、stop意図、収集終了介入、horizon端、Supervisor制動を同一扱い。 | 偽の停止能力、発進抑制、未学習stop確率の誤接続。 | 観測停止/環境停止意図/安全停止/収集介入/未来未観測を区別。B0はstop_probabilityを作らない。 | 各状態のmask/理由が異なる。停止支持数0ならNOT_EVALUATED。B0用Safety設定整合。 |
| D08 | P1 | 設計欠落（学習器・評価集計） | time_path_v1.py:L93–99; proposal §5 L68–69, §6 L80–83 | 短いfuture mask、速度帯sampling、異なるsupported数のmicrobatchを混在。 | アンカー平均と点平均を混同し、長期誤差や蓄積勾配の重みを誤解する。 | 現lossの目的を維持し重みを開示。蓄積はwindow内supported総数で正規化。 | 固定予測で、不均一supportの分割/一括lossと予測勾配が一致。horizon支持数を出す。 |
| D09 | P1 | 設計欠落（評価証拠） | proposal §5 L66–67, §6 L78–84 | 同コース反復を未知条件と呼ぶ、継承元表現のデータ出所を監査しない。 | 独立性・汎化・test隔離の過大評価。 | 6/2/2を固定条件のrun holdoutと明示。元checkpointの学習データ系譜と重複hashも監査。 | split/hash交差ゼロ、前処理のtest fitなし。系譜不明ならscratch基準を別報告。 |
| D10 | P1 | 設計欠落（評価分母） | proposal §6 L81–83, §7 L92–95; v4_10_tracking_trial_20260910.md:L84–95 | 棄却plan、長期教師欠損、ゼロ停止支持、補正前の失敗を集計から消す。 | 受理例だけは良いが実用不能なモデルを高評価する。 | raw誤差・accept条件付き誤差・coverage・入力無効/出力棄却・進行量を併記。 | 全評価anchorから分母を追跡。常時停止/定速baselineを含め、支持0をNAにする。 |


D01はGRUCellの数学的バグではない。現在のクラスは「invalid stepではhiddenを変えない」という宣言どおりに動く。しかしそれを時間履歴として再利用すると、固定グリッド上の穴の位置が表現に残らない。これは入力配列の「穴を詰めない」だけでは直らない。相対slot位置を有効tokenへ付与すれば、実際のsensorずれdtを追加せずに最低限の時間識別性を持たせられる。

D02では早着レコードのbag receiptを1.01秒、同headerの遅着を2.0秒、入力cutを1.5秒とした。global dedup前は使用可能なレコード1件、後は0件になる。遅着をそのまま使えば非因果的、利用時刻で除外すれば本来使えた早着まで失う。新time pipelineは未実装なので「新pipelineが実際に未来情報を漏らしている」とは断定しない。

D04はproposal自身が既に認識している不足。`runtime_ready=0`、`time_sec`の1.0を`True`へ置換しても値比較は通る。またcommand=FalseのweightsをTrueの構成へstrict loadしてもmissing/unexpected keyは0だった。これは未実装checkpoint loaderのバグの発見ではなく、weightsだけでは構成を復元できないことの確認である。

### 2.3 学習・走行の証拠が必要な仮説

| ID | 重要度 | 分類 | ファイルと行／提案節 | 発生条件 | 影響 | 最小修正 | 確認テスト |
|---|---|---|---|---|---|---|---|
| H01 | P2 | 未検証仮説 | time_path_v1.py:L67–73; proposal §3 L44 | 30段で遠方誤差や勾配の問題が増える可能性。 | 学習・追従性能低下の可能性。現時点で失敗の実証なし。 | 30段B0を維持。horizon別loss・grad norm・初期速度整合を測る。 | 同条件での位置/勾配診断。必要時だけstep embeddingまたはdirect-30 headと比較。 |
| H02 | P1 | 未検証仮説（入力不足の条件は明確） | proposal §3 L45, §6 L76–78 | 同一観測履歴から異なるroute/目標速度の教師を要求する。 | 単一決定論出力では意図を識別できず、平均化等が起こり得る。 | 初版は同固定コースの行動模倣と限定。意図制御が要件なら運用route/速度上限を入力契約に追加。 | 同じ観測に異なる意図が必要な例の監査。goalなしの全課題適合は主張しない。 |
| H03 | P1 | 未検証仮説 | time_path_v1.py:L50–66; proposal §8 L102 | 過去指令が教師policyの手がかりになり、閉ループで分布が変わる可能性。 | sensor無視や自己強化の懸念。因果的な過去指令それ自体はleakではない。 | command OFF/ONを初回P1比較へ。ONは実際に送出した過去指令のみ。 | 同split/提示数の比較、遅着/欠損/安全介入後の指令履歴、closed-loop復帰。 |
| H04 | P1 | 未検証仮説（演算意味は確定） | time_path_v1.py:L31–41; proposal §3 L46, §4 L54–57 | 低速で0.1秒位置差分のノイズが速度と同程度になる可能性。 | 速度振動・偽発進・不安定な方位/曲率。AWSIM実ノイズ量は未確認。 | B0を残し位置/速度不確かさと短区間集約を評価。独立速度headは必要時比較。 | 静止+合成ノイズ、定速円弧、停止→発進。速度教師との意味を揃える。 |


## 3. そのまま残してよい設計

`TimePathV1.forward` L61–77はtargetsを除去し、自分の出力を次stepへ入力して30点を生成する。新規テストL56–65のteacher隔離も今回通過した。ただし将来のdataset builderがteacher-only情報を入力特徴へ混ぜないことまでは、このテストでは証明しない。

`masked_time_loss` L80–99はmasked NaNを演算前に除去し、有効アンカー内→アンカー間の順で平均する。全無効でNoneを返すこと、無効出力位置への直接勾配が0であることは妥当。自己回帰の前stepが後のvalid点へ影響する間接勾配まで0にする必要はない。

`interval_speed` L31–41は観測時原点p0=0を含む完全な30点に対して、区間弦長速度として正しい。符号付きv_long、接線方向の瞬間速度、stop probabilityではない。固定0.1秒に対する係数も整合している。

3秒・30点、独立速度headなし、goalなしは、限定タスクの実験を不可能にする理由ではない。将来のモデル交換をP0の必須修正にしない。

## 4. 修正後の時間・座標・因果性契約

### 4.1 別々に保持する時刻

- `t_obs_ns`：教師原点・ego状態・出力座標の基準となる実カメラ取得時刻。
- `slot_time_ns`：4/10履歴の要求時刻。実際に選ばれたcapture時刻と区別する。
- `available_ns`：入力処理が利用可能になった時刻。bag receiptを同値だと断定しない。
- `t_freeze`：その推論入力を確定した時点。capture時刻とは別。
- `t_publish`、controller now、clock/epoch ID：推論遅延・配送遅延・age・reset判定用。

時刻ごとにclock domainも保持する。sensor/sim時刻とarrival/steady時刻が異なる場合、同じns単位でも直接減算・比較しない。対応関係またはイベント順序のcutを明示する。補間入力の利用可能時刻は少なくとも両端が揃った後とし、両端provenanceを保持する。

モデルに渡す各イベントは同一epoch、対象slotの仕様内、同じclock domainで利用可能時刻<=入力cutを満たすこと。将来到着するレコードを見て過去のtensorを変更しない。bag全体の対称補間を推論入力に無条件に使わない。

一方、t_obsよりわずかに後に取得したLiDARでも、入力cut以前に本当に使えた場合には、定義した入出力遅延契約次第で利用可能。header>t_obsという一点だけで未来正解漏洩と判定しない。過去限定を選ぶ場合は、その選択で20Hz LiDARと30ms閾値に起きる採用率変化を確認する。

既存50ms補間閾値は、converter L176–187では「各補間端点からtargetまで」の最大距離であり、端点間gap全体を50msに制限していない。最大gapは100msになり得る。値を勝手に変更するのではなく、この意味をschemaと監査出力へ記録する。

### 4.2 教師とmask

同一world/body基準に揃っている場合の教師は、

`p_i = R(yaw_obs)^T [p_world(t_obs+i*0.1) - p_world(t_obs)]`。

converter L559–562の回転式自体はこれと整合する。必要なのは、frame/child、車体上の基準点、epoch、補間両端が本当に一致するかの検査。base_linkと後輪軸中心等の一致を名前だけで仮定しない。

出力の最低契約は `xy_m[B,30,2]`、`xy_mask[B,30]`、整数nsの時間格子、原点frame/時刻、教師provenance。velocityやstopのmaskは独立させる。future_maskはloss/評価専用であり、推論用のsensor availability maskと混同してモデルへ渡さない。teacherに区間速度を作る場合、m0=Trueとして `interval_mask_i = xy_mask_(i-1) & xy_mask_i`、加速度なら必要な3点のANDを使う。

巡航データの介入除外はproposal §1の保守的なアンカー全体除外を初版で維持する。さらに、targetがブレーキ開始前でも補間端点が介入後にある点を採用しない。観測済み停止位置の重複はvalid、単なる未観測未来はinvalid。未知の未来を最後の点で埋めて停止教師にしない。

### 4.3 現在入力の欠損

既存 `ModelBatchV3.validate(require_current=True)` は現在camera/LiDAR/全ego特徴を要求する（L208–212）。time履歴で過去をmaskできることは、現在センサー欠損でも推論可能という意味ではない。初版は現在入力不成立を理由付きで棄却し、失効・停止を既存Supervisorへ接続する。部分ego特徴で推論する別設計を黙って導入しない。

## 5. 速度・停止・追従の契約

### 5.1 位置差分速度はB0で残すが、そのまま低速commandへ直結しない

`v_i = ||p_i-p_(i-1)||/0.1`は区間(i-1,i)の弦長速度。瞬間速度教師と比較するなら時刻・区間定義を合わせ、rawと集約版を両方残す。独立速度headはB1比較でよく、B0開始の必須条件にしない。

位置誤差の1座標成分を同分散σ²、隣接相関ρと仮定すると、差分速度誤差の分散は `2σ²(1-ρ)/dt²`。例えばσ=0.01m、ρ=0、dt=0.1sなら標準偏差は約0.141m/s。このσは説明用の仮定であり、AWSIMの実測精度ではない。ノルム化による静止時の正バイアスもある。

対応は精度改善の要求ではなく、固定の既存位置データから静止/低速時の差分分布・相関を測り、信頼できるvelocityとの意味を合わせた比較、0.3–0.5秒程度の区間集約候補、低速での方位/曲率未定義の扱いを実装すること。集約時間は未校正の比較候補で、制御遅延や発進の鈍化も評価する。

### 5.2 停止を5種類に分ける

1. **観測上のstationary**：教師として位置/速度がほぼ一定である状態。
2. **環境理由によるstop意図**：障害物・停止指示等に応じて止まる意思決定。状態stationaryとは別のラベル根拠が必要。
3. **Safety Supervisorの制動**：入力欠損、stale、障害物停止距離等による安全動作。
4. **収集終了の介入**：`brake_sim`以降と、その未来を含む巡航アンカーは除外。
5. **予測horizon/収録の終端**：先がないことは停止の正解ではない。

stop stateの初期監査用候補として、速度<=0.05m/sが0.5秒続くとstationary、>=0.10m/sでmovingへ戻すヒステリシスを提案する。ただしこれは**レビューで置いた未校正候補**。ノイズ帯と重なる観測はUNKNOWNとしてstop mask=Falseにし、位置教師は維持する。データに合わなければtrain/validation側のノイズ監査で固定し直し、testで調整しない。この分類閾値をそのまま実車の安全停止閾値に流用しない。

stop意図headの正例には、stationaryだけでなく環境理由の証拠が必要。支持例がなければstop意図性能はNOT_EVALUATEDとし、B0を止める条件にはしない。

B0は `stop_probability=None` を保つ。SafetyConfigのモデルstop入力要求を有効にするなら対応head/契約が必要で、欠損を架空の0確率で埋めない。B0用統合configは既存`enable_model_stop=False`の意味と整合させ、現在有効な安全機能を無断で外さない。

### 5.3 時間adapterは学習前に検証する

age `a=now-t_obs`。過去body座標から現在body座標への変換は、

`T_B(now)_B(obs) = inverse(T_W_B(now)) * T_W_B(obs)`。

各残存点の時刻は `i*0.1-a`。a=0.23秒なら0.2秒分を単にindex shiftするだけでは足りず、p(a)の補間と残り時刻の定義が必要。clock epochを跨いだ変換・古いplanの再stamp延命は禁止。

速度は元の観測時点列か正しい補間区間から求める。crop後の先頭点に対して新たに「現在原点0から0.1秒」の差分を取ると、追従位置誤差を速度に混入させる。PPの横制御と縦制御、またはMPCの参照は同一plan ID・同一時間軸で扱う。

旧 `v4_pp_reference_adapter.py` L114–118には終点で速度を0にする方針がある。有限試験経路では合理的でも、毎回更新する3秒予測の終端を「到着すべき停止点」と扱ってよいとは限らない。停止距離/期限切れの保守性は維持しつつ、prediction-horizon端とmission終点を別フィールド/理由で区別する。

低速で3秒の予測幾何が短くなる点も必須試験。一定0.15m/sの合成oracleなら全長0.45mで、旧試験のlookahead=1mには届かない。これはその速度・定速仮定での算術例であり、新モデルの実際の全軌道が0.45mという断定ではない。速度を上げて帳尻を合わせず、短い参照に対するPPのlookahead/終端挙動、停止→発進、縦制御を合成段階で確認する。

## 6. 5つの懸念への最小変更

| 懸念 | 今回必須 | baseline後の追加候補 | 採用条件 |
|---|---|---|---|
| 30段GRU | 段数維持、finite/grad norm/horizon別error、初期ego整合を検査 | decoder step embedding、direct-30 head等の小比較 | 長期誤差・計算量・閉ループが実際に改善 |
| goalなし | 固定コースの行動模倣に用途を限定、意図の衝突を監査 | 運用中に得られるroute/目標速度入力 | 分岐・目標速度を選び分ける要件がある |
| 位置差分速度 | 意味、時間区間、mask、低速不確かさ、controller接続を定義 | 区間集約、独立速度head、弱い整合loss | 発進停止/旋回/速度誤差が改善、raw位置性能を隠さない |
| 過去指令依存 | OFF/ON比較をP1へ。ONは実送出済み指令の因果履歴 | command dropout、復帰データ等 | 安全介入/欠損/閉ループでも改善 |
| 低速ノイズ | 既存精度を受容、UNKNOWN帯・mask・ゼロ長処理 | ノイズを考慮したloss、補助速度head | データの不確かさに対して効果を確認 |

このcoreはtrainとevalの双方で自分の予測を再入力しており、teacher forcingをtrainだけで用いる構造ではない。したがって「30段だからteacher-forcing由来のtrain/test不一致が必ず起きる」とは言わない。一方、閉ループで自分の行動が次の観測分布を変える問題は別に残る。これは模倣学習の既知の論点[2]だが、今すぐDAgger導入を必須にする根拠にはしない。

command OFFは依存の少ない診断baselineとしてアタシの第一候補。ただしONが悪いと決めつけない。同じ位置・センサーでも5/8km/h設定による目標の違いがある場合、OFF側が意図情報を失うこともある。実測速度と設定速度を区別し、モデルに何の行動を予測させるかを先に確定する。

## 7. 分割・重み・指標

### 分割と初期化

各速度設定6/2/2のrun分割は、同コース・同条件内holdoutとして条件付きで妥当。ただしtestは合計4runで、独立した未知シナリオ20件の証拠ではない。test ID、親データ、処理hash、履歴/未来窓を学習前に固定する。後で回復走行やnormalデータを混ぜる場合も出所を追跡する。

backbone継承は許可key/shapeの確認だけでなく、継承元checkpointがどのデータを見たかの系譜確認が必要。これが不明なら「test未見の汎化」とは主張せず、scratch基準を別に置く。ここでは実際に過去checkpointがtestを見たと断定していない。

### 重み

現lossは `L=(1/|A|) Σ_a [(1/n_a) Σ_i m_ai*(|e_x|+|e_y|)/2]`。

これはvalid anchorを均等に扱う目的であり、バグではない。しかしvalid=1点のanchorと30点のanchorが同じ総重みになり、1点あたりの重みは最大30倍違う。sampler確率q_aが変わるなら、horizon iの有効重みは `Σ_a q_a*m_ai/n_a` にも依存する。loss一本で長期精度を比較しない。

microbatchごとにこの平均を出して単純に等分蓄積すると、microbatch内のsupported数が異なる場合に一括batchと異なる勾配になる。蓄積window全体のsupported数で正規化する。集計の一致テストは同じ固定予測、またはBN/dropout等のbatch依存を固定して行う。train-mode BNのある全モデルの一括forwardとmicrobatch forwardが完全一致するという主張ではない。全無効microbatchではoptimizer/schedulerを進めず、既に蓄積した勾配を誤って破棄しない。BNの統計は勾配蓄積とは別問題。

自然frame分布、速度設定均等、run macroを区別する。設定5/8km/hは実測5/8km/h走行ではない。実際に採用された教師の速度分布・停止/発進/旋回支持数を再集計する。

### 指標

位置L1の座標平均とEuclidean ADEを混同しない。0.5/1/2/3秒の各horizonで同じ対象集合/支持率を示す。横/縦誤差はbase_linkのy/xなのか、教師経路の接線/法線なのかを明記する。旋回時には一致しない。停止点で接線未定義なら固定ルールまたはNAを使う。

raw全点の誤差、controller採用点の条件付き誤差、採用率、入力欠損率、出力棄却率、stale率を分離する。NaNや棄却例を誤差の分母から黙って消さない。支持0のstop/回復場面を誤差0として扱わない。教師を必要としないゼロ移動・定速・定曲率等の因果baselineを置く。

閉ループでは追従誤差だけでなく、定時間内の進行量/完走、介入/停止回数、逸脱・衝突・制約違反、発進成功、p50/p95/p99遅延とdeadline超過率を報告する。常時停止で安全指標だけ良くなるモデルを合格にしない。frameを独立サンプルとして狭い信頼区間を出さず、run単位の値と最悪runを出す。4 test runでの不確かさを隠さない。

## 8. 推奨構成と実装順序

```text
Raw event records (epoch + capture + arrival/available + sequence)
  ├─ teacher-only branch → epoch/frame/intervention-safe XY + independent masks
  └─ causal input cut → 共通time_history → RGB / 2D range+validity / ego / optional past sent commands
                           ↓
                 valid-only CNN + fixed-slot time encoding
                           ↓
                    既存fusionを再利用
                           ↓
                 30-step XY GRU（B0を維持）
                           ↓
             typed time plan + source stamp / config identity
                           ↓
      time_reference: age / ego-motion / interval semantics / feasibility
                           ↓
        Pure Pursuit＋既存縦制御、または比較用MPC
                           ↓
             既存Safety Supervisor → 単一command権限
```

teacher branchはモデル入力branchへ接続しない。V4の既存経路と既定挙動は維持し、新time経路は明示選択で有効化する。

| 段階 | 作業 | 合格基準 | 今できる範囲 |
|---|---|---|---|
| P0-A | teacher epoch/frame/介入境界、独立mask、raw event保持 | B01/B02/D02/D03の反例が安全に処理される | 合成レコードの単体テスト。大容量rawは不要 |
| P0-B | 時間slot表現、valid-only CNN、型付きconfig/plan | mask隔離・時間識別・teacher隔離・offline/runtime parity | CPUの小型fixture、checkpoint構成テスト |
| P0-C | time adapterとoracle PP/縦制御 | fractional age・短低速軌道・停止発進・resetを正しく扱う | 学習なしの合成/運動学モデル。実車権限なし |
| P1-prep | split manifest、loss集計、optimizer/scheduler/resumeの契約 | 固定予測でsupport不均一の一括/蓄積集計一致、test非使用、構成復元一致 | 現時点は実装と非学習テストまで |
| P1 | SSD後に実教師監査、有限予算B0、OFF/ON比較 | horizon/scene別結果、kinematic baseline比較、失敗開示 | SSD導入・学習実行許可後 |
| P2 | 同条件AWSIM閉ループ、旧モデル/oracle比較 | 進行・制約・停止・遅延・全分母を報告 | 実行条件が整ってから |
| P3 | B1/B2/B3、goal、stop意図、回復データ等 | 具体的なbaseline失敗に効き、別場面を悪化させない | 必要性確認後 |

最初のCodex作業はP0-A/B/Cを独立した小PRへ分割すること。今回のレビューはそれらの修正パッチを作成・適用したものではない。まず反例を回帰テストとして移植し、現在は問題を再現するassertを、修正後は望ましい挙動のassertへ変更する。

## 9. 未確認事項・追加証拠

実画像/LiDAR/pose配列と学習済みTimePath重みはZIPにない（START_HERE L26–31）。したがって、20runの教師採用率、frame/extrinsicsの実値、位置ノイズ量、未来mask分布、停止理由の支持数、30段学習の収束、実走行性能は未確認。

次に必要な証拠は、(a) clock/frame/補間端点/介入境界/arrivalの少量監査出力、(b) run×horizon×理由の採用数、実測速度と停止時差分分布、(c) 前処理とcheckpoint系譜のmanifest、(d) oracle時間追従と遅延注入試験、(e) SSD後の有限予算B0と閉ループ結果。大容量rawの転送や位置精度改善を、その前提として要求しない。

latencyの旧V4実績は原因切り分けの背景であり、新TimePathが同じ割合でstaleになると予測したものではない。既存Safety単体テストの成功も新time integrationの安全性証明ではない。

## 10. 原典と独自推論の区別

[1] TransFuser公式CVPR2021実装のGRU waypoint rolloutはtarget pointを使う。今回の30点/.1秒・goalなし・2D LiDAR・小型fusionを、原典の検証済み性能として扱えない。構造の参考と独自モデルの評価を分ける。

[2] Ross, Gordon, Bagnell, *A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning*, AISTATS 2011. 自分の行動により後の観測分布が変わる模倣学習上の問題の根拠。今回のモデルに特定手法が必須だという証明ではない。

[3] Ioffe, Szegedy, *Batch Normalization: Accelerating Deep Network Training by Reducing Internal Covariate Shift*, ICML 2015. mini-batch統計で正規化する機構の根拠。padding混入の具体的な指摘は、このZIPのコードと今回の合成再現に基づく。

公開一次資料URL（添付資料の内容は送信していない）：

```text
[1] https://github.com/autonomousvision/transfuser/blob/cvpr2021/transfuser/model.py
    https://arxiv.org/abs/2104.09224
[2] https://proceedings.mlr.press/v15/ross11a.html
[3] https://proceedings.mlr.press/v37/ioffe15.html
```

**最終判断：モデルの大型化より先に、「正しい時刻の正しい教師を、maskに忠実な表現で学び、正しい時間参照として追従器へ渡せる」ことを確定する。この条件を満たしたB0を、固定コース内の閉ループ成立性を調べるbaselineとして進める。**
