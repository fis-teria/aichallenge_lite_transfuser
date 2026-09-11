# Astra Proレビュー後の次工程

参照: [レビュー](<Astra Pro/astra_2026_1006_review_ja.md>)、[P0実装結果](time_path_p0_implementation.md)。
整理時の実装HEAD: `ab73206b07a1ca7cf7e73495ea785ef1385e4e9e`。
本資料は作業順序の整理。学習・実データ処理・AWSIMの再開指示ではない。

## 現在地点

P0の再現例に対する修正と、合成教師→時間参照→既存Pure Pursuit制御計算の試験は完了。
WSLの関連30件、全体1876件成功・4件skipの記録がある（今回再実行した結果ではない）。
これは実bagからモデル入力までの全工程や、実行時の安全統合を確認したという意味ではない。
新モデルは未学習。位置精度を現状のまま受け入れる方針と学習保留を維持する。

| レビュー項目 | 現状 | 次の境界 |
|---|---|---|
| B01/B02 | epoch補間両端・anchor/future frameの回帰修正済み | 実データのframe/epoch/基準点を確認 |
| B03/D01 | TimePathでvalid-only CNNとslot時刻表現を実装・合成検証 | 実履歴を同じslot仕様で組み立てる |
| D02/D03 | pre-dedup event保持・cut選択・独立XY/velocity maskを実装 | 実bag readerとDatasetへの接続、availability proxyの限界を明記 |
| D04/D05 | 型・30点設定・command設定の誤読込防止は実装 | 完全なcheckpoint/config/optimizer/RNG復元は未実装 |
| D06 | fractional age・旋回・短低速軌道・停止発進・resetの合成計算を検証 | 実body変換、遅延、feasibility/Supervisorは後続 |
| D07-D10 | 部分機能のみ。停止理由の全分類・学習集計・分割・評価系は未完成 | P1-prepで仕様化・合成検証 |
| H01-H04 | 効果・実害の大きさは未測定 | H03のOFF/ONは初回P1、H04は実教師監査、H01のモデル変更は後回し |

## 次に行う P1-prep（実データ学習なし）

### A. 教師・履歴を実際のDataset入力まで接続

対応D02/D03/D07/H02/H04。まず合成イベントと小型fixtureで実装する。
- 実カメラ取得時刻に対応する観測pose/egoを構成し、補間端点と利用可能時刻を記録する。
- TimeEvent→固定slot→画像/LiDAR/ego/指令tensor→TimePathをつなぐ。
- 同じイベント列・cutからoffline/runtime用の入力tensorが一致することを、共通関数で確認。
- 停止を観測stationary/環境stop意図/Safety制動/収集終了介入/horizon端に分類。UNKNOWNを許容。
- B0はstop_probabilityを生成せず、意図ラベルなしで停止能力を主張しない。
- 新しい教師経路を使うことを明示。旧combined validのデータだけから欠損XYを復元できたとは扱わない。
完了条件: 重複・遅着・欠落・reset・介入のfixtureでtensor/教師/mask/理由が再現。
注意: 既存の位置教師frameとPP要求のrear_axleを同一と仮定しない。推論出力のbody基準点と必要な変換を明記。

### B. 有効教師数に基づく損失・勾配蓄積と評価集計

対応D08/D10。現在のmasked_time_lossは有効アンカー平均として維持する。
- 各microbatchの平均を単純平均せず、蓄積windowの有効アンカー総数で重み付けする。
- 全無効microbatchをスキップしても、前の有効microbatchの勾配を消さない。
- optimizer/schedulerは実際の有効更新に合わせて進める。全無効windowは進めない。
- 固定予測で一括計算と分割計算のloss/予測勾配が一致する回帰テストを追加。
- BN/dropoutを含む全モデルのtrain-mode一括forwardとmicrobatch forwardの一致は要求しない。
- horizon別の支持数、raw誤差、入力欠損、出力棄却、採用率を集計。分母を失わず、支持0はNA。
完了条件: 有効数0/1/不均一の合成例で重み・勾配・更新回数・評価分母が一致。

### C. 完全な設定保存・復元と中断再開

対応D04/D05/D09。
- TimePath専用configと明示的なモデル構築入口を用意する。モデル作成前に、時間グリッド、座標/基準点、前処理、slot規則、command OFF/ONを型付きで検証。
- モデルだけでなくoptimizer/scheduler/RNG/step、split/教師manifest、継承元SHAとデータ系譜を保存。
- resumeと新規fine-tuneを区別。距離モデルのheadは流用しない。
- 合成fixtureで保存再読込と中断再開を検証。実コーパスでの学習は行わない。
完了条件: 設定違いを明示拒否し、同設定の状態を再現。データ系譜不明ならその旨を保持。

### D. run分割と比較実験設定の固定

対応D09/D10/H02/H03。
- 各速度設定10runをtrain6/validation2/test2に分ける案を固定manifest化。正式割当は原本IDとhash照合後。
- 同コース・同条件内holdoutと明記し、未知条件への汎化とは呼ばない。
- 速度設定・実測速度・自然frame分布・均等sampling・run平均を区別する。
- 継承元の学習データ系譜を確認。test未見を確認できなければscratch比較を別に置く。
- 最初のP1にcommand OFF/ONを置き、同split・初期化条件・提示予算で比較する。
- 何の意図を予測させるかを固定。OFF時に5/8km/h設定差を識別できると決めつけない。
完了条件: split/履歴/未来の越境なし、ID/hash交差なし、testで調整しない仕組み。

## SSD移行・保留解除後の P1

1. 実際の保存先とWSL backing volumeに容量監視を合わせ、20周アーカイブをhash照合して配置。
2. run×horizon×除外理由の採用数、実測速度、停止時の位置差分ノイズ、frame/取付姿勢、arrival proxyを監査。
3. 実教師でB0を有限予算で学習。command OFFを依存の少ない基準候補とし、ONも初回比較する。
4. ゼロ移動・定速・定曲率の因果baselineと比較し、全分母・horizon支持率・最悪runを報告。

有効教師数や停止理由支持数はraw画像数61,203とは別。未測定値を推定で埋めない。

## P2 / P3

P2: body基準点の実変換、実遅延、出力の実行可能性と既存Safety Supervisor、単一指令権限を統合した後にAWSIM。
完走/進行量、発進、逸脱/衝突、介入、stale率、p50/p95/p99遅延を記録。常時停止を成功としない。
P3: baselineで必要性が分かった場合だけ独立速度/停止head、整合loss、goal、回復データ、decoder変更を比較する。

## 次の具体的タスク

**P1-prep-A: TimeTeacherとTimeHistoryをTimePath専用Datasetへつなぎ、合成イベントから30点教師・独立mask・因果入力を返せるようにする。**
旧V3 Dataset/trainerの15点・combined maskを黙って流用しない。
Aでbatch契約を固定し、Bの損失集計、Cのconfig/factory/checkpoint、Dのsplit/比較設定へ進む。
Bの固定予測テストやDのmanifest仕様は、Aと独立に準備できる。
整理段階では追加学習・新規走行・外部レビュー送信は実行していない。
