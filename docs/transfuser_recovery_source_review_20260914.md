# TransFuserの復帰学習に関する一次資料の精査

2026-09-14。対象はユーザーが提示したPAMI版と公式`2022`実装の説明。
**主要なコード上の説明は正しい。ただし、角度拡張や微小操舵ノイズが復帰性能の主因だと
断定する根拠にはならない。今回の実測復帰データ収集を、それらだけで代替しない。**

公式`2022`ブランチを確認時のcommit
`9d413b2ad2d2d56c112b34a4a799be081800d77f`へ固定して照合した。
論文PDFと7個のソースのURL・サイズ・SHA-256は
[取得記録](evidence/transfuser_recovery_source_review_20260914/source_manifest.json)に保存した。
以下の「今回への判断」は、このコードと現行プロジェクトを比較した設計上の判断である。
新規AWSIM走行・学習・拡張の実装は本精査では実施していない。

## 提示された説明の判定

| 提示内容 | 判定と補足 | 一次資料 |
|---|---|---|
| ±20°の角度方向の視点拡張 | 正しい。既定では90%の確率で範囲内の角度を選ぶ。毎回±20°の両端だけへ回す意味ではない | [論文p.9、§4.7](https://www.cvlibs.net/publications/Chitta2022PAMI.pdf#page=9)、[config.py](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/config.py#L29) |
| RGBの切出し位置、LiDAR、waypoint、目標点を対応させる | 正しい。RGBは横方向のcrop移動、他の幾何量は対応する座標変換。画像平面内の回転とは異なる | [data.py](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/data.py#L211) |
| 教師操舵へ標準偏差0.001の正規ノイズを加える | 正しい。車両へ返す`control.steer`に加算される。CARLAの正規化指令で、radではない | [autopilot.py設定](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_autopilot/autopilot.py#L54)、[指令生成](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_autopilot/autopilot.py#L249)、[CARLA API](https://carla.readthedocs.io/en/0.9.10/python_api/#carlavehiclecontrol) |
| 先頭2点から操舵PIDの目標と速度を求める | `2022`の通常制御として正しい。2点の中点の方向で操舵し、2点間距離を0.5秒で割った速度を追う。停止・詰まり時には分岐がある | [model.py](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/model.py#L648) |
| 再予測と追従の組合せで復帰する | 妥当な機構の説明。ただし専用の復帰モードや、任意のずれから戻れる保証を確認した意味ではない | [実行時のPID呼出し](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/submission_agent.py#L362) |
| 横移動と向きのずれは別、教師だけの移動は不整合 | 今回の設計判断として妥当。教師の座標変換だけで別位置のカメラ/LiDAR観測や車両運動は生成できない | 下記の幾何・時間契約の検討 |

## 修正すべき解釈

### 角度拡張は、実車体をずらして復帰走行を取り直す処理ではない

公式の収集実装は左右・正面の3画像を連結し、学習時にその広い画像から横へずらして切り出す。
これにより仮想的な向きの違いを与えるが、物理的な横位置の変更や、その姿勢からの車両運動を
再シミュレーションする処理はここにはない。
[カメラ構成](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_autopilot/data_agent.py#L87)、
[画像連結](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_autopilot/data_agent.py#L173)、
[切出し](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/data.py#L545)。

したがって「向きの異なる見え方と進行方向の対応を学ぶ」は妥当だが、
「実際に外向きの速度・操舵状態から戻る時間軌道を新たに生成する」と読み替えない。
これは実装からの推論であり、角度拡張が無効だと述べているわけではない。

さらに論文§4.12では、最終構成から回転拡張を除いた影響は小さく、有意とは考えにくいと
述べている。対象は論文の走行ベンチマークで、今回のコーナー復帰条件を直接評価した実験ではない。
**「TransFuserの復帰は±20°拡張によって解決された」との因果的な結論は避ける。**
[論文p.14、アブレーションの説明](https://www.cvlibs.net/publications/Chitta2022PAMI.pdf#page=14)。

### ノイズ付きの実行指令と、教師として読む項目を分ける

公式では車両に渡す操舵へ微小ノイズを加える一方、measurementの`steer`保存には
加算前の値を渡す。これと、学習に用いる将来位置の出所は別に確認する必要がある。
[autopilot.py](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_autopilot/autopilot.py#L249)。

`data.py`のego waypointは、将来の保存フレームの車両行列を読んで現在座標系へ変換する。
ここで使う将来位置は、ノイズを含む教師制御下で実際に観測された車両位置である。
同じJSON内に予測waypointがあっても、それを学習教師に使うと推定してはいけない。
[保存行列](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_autopilot/data_agent.py#L282)、
[waypoint構築](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/data.py#L375)。

今回提案する意図的な操舵パルスは、この微小ノイズとは大きさ・時間構造・目的が異なる。
パルス適用中の将来軌道を「戻るべき正解」として学習させず、解除後の教師制御区間を採る。
過去に受けた外乱と実測操舵は履歴として残す。公式の0.001をAWSIMの0.001radへ転用しない。

### 先頭2点という点数だけを制御へ移植しない

公式の保存間隔は0.5秒で、既定出力は将来4点。通常制御は車両座標へ戻した
先頭2点の中点と、2点間距離の2倍を使う。
[保存周期](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_autopilot/autopilot.py#L33)、
[出力設定](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/config.py#L5)、
[制御計算](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/model.py#L655)。

こちらは0.1〜3.0秒の30点である。直進・等速5km/h・遅延なしという説明用の条件では、
先頭2点の中点は約0.21m先になり、公式の時間間隔なら約1.04m先になる。
これは時間間隔からの計算であり、AWSIMの実測値ではない。
速度算出の係数2もこちらへコピーすると、同じ等速条件で速度を本来の1/5に見積もる。

今回は固定目標5km/hと、時刻補正・距離に基づくPPの整合を維持する。
正しい復帰軌道を渡して追従できるかを検証し、PIDへの変更をデータ不足の対策に混ぜない。
現行契約は[時間教師](../src/aic_transfuser_lite/data/time_teacher_v1.py)と
[制御計算](../src/aic_transfuser_lite/control/time_trial_v1.py)を参照。

## 現行モデルへ角度拡張を入れる前の条件

現行の[Dataset](../src/aic_transfuser_lite/data/time_dataset_v1.py)は、1系列のRGBを
`[4,3,224,384]`、2D LiDARを`[4,2,750]`、egoを`[10,4]`で構成する。
egoは前後速度・横速度・yaw rate・実測操舵を含み、
[TimeBackbone](../src/aic_transfuser_lite/models/time_backbone_v1.py)が履歴を読む。
記録する指令履歴は`[10,3]`だが、比較対象の
[実行設定](../configs/control/time_path_segment_5kmh_20260914.json)では`command_history=false`。
履歴を保存することと、現行モデルがそれを使っていることを区別する。

| 論点 | 今回の要件 |
|---|---|
| カメラ | 元画像のFOV・内部/外部パラメータと残せる有効画角を確認。公式と同じ広い連結画像を現在持つ前提にしない。RGBの平面回転や黒い補完領域を、実際のyaw視点と呼ばない |
| LiDAR | 距離列を画像のように回転せず、角度・原点・有効maskの契約に従って再投影する。観測されていない領域や遮蔽物の裏を有効な観測へ変えない |
| 履歴と状態 | 同一サンプルの全時刻で同じ仮想視点の定義を保つ。座標変換と車体姿勢の変更を区別し、横速度・yaw rate・実測操舵との整合を確認する。操舵角へ拡張yawを単純加算しない |
| 時間教師 | 幾何変換が一致しても、姿勢を変えた車両が同じ時間内にその軌道を走れる保証はない。初期速度、曲率、操舵応答とPPの追従を確認する |
| split | 拡張はtrainで行い、元runと全派生サンプルを同じgroupに保つ。validation/評価用に訓練時のランダム変換を流用しない |

もう一つの違いは、公式はナビ目標点で予測を条件付けるのに対し、
[現行TimePath](../src/aic_transfuser_lite/models/time_path_v1.py)にはその入力がないこと。
今回のずれの主因とは未確定だが、原著と同じ入力条件の実験とは扱わない。
基準線や将来poseを推論入力へ足して復帰できたことにしない。
[公式の目標点設定](https://github.com/autonomousvision/transfuser/blob/9d413b2ad2d2d56c112b34a4a799be081800d77f/team_code_transfuser/config.py#L27)。

## 収集案への反映

[収集方法の設計](time_recovery_collection_method_20260914.md)に以下を反映した。

1. 収集PPと評価の実測速度帯をそろえる。目標5km/hでも前回の採用アンカーは約3.38km/h、
   失敗側は約4.58km/hだったため、設定値だけで同条件としない。
2. 通常PPに短い操舵パルスを加え、解除後にまだ外向きの状態から実測復帰を採る。
   初期候補は左右5〜25cm・外向き2〜4度。到達可能性は方式確認で検証する。
3. 解除済み指令の発行境界、過去の画像/LiDAR/ego/指令、未来3秒の教師区間を一緒に記録する。
   最初の0.5秒の除外理由も監査し、向き直り後だけ残ることを防ぐ。
4. 角度拡張は別の追加実験にする。まず実測復帰の追加だけを比較し、
   カメラ・履歴・動力学の条件が成立した後に、拡張単独/併用の効果を比較する。

前回の[採用状態の監査](time_corner_recovery_coverage_20260914.md)で、位置と角度が十分ある
60件は全て元ラインへ向いた組合せで、外向きは0件だった。これは対象の独立2run・118アンカーの
集計であり、手持ちの全データに外向きが存在しないという結論ではない。
今回の優先順位はこの実測の不足範囲に基づく。

| 比較段階 | 変えるもの | 固定するもの・判定 |
|---|---|---|
| 最初 | 既存データに実測復帰を追加 | 同じ初期重み・学習予算・評価run・制御設定で、外向き状態の時間別XY誤差と通常走行への影響を比較 |
| 追加条件成立後 | 角度拡張だけ、実測復帰＋角度拡張 | まずvalidationで選び、未使用の独立runとAWSIMで確認。拡張しただけのフレームを独立走行数へ数えない |
| 必要な場合 | E2E短区間走行から早期に教師へ引継ぎ | モデルが実際に訪れる状態を補う。引継ぎ後の教師走行をE2E完走として数えない |

「waypointの方向が戻る」だけで採用を決めない。車体の初期運動により、正しい復帰でも
最初は横ずれが増える場合がある。ピークずれ、復帰時間、監視余裕、操舵飽和、
介入なし完走と通常走行の悪化を評価する。

## 検証範囲と再確認方法

本作業は文献・ソース読解と設計文書の変更のみ。学習結果や新規実走による効果検証は未実施。
既存コードを変更していないため、`pytest`の再実行は行っていない。

取得した原本はGit対象外の`tmp/transfuser_recovery_source_review_20260914/`に置いた。
PDFはテキスト抽出して該当節を読んだ。ブランチ名だけを再取得すると版が変わり得るので、
再確認には取得記録のcommit固定URLを使う。

```powershell
pdftotext -layout tmp/transfuser_recovery_source_review_20260914/Chitta2022PAMI.pdf tmp/transfuser_recovery_source_review_20260914/Chitta2022PAMI.txt
rg -n -C 3 'angular viewpoint|removing the random rotation' tmp/transfuser_recovery_source_review_20260914/Chitta2022PAMI.txt
rg -n 'steer_noise|control.steer|self.save\(' tmp/transfuser_recovery_source_review_20260914/team_code_autopilot__autopilot.py
rg -n 'control_pid|desired_speed|aim =' tmp/transfuser_recovery_source_review_20260914/team_code_transfuser__model.py
```
