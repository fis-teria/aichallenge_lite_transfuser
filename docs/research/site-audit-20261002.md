# 公開サイトと実装の確認結果

確認日：2026年10月2日<br>
対象：[fis-teria/aichallenge_lite_transfuser](https://github.com/fis-teria/aichallenge_lite_transfuser)<br>
確認したmain：[8d9ab923e8cc2d01b28296a3032f65ab9438c384](https://github.com/fis-teria/aichallenge_lite_transfuser/commit/8d9ab923e8cc2d01b28296a3032f65ab9438c384)（2026年9月27日）

## 結論

既存サイトには17記事があり、システム構成、技術スタック、限定走行試験、TransFuser・DAgger・MPPIなどの短い論文紹介が揃っています。今回の調査では、これらを残しながら「E2E自動運転」「自動運転レース」「MPC・MPPIと安全監視」の3分野の見取り図を追加し、代表論文を深掘りする構成が適しています。

既存のTransFuser 2021記事は同じURLで詳しくし、ForzaETHとRobust MPPIは独立した記事として追加できます。現行の入力・出力契約や実験記録を論文の性能と混同しないことが、記事の判断基準です。

## 既存記事との役割分担

| 今回の記事 | 既存記事との関係 | 読者が判断できるようにすること |
|---|---|---|
| E2E自動運転の全体像 | システム構成記事から参照する分野横断の解説 | センサから直接操作する方式と、経路を出して別制御器で追従する方式の違い。教師情報の混入、模倣学習の分布ずれ、閉ループ評価 |
| TransFuser 2021詳解 | 既存の同名記事を更新 | 原論文の融合方式、入力・出力・評価条件、本プロジェクトと異なる部分 |
| 自動運転レースの全体像 | 新規 | 大域走行ライン、速度計画、局所回避、追従、車両同定、実行時間の関係。小型車・実車・シミュレーションの区別 |
| ForzaETH詳解 | 新規 | 統合スタックとして何を提供し、どの条件で検証され、何を移植候補と考えられるか |
| MPC・MPPIの全体像 | 既存の情報理論的MPC記事と相互参照 | Pure Pursuit、MPC、MPCC、MPPI、ロバスト追従、安全監視の役割分担 |
| Robust MPPI詳解 | 新規 | 既存の参照経路空間MPPIと異なる定式化、フィードバックを含む評価条件、適用に必要な追加要素 |

DINOv3とResNet18には既に図解記事があります。今回の3分野の説明から必要な箇所へリンクし、同じ内容を別記事へ繰り返さない構成にします。

## 実装に基づく説明の前提

### 1. TimePathの主出力は30点のXY経路

公開mainのTimePathV1は、観測時点のbase_link座標で、0.1秒から3.0秒先までの30点を予測します。形状は[B,30,2]、座標の単位はmです。GRUが座標差分を順に出力し、加算して経路を作ります。

点間距離から計算する値は区間の移動速度ノルムです。学習済みの速度Headや停止確率ではありません。また、原論文TransFuserと同じ目標点条件付きdecoderではありません。

根拠：[TimePathV1の実装](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/src/aic_transfuser_lite/models/time_path_v1.py)、[出力・教師分離のテスト](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/tests/test_time_path_v1.py)

### 2. センサ履歴と教師を分離している

画像、2D LiDAR、車両状態の履歴を使用します。無効な画像・LiDARスロットはCNNに入れず、マスクを使って履歴を処理します。forwardでは教師を除外し、指令履歴を使わない設定ではその入力をゼロ化・マスクします。

配布情報に記録されたbounded AWSIM用checkpointは、画像224×384、LiDAR750点、画像・LiDAR履歴各4、ego履歴10、hidden_dim128、融合2層・4headです。これらは配布checkpointの設定であり、すべての実験の共通条件ではありません。

根拠：[TimeBackboneV1](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/src/aic_transfuser_lite/models/time_backbone_v1.py)、[配布checkpointの記録](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/docs/model_distribution/current_time_path.json)

### 3. 経路予測と制御・安全監視は別の役割

通常経路はPure Pursuit等のモデル外処理へ渡します。任意のSLAM構成では局所占有地図に基づく回避経路を選択し、既存の操舵応答補償・レート制限と安全監視を通します。

現在のMPPIは参照経路の横方向形状など4係数を探索するreference-space方式です。256候補×3反復を用い、必要に応じて曲率を積分する候補へ切り替えます。Robust MPPIの実装や、完全な車両動力学に基づく操作系列のMPPIと同一視できません。重み付き平均経路も再検査し、未知セル・地図外は通行不可とします。移動障害物予測や路面の意味的な走行可能領域の判定は含みません。

MPPI用設定ではstop_v1と実測速度に基づく停止距離の判定が必須です。設定にある制御周期0.05秒や計画鮮度0.5秒は設定値であり、実測遅延ではありません。

根拠：[MPPI実装](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/src/aic_transfuser_lite/control/slam_mppi.py)、[構成と試験記録](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/docs/slam_mppi_avoidance.md)、[低速MPPI設定](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/configs/control/time_path_slam_mppi.json)

### 4. GNSS・IMU非依存の範囲を限定する

TimePath制御ノードは車速と実操舵角から局所的な位置・姿勢を積分します。これは車輪・操舵によるデッドレコニングであり、LiDAR odometryや大域自己位置推定ではありません。制御ノードにGNSS・IMU・外部poseを入力しないことは、シミュレータやAutoware全体がそれらを使用しないという意味ではありません。

モデルが受け取る横速度・yaw rateはAWSIMのVelocityReportに由来します。実車の車輪センサで同じ情報を得られるとは仮定できません。

根拠：[入力の由来と制限](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/docs/time_control_without_gnss_imu.md)

## 実測結果について維持する注意書き

- 132.3635秒の単独周回は、特定の試験条件での1回の記録です。一般的な完走率や障害物安全性を示しません
- 5km/h目標のrollout02では、静止箱の通過と通常追従への復帰を確認しています。外部supervisorの上限で終了しており、周回成功ではありません。複数配置の成功率、独立した接触カウンタ、実車体の最小距離は未測定です
- 20/15/15km/hは試験設定の上限です。最初の箱回避区間の実測最大速度は6.588km/hで、その後は予測範囲の短縮等によりSTOPPED_NO_LAPでした。「15km/hの箱回避に成功」とは書けません
- 保持経路を使う復帰処理には実装・テストの記録がありますが、最新版でのAWSIM復帰完了は未確認として記録されています
- 共通ILの6点waypoint・速度・停止Head、DINOv3関連の進捗は開発ブランチの内容です。現行配布モデルの性能と区別します。既存の32更新pilotは学習データ内のfitであり、実ログの停止Head学習、DINOv3公式重みとの比較、閉ループの性能改善は含みません

根拠：[試験記録](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/docs/slam_mppi_avoidance.md)、[保持経路復帰](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/docs/time_path_runtime_recovery.md)、[共通IL pilot記事](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/docs/site_src/articles/common-il-pilot-20260927.html)

## サイト更新方法と今回の検査範囲

記事の編集元はdocs/site_src/articlesとarticles.jsonです。公開HTMLを直接修正せず、Python標準ライブラリの生成処理で一覧・関連記事・公開ページを更新します。既存URL、過去の失敗、条件付きの結論は残します。

既存ソースのコピーに対して、次を実際に確認しました。

- サイト生成：成功
- 生成物一致検査：成功
- サイト用回帰テスト：29件成功
- 内部リンク等の構造検査：21ページ、601参照でエラーなし
- 記事本文にある21種類の過去commit固定GitHubリンク：取得成功

Git履歴を利用する通常の根拠検査、全体pytest、ブラウザ表示、ROS、AWSIMはこの確認には含みません。今回確認したテストソースの存在を、新しい走行試験の成功として扱いません。

公開先は[既存GitHub Pages](https://fis-teria.github.io/aichallenge_lite_transfuser/)です。確認対象mainの[配信workflow成功](https://github.com/fis-teria/aichallenge_lite_transfuser/actions/runs/36325100973)は確認できましたが、この調査段階では公開HTTPと配信本文の直接確認は完了していません。

正式な反映では、指定されたWindowsのサイト用checkoutで最新版と差分を照合し、生成・リンク・記事対応の検査を実行します。公開後は対象commitのActionsと配信された記事本文を確認します。今回の調査そのものによるモデル変更、学習、走行、GitHubへの変更はありません。

根拠：[サイトの追加・検査・公開手順](https://github.com/fis-teria/aichallenge_lite_transfuser/blob/8d9ab923e8cc2d01b28296a3032f65ab9438c384/docs/site/README.md)
