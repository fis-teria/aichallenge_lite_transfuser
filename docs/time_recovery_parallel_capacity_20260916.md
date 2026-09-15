# AWSIM復帰データ収集の並列化候補

以下は並列化前の容量調査。続く実装・2環境実走の結果は
[AWSIMを変更しない2環境試験](time_parallel_multiscale_20260916.md)を参照。

同じPCでまず2環境・各1台を比較する価値がある。3環境による3倍速は未確認であり、
現状の実測からはGPUの処理時間が制約になり得る。同一AWSIM内の3台収集はUnity側と
収集系の改修が必要で、現在の実行バイナリでそのまま利用できると確認したものではない。
本調査中も既存の60cm収集を継続し、並列環境の起動やランタイム変更は行っていない。

`graneple@192.168.3.10` の60cm右側r03走行中に、約2秒間隔で12回・約22秒測定した。
すべて走行中のbaseline区間で、faultなし。復帰中のピーク、複数環境、長時間の熱制限は未測定。
GPU使用率はAWSIM・RViz・デスクトップを含む装置全体の値であり、環境数に比例するとは限らない。

| 項目 | 1台収集中の実測 |
|---|---:|
| GPU | RTX 4060 Laptop / VRAM 8188MiB |
| GPU使用率 | 平均54.75%、50～60% |
| GPU使用メモリ | 最大1355MiB |
| CPU全体使用率 | 平均25.61%、最大28.21%（20論理CPU） |
| 利用可能RAM | 最小7.39GiB（システム全体15.25GiB） |
| シミュレーション時間 / 実時間 | 0.99990 |

別時点のcontainer CPUスナップショットはsimulator 168.53%、autoware 155.41%、
collector nodes 130.46%（100%は1論理CPU相当）。CPUは14物理core / 20論理CPU。
現在の制御側CPU割当は2つの物理coreに対応する4論理CPUであり、全体使用率の低さだけで
制御スレッドの余裕を断定できない。並列化では物理coreの割当も分けて検証する。

測定値は [parallel_capacity_profile.json](evidence/time_recovery_60cm_20260916/parallel_capacity_profile.json)。

| 案 | 利点 | 必要な確認・変更 |
|---|---|---|
| 同じPCにAWSIMを2環境、各1台 | 既存の1台収集を独立させやすい | ROS通信・clock・保存先・CPU割当の分離、GPUと記録I/Oの競合 |
| 同じAWSIM内でセンサ付き3台 | シーンや地図の処理を共有できる可能性 | 各車のセンサ・pose・control・TF・教師・監視・外乱管理を分離。車同士の干渉も評価 |
| 別PCに1環境を追加 | GPU/CPU/RAMの競合を避けやすい | 同一実行条件、ROS通信の分離、転送先の競合。別PCの余力は本調査では未計測 |

AWSIM Labs公式文書ではROS2ノード・シミュレーション時計・topicの構成が説明されている。
センサを各車へ配置する仕組みはあるが、それだけで今回の3台同時収集が完成するわけではない。
NPCの追加と、Camera/LiDAR/ego履歴/教師を各車に対応付けて記録することを区別する。
[ROS2接続の公式説明](https://autowarefoundation.github.io/AWSIM-Labs/main/Components/ROS2/ROS2ForUnity/)、
[センサ追加の公式説明](https://autowarefoundation.github.io/AWSIM-Labs/main/Components/Vehicle/AddNewVehicle/AddSensors/)。

現行ローカル実装は `tools/run_time_recovery_awsim.py` の起動時に `ROS_DOMAIN_ID=1` を指定し、
`tools/time_recovery_collector_node.py` はdomain 1以外を拒否する。
operatorの開始条件も実行中containerなしを要求する。したがって二重起動するだけでは対応できない。
これらを単に削除せず、実行単位の通信分離と権限・監視確認へ置き換える必要がある。
同じDDS空間に複数の `/clock`、姿勢、制御topicを混在させない設計にする。

別プロセスのAWSIM環境Aをdomain 1、環境Bをdomain 2とする例では、各環境のAWSIM・
制御・監視・rosbag・RVizの全プロセスを対応するIDへ揃える。同名topicを維持しても、
異なるdomain間の通常のDDS通信は分離される。
[ROS 2公式説明](https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Domain-ID.html)。
同一Unity/AWSIMシーンに3台置く案では、共通clockに対して車ごとのtopic名前空間とTF frameを
分けるのが基本。プロセスの環境変数 `ROS_DOMAIN_ID` を変更するだけで3台を個別のdomainへ
分けられるとは扱わない。この調査では実際の二重起動やdomain分離試験はまだ実施していない。

保存先は1runのraw約1.37～1.38GB、転送用圧縮約0.58GB。
回収後の実行先空きは約14GiBなので、並列時は録画とpackが重なる分の余裕が小さい。
複数収集に共通の空き容量予約と転送順序を設け、現在の実走中10GiB以上という条件を維持する。

推奨する比較は、1環境の現状値を基準に、隔離した2環境を同じ時間だけ動かし、
実時間あたりの「因果的な入力・教師検証に通った復帰イベント数」と55～65cm帯の
Cameraアンカー数を比べる方法。RTF、clock/制御間隔、センサ欠損、停止監視、I/Oも併記する。
仮に2環境が各RTF 0.8で同じ採用率を維持できれば、走行部分は理論上1.6倍になるが、
これは計算例であり実測速度向上ではない。初期化・転送・不採用イベントを含めた実時間で判断する。
