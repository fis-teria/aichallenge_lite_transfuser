# V4 shadow ROS2 node

既存ament_pythonパッケージ `aic_e2e_runtime` に `v4_shadow_node` と
`v4_shadow.launch.py` を追加した。別の独自runnerを起動窓口にしない。
既存MPC/controller launchは変更せず、並列のROS nodeとしてinclude/起動する。

```bash
colcon build --packages-select aic_e2e_runtime
source install/setup.bash
ros2 launch aic_e2e_runtime v4_shadow.launch.py config_file:=/absolute/reviewed.json
```

設定雛形はshare/aic_e2e_runtime/config/v4_shadow.example.json。
enabled=falseで配布する。実node名・Odometry topic・pose frame/根拠・command意味・
一意の出力ファイル・有限envelopeを確認して指定する。UNKNOWNを仮値で有効化しない。
既存固定step500 loaderのcheckpoint配置を再利用し、ランダム重みへfallbackしない。
ROS/PyTorch/既存Python依存は実行環境側に必要。環境更新は行わない。

親ROS processはsubscription・graph監視を担当し、別spawn childが既存
SpatialInputV4 / ShadowObservationJoin / ShadowSession / fixed loaderを再利用する。
queueは各64件。満杯・graph fault・reset・期限切れ・worker終了で有限sessionを終了。
終了時は所有childだけをjoin→TERM→KILL。ROS受信をmodel forwardで塞がない。
親でfaultを検出した後のchild結果は採用せず、node再起動で勝手に期限更新しない。
このnodeからAWSIM/controller/gear/modeを起動・変更・送信しない。

出力はJSONLの未補正V4経路と処理状態。車両制約が未設定なのでShadowBridge(None)を
使用し、追従commandを有効化しない。MPCへV4経路を入力する構成ではなく、
MPCが別途走行している間のV4 shadow記録用nodeである。
通常ROS timerのgraph監視に加え、既存monotonic freshness checksを維持する。
外側の120秒上限・AWSIM停止・累積試験予算は試験起動側の責任であり、本nodeの
追加だけで車両停止確認済みとはしない。

現AWSIM単独ではcommand publisherとOdometryがないことを前回実測済み。
外部制御器を含むlaunch側で提供する必要がある。GNSS/IMUによるpose生成は別途
確認が必要で、単なるtopic remapでOdometryに見せかけない。

限定tests：tests/test_v4_shadow_package.pyと既存transport/join/session/bridgeテスト。
実checkpoint読取・推論・AWSIM走行は今回のパッケージ整備では行わない。
