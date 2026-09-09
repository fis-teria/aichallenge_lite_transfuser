# 車速・操舵状態による局所Odometry

## 確認した契約

Windows正本: fis-teria/aichallenge_lite_transfuser、branch codex/windows-wsl-training-sync。
開始HEAD: 0c93c662d147b6820c863110f9cf9b9e7bbdece3。
既存未追跡の2監査レポートは前の作業成果として保全し、今回一緒に記録する。

SSH先は読取のみ。`~/git/autononous_ai/aichallenge-racingkart` HEAD
`4af395eee10f928c7fc7225760adfa04c4c07ff4` の現ファイルを確認した。
`aichallenge/workspace/src/aichallenge_submit/racing_kart_description/config/vehicle_info.param.yaml`
にはwheel_base=1.087m（前後輪中心間）、max_steer_angle=0.64radがある。
URDFはbase_linkのvisual originしか示さず、実simulator車速基準点と後輪中心線の横オフセットはUNKNOWN。

今回読み取った現ファイルSHA256:

- vehicle_info.param.yaml: `29e480278da2d7a04f936eaa7adf8a47f2690ba51b129646d74ac23368f3adca`
- vehicle.xacro: `9481faefeab06c268a607201b56c17973442a0056d8df909ca10c4cd60acc9bb`

保存DLL由来のVehicle.SteerAngleはactualSteerAngleを返し、同じ値が両前輪のUpdateWheelSteerAngleへ渡る。
VehicleReportRos2Publisherは`-SteerAngle * PI/180 + noise`をSteeringReportへ格納する。
要求Commandではなく適用後操舵角の報告（noise付き）であり、rad、ROS左旋回正の符号を使う。
VelocityReportのbody-frame vx/vyはm/s。Heading_rateは今回の局所Odometryでは診断専用。
競技使用可否はユーザー確認に基づく。独立した競技審査や実車精度保証ではない。

## 実装

- `steering_odometry_v4.py`: ROS非依存の有限buffer・時刻結合・bicycle yaw計算。
- velocity header stampを基準に、SteeringReport.stampの前後値を補間。同時刻は直接使用。外挿なし。
- 最大結合待ち/補間間隔250ms、buffer各32件。欠損・順序逆転・同時刻競合は理由付きfault、clock resetでcacheと原点を破棄。
- 単位: m/s, rad, m, ns。raw headingの±1256やNaNを積分せず、その値はtraceへ保存。
- 既存SE(2)積分を再利用。並進は報告vx/vyを保持する（vyを0と推測しない）。
- curvature k=tan(delta)/L、基準点が後輪中心線から左へyだけ離れている場合、w=vx*k/(1-y*k)。中心線上なら通常のvx*tan(delta)/Lになる。縦オフセットはこのvx関係に現れない。無横滑りbicycle近似であり実物理の完全再現ではない。
- `/v4/local_odometry` twist.angular.zも推定値へ変更。GNSS/IMU/地図・Command購読・actuator出力なし。V4本体のego入力heading_rateは今回変更しない。
- stamp、補間元stamp/重み、操舵角、raw heading、推定yawを記録。

## 現在の有効化状態

`v4_shadow_node`からconfigの`local_odometry_geometry`を渡せるようにした。
必須fieldはwheelbase_m、reference_left_offset_m、evidence。
**現行設定には追加していない。未指定ならLOCAL_ODOMETRY_DISABLED REFERENCE_GEOMETRY_REQUIREDを出し、poseを発行しない。旧heading積分へfallbackしない。**
基準点の横オフセットを未確認のまま0と扱わないための状態であり、追加ライブ接続を実施したわけではない。
単独entrypointもgeometry未指定で無効。現段階は親V4からの明示config指定が有効化経路。

## 検証手順

```powershell
./tools/sync_to_wsl.ps1 -CheckOnly
./tools/sync_to_wsl.ps1
ssh codex-wsl 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_local_odometry_v4.py tests/test_steering_odometry_v4.py'
ssh codex-wsl 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q'
```

合成fixtureはL=2m等の人工値。直線/左右旋回/後退/停止、巨大raw heading非依存、補間、非有限値、重複、逆順、期限、有限buffer、reset、特異幾何を検証する。
実ROS・AWSIM走行・実精度評価はNOT_RUN。AWSIM改変、SSH先への反映、pushなし。
同期scriptは無変更、固定Datasetルートの存在判定だけを行うことを静的確認済み。実行結果は後記する。

## 実行結果

実装・限定検証commit: `c9aa7d9c860ba7ea32059952890945ff87ceae0e`。
既定CheckOnlyはCHECK_OK、通常同期はSYNC_OK、WSL HEAD一致。
WSL lock付き限定合成テスト: **35 passed in 7.95s**。
同commitでWSL lock付き全pytest: **1776 passed, 4 skipped, 51 warnings in 82.14s**。
skipはOSQP実solver、Draft2020 validator、jsonschema、公式Tiny packageの未導入/未指定による既存条件。

既定同期によるDatasetルートの存在確認を実施。
本実装・限定合成検証ではDataset内容・raw・sensor・checkpointの読取りは未実施。
フルpytestは既存suiteの通常実行であり、追加実モデル推論・ライブセンサ取得・ROS起動の許可や結果を意味しない。
