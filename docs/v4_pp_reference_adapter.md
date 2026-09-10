# V4 → 既存Pure Pursuit：参照準備段階

基準版: `9e1723242ef2e7993af3a4210195905bab012880`。
今回の追加はROS非依存の参照Adapter。V4・既存PP・AWSIMは変更しない。
**PPへのROS接続・走行は未完了**。非null出力もRUN許可ではない。

## 入出力と再利用

- `control/v4_pp_reference_adapter.py` は `ShadowBridge.accept()` の幾何検査、
  折線再サンプリング、観測時刻poseによる固定局所frame変換を再利用する。
- 入力は既存 `Plan` / `PathPose`。sourceは `FIXED_V4_UNCORRECTED`、XYは20×2 m。
  既存PLANログやRViz Pathだけから欠落するclock/epoch/pose根拠を推測しない。
- 出力は不変 `PreparedReference`。元XYは変更せず、plan識別子・観測時刻・
  monotonic生成時刻・clock/epoch・期限・frame・変換差を保持する。
- 最後の接線は最後の非零線分から採用する。終点にゼロ長線分を加えない。
- 速度はモデル予測ではない明示試験方針。既存Limitsの上限、曲率横加速度、
  残長、制動遅延・実効制動から上限を計算し、終点速度を0にする。
  これは参照速度であり、実車減速成立やjerk制約の証明ではない。
- 消費側は入力更新とは独立して毎周期 `poll()`。期限切れ・時計巻戻り・
  epoch変更・新規不正入力で保持参照を破棄し、last-validへ戻さない。

## 未接続部分

`config/v4_pp_reference_trial.json` は無効状態の設定案であり、実行launchではない。
実Limitsはnull。fixture値を現AWSIM校正値として使わない。

後続実装は以下を必要とする。

1. 同じforwardのPLANへclock/epoch/pose根拠を結合する専用搬送。
2. 現在SLAM外挿poseと実速度・実操舵の時刻整合済み状態。
   `/v4/slam_odometry` の未設定twistを車速0と解釈しない。
3. Vehicle root、後輪中心、既存PPの基準点・frame契約の明示変換。
4. 実根拠のLimits、現在状態との追従可否検査、PPの短horizon対応設定。
5. 既存PPの専用Trajectory購読・shadow command出力・override無効化。
   標準Trajectoryにepoch/期限はないため、搬送時と消費側の失効処理が必要。
6. 不正参照で既存PPが停止要求へ移ることを確認後、単独送信元でsim接続。

この段階にpublisher、engage、モデル推論、障害物監視解除はない。
ROS/実scan/既存PP実行/AWSIMはNOT_RUN。

## 検証コマンド

Windows commit後に既定CheckOnly・同期を行い、WSLで:

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_v4_pp_reference_adapter.py tests/test_path_control_bridge.py
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

人工fixtureのみ。直線・左右円弧、raw不変、終端速度、観測pose変換、
複数消費周期、入力不正、未知契約、失効を検証する。
