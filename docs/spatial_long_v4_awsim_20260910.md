# V4-20 AWSIM observation trial

ユーザーのAWSIM試験依頼に基づく、20 sim秒の実入力shadow試験。
遠方validation MAEが16 epochより小さい12 epochを固定する。
重みSHA256: `07f2a04b9da0e2673036a1f0de6e13d841f1c4f84aa12e7a33ff61c070286c33`。

`v4_shadow_node`の明示`long_model`設定だけで46点loader/sessionを選択する。
設定は`checkpoint`絶対pathと上記`sha256`。既定2m loaderは従来どおり。
46点のraw XY[m]を同じ観測時刻SLAM姿勢で表示。固定2m PP adapterへは接続しない。
既存PPのみが車両を操縦し、V4-20は受信・推論・診断出力のみ。
停止は所有AWSIMのfreeze/killで、制動成功やV4閉ループ走行ではない。

Windows commit・CheckOnly・通常同期後、WSLで：

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

独立配布先のHumble packageをbuildし、既存run27 runnerを専用runへ複製する。
checkpoint mountを12 epochへ変更し、shadow PP connection起動を省く。
host wall上限180秒、probe移動上限20秒、shadow wall150秒を維持する。
結果と正確な実行コマンドは試験後に追記する。
