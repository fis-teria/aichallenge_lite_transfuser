# 最新成立入力を優先するV4 shadow join

## 変更範囲

基準HEAD: `803eac1529d6150e208529681abc0175d450d05a`。
`ShadowObservationJoin` と純粋合成testsのみを変更。
ROSの購読・配送タイマー、AWSIM、モデル、重み、制御publisherは変更しない。

- 最大16件の候補を新しい順に検査し、成立する最新cameraを一度だけsessionへ渡す。
- 最新が入力待ちでも、古いが未処理・期限内・成立済みの候補を選べる。
- 選択したcameraより古い候補は `SUPERSEDED_BY_READY` で終了する。
- 欠落したbracket/scanは元受信から最大300msの期限内だけ待つ。
  固定grid不適合等は即時拒否し、後続を期限までブロックしない。
- 1 tickあたり最大1回。推論中に入力を書き換えず、次回は新しいcutoffで検査する。
  同じ画像をsensor更新だけで再推論しない。配送の最短20msは維持するため、
  callback到着と完全同時の推論開始を保証する変更ではない。
- 各sensorの64件履歴は保持。epoch/frame/timestampとdecoded valueが同じ再配送だけ
  重複排除し、元のavailabilityを保持する。同時刻の異なる値は理由・role・時刻を記録し、
  FAULTをラッチする。明示resetなしで継続しない。

## 100ms履歴契約

phase、100ms grid、40ms camera許容差、30ms LiDAR許容差、50ms ego補間許容差を維持。
105ms cameraを100ms取得と偽装したり、9Hz timerを追加したりしない。
off-grid画像は今回も採用しない。既存SIM_GRID_MISSING_V2の欠損slot処理を維持し、
捨てた候補を観測済みの履歴として補充しない。全cameraで推論する変更ではない。
品質・実効Hz・実測遅延改善はAWSIM再試験までUNKNOWN。

## 検証手順

Windowsで今回変更をcommit後、既定syncを変更せず使用する。

```powershell
.\tools\sync_to_wsl.ps1 -CheckOnly
.\tools\sync_to_wsl.ps1
ssh codex-wsl 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q tests/test_shadow_observation_join_v4.py tests/test_shadow_delivery_v4.py tests/test_shadow_ros2_transport_v4.py tests/test_publisherless_shadow_v4.py tests/test_v4_shadow_package.py'
```

既定syncのDataset操作は固定rootに対する `test -d` のみであることを静的確認。
Dataset内容・raw・sensor・checkpointの読取、実ROS・実推論・走行は検証に含めない。

## 実行結果

検証commit: `513a4ec8d98e312d4fcdd8316c65b1b8d80fddb5`。
Windows commit → CheckOnly (`CHECK_OK`) → 通常同期 (`SYNC_OK`) を実施。
既定同期によるDatasetルートの存在確認を実施した。
既存Dataset内容・raw・sensor・配布checkpointの読取りは未実施。
テスト内部の人工データ・一時モデル生成は実運用データの検証とは別である。

- 上記関連tests: **58 passed in 4.49s**。
- WSL共有lock下の全体tests: **1741 passed, 4 skipped, 51 warnings in 70.52s**。
- skip: OSQP未導入、Draft2020 validator不足、jsonschema未導入、公式Tiny package未指定。
  これらをPASSとしない。追加依存の導入は行わなかった。

全体検証の実行コマンド:

```powershell
ssh codex-wsl 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q --junitxml=tmp/v4_latest_ready_junit.xml'
```

JUnitはWSLとWindowsの `tmp/v4_latest_ready_junit.xml` に保存（Git非追跡）。
AWSIMホストへの配布・install更新、実ROS、固定V4の実入力推論、走行はNOT_RUN。
次は同じPP走行＋V4 shadow構成でjoin待ち・推論間隔・重複/競合理由を再測定する。
今回pushは行っていない。
