# 10km/h教師104窓を追加する再学習

`configs/time_path_p1/native_obstacle10_replay_20260918.json` は、従来の通常・発進・復帰
60,608提示/epochをすべて維持し、旧障害物338窓と新しい10km/h教師104窓を
それぞれ10回提示する別実験。合計65,028提示/epoch、3 epochs、6,099更新。
新規104窓はコーン周辺74・通常30。繰り返しや重複窓は独立イベント数ではない。

初期モデルは復帰・発進保持の基準モデル
`runs/time_launch_protection_v2_20260916/launch_balanced/epoch_03.pt`
（SHA256 `1fe12ca066791d3eb8907c6921120e2cfbb38925c422d5be54940f4acbdd120a`）。
前回の障害物再学習は発進の操舵余裕の保持基準を満たしていないため、初期値に採用しない。
旧338窓は学習データとして引き続き使う。旧設定・重み・出力を上書きしない。

バッチ32、FP32、seed42、LR1e-5、最大3時間。入力・モデル・損失は前回と同じ。
既存の復帰補助損失も維持。30点の将来XYを学習し、観測速度は監査用に保持する。
停止・行動モードは未確定のままで、今回それらの学習を追加しない。

選別manifestと原本をSHA256確認し、442窓すべての入力履歴と教師を再現する。
旧1配置グループと新2配置グループはすべてtrain専用。旧validationの割当を保ち、
testは使用しない。近接正面（前方6m以内、横±1m以内）の新規採用は0件という
不足は残る。品質保留568窓や除外1窓を混ぜない。

通常11,505フレーム/4走行、復帰8,962フレーム/42走行、発進210条件で旧モデルと
各epochを比較する。旧保持基準を緩和せず、全体・コーンおよび収録run別の教師適合を
出す。教師適合は学習データ上の評価であり、未知障害物での回避成功を意味しない。
条件を満たす候補がなくてもruntimeは変更しない。AWSIMでの再評価は別工程。

## 実行

Windows正本でcommit後、cleanなWindows同期用cloneから`tools/sync_to_wsl.ps1`を使用。
無関係のstaged変更を保持するため、元checkoutの変更を取り消さない。
WSL checkoutは `/home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin`。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh timeout --signal=INT --kill-after=60s 10800s \
  env PYTHONPATH=src .venv/bin/python -u tools/train_time_native_replay.py run \
  --root /home/thistle/e2e_autonomous \
  --plan configs/time_path_p1/native_obstacle10_replay_20260918.json
```

出力: `/home/thistle/e2e_autonomous/runs/time_native_obstacle10_replay_20260918/`。
`preparation/proof.json`に入力・提示数・source commit、`training/verification.json`に
完了時の更新数と重みハッシュ、`evaluation/selection.json`に保持判定を記録する。
学習中断時のみ同じソース・設定で`train --resume`し、その後`evaluate`する。
`run`や`prepare`で既存出力を上書きしない。

## 起動前検証

WSLの全pytestは3,262 passed / 4 skipped（128.37秒）。初回起動はソースガードで
停止し、学習出力は未作成。旧基準commit以降の保護対象差分は、別診断ツールのみが
使用する`training/native_fit_v1.py`の新規追加だけで、今回の学習経路からは参照しない。
モデル・入力・既存損失・旧設定が同一であることを確認して、今回のソースガード基準を
`dfe2cfa89417d84c87343522f3694bee4246cd3d`に更新した。ガード自体は維持する。
