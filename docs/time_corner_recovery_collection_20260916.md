# 全コーナー進入の復帰収集モード

コーナーを必須対象として台帳管理し、近接地点を別の周に割り当てる。
各周のイベント数は初期3以下、40 mの準備開始間隔を維持する。
失敗・見送りは取得済みにせず、実測教師を検証してから未取得地点の次周計画を作る。
AWSIM本体、5 km/h速度目標、停止領域・センサ・操舵監視を変更しない。

既存の横ずれのみの指定は既定値のまま再現する。追加した `target_heading_rad` は
実測正常走行の向きに対する誤差で、準備経路をHermite曲線で作る。経路は教師ではない。
教師は正常経路のPPへ戻した後のセンサ観測と実測将来位置だけを用いる。
目標横ずれ±5 cm、指定角±許容角を0.25 s保持し、復帰後10 sまで安定確認する。
ラベルには因果的な入力と3 s全将来の検証を引き続き要求する。

初期カタログは基準コースの曲率と向きの変化を確認した11区間。
同方向の複合コーナーは入口を複数に分けるため、競技のセクション番号とは異なる。
初期条件は横20 cm・外向き4度、許容角1度。各地点の地図審査と実走確認が必要。
旧コードの25〜300 m制限を20〜325 mへ拡張したが、実測guideの全区間支持と
準備・復帰候補の地図審査を必須とする。周境界を跨ぐ外乱は対象外。

二つのAWSIMはROS_DOMAIN_ID=1/2と個別Dockerネットワークで分離し、
同じコーナーを別runでtrain/validationに割り当てる。未見コーナーへの汎化評価ではない。
最大12 run（初期8、再試行4）、各run最大30分。生データは2 runずつWSLへ転送・照合する。
カタログ完了は、各splitの独立runにおいて各コーナー60以上の有効復帰anchor、
かつ入口付近（予定release -0.5〜+3 m）の目標横ずれ・角度・速度に一致する
有効anchorが3以上あること。完走・外乱コマンド送信だけでは完了にしない。

Windowsで編集・コミットし、`tools/sync_to_wsl.ps1`で同じcommitへ同期する。
WSL native repositoryで実行するコマンド:

```bash
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/plan_time_corner_recovery.py \
  --catalog configs/collection/corner_recovery_20260916.json \
  --base /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs/base.csv \
  --output /home/thistle/e2e_autonomous/runs/time_corner_recovery_20260916/plans
```

各planに対して `tools/generate_time_large_recovery_reference.py` を実行する。
正常走行源は `codex-time-recovery-sites-normal-n03` と既存のハッシュ証明を使う。
ROS wiring smokeは `tools/smoke_time_large_recovery_ros.py` をnetwork noneの公式環境で実行する。
実走は `tools/run_time_recovery_awsim.py --parallel-plan ... --ros-domain-id 1|2`。
WSLで `tools/audit_time_recovery_collection.py`、causal replay、materializeを経て
`time_corner_recovery_v1.corner_coverage` で進捗を確定する。
未取得分の計画は上のplanコマンドに `--coverage coverage.json` を付けて別outputへ出す。
残りrun予算はキャンペーン台帳で別途消費・確認する。

実装・地図審査・実走結果の証拠は本書へ追記する。現時点では動的な全地点成功を主張しない。
