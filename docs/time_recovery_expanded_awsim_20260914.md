# 復帰データ統合モデルのAWSIM比較

ユーザー依頼により、`graneple@192.168.3.10`で追加学習前後の復帰性能を確認する。
Windowsを編集正本とし、Linuxでの試験・保存記録の解析はnative WSLを使用する。
AWSIM実行は指定された`.10`で行い、既存のdirty checkoutと過去の実験を保全する。

## 固定する条件と実行順序

- 追加前checkpoint SHA256: `c2fdb6fd1daf525524fe44d3f793d84b482760aa1f9f1fa84d5315222ab9fbe0`。
- 追加後checkpoint SHA256: `7ec445b6ab955e76d0d71efbd8f2f1dd3066ebe365516df38df8cf18adb56f44`。
- 生の時間軌道30点、固定目標5km/h、`stopping_preview_segment_v1`、
  `awsim_understeer_v1`、既存の操舵応答補償・停止領域監視を共通にする。
- まず通常のE2E走行を各1回。1周、監視停止、進捗停止、既存600s上限のいずれかまで記録する。
- 次に左右各1回ずつ両モデルを比較する。教師PPで同じコーナーへ進み、既存の有限操舵パルスで
  小さな横ずれ・向きずれを作った後、E2Eへ一度だけ制御を渡す。引継ぎ後はE2Eが走行する。
  教師が準備した区間とモデル自身の走行区間は明示的に分け、教師区間をE2E完走と数えない。
- 初回は通常2回＋左右復帰4回を上限とし、走行失敗の無条件再試行や監視条件の緩和はしない。
  準備・接続失敗は原因を確認して修正し、走行成績に混ぜない。
- 走行経路は通常のAutoware RVizに`/visualization/time_path/raw_path`を表示する。
  参照経路・評価用自己位置はモデル入力へ加えない。

## 記録と判定

走行前にsource/install/checkpoint/sceneのhash、GPU、ROS publisherの所有権、RVizの購読、
空き容量を確認する。隔離ROSの実モデルPath・異常制動smokeを実行し、simulation起動前に閉じる。
閉じた走行記録をWSLへ移してhashを確認し、同じ制御計算の再生と復帰区間の横ずれ・向き・
復帰時間・停止理由を集計する。制御が成立したことだけで復帰成功とは判定しない。
新しい実行記録を学習splitへ自動追加せず、元のtestと評価予約は未使用のまま保持する。

実行前の環境はGPU正常、動作中containerなし、空き約22GB。
既存checkoutのHEADは`4af395eee10f928c7fc7225760adfa04c4c07ff4`で、既存の編集を保持する。
新しい専用deploymentを使い、今回のowned process/containerだけを終了する。

実行コマンド・実測結果・未解決事項は以下に追記する。

## 復帰試験の引継ぎ実装

`run_time_path_awsim_trial.py --recovery-side left|right`を追加する。
実行先の`references/{side}.json/.csv`は今回収集に用いた同じ有限パルス参照を使用する。
`recovery_assets.json`で教師PP install・launch・参照のhashを固定し、実際にロードした
PPの上限速度5km/h・速度ゲイン4を公式start前に検証する。

最終command publisherは常に`/time_path_controller`の1つ。
教師PP入力も既存0.5rad・0.8rad/s・停止領域監視を通す。公開したパルス0の境界から
150ms以降を観測した最初のモデル出力でE2Eへ切り替える。品質による選別をしない。
引継ぎをPP計算の前にラッチするため、モデル軌道が不適切でも教師へ戻さない。
0境界から1s以内に引き継げなければ停止し、パルスや教師運転を延長しない。
参照投影は引継ぎ後には診断専用で、範囲外などの診断失敗がE2E制御を止めることもない。
生のモデル軌道は準備中も通常RVizへ表示する。

```bash
# native WSL: Windowsでcommit/sync後
tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q

# .10: 検証済みの新規deploymentごとに、固有run IDで実行
timeout --signal=TERM --kill-after=10s 710s python3 \
  "$deployment/$source/tools/run_time_path_awsim_trial.py" \
  --deployment "$deployment" --run-id "$run_id" --display :1 \
  --config configs/control/time_path_expanded_5kmh_20260914.json --recovery-side left
```

## 通常走行の暫定観測

`source_50d6b22`・それぞれのepoch3で各1回を実施。
旧モデル`codex-time-expanded-base-normal01`は最大0.107m/s、追加後
`codex-time-expanded-candidate-normal01`は正加速度commandなし。
どちらも発進時の`STEERING_FEASIBLE_LOOKAHEAD_MISSING`で進捗停止し、完走は0/2。
この条件ではコーナー復帰性能を測れていないため、別途予定した教師準備後の引継ぎ試験を行う。
両方で通常`rviz2`のraw Path購読を確認した。無条件に再試行はしない。

## 保存記録の評価方法

Windows commit `39f3aa1`をnative WSLに同期し、`pytest -q --disable-warnings`は
2,349 passed / 4 skipped（87.94s）。同sourceを両方の復帰試験deploymentに配置し、
source/install一致と実モデルによる隔離ROS smokeが両方PASS。

復帰の判定は、教師PPの実測nominal guideに対する横ずれ5cm以内・向きずれ2度以内を
速度0.5m/s以上で1秒間維持すること。250ms以上の観測欠損で連続性を切る。
さらに10秒区間を観測でき、区間末尾もこの条件内にある場合に`RECOVERED_10S`とする。
最初から許容内、停止して位置だけ戻った、途中で記録が切れた場合は成功に数えない。
これは道路端までの安全距離や一般的な成功率を表す指標ではない。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/evaluate_time_recovery_takeover.py \
  --run ../runs/time_recovery_expanded_awsim_20260914/raw/codex-time-expanded-base-left01 \
  --run ../runs/time_recovery_expanded_awsim_20260914/raw/codex-time-expanded-candidate-left01 \
  --run ../runs/time_recovery_expanded_awsim_20260914/raw/codex-time-expanded-base-right01 \
  --run ../runs/time_recovery_expanded_awsim_20260914/raw/codex-time-expanded-candidate-right01 \
  --reference-root ../runs/time_recovery_expanded_awsim_20260914/references \
  --output ../runs/time_recovery_expanded_awsim_20260914/evaluation
```

引継ぎ後の記録だけで既存PP・操舵応答・マッピングを再計算し、拒否を含めて照合する。
最初の停止領域拒否は実際のscan・発行操舵・実測運動を用いて再評価する。
rawモデル軌道・同じguideに対する初回予測の向きも診断し、教師区間をモデルの成績へ含めない。
