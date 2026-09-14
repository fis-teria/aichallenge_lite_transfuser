# 条件付き反復外乱の収集・検証

## 現在の状態

ユーザーが2026-09-15に[提案方針](time_random_recovery_collection_proposal_20260914.md)による収集から検証までを承認した。未使用の固定位置r52〜r61は実行せず、新しい計画とrootで進める。最終目標は通常E2E完走。そのための追加教師を増やす作業であり、本タスクではモデル学習やモデルの完走性能を主張しない。

実装commit`9e80849`のnative WSL重点テストは58 passed / 1 warning、6.66s。運用追加`886e1ff`の全pytestは2460 passed / 4 skipped / 65 warnings、92.70s、exit0。ただしその後の実データ準備で、正常guideの先頭が60.051293691840584mで、開始65mに対する5m余白を満たさず停止した。走行はまだ未実施。空の準備出力とログを保全して、開始下限を66mへ移す。検証を弱めず、runtimeと準備で共通のguide範囲検証と端数境界の回帰テストを追加した。次は同修正の全pytest・再準備・専用rootでの実走確認。

`a192ee3`の全pytestは2460 passed / 1 failed / 4 skipped。新規テストが必要終端ちょうど132mを不足と誤判定する期待値になっていたため、132mは有効、131.999mは無効へ修正した。実装条件は変更していない。失敗ログは`runs/time_random_recovery_20260915_full_a192ee3.log`に保全。初回準備の空の出力は`{runs,raw}/time_recovery_random_20260915_failed_prepare_886e1ff`へ保全済みで、収録runは0。

## 変更の根拠・範囲

既存収集は1周に1回しか外乱を入れず、集計も最初の解除時刻を前提としていた。1周あたり複数回の有効教師を作るには、有限スケジューラと複数イベントの境界管理が必要。既存単発機能の既定値と教師の実測生成は保ち、追加形式を明示して分岐する。成功条件は「複数の独立イベント、有効な実観測入力と将来30点、再現可能な抽出」。フレーム数だけを性能改善と扱わない。

変更所有者は主agentのみ。ROSの座標系・topic・メッセージ・QoS・クロック・唯一の制御発行元を維持する。操舵はROS入力rad、速度はm/s、基準横ずれは左正m、姿勢差はrad。センサ期限、actuator clamp、停止領域監視、overspeedを変更しない。提案が後段監視や発行時確認に失敗したら状態を進めない。

## 固定した初回計画

- 実行先 `graneple@192.168.3.10`。root `/home/graneple/e2e_autonomous/time_recovery_random_20260915`。
- 最大2run、各1周＋将来末尾＋停止、各最大3イベント。runごとのseedとsplitを走行前に固定する。
- `codex-time-recovery-random-r62`: seed915062、train。`codex-time-recovery-random-r63`: seed915063、validation。同一runをframeやeventでsplitしない。
- 目標5km/h、`aligned_gain4_v1`。外乱±0.10rad、最大2s・plateau1.5s・release0.15s、復帰区間10s。既存横ずれ上限0.25m・向き上限4度を維持。
- 開始候補は正常guideの検証済み範囲内、コース進行66〜111m。候補の遅延0.5〜1.5sをseedで生成し、左右をshuffleする。有限3回の計画に左右双方を含める。
- 開始前に位置差5cm・向き差1度以内、速度1.15〜1.4m/sを1s維持。直前150ms以内の正常発行で停止領域余裕0.5m以上、nominal操舵に0.10radのheadroomがあることを要求。実際の外乱候補にも既存の停止領域監視と発行時確認を通す。
- 復帰は位置差5cm・向き差2度以内、速度1.15〜1.4m/sの1s連続維持を確認。復帰区間10sを終え、さらに将来3s＋候補遅延を待つ。未復帰なら反復を打ち切って記録し、通常PPと監視を維持する。強さ・長さの自動増加や回数予算の延長はしない。
- 1run最大1800sim秒、外側1980秒＋終了猶予20秒。同時走行1。開始時空き12GiB、実行中10GiB、bag2GiB/runという既存運用。

既存実装sourceを保全した専用コピーへ変更したruntimeファイルのみ適用し、実行manifest・commit・全ファイルhashを保存する。問題があれば該当runを停止・分類し、以前のrootはそのまま使用可能。元AWSIM checkoutのdirty状態と歴史的containerは保全する。停止時は今回の実行が所有するものだけ既存runnerで終了する。

## 収集後の検証

Camera・LiDAR・ego履歴は実際にずれた位置の観測。外乱ゼロ指令のROS発行時刻をイベントごとに記録する。これはAWSIM actuatorへ実際に適用された時刻とは区別する。

解除後150ms以降で、未来3sすべてが外乱ゼロの教師制御かつ実測位置で支持されるanchorを採用する。未復帰イベント、非教師区間、次外乱、停止、時刻欠損をまたぐ教師を除外する。過去の外乱中の観測は因果性・センサ監査に従って履歴として利用可能。採用anchorへevent IDを保存する。

原本をWSLへ転送後、全hash・ディレクトリ・SQLiteを照合する。ユーザーが既に承認した運用に従い、照合済みの該当AWSIM側原本のみ移動済み記録へ置換する。native WSL `/home/thistle/e2e_autonomous/{raw,runs}/time_recovery_random_20260915`へ原本・圧縮・監査・教師を保持する。

bag構造監査、全候補の因果再生（最大1024/run）、イベントごとの実測ずれ・復帰確認、両正常実走に対する状態分布、教師30×2と全入力の生成・再照合、代表イベントの軌跡を確認する。成功・失敗・未投入の全イベント数を報告する。既存test4runとr48/r49は開かない。

## 実行コマンド

```powershell
tools/sync_to_wsl.ps1 -CheckOnly
tools/sync_to_wsl.ps1
```

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

全pytestのexit0・実行commit・ログhashを持つ`test_gate.json`を生成した後、native WSLで準備する。実行済みrootやrun IDを再使用しない。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/collect_time_random_recovery.py prepare \
  --plan configs/time_path_p1/random_recovery_20260915.json \
  --test-gate /home/thistle/e2e_autonomous/runs/time_random_recovery_20260915_test_gate.json
```

新規remote rootへ固定計画、準備archive/manifest、test gate、運用script2本と既存`move_pair.py`を転送する。`setup`は既存sourceを照合・複製し、変更runtime3ファイルだけ適用して新manifestを生成する。

```bash
python3 /home/graneple/e2e_autonomous/time_recovery_random_20260915/collect_time_random_recovery.py setup \
  --plan /home/graneple/e2e_autonomous/time_recovery_random_20260915/collection_plan.json
```

Windowsで次のコマンドを一度実行する。各runの完走・停止・複数復帰の確認後に次runへ進み、2runの転送照合・既承認の整理・native WSL監査と教師生成を行う。失敗時は原本と結果を保全して分類し、無条件再実行しない。

```powershell
python -X utf8 -u tools/collect_time_random_recovery.py collect --plan configs/time_path_p1/random_recovery_20260915.json
```

WSL監査を独立に実行する場合は同ツールの`audit`をworktree lock下で使う。既存出力があれば上書きを拒否する。本収集ゲートは各run2イベント以上、全投入イベント復帰確認、各イベント有効教師60件以上、両正常実走と一致する外向き状態が各run1件以上。これはモデルの走行成績ではない。現在、新形式の実走・収集結果は未確定。
