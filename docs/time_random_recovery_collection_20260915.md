# 条件付き反復外乱の収集・検証

## 完了結果

2026-09-15に承認された収集から検証までを完了した。AWSIM実行先は `graneple@192.168.3.10`、データ検証・教師生成はnative WSLのworktree lock下で実施。目標5km/h、既存の停止監視、通常RViz、1周＋将来末尾＋正常停止を維持した。修正版の有限2run計画はsealed。追加run・モデル学習・モデルの走行評価は実施していない。

| run | 分割 | 周回時間 | 復帰確認 | 有効教師 | 両正常実走に対する外向き対象 |
| --- | --- | ---: | ---: | ---: | ---: |
| r64 | train | 279.08s | 3/3 | 273 | 7 |
| r65 | validation | 278.98s | 3/3 | 243 | 6 |
| 合計 | 独立2run | 各1周完走 | 6/6 | 516 | 13 |

修正版では投入6・復帰成功6・未復帰0・未投入0。1件は実観測のCamera/LiDAR/ego履歴と、0.1s間隔・未来0.1〜3.0sの実測XY **30×2** の組。562候補から516件を採用した。除外46件は、入力欠損5件、区間開始直後150msの余白13件、未来の教師制御の連続性不足28件。分類はこの順序の排他的集計で、個々の失格条件は重複し得る。

全6イベントの教師数は89/92/92件と61/93/89件。各イベント60件以上の事前ゲートを満たした。r65の最初のイベントには、未来制御の条件を満たさない候補29件（うち1件は入力欠損として集計）があり、採用を61件へ絞った。条件を緩めて補充していない。

解除後の横ずれ最大値は各イベント約10.3〜11.5cm、位置・向き・速度の1s維持による復帰確認は解除後約4.28〜5.36s。外向き対象は、両正常guideで横ずれ5〜25cm・向き2〜4度かつ両者が同符号となる採用anchor。モデル失敗場面の一部に対応する限定的な状態指標であり、516件が独立516ケースという意味ではない。

原本全hash・構造・SQLite、全候補の因果監査、イベント境界、教師shape/有限値/全点mask、全入力valid、LiDAR/ego/command/dtの有限値、参照範囲、生成済み全ファイルの再hashがPASS。各イベント1代表、計6anchorの全入力tensorはrawからの再生とキャッシュが完全一致した。図を目視し、6つの復帰曲線と実画像・未来教師を確認した。

結果は[収集index](evidence/time_random_recovery_20260915/collection_index.json)、[入力再生・教師再照合](evidence/time_random_recovery_20260915/post_collection_verification.json)、[採否内訳](evidence/time_random_recovery_20260915/selection_and_visual_details.json)、[復帰曲線](evidence/time_random_recovery_20260915/recovery_events.png)、[画像と未来教師](evidence/time_random_recovery_20260915/representative_inputs_and_teachers_v2.png)を参照。

## 保存先・環境の終了確認

- 原本: `/home/thistle/e2e_autonomous/raw/time_recovery_random_confirmed_20260915/{run_id}`。
- 教師・監査: `/home/thistle/e2e_autonomous/runs/time_recovery_random_confirmed_20260915`。`materialized/{run_id}`と`prepared/{train,validation}/{run_id}`へ生成した。
- 圧縮原本: 同runs配下 `random_pair01_20260915.tar.gz`、1,053,589,930 bytes。展開原本の通常ファイル合計2,406,735,253 bytes。SHA256 `ca269977a3aa54c857f534580cbec8c567e400c9ad64ae98e2c3df3ed5cbf9a8`。
- AWSIM側はWSLの全照合と削除直前再照合を通したr64/r65だけを移動済みmarkerへ置換。診断用r62原本・圧縮は保全した。
- 実行先の元checkout HEAD/dirty状態、歴史的114コンテナ・39composeを保全し、稼働コンテナ0・今回の余分なコンテナ0。専用runtime555ファイルと参照hashも再照合した。終了時空き容量は約19.14GB。

終了処理中の `NOMINAL_FIXED_SPEED_MISMATCH` はr64に6行、r65に5行。すべて保存済み正常停止より後、実速度0・目標速度0。走行結果のfaultは両runともnull、bag終了確認済み。採用教師へ混入していない。

ホスト終了照合の初回はDocker `ps` のMounts表示順の違いを変更と判定した。全差分が同じmountトークンの並び替えであることを確認し、個数も保つソート比較へ修正。他の項目は厳密比較を維持し、[再照合](evidence/time_random_recovery_20260915/host_final_v2.json)がPASS。初回結果も保全した。代表図は共通のXY軸範囲で見やすくしたv2を採用し、旧図・教師・splitも保全している。

## テスト・診断履歴

修正commit `404ea7cf51097c74cb0ba0a26753fa5b8d219666` はnative WSL全pytest **2464 passed / 4 skipped / 65 warnings、93.06s、exit0**。準備・555ファイルの専用runtime配置・公式Docker内のimport/有限3イベントの数学smokeもPASS。修正版の固定計画SHA256は `ef66b8a103a8aa31f66817d47cb02634a5f9f92dd31da2bbccbb56563ce48eb5`、overlay archiveは `3832f505dbff9315e91ee557110bc1bf665b494a6c04522f50821d72ec93813a`。gateは `/home/thistle/e2e_autonomous/runs/time_random_recovery_20260915_test_gate_404ea7c.json`。実走・WSL検証のpipelineもexit0。以下は過去の試行記録であり、消費済みroot/IDを再実行しない。

初回r62の診断用archiveをWSLへ転送し、61ファイル・1,195,532,811 bytesの全hash・構造・SQLiteを照合済み。archive SHA256は `14ad8f056baefb04f2a9fdad508a2350654f9ffd12f72cd77218bb57d82a2689`。AWSIM側原本も保全した。初回の収集不足は以下に記録し、以降の準備履歴にある「次はr62/r63」は当時の予定であり、現在は実行しない。

初回r62は279.04sで1周し正常停止・bag終了したが、反復収集ゲートは不成立。実測では解除後4.34sから復帰確認が97行あった。9秒台の0.205sの発行間隔で、過去に成立した1s確認まで消したのが原因。現在の安定区間を欠損でリセットしつつ、既に観測した復帰確認は保持する修正を行った。次の外乱には独立した新しい1s安定確認と3s余白が必要で、欠損をまたぐ教師も従来どおり除外する。閾値・速度・停止監視は変更していない。r62の投入1イベントは収集処理の診断資料とし、上記6イベント・516教師には含めない。

初回pipelineはr63を開始する前にexit1で終了し、r62を診断資料として保全した。修正確認は別root `time_recovery_random_confirmed_20260915`、新規r64(train)/r65(validation)、各最大3外乱・1周・最大2runの新しい有限計画とする。seed・split・外乱・監視・教師品質ゲートを保ち、初回IDを再実行しない。固定計画は`configs/time_path_p1/random_recovery_confirmed_20260915.json`。r62は修正後の学習・検証件数へ混入させない。

`c7e3c5aa6b5467689d4692522026f11973e2d0f8`で全pytest2461 passed / 4 skipped / 65 warnings、92.65s、exit0。native WSL準備も成功。固定計画SHA256は`ae3e5bdded5b4890d736d9e31cb9e0ff794213513a383b7ad7c3d408b60b078b`、overlay archiveは`976c7903835d2876e55fd6189af3f00930f497618f54bda8ea5521d879f30d04`。使用gateは`/home/thistle/e2e_autonomous/runs/time_random_recovery_20260915_test_gate_c7e3c5a.json`。次は専用root配置・公式イメージsmoke・r62/r63の実走。

専用root配置は555ファイルのmanifestで完了。公式Dockerイメージ内で変更3ファイルの構文・import・有限3イベントの数学smokeがPASS、exit0。実走の検証とは区別する。`tmp/time_random_recovery_20260915_deploy.log`とremote `official_image_smoke.json`へ保存した。次はr62/r63を順番に収集する。

ユーザーが2026-09-15に[提案方針](time_random_recovery_collection_proposal_20260914.md)による収集から検証までを承認した。未使用の固定位置r52〜r61は実行せず、新しい計画とrootで進める。最終目標は通常E2E完走。そのための追加教師を増やす作業であり、本タスクではモデル学習やモデルの完走性能を主張しない。

実装commit`9e80849`のnative WSL重点テストは58 passed / 1 warning、6.66s。運用追加`886e1ff`の全pytestは2460 passed / 4 skipped / 65 warnings、92.70s、exit0。ただしその後の実データ準備で、正常guideの先頭が60.051293691840584mで、開始65mに対する5m余白を満たさず停止した。走行はまだ未実施。空の準備出力とログを保全して、開始下限を66mへ移す。検証を弱めず、runtimeと準備で共通のguide範囲検証と端数境界の回帰テストを追加した。次は同修正の全pytest・再準備・専用rootでの実走確認。

`a192ee3`の全pytestは2460 passed / 1 failed / 4 skipped。新規テストが必要終端ちょうど132mを不足と誤判定する期待値になっていたため、132mは有効、131.999mは無効へ修正した。実装条件は変更していない。失敗ログは`runs/time_random_recovery_20260915_full_a192ee3.log`に保全。初回準備の空の出力は`{runs,raw}/time_recovery_random_20260915_failed_prepare_886e1ff`へ保全済みで、収録runは0。

## 変更の根拠・範囲

既存収集は1周に1回しか外乱を入れず、集計も最初の解除時刻を前提としていた。1周あたり複数回の有効教師を作るには、有限スケジューラと複数イベントの境界管理が必要。既存単発機能の既定値と教師の実測生成は保ち、追加形式を明示して分岐する。成功条件は「複数の独立イベント、有効な実観測入力と将来30点、再現可能な抽出」。フレーム数だけを性能改善と扱わない。

変更所有者は主agentのみ。ROSの座標系・topic・メッセージ・QoS・クロック・唯一の制御発行元を維持する。操舵はROS入力rad、速度はm/s、基準横ずれは左正m、姿勢差はrad。センサ期限、actuator clamp、停止領域監視、overspeedを変更しない。提案が後段監視や発行時確認に失敗したら状態を進めない。

## 修正版の実行条件

初回r62だけを実行し、r63は未使用。修正確認ではrun IDをr64/r65、remote/WSL root名を`time_recovery_random_confirmed_20260915`へ変更した。同じseed915062/915063と他の条件を維持した。以下の2runは収集・検証済みでsealed。

- 実行先 `graneple@192.168.3.10`。root `/home/graneple/e2e_autonomous/time_recovery_random_confirmed_20260915`。
- 最大2run、各1周＋将来末尾＋停止、各最大3イベント。runごとのseedとsplitを走行前に固定する。
- `codex-time-recovery-random-r64`: seed915062、train。`codex-time-recovery-random-r65`: seed915063、validation。同一runをframeやeventでsplitしない。
- 目標5km/h、`aligned_gain4_v1`。外乱±0.10rad、最大2s・plateau1.5s・release0.15s、復帰区間10s。既存横ずれ上限0.25m・向き上限4度を維持。
- 開始候補は正常guideの検証済み範囲内、コース進行66〜111m。候補の遅延0.5〜1.5sをseedで生成し、左右をshuffleする。有限3回の計画に左右双方を含める。
- 開始前に位置差5cm・向き差1度以内、速度1.15〜1.4m/sを1s維持。直前150ms以内の正常発行で停止領域余裕0.5m以上、nominal操舵に0.10radのheadroomがあることを要求。実際の外乱候補にも既存の停止領域監視と発行時確認を通す。
- 復帰は位置差5cm・向き差2度以内、速度1.15〜1.4m/sの1s連続維持を確認。復帰区間10sを終え、さらに将来3s＋候補遅延を待つ。未復帰なら反復を打ち切って記録し、通常PPと監視を維持する。強さ・長さの自動増加や回数予算の延長はしない。
- 1run最大1800sim秒、外側1980秒＋終了猶予20秒。同時走行1。開始時空き12GiB、実行中10GiB、bag2GiB/runという既存運用。

既存実装sourceを保全した専用コピーへ変更したruntimeファイルのみ適用し、実行manifest・commit・全ファイルhashを保存する。問題があれば該当runを停止・分類し、以前のrootはそのまま使用可能。元AWSIM checkoutのdirty状態と歴史的containerは保全する。停止時は今回の実行が所有するものだけ既存runnerで終了する。

## 収集後の検証

Camera・LiDAR・ego履歴は実際にずれた位置の観測。外乱ゼロ指令のROS発行時刻をイベントごとに記録する。これはAWSIM actuatorへ実際に適用された時刻とは区別する。

解除後150ms以降で、未来3sすべてが外乱ゼロの教師制御かつ実測位置で支持されるanchorを採用する。未復帰イベント、非教師区間、次外乱、停止、時刻欠損をまたぐ教師を除外する。過去の外乱中の観測は因果性・センサ監査に従って履歴として利用可能。採用anchorへevent IDを保存する。

原本をWSLへ転送後、全hash・ディレクトリ・SQLiteを照合する。ユーザーが既に承認した運用に従い、照合済みの該当AWSIM側原本のみ移動済み記録へ置換する。native WSL `/home/thistle/e2e_autonomous/{raw,runs}/time_recovery_random_confirmed_20260915`へ原本・圧縮・監査・教師を保持する。

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
  --plan configs/time_path_p1/random_recovery_confirmed_20260915.json \
  --test-gate /home/thistle/e2e_autonomous/runs/time_random_recovery_20260915_test_gate_404ea7c.json
```

新規remote rootへ固定計画、準備archive/manifest、test gate、運用script2本と既存`move_pair.py`を転送する。`setup`は既存sourceを照合・複製し、変更runtime3ファイルだけ適用して新manifestを生成する。

```bash
python3 /home/graneple/e2e_autonomous/time_recovery_random_confirmed_20260915/collect_time_random_recovery.py setup \
  --plan /home/graneple/e2e_autonomous/time_recovery_random_confirmed_20260915/collection_plan.json
```

Windowsで次のコマンドを一度実行する。各runの完走・停止・複数復帰の確認後に次runへ進み、2runの転送照合・既承認の整理・native WSL監査と教師生成を行う。失敗時は原本と結果を保全して分類し、無条件再実行しない。

```powershell
python -X utf8 -u tools/collect_time_random_recovery.py collect --plan configs/time_path_p1/random_recovery_confirmed_20260915.json
```

WSL監査を独立に実行する場合は同ツールの`audit`をworktree lock下で使う。既存出力があれば上書きを拒否する。本収集ゲートは各run2イベント以上、全投入イベント復帰確認、各イベント有効教師60件以上、両正常実走と一致する外向き状態が各run1件以上。今回すべて成立した。データ品質の判定であり、モデルの走行成績ではない。

追加の入力再生・図生成・ホスト照合は[evidenceの手順](evidence/time_random_recovery_20260915/README.md)に記載。小さなJSON/図/ログ/解析scriptだけをWindowsへ戻し、原本・配列・重みはGitへ追加しない。

## 適用範囲と次の判断

反復外乱からの実測教師収集は2runで成立した。コース進行約67/87/108m、目標5km/h、横ずれ約10cm、左向き外乱4・右向き2という範囲に限る。全周任意位置、大きな逸脱、異なる速度・環境への汎化は未評価。既存test4runと予約r48/r49は未使用。

次に既存データと統合する場合は、このrun単位splitとイベントIDを保持し、正常走行と復帰の比率・位置・左右の偏りを明記する。通常走行の完走・モデルの復帰改善は、再学習と未使用条件でのAWSIM比較によって別に確認する。本タスクでは収集した教師の品質までを確定した。
