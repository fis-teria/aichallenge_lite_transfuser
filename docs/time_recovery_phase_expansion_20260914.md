# コーナー進入・出口の復帰データ追加

## 現在の状態

pair1の左右2runを完走・WSL移動・監査・教師生成まで完了。185件の復帰教師を用意した。ユーザーから反復ランダム外乱方式の提案があり、次の収集方式を検討中。固定12run計画の残り10runは未実施。学習方法比較では既存データの再配分に効果があったが、外向き6件の3秒先精度は悪化した。既存復帰はコース進行88m付近・横ずれ約10〜12cmへ偏るため、今回は実際の観測状態の種類を増やす。元の最終目標は通常E2E完走であり、この収集の成否をモデルの完走性能とは扱わない。

実行の正本はこの文書と[固定計画](../configs/time_path_p1/recovery_phases_20260914.json)。完了済みの旧キャンペーンと評価予約は再利用しない。

- 固定計画SHA256: `b2d1ef68d39001d8677814f45703f247b836ef5846044700ffe3577711e21c27`。
- `2ba8685`のWSL全テストは2420 passed / 4 skipped、88.25秒。参照4種類の生成と実データのguide範囲チェック、旧source554ファイルの照合・専用root複製は成功。
- 最初のWindows側起動でLinuxパスがbackslashへ変わり、remote Pythonがファイルを開く前に失敗した。`tmp/time_recovery_phase_pair01_20260914.log`を保全。両台帳はattempts空、r50出力も未作成で、走行は消費していない。
- `b09b7ae`でSSH・SCP・状況確認へ渡すpathを明示的にPOSIX化した。PureWindowsPathを使う回帰テストを追加し、WSL全テストは2423 passed / 4 skipped / 65 warnings、86.92秒、exit0。収集source554ファイルは変更していない。
- 進入側pair1は`COLLECTION_PAIR_COMPLETE 1`、Windows側処理exit0。進行ログは`tmp/time_recovery_phase_pair01_run_20260914.log`。左右の入力・教師185件はnative WSLで全hash再照合済み。
- 収集中にユーザーから「一定時間ごとのランダム外乱と復帰区間の抽出」を提案された。[検討案](time_random_recovery_collection_proposal_20260914.md)を整理した。以後を条件付きランダム方式の少数run試験へ移すか固定計画を続けるか選択を待っている。pair2はまだ開始しない。全12run用の`finalize`は未実行。

## pair1の実測結果

| run | 事前split | 周回時間 | 有効復帰教師 | 両正常実走基準の外向き対象 | 最大横ずれ絶対値 | 解除後の復帰確認時刻 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| earlyleft-r50 | train | 278.99s | 92 | 2 | 10.38cm | 4.430s |
| earlyright-r51 | train | 279.02s | 93 | 3 | 10.75cm | 4.465s |

run IDの接頭辞は`codex-time-recovery-`。復帰確認時刻は、既存の位置・向き条件を1秒維持できた時点。最大横ずれは外乱解除後1.095s／1.280sに発生しており、解除直後だけ、または横ずれが単調減少する区間だけへ絞ると復帰初期の実観測を落とす。

185件は独立185走行ではなく、独立2run内のCamera anchor。うち厳しい「横ずれ5〜25cmかつ向き2〜4度が外向きで、正常2runの双方で一致」の対象は5件。追加のvalidationはまだ0run。収集回数と状態の多様性を分けて評価する必要がある。

全runで正常停止・bag終了・因果再生・センサ入力・30点の実測将来教師を確認した。生成後の16ファイルのhash再照合も一致。監査の詳細は[集計](evidence/time_recovery_phase_expansion_20260914/pair01_20260914_summary.json)、[準備ゲート](evidence/time_recovery_phase_expansion_20260914/pair01_20260914_prepared.json)、[終了後確認](evidence/time_recovery_phase_expansion_20260914/pair01_post_collection_inspection.json)。旧集計の`calibration_gate_pass`は3件以上を要求するためr50ではfalseだが、今回開始前に固定した本収集ゲートは1件以上であり、2runともPASS。事後の基準変更ではない。

監視ログに出た`NOMINAL_FIXED_SPEED_MISMATCH`は左右各6行で、すべて停止確認を保存した後の終了処理中。最初の該当行はr50で停止確認保存から0.125s後、r51で0.195s後。該当行の実速度・発行目標速度はともに0m/s、phaseはinvalid。解除後10秒の復帰区間には非教師追従行が0行で、これらの終了行は今回の教師へ入っていない。終了順序に伴う診断表示は記録したまま保全し、このタスクでは監視を変更していない。

原本2,364,540,283 bytesと圧縮1,043,394,650 bytesをWSLに保存。archive SHA256は`c411b14347348a66e959f32a240b2dd5d52d6f06110b1ed1b0683c5b46cb8e79`。全ファイル・ディレクトリ・SQLiteの[照合](evidence/time_recovery_phase_expansion_20260914/pair01_20260914_verified.json)後、対象2runだけを既承認の移動運用で整理した。.10の空きは21,168,037,888 bytes（約19.7GiB）。元checkoutのHEAD・dirty状態、既存container IDと停止状態は一致、稼働containerは0。専用rootのsource554ファイルも両rootで一致している。[ホスト確認](evidence/time_recovery_phase_expansion_20260914/pair01_post_collection_host.json)。

## 変更の必要性と範囲

変更対象は、既存収集ツールを2つの新規位置へ適用する設定・運用だけ。既存データの反復では観測した位置・姿勢の種類を増やせないため、コーナー内の開始位置を変える。モデル、教師の作り方、損失、制御器、監視閾値を変更する必要はない。失敗走行の全状態を今回だけで覆うとは仮定しない。

- 前半: コース進行76m。正常実走の姿勢変化は旋回への進入側。
- 後半: コース進行104m。正常実走で大きな旋回を終えた側。
- 各位置で左右両方向、2train＋1validationの計3run。全12runを開始前に8train／4validationへ固定する。
- 初回4runも事前割当済みのtrain。振幅・持続時間の再調整を目的とした校正runではない。モデルの結果を見てsplitを変更しない。
- 初回を含む各runで、1周・正常停止・bag正常終了、1秒維持を含む復帰確認、因果的に有効な教師60件以上、両正常実走を基準に外向き教師1件以上を要求する。既存本収集と同じ「目標0件なら反復を止める」方針。これは走行監視の緩和ではない。
- 原因が未知の失敗や上記不成立時には次pairを開始せず、証跡を保存して原因を分類する。同じ失敗run IDを再実行しない。

収集sourceは既に実行済みの`a1c9e5ad0a5c274826601338355b75d34b268186`を固定し、554ファイルを各開始前に照合する。新規参照JSONでは`steering_pulse.config.start_s_m`だけ変更する。現在のWindows/WSL学習コードをAWSIM教師へ一括配備しない。

実行先は`graneple@192.168.3.10`。新規rootは`/home/graneple/e2e_autonomous/time_recovery_phase_early_20260914`と`time_recovery_phase_late_20260914`。共通計画と集計ゲートは`time_recovery_phase_expansion_20260914`。元AWSIM checkoutのHEAD・既存dirty状態・歴史的に停止したcontainerを保全する。

## 維持する制御・教師条件

目標速度5km/h、`aligned_gain4_v1`、外乱ROS操舵入力±0.10rad、最大2秒・一定値区間1.5秒、解除0.15秒、復帰区間10秒。横ずれ上限0.25m、向き上限4度など全て既存値。車体座標の左正・前方正、角度rad、速度m/s、シミュレーション時刻と収録available時刻を維持する。

既存ノードの単一制御発行元、actuator clamp、状態・センサtimeout、停止領域、NaN等の監視を維持。異常時は既存supervisorが停止させる。シミュレータ以外の実機デバイスがあれば開始しない。通常RVizに教師基準経路・観測経路を表示する。今回E2Eモデルによる走行はしない。

1runは1周と将来教師用の末尾・停止を収録し、シミュレータ時間最大1800秒、外側1980秒＋終了猶予20秒。同時走行1、2runごとに梱包する。原本と圧縮版をnative WSLで全ファイルhash・構造・SQLite照合後、ユーザーが既に承認した移動運用に沿って該当するAWSIM側の原本だけを移動済み案内へ置換する。空き容量12GiBを開始条件、既存10GiBを実行中の下限とする。

学習用入力は実際にずれた位置のCamera・LiDAR・ego履歴、教師はそこからPPが復帰した実測将来30点（0.1秒間隔、3秒、XY m）。解除後150ms除外と、全未来が教師制御区間に入る条件は維持する。教師だけの位置ずらしや、欠損未来の外挿はしない。

## 実行方法

新しい運用コード・設定・テスト・この文書だけをWindowsでcommitし、同期する。native WSLでは必ずworktree lockを保持する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/collect_time_recovery_phases.py prepare \
  --plan configs/time_path_p1/recovery_phases_20260914.json
```

準備した参照archive・manifest・固定計画・運用script・既存`move_pair.py`を新規remote hubへコピーし、以下を一度実行する。`setup`は新規root作成と旧収集sourceのhash照合を行い、既存root上書きを拒否する。

```bash
python3 /home/graneple/e2e_autonomous/time_recovery_phase_expansion_20260914/collect_time_recovery_phases.py setup \
  --plan /home/graneple/e2e_autonomous/time_recovery_phase_expansion_20260914/collection_plan.json
```

Windows正本で各pairを順番に実行する。`collect`は2runの完走分類・梱包・WSL転送照合・対応原本の整理・WSL教師監査生成を行う。pair1〜6を使い、成功済みpairや失敗runの無条件再実行はしない。

```powershell
python -u tools/collect_time_recovery_phases.py collect --plan configs/time_path_p1/recovery_phases_20260914.json --pair 1
```

監査だけを独立に実行する場合は、native WSLで`audit --pair N`を同じ固定計画とlockの下で使う。全6pair後に一覧を確定する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/collect_time_recovery_phases.py finalize \
  --plan configs/time_path_p1/recovery_phases_20260914.json
```

native WSLの原本は`/home/thistle/e2e_autonomous/raw/time_recovery_phase_expansion_20260914`、監査・圧縮・教師・入力は`/home/thistle/e2e_autonomous/runs/time_recovery_phase_expansion_20260914`へ保存する。過去のtest4run、r48/r49には触れない。今回の範囲は追加収集と教師準備までで、再学習・モデル評価は未実施。
