# 外向き復帰教師の追加収集（2026-09-14）

追加収集を完了した。校正2本に続く本収集12本はすべて1周・正常停止・bag保存を完了し、
native WSLへ照合付きで移動した。train 729件、validation 186件の入力・教師を生成済み。
評価予約2本は原本の監査までとし、学習用には生成していない。以下に実施条件と結果を記録する。

前回の±0.08 rad校正では、正常実走2本の双方を基準にして横ずれ5〜25cm・外向き2〜4度を
満たす有効cameraアンカーは左右各1件だった。ユーザーの追加収集依頼に従い、まず既存実装内の
±0.10 rad・最大2秒・一定値区間1.5秒を左右各1本で校正する。
固定目標5km/h、150msの解除後除外、状態・時間・操舵・停止領域監視を維持する。

校正IDは`codex-time-recovery-pulseleft-r36`、`codex-time-recovery-pulseright-r37`。
AWSIMは`graneple@192.168.3.10`の新しい`time_recovery_pulse_a010_20260914`内で実行し、
既存の校正キャンペーンは封印したまま保全する。今回の校正の上限は2回、同時実行は1回。
1周と将来軌道の末尾・停止を記録し、最大1800秒のシミュレータ時間制限を維持する。
通常のRVizに経路を表示する。

校正の採用条件は、各runで上記目標状態の有効アンカーが両正常基準に対して3件以上残り、
正常復帰・1周・停止を満たすこと。成立後に本収集のrun単位のtrain/validation/評価予約を
固定する。校正runを最終評価へ割り当てない。未成立の場合は無変更の大量反復へ進まない。

実行sourceは既に全体pytestが通過した`a1c9e5ad0a5c274826601338355b75d34b268186`を固定し、
554ファイルのSHA、速度整合性、参照経路、空き容量を各実行前に照合する。
新しい集計処理はWindowsでコミットしてからnative WSLに同期し、worktree lock下で検証する。

```bash
# AWSIM実行先、新しい専用root内（左右を一つずつ実行する）
python3 /home/graneple/e2e_autonomous/time_recovery_pulse_a010_20260914/start_a010_trial.py \
  --run-id codex-time-recovery-pulseleft-r36

# native WSLのrepo root
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  docs/evidence/time_recovery_expansion_20260914/summarize_expansion.py --smoke-only

# bag監査、causal replay、状態監査は前回の文書と同じCLIで、新しいraw/outputを指定する。
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  docs/evidence/time_recovery_expansion_20260914/summarize_expansion.py \
  --analysis /home/thistle/e2e_autonomous/runs/time_recovery_expansion_20260914 \
  --raw /home/thistle/e2e_autonomous/raw/time_recovery_expansion_20260914 \
  --alternate-guide /home/thistle/e2e_autonomous/runs/time_steering_pulse_20260914/alternate_nominal_guide_r31.json \
  --runs r36 r37 \
  --output /home/thistle/e2e_autonomous/runs/time_recovery_expansion_20260914/calibration_summary.json
```

原本は2本単位で梱包し、native WSLでSHA・サイズ・構造・SQLiteを照合した後に、AWSIM側の
対応原本だけを移動済み案内へ置換する。大きなデータをGitに含めない。

## 校正結果と本収集の固定計画

| 校正run | 1周 | 有効アンカー | 両正常基準で外向き目標を満たす数 | 復帰の1秒維持確認 |
|---|---:|---:|---:|---:|
| r36・左 | 278.97s | 94 | 3 | 解除後4.145s |
| r37・右 | 279.06s | 92 | 3 | 解除後4.550s |

両runで校正の採用条件を通過した。原本2,378,280,298 bytesと圧縮1,049,140,225 bytesを
native WSLで照合し、AWSIM側の原本を移動済み案内へ置換した。
校正の186件は本収集のtrain/validation/評価予約へ混ぜない。
[校正集計](evidence/time_recovery_expansion_20260914/calibration_summary.json)、
[保全確認](evidence/time_recovery_expansion_20260914/pair01_20260914_verified.json)。

![校正前後のカメラ観測状態](evidence/time_recovery_expansion_20260914/calibration_comparison.png)

緑は表示範囲内の目標状態、丸は採用cameraアンカー、×は最初の150msによる除外。
右側は横ずれと向きの符号を反転して比較しやすく表示した。

本収集は新しい`time_recovery_outward_20260914`内で最大12回、左右各6本とする。
`r38`〜`r45`の8本をtrain、`r46`〜`r47`をvalidation、`r48`〜`r49`を評価予約とし、
最初の本収集前に[固定計画](evidence/time_recovery_expansion_20260914/production_plan.json)を保存した。
計画ファイルのLFでのSHA256は`04d757dde75e6b42062a6966d5f9d05bdad1649c7e4d51980f47ded5f68bb462`。
校正summary・計画・source・参照経路のhashを各run前に検証する。
本収集で目標アンカーが0件となった場合や復帰・完走に失敗した場合は追加反復を止める。
収集後の件数に応じてsplitを付け替えない。

同じコースの同じコーナーにおける小ずれ復帰を反復収集するため、別コース・大きな横ずれへの
一般化を示すデータではない。独立runであっても状態の類似性と時間相関は残る。
モデルによる走行、学習、モデル性能評価はこの収集に含めない。

train/validationの採用runでは既存の`materialize_recovery_run`で全教師を再生成し、
`_prepare_run`で全参照画像・LiDARを読み出して入力を準備する。
アンカーID・全入力の有効性・未来30点のmaskを照合し、評価予約は学習用生成から除外する。
単独runの準備成果物であり、既存コーパスとの統合manifestと再学習は別途実施する。

追加した集計のsmokeと前回r34/r35の実データ回帰は通過した。
native WSLの全体pytestは`bb2a2da`で2321 passed / 4 skipped（85.89s）。
制御sourceは校正・本収集とも`a1c9e5a`を固定する。

## 本収集の転送・生成コマンド

各pairが正常停止し、`move_pair.pack`で閉じた2本を梱包してから実行する。
pair番号2〜7は固定計画の順序に対応する。既存出力を上書きする再実行は拒否する。

```powershell
# Windowsの正本repo。WSLの照合完了後、対応するAWSIM側の原本だけを移動済み案内へ置換する。
python -X utf8 docs/evidence/time_recovery_expansion_20260914/ship_closed_pair.py --pair 2
```

```bash
# native WSLのrepo root。全候補の監査とtrain/validationの教師・入力生成を行う。
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  docs/evidence/time_recovery_expansion_20260914/audit_new_pair.py --pair 2
```

生成先は`/home/thistle/e2e_autonomous/runs/time_recovery_expansion_20260914`内の
`materialized/<run_id>`と`prepared/<split>/<run_id>`。
原本は`/home/thistle/e2e_autonomous/raw/time_recovery_expansion_20260914/<run_id>`に保持する。

全pairの監査後に、以下で最終一覧を生成する。全12本の割り当て、原本の照合記録、
生成ファイルのhash・アンカーID・教師shape/maskと評価予約の除外を再確認する。
成果物は同じanalysisディレクトリの`collection_index.json`。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  docs/evidence/time_recovery_expansion_20260914/finalize_collection.py
```

## 完了結果

最終一覧の状態は`COMPLETE_VERIFIED`。収集前のrun割り当てを保持したまま、
全12本の復帰・完走・停止、原本の照合記録、生成した全ファイルのSHA256、
アンカーID、入力の有効性、教師`[N, 30, 2]`と全点のmaskを確認した。
原本と圧縮ファイルの全内容は転送時に照合済みで、最終一覧作成時には大型archiveのhashを
再計算していない。生成ファイルのhashは最終一覧作成時にも再計算している。

| 用途 | 独立run数 | 有効cameraアンカー数 | 両正常基準で外向き目標を満たす数 | 教師・入力の生成 |
|---|---:|---:|---:|---|
| train | 8 | 729 | 23 | 全件生成・照合済み |
| validation | 2 | 186 | 6 | 全件生成・照合済み |
| 評価予約 | 2 | 184 | 5 | 原本監査のみ、未生成 |

「有効アンカー」は解除後150msを除外し、causal replay・センサ入力・未来3秒の実測位置が
成立した復帰区間のサンプル数。1周全体の画像枚数や独立した復帰事象数とは異なる。
「外向き目標」は横ずれ5〜25cm・同符号の外向き2〜4度を両正常実走基準で満たす状態。
train/validationの915件すべてがこの厳しい外向き条件に該当するわけではなく、該当は29件。
校正r36/r37の186件・外向き6件は、この表のどの用途にも含めない。

| run | 用途 | 有効アンカー | 外向き目標 | 1周時間 |
|---|---|---:|---:|---:|
| r38・左 | train | 93 | 3 | 278.98s |
| r39・右 | train | 92 | 4 | 279.05s |
| r40・左 | train | 91 | 2 | 278.97s |
| r41・右 | train | 87 | 3 | 279.07s |
| r42・左 | train | 89 | 3 | 278.97s |
| r43・右 | train | 92 | 3 | 279.06s |
| r44・左 | train | 92 | 2 | 278.99s |
| r45・右 | train | 93 | 3 | 279.06s |
| r46・左 | validation | 93 | 3 | 278.97s |
| r47・右 | validation | 93 | 3 | 279.05s |
| r48・左 | 評価予約 | 93 | 2 | 278.99s |
| r49・右 | 評価予約 | 91 | 3 | 279.05s |

教師は0.1秒間隔・3秒先までの30点、現在車体座標のXY単位m。
教師だけを人工的に横へずらさず、AWSIMで実際にずれた車両の画像・LiDAR・状態と、
そこから教師Pure Pursuitが復帰した実測将来軌道を組にした。
本収集の横ずれピークは約10〜12cmであり、目標条件の上限25cmまでを埋めたわけではない。

今回の14本（校正を含む）の原本は合計16,633,558,489 bytes、保管圧縮archiveは
7,340,494,307 bytes。両方をnative WSLに保管し、各pairの全ファイル・構造・SQLite照合後に、
AWSIM側の対応原本と転送用archiveだけを移動済み案内へ置換した。
本収集キャンペーンは12本で封印済み。終了確認時の実行ホスト空きは約21.2GiBで、
実行中containerと本収集プロセスはなかった。
元のAWSIM checkoutのHEADと取得可能なGit状態は開始時と同じ。
以前から読み取れないログディレクトリはこのGit状態確認の保証範囲に含めない。

証拠:
[最終一覧](evidence/time_recovery_expansion_20260914/collection_index.json)、
[最後のpairの監査](evidence/time_recovery_expansion_20260914/pair07_20260914_summary.json)、
[評価予約の生成除外](evidence/time_recovery_expansion_20260914/pair07_20260914_prepared.json)、
[実行ホスト終了状態](evidence/time_recovery_expansion_20260914/final_host_state.json)。

最終一覧の作成処理は、最後のpairが未検証の状態では完成一覧を出力せず失敗することを
native WSLのsmokeで確認し、全pairの監査後に実データで成功した。
前述の2321 passed / 4 skipped以降、学習・制御ライブラリの変更はない。

今後は既存データとの統合manifestを作り、run単位splitと校正・評価予約の除外を維持して、
追加データなし／ありを同一条件で再学習・比較する。
今回はデータの追加・監査・準備までであり、モデルの再学習・予測性能評価・E2E走行は未実施。
同じコーナーの近接した状態を繰り返したため、この件数だけで復帰能力の十分性は判断しない。
