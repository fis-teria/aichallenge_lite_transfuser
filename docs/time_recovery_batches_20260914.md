# 復帰教師の追加収集 — 2周ごとのWSL移動

## 完了結果（2026-09-14）

予定した追加6条件を各1周収集し、全6本が正常停止・bag closeまで完了した。
2本ごとにWSLで全件照合・教師再現を行い、AWSIM側の原本を移動済み案内へ置換した。
全364ファイル・リンクを照合済み。bagは8.36GiB、設定/ログを含む原本は8.67GiB、
圧縮バックアップ3.83GiBをWSLに保持する。AWSIM側の空きは20.92GiB、
WSL内の空きは695.68GiB。WSLの実体は `F:\WSL\Ubuntu-22.04-Recovered` にある。

| 条件 | run | hold中央値（基準折線から） | 復帰後5mの最大横ずれ | 事前基準 |
|---|---|---:|---:|---|
| 直線・左40cm | r21 | +44.60cm | 5.91cm | 通過 |
| 直線・右40cm | r22 | -45.49cm | 7.02cm | 通過 |
| 右カーブ・左20cm | r23 | +35.81cm | 6.35cm | 通過 |
| 右カーブ・右20cm | r24 | -8.53cm | 9.20cm | 保留: ずれ量と誤差低減が不足 |
| 右カーブ・左40cm | r25 | +57.07cm | 7.55cm | 通過 |
| 右カーブ・右40cm | r26 | -31.05cm | 11.89cm | 保留: 復帰後10cmを超過 |

主候補4本と補助2本の明示的な一覧は
`docs/evidence/time_recovery_batches_20260914/collection_index.json` に保存した。
右カーブの補助データにも、通常走行から20cm指定で約22cm、40cm指定で約44cmの軌跡変化を確認。
基準折線からの横ずれと通常走行との差は別の指標である。後付けの補助比較で事前判定は変更していない。

全320候補の入力履歴/未来3秒を監査し、319候補が有効（主候補218、補助101）。
残る1候補はcurrent sensor欠損で除外した。全6runで画像/LiDARから各3件、計18件のtensor再現が成功。
画像24,249件、LiDAR 51,134件を確認し、不正データ・capture時刻逆行は0。
目標速度は5km/h、実測移動速度中央値は各run約3.52km/h。記録した走行中判断時間の最大は98.12ms。
同じrun内の候補は重複する時間窓で、319回の独立した復帰試行ではない。

今回の8試行/16GiB予算に対して6試行/8.36GiBで終了し、追加枠2試行は使用しなかった。
AWSIM側の収集processと起動containerは0。既存114停止containerと、元repositoryの
HEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`・既存変更155件（porcelain normal）は維持した。
実走source `0484dc369f66282e05faa61dc98c80d687a9c6c4`、車体asset、制御・安全監視条件は全6本で同一。

これは不足していたずれ量・カーブ条件を補う初期収集であり、学習量の十分性やモデル性能は未判定。
学習用materializeとsplitは未実施。補助2本を主教師へ採用する前に、曲線の基準線・
投影・評価条件の扱いを整理する必要がある。raw yaw異常の履歴除外と因果的入力選択は継続が必要。

## 実施した収集計画

ユーザーは、2周ずつ収集し、検証後にWSLへ移して容量を戻す運用を承認した。
既存の有効な条件は同じ直線の左右20cmで、正常完走2本と部分復帰2本。
今回の最初の不足は横ずれ量と場所の偏り。新しい6条件を初期補強の目標とし、
同じ条件の無目的な周回は追加しない。学習用データ全体の十分性は後のモデル評価で判断する。

| 組 | 候補条件 | 終了後の判断 |
|---|---|---|
| 1 | 直線の左40cm・右40cm、各1周 | 実際のずれと復帰、原本、教師再現を検証してWSLへ移動 |
| 2 | 右カーブからの左20cm・右20cm、各1周 | 同じ検証を行い、曲線条件の実測成立を確認 |
| 3 | 同じ右カーブからの左40cm・右40cm、各1周 | 小さいずれの結果と通過余裕を確認後に実施 |

今回の実行上限は診断失敗を含む8試行、各1周・2GiB、合計16GiBの記録予算。
基本は上記6試行で、追加枠2本は原因が分かった独立の修正確認にだけ使用する。
同じ最初の物理的失敗を2回繰り返した条件は再試行せず、原因と不成立区分を記録する。
各試行30分sim / 31分wall / 33分outer、全体の起動可能期限は開始後4時間。
2試行を超えてAWSIM側に原bagを蓄積しない。最低空き10GiBを維持し、
転送中に次の走行は始めず、WSLの全ファイル一致を確認してから元データを移動済み案内へ置換する。

目標速度5km/h、自車1台、NPCなし、公式PP、通常RVizの3本のPath表示を継続する。
camera/LiDAR/IMU/pose/velocity/steering/名目・最終command/clock/TFを元時刻で記録する。
JudgeLogの1周を確認して4秒追加記録後に正常制動し、3秒の停止確認とbag closeを必要とする。
学習・split・materialize・モデル推論は今回の作業範囲に含めない。

## 事前に確認した根拠

`.10`の空きは約21.02GiB、起動中containerと学習processは0。
旧20記録はWSLへ移動済みで、AWSIM側は移動済み案内だけを保持している。
元repositoryの既存変更は保持し、専用source/installと参照CSVだけで実行する。

既存生成器で地図を検査し、直線40cmの左右とも従来のcenter clearance 1.4mを満たした。
右カーブ候補も左右20/40cmで同じ条件を満たした。これは地図上の候補確認で、走行成立ではない。
直線の範囲は基準s=5.968〜21.968m、右カーブは275.533〜291.533m。
各区間はapproach4m / hold6m / recovery6m、復帰後5mも検査する。
右カーブの最大基準曲率は0.10358/m。候補選択の最大曲率は0.12/mとし、
収集場所の種類を変える。車体・監視の数式や閾値は変更しない。

旧r19/r20の当該右カーブの通常走行では、復帰後に相当する5mの最大横誤差は約7.1cm。
一方、他の左カーブ候補には通常走行だけでも復帰後10cmを超える場所があり、
今回の条件比較には採用しない。旧r19/r20のbagはWSLで保持する。

## 採用判定

- 参照の符号側への実測hold中央値が指定offsetの半分以上（20cmなら10cm、40cmなら20cm）。
- recovery後5mの最大絶対offsetが10cm以下、holdからの絶対誤差低減が5cm以上。
- 1周、正常停止、bag close、全原本の転送一致、単一epoch。
- 復帰区間の全候補を上限256件まで監査し、入力履歴と未来3秒30点を確認。
  各run代表3件は元画像・LiDARから実tensorを組み立てる。
- nominalの目標値と実測速度を分けて記録する。指示offsetと実測offsetも分ける。
- 途中終了、意図した符号・大きさが成立しないものは完走採用例へ数えず、診断/部分候補として保管。

基準コース上の横誤差は全車体の無接触証明ではない。既存の前方scan監視・操舵上限・
capture/receipt/skew・100ms判断期限・正常制動を保持する。
意図的に寄せるapproach/holdを復帰の正解に含めず、教師は実測未来位置を使う。
raw yaw異常履歴の除外は既存probeで維持し、学習converterへの直接投入はしない。

## 必要な変更と検証

既存CLIは20cm直線に固定されているため、新しい参照を再現可能に生成できない。
変更は `tools/generate_time_recovery_collection.py` のoffset/geometry/曲率選択引数に限定する。
既定値・位相長・map clearance・基準CSV・速度・wrap末尾・実走controllerは保持する。
既定20cmのCSVバイト一致と、40cm/曲線の出力shape・位相・速度・地図適合をWSLでsmoke確認する。
不正offset/NaN曲率が拒否されることも確認し、full pytestを1回実施する。
切戻しは現在のrunを正常停止し、前の参照directoryとsourceを再選択することで行う。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/generate_time_recovery_collection.py \
  --inputs /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs \
  --output /home/thistle/e2e_autonomous/runs/time_recovery_batches_20260914/references_straight040 \
  --offset-m 0.4
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/generate_time_recovery_collection.py \
  --inputs /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs \
  --output /home/thistle/e2e_autonomous/runs/time_recovery_batches_20260914/references_curve020 \
  --offset-m 0.2 --geometry right_curve \
  --maximum-base-curvature-inv-m 0.12 --preferred-base-curvature-inv-m 0.07
```

生成物と全試行台帳はWSLの `runs/time_recovery_batches_20260914/`、原本は
`raw/time_recovery_batches_20260914/` へ保存する。2周ごとの転送・削除は既存の照合手順を使い、
対象runの全ファイルとdir構造・symlinkを確認してから、元位置には案内だけを残す。

## 実行記録

参照生成のsourceは `8333fae192799f85dbcd9cde921b1e368f11920b`。
WSL smokeは既定20cmのバイト一致、全8参照のshape・速度・位相と、接続部分を含む
0.1m刻みの地図clearance、不正値の拒否を確認した。
同じsourceの全pytestは **2222 passed, 4 skipped, 63 warnings (76.95s)**。
skipsは未導入のOSQP、jsonschema 2件、optional公式LiDAR小袋による。
実走sourceは検証済み `0484dc369f66282e05faa61dc98c80d687a9c6c4` を維持する。

追加の収集台帳と転送は `docs/evidence/time_recovery_batches_20260914/` の補助手順を使う。
`start_owned_run.py` は1本だけ起動し、8試行/16GiB/2本滞留/期限と参照SHAを確認する。
`move_pair.py` は明示された2本に限定し、pack、WSL verify、限定mount内cleanupを別々に実行する。
`summarize_pair.py` は上記の採用条件を数値で判定する。途中終了も削除せずWSLへ保管する。

```bash
# AWSIM host: 1本完了後に次を起動。run IDは再利用しない。
python3 /home/graneple/e2e_autonomous/time_recovery_collection_20260913/start_owned_run_20260914.py \
  --run-id codex-time-recovery-left040-r21 --profile straight040 --side left
# AWSIM host: 明示した2本がclose済みの時だけ圧縮。
python3 /home/graneple/e2e_autonomous/time_recovery_collection_20260913/move_pair_20260914.py pack --pair 1 \
  --runs codex-time-recovery-left040-r21 codex-time-recovery-right040-r22
# packのarchive/snapshot/shippingをWSLへ転送してから、記録されたsnapshot SHAを指定。
tools/with_wsl_training_lock.sh .venv/bin/python \
  docs/evidence/time_recovery_batches_20260914/move_pair.py verify --pair 1 \
  --runs codex-time-recovery-left040-r21 codex-time-recovery-right040-r22 \
  --snapshot-sha256 "$SNAPSHOT_SHA256"
```

verify後に既存 `audit_time_recovery_collection.py` と `causal_replay_probe.py` を各runに実行し、
`summarize_pair.py --pair 1 --runs RUN1 RUN2` で実測成立と全入力候補/3秒未来を確認する。
これらのPython実行は全てWSLのworktree lock内で行う。
cleanupは検証receiptと採用区分を含む案内を準備した後、同じsnapshot SHAとrun IDで実行する。
専用rootのみmount、network none/read-only rootfs/cap-drop ALL/DAC_OVERRIDEのみの一時containerで
元ファイル全件を再照合してから移動済み案内へ置換し、転送用archiveだけ削除する。
完了receiptをWSLにも保管し、台帳を `WSL_MOVED` に更新してから次の組へ進む。

### 第1組: 直線40cm — 収集・検証・移動完了

| run | 実測hold中央値 | 復帰後5mの最大横ずれ | 有効な3秒未来候補 | 周回 | 採用 |
|---|---:|---:|---:|---:|---|
| left040-r21 | +44.60cm | 5.91cm | 58 | 373.79s / 1周 | 可 |
| right040-r22 | -45.49cm | 7.02cm | 57 | 373.79s / 1周 | 可 |

目標5km/h、実測移動速度の中央値はそれぞれ3.522/3.520km/h。
各runは単一epochで、全候補115件が入力履歴と30点の実測未来を満たした。
画像/LiDARからのtensor再現は各3件成功。raw yaw異常は各run2メッセージあり、
既存の異常履歴除外を維持した。復帰候補へは混入していない。

2本のbag合計2,985,860,674B、設定/ログ等を含む全122ファイル・リンクの内容は
3,096,439,634B。転送archiveは1,372,071,438Bで、全ファイル・構造・SQLiteをWSLで照合した。
原本の全件再照合後にAWSIM側を移動済み案内へ置換し、native全件を再度検証した。
AWSIM側の空きは20.90GiB。移行先は `raw/time_recovery_batches_20260914/`。

終了後のheartbeatに `NOMINAL_FIXED_SPEED_MISMATCH` が残ったが、両runの正常停止時の
`result.last_control` はfaultなし・速度0・3秒停止確認済み。
r21では正常停止sim425.869990481sの後、clock停止後425.979990478sの時刻で不一致が発生した。
停止後の終了処理と走行中の異常を区別し、採用判定は固定済みresultと元時刻の有効位相で行う。

転送補助は内部ログsymlinkに対応するため `8360a65703850a5bd0493f717a17eabe9cc10950` で修正した。
旧bagの全60 manifest項目をWSLで再照合し、外部参照・重複run IDの拒否もsmoke確認済み。
記録器のmanifestはリンク先内容のSHA、移動用snapshotはリンク自体と実体の両方を保持する。

判定値は `docs/evidence/time_recovery_batches_20260914/pair01_20260914_summary.json`、
全件照合と削除後確認は同prefixの `verified.json` / `postcheck.json`、実測図は
`pair01_20260914_measured_recovery.png` に保存した。

### 第2組: 右カーブ20cm — 収集・検証・移動完了

| run | 実測hold中央値 | 復帰後5mの最大横ずれ | 有効な3秒未来候補 | 周回 | 採用 |
|---|---:|---:|---:|---:|---|
| left020-r23 | +35.81cm | 6.35cm | 52/52 | 373.94s / 1周 | 可 |
| right020-r24 | -8.53cm | 9.20cm | 50/51 | 373.59s / 1周 | 診断用 |

右20cmは、指示方向へのずれが10cm未満で、復帰後の最大誤差に対する5cm以上の低減も不成立。
通常走行の左寄りの横誤差と相殺される観測結果であり、成功例として採用しない。
1候補は `CURRENT_SENSOR_MISSING` で除外した。教師の30点自体は51件で成立していたが、
入力を満たす50件と区別する。両runとも単一epoch、代表3件のtensor再現は成功した。

実測の移動速度中央値は左3.520/右3.523km/h、目標は両方5km/h。
2本のbag合計3,000,888,898B、全121ファイル・リンクは3,111,497,072B。
1,372,443,026Bのarchiveを転送し、第1組と同じ全件一致・SQLite・削除後native再確認を通した。
AWSIM側を移動済み案内へ置換後、空きは20.99GiB。

実測control poseの対象区間（approach手前2m〜recovery後5m）を地図上で調べたところ、
左は半径2.0m、右は半径1.6mの円内が全サンプルで空きセルだった。
右の半径1.8mは不成立。前方scan監視の最小ray marginは左1.031m/右0.760mで、走行faultはなし。
40cmの参照自体は事前に接続部を含めて半径1.4mを検証済み。
これらを根拠に予定の40cm各1周へ進める。全車体・全時刻の無接触証明ではない。

判定と原本照合の証拠は第1組と同じprefix形式で `pair02_20260914_*` に保存した。
地図と前方監視は `pair02_20260914_measured_margin.json`、比較対象の旧通常走行は
`curve_baseline_margin_20260914.json`。同じ不成立条件の無目的な追加収集は行わない。

補助診断として、旧r19/r20の同一右カーブ通常走行を、共通の基準sで補間して比較した。
実測holdの通常走行からの変化は左+21.94cm/右-22.04cm、復帰後の通常走行との差は
両側とも最大約3.2cm。右20cmにも意図した軌跡変化の情報がある。
上表の-8.53cmは基準折線に対する絶対位置であり、通常走行に対する変化量ではない。
両者を混同してデータ破損・車両が寄らなかったと判断しない。
基準折線の頂点付近では、別のposeでもprojectionのsが同じになる場合を観測した。
比較では同一capture poseの重複だけを除き、通常走行側はsの単調増加を確認した。
約5mの復帰後窓のうち双方で記録された共通範囲のみ使用し、外挿していない。
これは収集後の補助診断であり、事前の採用基準やprimary区分は変更していない。
共通する追従誤差と投影の影響はこの比較だけでは個別に分離できない。
証拠は `curve020_nominal_comparison.json`、再現手順は以下。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  docs/evidence/time_recovery_batches_20260914/compare_curves_to_nominal.py --pair 2
```

### 第3組: 右カーブ40cm — 収集・検証・移動完了

| run | 実測hold中央値 | 復帰後5mの最大横ずれ | 有効な3秒未来候補 | 周回 | 採用 |
|---|---:|---:|---:|---:|---|
| left040-r25 | +57.07cm | 7.55cm | 51/51 | 374.17s / 1周 | 可 |
| right040-r26 | -31.05cm | 11.89cm | 51/51 | 373.46s / 1周 | 補助用 |

右40cmは、復帰後10cm以内という事前条件だけを満たさなかった。
通常走行との補助比較では、holdの軌跡変化が左+43.87cm/右-44.33cm、
復帰後の通常走行との差は最大6.36cm/6.46cmだった。
右20cmと同様に、基準線・通常走行・追加offsetの関係を残して補助区分で保存した。
比較の再現は `compare_curves_to_nominal.py --pair 3`。

2本とも単一epoch、全候補の因果入力/30点未来と代表3件のtensor再現を確認した。
raw yaw異常は左2/右5メッセージあり、復帰候補の入力には混入していなかった。
地図上の実測位置は左半径2.0m、右半径1.6mの円内が全サンプルで空きセル。
前方scan監視の最小ray marginは対象区間の手前2mも含め左1.094m/右0.665m。
全車体・全時刻の無接触証明は行っていない。

bag合計2,991,730,242B、全121ファイル・リンク3,102,301,579B、archive1,370,600,133B。
前2組と同じ全件一致、SQLite、入力/未来再現、原本削除後のnative全件再照合を通した。
結果と図は `pair03_20260914_*`、補助比較は `curve040_nominal_comparison.json`。
最終台帳は `campaign_20260914.json`、host停止・容量確認は `campaign_host_final_20260914.json`。
台帳は `COMPLETE`。通常RVizの基準・収集用・実測Pathは保存画像 `pair01_normal_rviz.png` でも確認した。
画像中の既存Time/V4表示名・RaceTrajectoryの数値は保持された表示設定であり、今回のモデル実行や目標速度の証拠には使わない。
