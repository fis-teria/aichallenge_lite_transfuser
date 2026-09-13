# 復帰教師の追加収集 — 2周ごとのWSL移動

## 現在の実行計画

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
python3 move_pair.py pack --pair 1 \
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
