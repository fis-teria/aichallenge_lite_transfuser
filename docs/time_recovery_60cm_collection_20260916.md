# 60cm復帰データ収集（1周3回）

実収集・WSL転送・教師生成・run単位分割を完了した。右側2周、左側2周で、
各周3イベントすべての復帰と教師検証が成立した。調整走行を含む全8周は `COMPLETE_LAP`、
凍結済み `result.json` のfaultは全件null。途中で不成立となったイベントは教師から除外した。

採用は **19復帰イベント・1,780 Cameraアンカー**。アンカーは約10Hzの連続フレームであり、
1,780個の独立した復帰事例ではない。実測55～65cm帯は111アンカー。
3イベント成立の4周だけでは1,126アンカー・同帯65アンカーで、残りは調整周回の成立イベントである。

| 分割 | run数 | 採用復帰イベント | 採用アンカー | 実測55～65cm帯 |
|---|---:|---:|---:|---:|
| 学習用 | 3 | 8 | 749 | 48 |
| 検証用 | 5 | 11 | 1,031 | 63 |
| 合計 | 8 | 19 | 1,780 | 111 |

| 実行順・run末尾 | 分割 | 復帰成立 / 計画 | 採用アンカー | 55～65cm帯 | 不成立箇所 |
|---|---|---:|---:|---:|---|
| 1: left-r01 | 検証 | 1 / 3 | 94 | 8 | S00準備未達、後続未実施 |
| 2: right-r01 | 検証 | 2 / 3 | 186 | 8 | P02準備未達 |
| 3: right-r02 | 学習 | 3 / 3 | 281 | 11 | なし |
| 4: left-r02 | 学習 | 2 / 3 | 188 | 15 | P02復帰確認失敗 |
| 5: right-r03 | 検証 | 3 / 3 | 282 | 10 | なし |
| 6: left-r03 | 検証 | 2 / 3 | 186 | 15 | P04復帰確認失敗 |
| 7: left-r04 | 学習 | 3 / 3 | 280 | 22 | なし |
| 8: left-r05 | 検証 | 3 / 3 | 283 | 22 | なし |

run共通接頭辞は `codex-time-recovery-60cm-d60-g03-`。
成立した右側の地点はP00/P03/P02（release 60/164/247m）、左側はP00/P05/P03（60/108/164m）。
いずれもsettling 4m。復帰完了は通常PPへ実際に切り替えた後、横ずれ10cm以内・向き差2度以内・
所定速度範囲内を連続1秒満たすことを10秒窓で確認した。

全採用入力は因果的なCamera・LiDAR・ego履歴の再現検査に合格し、教師は未来3秒の
`[N, 30, 2]` のXY座標（m）。全点finite、全mask有効。採用イベントと教師の未来3秒を含む
制御publication間隔は最大約85ms、150ms超なし。これは送出時刻の間隔であり、
アクチュエータ到達遅延の測定ではない。

raw合計は **11,014,603,154 bytes（約11.01GB）**。全ファイルSHA・ディレクトリ構造・SQLite検査を
通過してnative WSLへ保存した。検証済みの今回のrawと一時tarを実行先から除去し、
空きは15,078,858,752 bytes（約14.04GiB）。全container停止、実行先の元repoのHEAD/Git状態、
配置556ファイルと過去40cm収集の共通94ファイルが変わっていないことを確認した。

証拠は [集計](evidence/time_recovery_60cm_20260916/collection_index.json)、
[保存ファイルのSHA一覧](evidence/time_recovery_60cm_20260916/manifest.json)、
[実測復帰と採用アンカーの図](evidence/time_recovery_60cm_20260916/accepted_recovery_anchors.png)、
[通常RViz・右側](evidence/time_recovery_60cm_20260916/rviz_right-r02_event3.png)、
[通常RViz・左側](evidence/time_recovery_60cm_20260916/rviz_left-r04_event3.png)。
rawは `/home/thistle/e2e_autonomous/raw/time_recovery_60cm_20260916`、
生成物・因果再現ログは `/home/thistle/e2e_autonomous/runs/time_recovery_60cm_20260916` の
`materialized/`、`prepared/train/`、`prepared/validation/` とrun別レポートに保持した。

**未解決点は停止地点S00の左60cm教師が採れていないこと。** また、右側P00/P02の
55～65cmアンカーは1イベントあたり1～3件と少ない。同一地点を両splitに含むため、
この件数から未知地点への一般化や学習済みモデルの復帰成功率を結論できない。
今回の成立結果は教師PPの走行収集であり、既存データとの統合・再学習・モデル評価は次の作業となる。

2026-09-16のユーザー指定により、AWSIMで左右60cmの横ずれからの復帰を収集する。
1周の計画イベント数は3で固定。目標速度5km/h、30分上限、既存の停止・時刻・姿勢監視を維持する。
今回の範囲は収集、native WSLへの検証付き転送、教師生成、run単位のtrain/validation分割。
モデルの再学習、既存データとの結合、学習モデルによる復帰試験は含めない。

実行先は `graneple@192.168.3.10`、専用rootは
`/home/graneple/e2e_autonomous/time_recovery_60cm_20260916`。
過去の20cm・40cm収集rootを保全する。最大8試行（調整走行を含む）、左右それぞれ2回の
3イベント成立を目安とし、イベント数の増加は行わない。

実行コードは遅延対策済み `adfc444818a26bae021d463cca5312b5d37a9f9a`。
Windows/WSLの開始時HEADは `e68e4ea2482e3da6e3098fdd8c10c7e872d278a8` で、
src/tools/configs/testsに差分がないことを確認した。
既存のnative WSL全体テスト2673 passed / 4 skippedのログSHAと、公式ROS接続試験を再照合した。
556個の配置ファイルは40cm実行時と同一SHA。既存の接続・遅延検証はコードの証拠として再利用し、
60cmの実走成立とは区別する。収集用の新規ランタイム変更はない。

開始前にGPU（RTX 4060 Laptop、driver 595.91.07）、実行中containerなし、
Windows/WSLのcleanなGit状態、通常RVizが利用するX表示を確認した。
収集開始時空き12GiB以上、実走中10GiB以上、bag上限2GiBを維持する。
保存容量は最大2run、実際には1runごとにWSLへ転送し、全ファイルSHA・構造・SQLite検証後に
今回の転送済みrawと一時転送tarだけを実行先から除去する。

教師は実際にずれた位置でのCamera・LiDAR・ego履歴と、通常PPへの切替後の実測未来3秒・30点。
人工的な準備経路を教師には使わない。目標60cmと、因果的に選択したCameraアンカーの
実測55～65cm帯を分けて集計する。横ずれの基準は照合した通常走行ラインであり、道路中心の真値ではない。
同一runをtrainとvalidationの両方に入れない。ただし同じ地点は両splitに含まれるため、
未知地点への一般化を検証する分割ではない。

初回左側はP00（release 60m）で復帰し、S00（118m）の準備が `TARGET_NOT_REACHED` で終了した。
1周は完走し、停止監視発報なし。S00は準備中に最大69.30cmまでずれ、release+1mの期限時点で
66.55cm・向き差-0.03562radとなり、目標±5cm・向き差±2度の成立が間に合わなかった。
準備中の制御publication間隔は最大90ms、150ms超0件。初回P00の94アンカー（55～65cm帯8件）
だけを検証用に採用し、S00と未実施の3地点目は教師に含めない。

S00のsettlingを2mから4mへ変更した118m候補、およびrelease 120/122/124/126m候補は
既存の半径1.4m地図検査に不合格だった。監視・目標許容値・期限は変更せず、S00の60cm収集を保留した。
右側P03の従来位置160mも60cmでは地図検査に不合格。156～186mの15候補を調べ、
地図検査と通常走行の開始速度条件を満たす最も近い164mを選んだ。
左側の代替もP00/P03/P02（release 60/164/243m、settling 4/4/2m）で地図検査を通過した。
地図検査は静的な準備経路と候補復帰経路に対するもので、実車両の動的復帰や全周安全の証明ではない。

右側の初回はP00/P03で復帰し、P02（243m）で準備未達となった。
release+1m時点では横ずれ-60.23cmまで収まったが、向き差0.03865rad（約2.2度）が残った。
準備中publication間隔は最大70ms、150ms超0件。P00/P03の186アンカーだけを検証用へ採用した。
P02をrelease 247m・settling 4mへ変更すると、右側r02/r03の両方で3復帰と完走が成立した。
各runの281/282アンカー、55～65cm帯11/10件を学習用/検証用へ分けた。

同じ247m条件で左側r02も試したが、P02の10秒後に横ずれ11.19cmが残り、
10cm以内・向き差2度以内・速度範囲内を連続1秒満たす完了条件に不合格となった。
復帰中publication間隔は最大70ms、150ms超0件。失敗P02は教師0件とし、
先に完了したP00/P03の188アンカー（55～65cm帯15件）だけを学習用へ採用した。
準備・復帰の期限や許容値は変更せず、左側3地点目の候補をP04（295m・settling 4m）へ移した。
通常走行の準備区間での向きの変化幅は約31.6度で、247m候補の約87.4度より小さい。
地図検査と開始速度条件を通過済み。向きの変化の比較だけで動的な復帰成立とは扱わない。

P04（295m）の左側r03も10秒終了付近で10.45cmの横ずれが残り、連続1秒の安定確認に
不合格だった。P04は教師0件とし、P00/P03の186アンカー（55～65cm帯15件）を検証用に採用した。
次に成立済みP00/P03を固定し、中間と後半の15候補を地図・開始速度・通常走行の向き変化で比較した。
地図と速度条件に合格した候補のうち、準備区間の向き変化＋復帰区間の向き変化×2が最小の
P05（108m）を選択した。これは候補選択の指標であり、安全性や動的復帰の証明ではない。
P05の準備/復帰区間の向き変化の総量は約30.65/21.25度。最終候補の左側は
P00/P05/P03（release 60/108/164m、settlingはすべて4m）とした。
この条件の左側r04/r05は3イベントすべて成立し、280/283アンカー、55～65cm帯は各22件を採用した。

実行途中の並列化に関する質問には、1台収集中のGPU/CPU/RAMと時間進行を測定し、
[並列化候補の調査](time_recovery_parallel_capacity_20260916.md)へ整理した。
別AWSIM環境はROS domainを分け、同じAWSIM内の複数車は車別topic/TFを分ける方針。
この収集では二重起動やdomain固定値の変更は行っていない。

Windowsのoperator入口（大きなデータはignoredなWSL出力先に保存）:

```powershell
python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py check
python tmp/time_recovery_60cm_20260916/prepare60.py
python tmp/time_recovery_60cm_20260916/setup60.py
python tmp/time_recovery_60cm_20260916/start60.py --run RUN_ID
python tmp/time_recovery_60cm_20260916/watch60.py RUN_ID
python tmp/time_recovery_60cm_20260916/wait_ship.py --run RUN_ID --pair PAIR_ID --prefix TRANSFER_PREFIX
python tmp/time_recovery_60cm_20260916/status60.py
python tmp/time_recovery_60cm_20260916/audit_run.py --run RUN_ID --split train
```

これらは実行履歴の入口であり、完了済みrunへ再実行しない。`watch60.py` と
`wait_ship.py` は開始後に別のPowerShellプロセスで実行した。新しい収集では一意なrun/rootと
有限試行数を設定する。補助スクリプトの保存先は evidence内の `operators/`、既存operatorへの
依存は `operator_dependencies/tmp/`。再現時はそれぞれ元のtmp階層へ配置する必要があり、
evidence内から直接実行する配置ではない。センサデータ・参照元の通常走行・教師配列はWSLに保持する。

参照経路生成・因果的な入力/教師検証は `tools/with_wsl_training_lock.sh` を通し、
`/home/thistle/e2e_autonomous/e2e_lite_transfuser` のnative WSL環境で行う。
operatorは既存の `run_time_recovery_awsim.py --speed-policy aligned_gain4_v1 --separate-cpus` を起動する。
通常RVizには通常経路・準備経路・実測軌跡と外乱地点マーカーを表示する。
PREPマーカーは準備開始位置であり、60cm到達の証拠はCameraアンカーの実測値で確認する。

転送・教師生成後の集計は同じWSL lock内で `finalize60.py`、`plot60.py` の順に実行し、
Windowsから `finish_host.py` で全runの転送済み状態と元の実行環境を確認する。
`collect_evidence.py` で小さな証拠ファイルだけを保存し、Windowsでコミットした後に
`sync_native_transport.py check` / `sync_native_transport.py sync` で公式同期処理へ渡す。
同期後の読み取り検証は以下で再実行できる。

```powershell
wsl.exe -d Ubuntu-22.04-Recovered -u thistle -- bash -lc 'cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python docs/evidence/time_recovery_60cm_20260916/operators/verify_final.py'
```
