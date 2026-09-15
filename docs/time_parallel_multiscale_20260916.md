# AWSIM 2環境と12・20・40・60cm教師の再学習

ユーザー指定: AWSIM本体・シーン・車両・センサファイルは変更しない。
実行先は `graneple@192.168.3.10`、学習・教師検証はnative WSL。

並列環境は既存CLI `--ros2-base-domain` で車両domainを1/2に設定する。
各環境に専用Docker network namespaceを持たせ、既存のdomain 0管理通信も隔離する。
通常の `make dev` / 公式開始処理を使い、外側のcompose override、収集ノード、
保存先、CPU割当だけを変更する。AWSIMの実行バイナリや設定の差し替えは行わない。
2環境の初期試験は左右60cmを各1周・各3イベントとし、無制限の連続収集は開始しない。
既存のセンサ・計算・操舵・停止監視は維持する。通常RVizは環境別の新規ウィンドウを識別する。

再学習は、既存の通常走行＋復帰cacheへ、検証済みの4収集群を追記する。
「12cm」は操舵外乱収集の通称であり、全アンカーが正確に12cmずれている意味ではない。
全既存splitを維持し、failed run/eventを除外する。追加教師は未来3秒・30点の実測XY。
rawを再ハッシュし、採用された全入力履歴を原bagから再生成してprepared配列と比較する。

初期重みは従来と同じcommand-off epoch10、3 epochs、batch32、float32、seed42、lr3e-5。
従来の通常36,726枠と復帰8,920枠/epochを維持し、復帰枠の25%を既存の外向き教師と
新規イベントの採用区間先頭1秒へ配分する。残りの枠でも全unique復帰アンカーを提示する。
補助損失は既存のPP操舵相当・遠方横位置のgeometry設定を維持する。
best選択は従来6runの3秒誤差、追加validationは選択後診断のみ。封印testは読まない。
比較対象は従来の `balanced_geometry`。追加データと提示配分の変更を伴う1 seedの比較であり、
データ量だけの効果やAWSIM完走率を推定する実験ではない。
並列試験でこれから収集するデータは、この学習の途中には混ぜない。

Windowsでコミット後、公式sync処理を通して同一コミットをWSLへ同期する。
以下はnative WSL repoから、各コマンドを `tools/with_wsl_training_lock.sh` で囲んで実行する。

```bash
env PYTHONPATH=src .venv/bin/python -m pytest -q
env PYTHONPATH=src .venv/bin/python -u tools/train_time_multiscale_recovery.py audit \
  --plan configs/data/recovery_multiscale_sources_20260916.json --root ..
# auditのresolved_plan.jsonをWindowsのconfigs/time_path_p1へ保存・commit・再同期してから:
env PYTHONPATH=src .venv/bin/python -u tools/train_time_multiscale_recovery.py prepare \
  --plan configs/time_path_p1/recovery_multiscale_20260916.json --root ..
timeout --signal=TERM --kill-after=20s 7200s env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_multiscale_recovery.py train \
  --plan configs/time_path_p1/recovery_multiscale_20260916.json --root ..
env PYTHONPATH=src .venv/bin/python -u tools/train_time_multiscale_recovery.py compare \
  --plan configs/time_path_p1/recovery_multiscale_20260916.json --root ..
```

## 入力・教師の検証結果

追加31run・73イベント・6,660アンカーについて、raw再ハッシュ、採用入力履歴の全再生成、
既存prepared配列との完全一致を確認した。既存のrun単位splitを維持した。

| 追加収集群 | train | validation |
|---|---:|---:|
| 約12cmの操舵外乱 | 1,441 | 558 |
| 20cm（遅延対策の確認走行を含む） | 1,282 | 565 |
| 40cm | 564 | 470 |
| 60cm | 749 | 1,031 |
| 追加合計 | 4,036 | 2,624 |
| 既存と統合後 | 42,172 | 15,399 |

復帰のunique trainは5,446。全uniqueを提示し、復帰枠8,920のうち2,230枠を
473アンカー（既存外向き35＋新規イベント先頭1秒438）へ配分する。
通常走行36,726枠と従来6runの選択用validation 12,346は従来通り。
実測教師から求めたPP操舵と補助損失用の操舵計算は、全5,446点で最大差
`2.044091901298728e-08 rad`。初期重みの選択用validation指標も従来と完全一致した。

追加cache: `/home/thistle/e2e_autonomous/datasets/cache/time_recovery_multiscale_20260916`。
manifest SHA256: `f2b463b73f1d1cb7993b151754ca16488fcbd5748887375ee71b30d56316efc9`。
WSLの全回帰テスト: **2,696 passed, 4 skipped**。4件は既存環境の任意依存・fixture不足。

## AWSIMを変更しない2環境試験

実行root: `/home/graneple/e2e_autonomous/time_recovery_parallel_20260916`。
収集コードcommit: `5e4cea7f030994c2ee4f58fb6fd5af4b170bb1e5`。
AWSIMディレクトリの全1,089ファイル・664,111,503 bytesを開始前後にSHA256比較し、
全件一致した。元のAIチャレンジrepoのHEAD・Git statusも不変。

| 環境 | 車両domain | collector CPU | simulator / Autoware CPU | 修正版の結果 |
|---|---:|---|---|---|
| 左60cm | 1 | 0-3 | 8-9,12-15 | 1周完走、3/3復帰、正常停止・bag終了 |
| 右60cm | 2 | 4-7 | 10-11,16-19 | 1周完走、3/3復帰、正常停止・bag終了 |

CPU割当は両環境・収集側とシミュレータ側で重複しない。各環境にunprivilegedな
`--network none`のnamespace保持containerを作り、simulator、Autoware、collector、
公式開始コマンドをその環境の `--network container:<holder>` へ接続した。
ドメイン0の無害な検査topicでは互いのメッセージを受信しないことを確認した。
実走中も各環境のnetwork namespaceが異なり、同一環境内では一致した。
各domainのclock・Camera・LiDAR・車速publisherはそれぞれ1つ、制御subscriberも
その環境の車両で、通常RVizのウィンドウIDも環境ごとに異なる。

初回のdomain 2起動では、収集側がノード名を `awsim_d2` と誤って推定したため、
開始前監視で停止した。実グラフと `MultiDomainROS2Manager` の実装では
ノード名はDDS domainではなく車両indexで決まり、1台構成では双方 `awsim_d1`。
この対応を収集側だけで修正し、回帰テストを追加した。
初回の左は1周・3復帰完了、右は未走行の記録として保全。
修正版2走行を含む試行予算は計4回で終了した。

同時走行を含む110測定点のうち、両車走行中・faultなしは97点。
その区間でGPU平均99.87%、シミュレーション時間/実時間は左1.00018、右0.99997。
全測定中のVRAM最大2,804MiB、利用可能RAM最小4.662GiB。
走行処理の並列化は成立したが、初期化・転送・教師の採否を含む全工程の2倍速を
示す値ではない。GPUはほぼ使い切っており、3環境の余力は確認していない。

実行した外側launcherのコマンド（保存済みroot・既存run IDは上書き禁止）:

```bash
export PYTHONPATH=/home/graneple/e2e_autonomous/time_recovery_parallel_20260916/source/src
timeout --signal=TERM --kill-after=20s 1980s python3 /home/graneple/e2e_autonomous/time_recovery_parallel_20260916/source/tools/run_time_recovery_awsim.py \
  --campaign-root /home/graneple/e2e_autonomous/time_recovery_parallel_20260916 \
  --run-id codex-time-recovery-parallel-d2-right-r02 --side right \
  --speed-policy aligned_gain4_v1 --separate-cpus --ros-domain-id 2 \
  --parallel-plan /home/graneple/e2e_autonomous/time_recovery_parallel_20260916/parallel_plan.json
# 先行環境のdrive_authorized.jsonとrviz_window.jsonを確認後、別プロセスで:
timeout --signal=TERM --kill-after=20s 1980s python3 /home/graneple/e2e_autonomous/time_recovery_parallel_20260916/source/tools/run_time_recovery_awsim.py \
  --campaign-root /home/graneple/e2e_autonomous/time_recovery_parallel_20260916 \
  --run-id codex-time-recovery-parallel-d1-left-r02 --side left \
  --speed-policy aligned_gain4_v1 --separate-cpus --ros-domain-id 1 \
  --parallel-plan /home/graneple/e2e_autonomous/time_recovery_parallel_20260916/parallel_plan.json
```

launcherは専用namespace保持containerが作成済みで、planの所有ラベル・domain・CPU・
接続先が一致することを要求する。環境ごとの出力、DDS profile、RViz設定を外側から
マウントする。AWSIMのバイナリ・設定・シーンは差し替えない。
再実行では新しいroot/run IDと有限のplanned_runsを作り、同じ所有・空き容量確認を行う。

## 再学習の結果

WSLで3 epochs、4,281更新、136,938アンカー提示を正常完了した。
初期重みは従来と同一。最良はepoch 3で、保存重みの再読み込み後の予測も完全一致。
学習・各epoch検証・再読み込み確認は2,755.88秒（約45.93分）。
開始前のcache検証・初期重み再評価を含むtrainコマンド全体は2,972.14秒（約49.54分）。

重み: `/home/thistle/e2e_autonomous/runs/time_recovery_multiscale_20260916/training/best.pt`。
SHA256: `685f8b8f9936ab7e272be12616982374fd8a8d61fbe4a304b25475dc2a206ba8`。
WindowsやGitへ重みはコピーしていない。

従来6runの選択用3秒先XY誤差（各run同重み）は、旧モデル0.0486613mに対し、
新モデルはepoch 1で0.0575691m、epoch 2で0.0534532m、epoch 3で0.0523710m。
この評価集合では旧モデルより約7.62%悪化している。群別の比較結果は後述。
各epochの不正予測数は0。封印testは読んでいない。

## 同じvalidationでの旧・新モデル比較

同一の15,399アンカーを両モデルへ入力。旧・新とも保存済みの選択用validation予測と
完全一致した。以下は各runを同じ重みで平均した3秒先のユークリッド位置誤差。
入力が不適格、または教師が支持していない点は既存のmaskで除外する。
新規validationはbest選択には使用していない。

| 評価群 | run / アンカー | 旧XY誤差 cm | 新XY誤差 cm | 変化 | 旧→新 PP操舵誤差 rad |
|---|---:|---:|---:|---:|---:|
| 通常走行 | 4 / 12,237 | 5.519 | 5.771 | +4.56% | 0.017416 → 0.017355 |
| 従来の復帰 | 5 / 538 | 3.352 | 3.741 | +11.59% | 0.003987 → 0.003701 |
| 新規約12cm | 2 / 558 | 8.617 | 7.817 | -9.28% | 0.007733 → 0.007311 |
| 新規20cm | 3 / 565 | 7.947 | 3.830 | -51.81% | 0.007802 → 0.002920 |
| 新規40cm | 3 / 470 | 10.308 | 5.141 | -50.13% | 0.011501 → 0.004963 |
| 新規60cm | 5 / 1,031 | 13.564 | 5.921 | -56.35% | 0.015259 → 0.007634 |

PP誤差は観測時点の教師PP操舵との差の絶対値。教師側の評価可能な母数を固定し、
モデルの経路がPPで拒否された場合は0.6radを割り当てる。通常走行のPP支持数は8,226で、
5km/h用の速度条件外などは対象外。両モデルとも通常群の候補拒否は53件、復帰群は0件。
これは位置・操舵のオフライン一致度であり、障害物監視・操舵遅延・完走率の評価ではない。

60cm群には、復帰が進んで横ズレが小さくなった時刻も含まれる。
既存のハッシュ固定済み観測状態をアンカーIDで対応させ、実際に55〜65cmずれている
validationだけを別途集計した。ここでもbestや学習条件は変更していない。
ズレの基準は収集で使用した実測の通常走行線であり、道路中心の真値ではない。

| 実際の観測ズレ | run / アンカー | 旧→新 3秒先XY誤差 cm | 旧→新 PP操舵誤差 rad |
|---|---:|---:|---:|
| 左右合わせて55〜65cm | 5 / 63 | 25.385 → 18.641 | 0.062834 → 0.050664 |
| 左へ55〜65cm | 3 / 45 | 23.843 → 15.828 | 0.052904 → 0.037208 |
| 右へ55〜65cm | 2 / 18 | 27.699 → 22.859 | 0.077730 → 0.070847 |

大きくずれた観測でも改善はあるが、平均位置誤差18.6cm、特に右側22.9cmが残る。
新規復帰群の平均改善だけで復帰問題が解決したとは判断しない。
通常走行と従来復帰のXY精度にも小幅な悪化があるため、現行モデルの置き換えは
新モデルでのAWSIM完走・停止監視の確認後に判断する。
本実験は1 seed・同一コースのrun分離検証であり、未収集地点や別コースへの一般化は未確認。

## 並列収集データの検証

WSLで全4試行のファイル構成・ハッシュ・SQLiteを検証し、因果的な入力履歴と
実測未来3秒・30点の教師を再構成した。修正版の同時走行2本は、いずれも各イベント60以上の
採用Cameraアンカーと実測目標帯のアンカーがあり、3イベントすべてが収集基準を通過した。

| 試行 | 用途 | 採用アンカー | 55〜65cm帯 | 採用イベント |
|---|---|---:|---:|---:|
| 初回 domain 1 左 r01 | 次回train候補 | 262 | 20 | 3 |
| 初回 domain 2 右 r01 | 開始前エラーとして除外 | 0 | 0 | 0 |
| 修正版 domain 1 左 r02 | validation | 242 | 21 | 3 |
| 修正版 domain 2 右 r02 | validation | 242 | 7 | 3 |

同時走行分は計484アンカー・6イベント・目標帯28アンカー。
今回の再学習・モデル選択・上記比較には、この新たな並列収集データを追加していない。

同時走行の復帰区間＋将来3秒における制御出力のsim時間間隔は最大約90ms、
実時間間隔は最大約71.93msで、150ms超は0件。Camera/LiDARの不正メッセージと
逆行ヘッダは0件。ただしfreeze時点の `CURRENT_SENSOR_MISSING` が左31・右36あり、
左の `RAW_HEADING_RATE_INVALID` 9件、右の教師補間不足なども除外した。
条件を緩めて採用数を増やしていない。単独走行時と同じ採用率や全工程2倍速は主張しない。

rawは `/home/thistle/e2e_autonomous/raw/time_recovery_parallel_20260916`、
監査・preparedは `/home/thistle/e2e_autonomous/runs/time_recovery_parallel_20260916`。
検証後に実行先の該当原本と一時転送archiveを整理し、空き容量は約14.08GiB。
最終確認でもAWSIM全ファイルと元repoのHEAD/statusは不変、稼働containerは0。

今回のAWSIM完走結果は公式PPによる教師収集の結果。新モデルの実走はまだ行っていない。
次の判定は、従来の固定5km/h・PP・監視条件で新モデルを実走させ、完走できるかの確認。

数値と検証記録は [compact evidence](evidence/time_parallel_multiscale_20260916/) に保存。
重み・raw bag・入力配列はnative WSLに保持し、Gitへ追加しない。
