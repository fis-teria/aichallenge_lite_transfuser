# MPPI V45 収集教師の位置・速度推定と停止判定の修正

対象はPC10 AWSIMの収集用教師 `lidar-motion-intent-r2`。
学習モデルの重み、AWSIM本体、衝突マージンは変更しない。
目標5km/hで、以前停止・後退を繰り返したカート入口と箱の2配置を通過した。
カート1イベントから教師候補314窓を確保。箱は最小距離監査が未対応のため、自動採用を保留する。

## 変更

- LiDAR位置履歴の隣接移動距離を足す速度推定を、過去0.6sのXYベクトル傾きの中央値へ変更。
  位置も最新観測時刻に合わせて推定する。0.2m/s未満は静止扱い。
  3点未満または履歴0.2s未満は速度未確定として0m/sにする。
  観測の間隔が空くケース、3km/h移動、旋回、発進・停止、外れ値、時刻逆転を回帰試験対象とする。
- 採用済みのAVOIDは、同じ新鮮な前方干渉対象が20km/h以下なら継続できる。
  開始条件の5km/h境界を毎回またいで回避を解除することを防ぐ。
  新しい対象への開始条件、観測timeout、全候補の衝突検証は維持する。
- 後退開始の4sタイマーは、収集モードでは新鮮なCMA前進速度要求で判定する。
  指令が1m/s未満、停止、欠落、古い場合はタイマーを解除する。
  開始済みの復帰はこの条件で打ち切らず、既存の停止・ギア切り替え処理へ任せる。
- r1実走で、戻り軌道の保持条件が新しいAVOID探索を禁止し続ける停止を確認した。
  r2では戻り軌道の実行中でも、AVOID条件が成立した場合は回避探索を再開する。
  切り替えは既存の軌道・車体衝突検証を通った場合だけ行う。

有効化は `teacher_v45.launch.py` の3パラメータに限定する。
通常ノードの既定値はfalse。`check_runtime.py` で実ノードの有効化と既存6マージンを確認する。
`source_manifest.json` は現在のSHA256と変更前のupstream SHA256を保持する。

## 保存入力の再生

実装commit: `b3f3cd8b1e1ae4cd8ebffd17f610ad34118fc160`。
記録済みLiDARの同じtrack ID・source時刻の履歴を新しいC++関数に通した結果。
これだけで実走の通過を証明するものではない。

| 場面 | 保存履歴の点数 | 旧MPPIの速度 | 修正版の速度 |
|---|---:|---:|---:|
| コーナー入口の停止カート、source 48.699998911s | 4 | 5.726km/h | 1.101km/h |
| 直線の箱、source 42.149999057s | 11 | 7.479km/h | 0km/h |

カートは静止の完全な推定には達していないが、両方とも早期AVOIDの5km/h境界を下回る。
最初の後退前4sの通常指令はカート378件、箱381件すべて1m/s未満。
旧判定は基準経路速度だけで後退へ入ったが、修正版ではこの指令履歴から後退を開始しない。

WSL証跡:
`/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/fix-r1/saved-replay-r2.json`。

## 検証と再収録

- 最新r2のWSL全体pytest: 3,123 passed / 4 skipped / 84 warnings、111.83s。
  4 skipは既存の任意依存・公式package未提供。実装commitは下記。
- 既存recovery単体試験: 27 passed / 1 deselected。
  除外1件は同梱していない元`mpc_controller.py`から定数を読む比較テスト。
- r2のPC10 C++: colcon集計391 tests / 0 errors / 0 failures / 0 skipped。
  新しい戻り軌道からの再探索テストも実ノードを含む試験で成功。
  非走行domain97と実走domain1で、教師3 consumer・既存6マージン・修正3項目の有効化を確認。
- r2実装commit: `7c328eaf4bfca7139887cfb98037fa6cba49d0dd`。

| run suffix (`lidar-v45-pc10-`) | 教師版 | 通過 | 後退 | 公式crash/wall/over | 教師候補窓 |
|---|---|---|---:|---|---:|
| cart-entry-b01 | 元V45 | 未通過 | 7区間 | 0/0/0 | 0 |
| cart-entry-r1a | r1 | 未通過・停止保持を確認後中断 | 0区間 | 0/0/0 | 0 |
| cart-entry-r2a | r2 | 通過・指定地点到達 | 0区間 | 0/0/0 | 314 |
| box-b01 | 元V45 | 未通過 | 7区間 | 0/0/0 | 0 |
| box-r2a | r2 | 通過・指定地点到達 | 0区間 | 0/0/0 | 0（距離未検証） |

r1は最小近似車体間距離0.93m、Camera 1,212観測、停止要求0。
409ファイル / 460,852,145 bytesをWSLへ転送し全SHA256一致。
未通過なので成功教師へ昇格させない。再ビルド容量確保のためPC10 rawコピーは再照合後に整理した。

r2のカートは走行81.274s、最小近似車体間距離0.521m、Camera 858観測。
箱はCamera 725観測。双方の停止要求0、地図上の車体逸脱0m、公式判定passed。
これは2つの静止障害物配置の試験で、移動NPCや全コース完走の確認ではない。
314窓は1イベント由来の重複窓であり、独立した314回避を意味しない。
画像 `[1,4,3,224,384]`、LiDAR `[1,4,2,750]`、実測未来XY `[30,2]` m、dt=0.1s。

WSL保存先: `/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918/`。

- `collected/lidar-v45-pc10-cart-entry-r2a/`: 409ファイル / 239,127,730 bytes。
- `collected/lidar-v45-pc10-box-r2a/`: 407ファイル / 188,747,094 bytes。
- 2run計816ファイル / 427,874,824 bytes、全SHA256一致。
- `audit/<run-id>/`: 監査JSON・時間窓索引・実測future教師NPZ。
- `fix-r2-pytest.log` と `fix-r2-vendor-recovery.log`: WSLテスト結果。

箱のrawと実測futureは保存済みだが、車両V2Xへ物体が現れないため最小距離がnull。
既存監査の0.30m条件を満たしたと仮定せず、教師候補マスクは全てfalseに保つ。
既存学習splitへの登録や再学習はこの作業では実行していない。

終了時、AWSIMバイナリを含む保護対象7ファイルは元のSHA256と一致。
収集コンテナの残存0、PC10空き約3.02GiB。最新source 323ファイルとruntimeのハッシュを再確認した。

PC10配置:
`/home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/{source,runtime}`。

再現コマンド:

```bash
cd /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918
python3 source/tools/build_mppi_v45_overlay.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/runtime

python3 source/tools/collect_mppi_v45.py \
  --awsim-repo /home/graneple/git/autononous_ai/aichallenge-racingkart \
  --runtime /home/graneple/e2e_autonomous/mppi_v45_collection_fix_20260918/runtime \
  --scenario /absolute/path/to/scenario.yaml --run-id lidar-v45-pc10-example-new \
  --speed-cap-kmh 5 --wall-timeout-s 480 --run-budget-gib 0.75 --free-reserve-gib 2 --execute
```

ビルド・runは新規出力先が必要。既存成果物を上書きしない。
前回と同じ位置・向き・seed・通過地点・5km/hでカート入口と箱を比較する。
前回3runのraw146ファイルはWSL側とSHA256を再照合後、PC10側コピーを整理した。
全記録と元教師ソースはWSL側に保持している。
箱はnative車両V2Xに存在せず、既存の車体間距離監査では距離未計測として扱う。
