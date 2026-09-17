# 回避教師の収集シナリオ

修正中の教師MPPIに接続するための、H2H静止NPCシナリオ生成器。
Windowsの正本は `configs/collection/avoidance_static_v1.json` と
`tools/prepare_avoidance_scenarios.py`。教師MPPI、AWSIM、既存H2Hシナリオを編集せず、
未作成の出力ディレクトリへシナリオ、収集キュー、入力ハッシュを生成する。
このコマンドはシミュレータ・ROS・学習・Dockerを起動しない。

## 収集条件

- 直線・コーナー入口・旋回中・出口を各4地点、計16地点。
- 各地点の前後1.5 m、左右・中央の3配置。道路境界で同じ位置に丸められた候補は重複として除外する。
- 教師速度上限は5 / 8 / 10 km/h（1.388889 / 2.222222 / 2.777778 m/s）。
- 物体は `static_physical` のNPC 1台。既存の車体形状と0.30 m以上の評価離隔を使用する。
  地図に重なる位置、Referenceと干渉しない位置、同じ断面に通過候補がない位置は収集キューへ入れない。
- 最大288条件から上記の除外を行う。これは収集候補数であり、成功イベント数ではない。
- 最初は警告のない5 km/h条件を地点ごとに順番に選んだ最大72条件。
  起動警告のある配置は別途 `probe-placement` で確認してから扱う。
- 追加で、同一区分の3地点にNPCを配置する5 / 8 / 10 km/hの条件を生成する。
  それぞれの単独配置で起動・回避・復帰を確認してから使う。

自車の開始位置は既存start_grid 1。今回の生成器は自車を強制的に横移動させない。
回避で発生した横ずれからReferenceへ戻る部分を連続収録する。
移動車両、任意形状の箱・コーン、意図的な通路閉塞はこのセットに含めない。

## 分割と教師の採用

コース終盤の4地点をvalidation、残り12地点をtrainとして生成前に固定する。
同じ地点・関連区間の速度違い、前後・左右位置違い、再試行を同じ区分に保つ。
区分間の配置がReference上で12 m未満なら生成を拒否する。ゴールをまたぐ距離も確認する。
同一run内にtrainとvalidationの障害物を混在させない。
validationは「既知の地図内の回避配置を分けた開発用評価」であり、未見地図・封印testではない。
既存データと統合するときは、過去の同じ配置・関連区間も照合して再分割する。

通常の終了地点は障害物の25 m先。ゴール付近は最初の回避・復帰を切らないよう2周を記録する。
各走行のシミュレーション上限は30分、外側の実時間上限は40分。
後者は収集runnerが適用する値で、H2H YAMLだけでは強制されない。
低速や停止で上限に達したrunは未通過として残し、成功数に加えない。

採用対象は、接触・道路はみ出しがなく、必要なCamera/LiDAR/ego履歴と将来3秒が揃った実測教師。
後退・緊急停止・末尾の不完全な未来は既存 `audit_lidar_v2x_obstacles.py` の条件で除外する。
通過だけでなく、回避後のReferenceへの戻りも走行ログで確認する。
回避不能時の停止教師は停止意図のラベル設計を別途行い、停滞や後退前停止をそのまま正解にしない。

## 生成

H2Hがあるホストで次を実行する。`REFERENCE` はシナリオ位置を決めるCSVであり、
教師バイナリを古いV44へ固定する引数ではない。現行の基準CSVのSHA-256をplanに固定している。
修正した教師のReferenceを変更した場合は、位置・分割・地図検査をやり直す。

```bash
python3 /path/to/prepare_avoidance_scenarios.py \
  --plan /path/to/avoidance_static_v1.json \
  --h2h-repo /home/si26-pc008/git/autonomous_ai/aichallenge-racingkart \
  --teacher-reference-csv "$REFERENCE" \
  --output /path/to/new_suite
```

出力:

- `scenarios/*.yaml`: 通常のH2H形式、map位置・向き・期待結果・収録指定。
- `native/*.json`: 既存コンパイラで変換したAWSIMシナリオ。AWSIM本体への書込みはない。
- `placements.json`: 除外を含む全配置、位置[m]・向き[rad]・横位置・断面の通過候補。
- `collection_queue.jsonl`: split、必要な速度上限[m/s]、時間上限、警告、前提条件。
- `pilot.json`: 最初に試す単独条件の順序。validationを学習へ混ぜない。
- `summary.json`, `manifest.json`: 件数、制約、入力と生成物のSHA-256。

H2Hのyaw境界だけは度数法なので、生成時にradからdegreeへ明示変換する。
障害物配置に使う教師ReferenceとH2Hの進捗Referenceは別々に記録する。
断面上の隙間は旋回中の車体の通過可能性を証明せず、生成成功は実走成功を意味しない。

## 教師修正後の接続

生成した各条件は `execution_ready=false`。既存runnerが固定している旧V44をそのまま使わず、
修正済み教師の実行ファイル・設定のidentityを収集runnerに明示して接続する。
速度はH2Hシナリオの名前から推測せず、キューの `teacher_speed_cap_mps` を教師launchへ渡し、
実ノードのパラメータと照合する。H2H標準YAMLに未対応の速度フィールドを追加してはいない。
マージンは既存設定、物体入力はLiDAR V2X、native V2Xは評価専用、E2Eはshadowを維持する。
教師修正の前後は別run・別identityで保存する。

1. 修正済み教師のidentity、速度、収録topic、有限時間・容量上限を接続する。
2. 配置を起動probeし、実際のnative V2X位置と想定の位置を照合する。
3. pilotを1条件ずつ収録・監査し、成功イベント数を集計する。
4. 単独条件を拡張し、確認できた配置だけ3台条件へ進む。

汎用のH2H `run` だけではこの教師・速度・容量契約は適用されない。
修正中の教師や稼働中AWSIMへ、自動で接続・起動する機能は設けていない。

## 検証

Windowsでこの変更をcommitし、公式 `tools/sync_to_wsl.ps1` で同一commitを同期する。
既存の別作業のindexがある場合は保全し、cleanなWindows transport cloneから同期する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser_lidar_v2x_margin
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

単体試験は位置shape、角度・速度の単位、分割漏洩、周回境界、pilot順序、既存出力保全を確認する。
実H2Hでの生成結果と実走前に残る確認は、別の検証JSONに記録する。

## 作成済みセット

生成sourceは `b16cac5991b5a0d84f3219db7734984a4bb2f487`。
SI26の既存H2Hで16地点・96配置要求を検査し、境界で重複した6配置を除外。
90配置から単独270条件（train 198 / validation 72）、3台配置3条件を生成した。
H2Hの構文・意味検査とAWSIMシナリオへの変換を全273条件で確認した。
地図・Reference・H2H Python入力のハッシュは生成前後で一致した。

pilotは5 km/hの72条件（train 52 / validation 20）。過去の起動不良付近というH2H警告がある
27条件はpilotに含めず、後続の配置probe対象としてキューに残した。
既存採用データとは未統合で、新しい実走成功イベントはまだ0件。

SI26の保存先:
`/home/si26-pc008/e2e_collection_scenarios/avoidance_static_v1_b16cac5/`

WSLで新規20試験と全体 **3,067 passed / 4 skipped / 84 warnings** を確認した。
全体所要149.91秒。skipは既存のOSQP・JSON Schema validator・optional Tiny環境の不足。
教師の修正作業、稼働環境、AWSIMの起動は操作していない。

詳細は [検証記録](avoidance_collection_scenarios_validation.json)。
