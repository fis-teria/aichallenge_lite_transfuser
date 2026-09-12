# 20周アーカイブの検証と時間教師の分割

2026-09-13の依頼に基づき、SSD移行後の原本転送・整合性検証・教師生成・分割を行う。
以前の転送保留はこの範囲で解除。optimizerによるモデル学習・AWSIM走行は含まない。

## 固定条件

- 原本: `time_teacher_20laps_20260911.tar`、20,207,984,640 bytes。
- archive SHA-256: `2d22e3273ebdeebc920716974e4389e696d324fd0b0c2f9ae74c95c540ddc6fd`。
- 内部MANIFEST SHA-256: `476321a3cd619bf2300779cf9cb498cff97a89e4e51a97238c8612be449b9581`。
- 全948ファイル（MANIFEST自己を含む）。外側SHAと全ファイルのサイズ/SHAを検証し、成功時のみ展開先を公開する。
- WSLは `F:\WSL\Ubuntu-22.04-Recovered`、生成処理はnative Linux filesystemで行う。
- 固定分割は `configs/time_path_p1/split_draft.json` のまま。各速度設定6/2/2周、合計train12/validation4/test4。
- split identity: `0339b8cadc704b5b418279fd3c058008060fecd4268b30d7e3e6235aabab80b0`。
- 同じコース・同条件の別run holdout。未知コースへの汎化評価ではない。

## 教師と入力参照

撮影時刻を基準に0.1秒刻み・30点・未来3秒のXY[m]と実測縦速度[m/s]を生成する。
座標系は`base_link_at_observation`。XY/velocity/intervalに別々のmaskを持つ。
撮影時刻はヘッダのsim clock、利用可能時刻はbag receipt proxyとして区別する。
freezeはcamera receipt + 明示された固定delay。CLI既定0nsは診断用として残し、今回の本出力は50,000,000nsを指定する。
これは受信後50ms待機するoffline構成であり、実測した処理完了時刻を意味しない。
両端がfreezeまでに届いた観測poseとegoだけを補間し、教師の未来情報を入力へ混ぜない。
収集終了の`brake_sim`以降と、未来3秒がその介入を含むアンカーを除外する。
入力欠損・教師欠損のアンカーもJSONLとmaskへ保存して分母を保持する。
停止意図は収集指令から推定せずUNKNOWN。collection interventionは別理由として保存する。

`train/<run>/`、`validation/<run>/`、`test/<run>/`に以下を置く。

- `raw/`: 検証済み原本runへのsymlink。Camera/LiDARの原本は複製しない。
- `anchors.jsonl`: 全撮影アンカー、bag row ID、時刻、因果的履歴参照、採否と理由。
- `teachers.npz`: `[N,30,2]` XY、独立mask、速度、アンカー時刻。
- `audit.json`: 有効数、除外理由、センサ形状、SQLite整合性、実測速度、実データ再構成smoke。

トップレベル`split_verified.json`は全20bagの移行先SHAを照合後に`sources_verified=true`となる。
`audit.json`の`usable_partial`は入力参照が成立しXYを1点以上支持する数、`usable_full`は30点すべてを支持する数。
全画像の形状/encodingとLiDAR geometryを逐次検査し、選択アンカーでは実画像・LiDARをP1共通入力関数まで通す。
全アンカーのtensor cacheや学習loaderの最適化を行った証拠ではない。
実推論側にも同じ入力確定条件と観測時刻からの経過時間の扱いが必要であり、今回ROSへは適用していない。
testの処理は形式・整合性の検証と教師生成に限定し、モデル選定・評価には使わない。

## 再現コマンド

Windowsでコミット後、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`。
以下はWSL native checkout `/home/thistle/e2e_autonomous/e2e_lite_transfuser`から実行する。
出力先は新規ディレクトリを指定し、既存出力・失敗時partialを上書きしない。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q \
  tests/test_time_archive_v1.py tests/test_time_sqlite_reader_v1.py tests/test_time_corpus_v1.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/prepare_time_corpus.py extract \
  --archive ../datasets/archives/time_teacher_20laps_20260911.tar \
  --destination ../datasets/raw/time_teacher_20laps_20260911 \
  --archive-sha256 2d22e3273ebdeebc920716974e4389e696d324fd0b0c2f9ae74c95c540ddc6fd \
  --manifest-sha256 476321a3cd619bf2300779cf9cb498cff97a89e4e51a97238c8612be449b9581 \
  --report ../datasets/archives/time_teacher_20laps_20260911_verification.json
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/prepare_time_corpus.py materialize \
  --raw-root ../datasets/raw/time_teacher_20laps_20260911 \
  --destination ../datasets/processed/time_teacher_20laps_20260913 \
  --split configs/time_path_p1/split_draft.json --freeze-delay-ns 50000000
```

## 検証結果

コード実行commit: `82ba8b8ef651314e33d32bf849f3253d4d1f2a04`。
WSL native checkout・共有lock経由で、新規回帰14 passed（2.96秒）、全体1976 passed / 4 skipped / 62 warnings（75.74秒）。
skipは既存のOSQP、JSON Schema関連2件、任意official package。

原本転送は正常終了。外側archive SHA、内部MANIFEST SHA、全948ファイルのサイズ/SHAが一致した。
展開後の20bagも固定split記載のSHAと一致した。転送途中の`.partial`は照合後に`.tar`へ確定した。

入力確定を画像受信と同時（0ms）にすると、5kmh_run01の3,907アンカーすべてで入力条件が成立しなかった。
内訳は速度補間に必要な次の実測値が未到着3,891件、現在センサ欠損16件。
同じ学習runを20枚おきに196アンカー抽出し、0/20/50/100msを比較した。
20/50/100msでは196/196入力が成立し、各191件が未来3秒の教師を完全支持した。
testを見て条件を選定していない。50ms条件の全1周再生成では3,791件が入力・全30点を支持し、実画像/LiDARからの再構成2件もPASS。
0ms診断と1周pilotは別ディレクトリに保存し、本出力へ混ぜない。

20runの最終件数・出力監査は生成完了後に追記する。
