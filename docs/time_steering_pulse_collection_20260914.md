# 外向き復帰を採る操舵パルスの実装・方式確認

2026-09-14。[収集計画](time_recovery_collection_method_20260914.md)の第2段階。
実行は`.10`のAWSIM、データ監査と学習はnative WSL。角度拡張は今回の変更に混ぜない。

## 実装

`time_steering_pulse_v1.py`はROS非依存の、1回だけの滑らかな操舵入力パルスを提案する。
提案状態は不変で、collectorが監視を通して実際に指令を発行した場合だけ確定する。
再試行や未発行の提案によって、開始・解除済み状態が進まない。

通常参照を常時追う教師PPの入力へ加算し、その後に既存の操舵角±0.5rad、操舵速度0.8rad/s、
加速度±1m/s^2と停止領域監視を通す。100ms判断期限、センサ期限、唯一のpublisherを維持する。
開始前後のカメラ/LiDAR/ego/指令は連続記録する。

| 状態 | 発行する外乱 | 教師phase |
|---|---|---|
| waiting | 0 | baseline。速度・位置・向きの開始条件を満たすまで待つ |
| active | 指定振幅×sin²(πt/T) | hold。未来教師にしない |
| releasing | 解除要求時の値から短時間で0へ | hold。解除途中も未来教師にしない |
| recovery | 0 | recovery。実測未来を監査する |
| complete / skipped | 0 | baseline。同じ走行で再投入しない |

外向きの目標状態への到達、横ずれ・向き・速度・進捗の上限のいずれかで解除を要求する。
sim時間上限またはその2倍のwall時間上限では外乱を0にする。未達による延長・増幅は行わない。
走行開始時に周回境界直前へ投影される場合があるため、開始区間の手前を一度観測してから有効化する。

通常走行との差は、先に取得した実測基準走行の進捗・横位置・車体yaw `[N,3]`から計算する。
固定CSVとの差だけで外向きを分類しない。基準のrun ID・原controlのhashを参照へ記録する。
基準データはteacher/debug専用で、学習モデルの入力へ追加しない。

## 教師境界と互換性

新しい記録は`annotation_schema=measured_steering_pulse_v1`を付け、最終指令の発行sim時刻・
monotonic時刻・連番をphase境界へ使う。外乱要求時刻や計算開始時刻を解除完了時刻に流用しない。
指令連番の欠落、逆行、非0外乱が残った教師phase、不明な形式の混在はエラーとする。
同一sim時刻でphaseが競合する区間、150msを超える記録の空白は教師にしない。

旧記録のdecision時刻に基づく処理は維持する。新旧ともphase開始後150msの除外を初回は維持し、
新方式の先頭0.5秒で外向き状態を落としていないかを監査する。
この150msを通信の物理遅延が証明された値とは扱わない。
過去の外乱に由来する実測操舵やyawの応答は初期状態として残す。
単一の`intervention_ns`へ解除時刻を代入せず、期間の集合で未来0.1〜3秒を検査する。

`collection_phase_windows()`をmaterializer、閉じたbag監査、因果replayの3か所で共有した。
パルスのない通常参照（空intervals）も監査できるようにした。
CSVを教師へ置換せず、形状 `[30,2]`、0.1秒間隔、m、観測時点base_linkという契約を維持する。

## 有限試行と採否

[速度整合](time_recovery_speed_alignment_20260914.md)の通常2走行が通った後、
左右各2回までを校正と方式確認に使う。各1周＋未来4秒、停止3秒、bag 2GiB上限。
初期計画の0.03〜0.05rad・0.5〜1秒は仮値であり、発行前の応答見積もりで実行値を固定する。
実装が許す校正範囲は振幅最大0.1rad、期間最大2秒、横ずれ上限最大0.3m、向き上限最大6度。
これはその最大値の試行を自動で繰り返す指定ではない。
各試行の値と変更理由を結果欄へ残す。同じ未達・失敗を無変更で反復しない。

開始は目標角付近の通常走行状態からとする。取得データは左右の実測横ずれ5〜25cm、
外向き2〜4度を当初目標に分類するが、未達を成功へ数えない。
初期校正runは最終評価へ使わない。採用アンカー数、外向きの状態、先頭0.5秒の除外、
復帰のピークずれ・時間・監視拒否を報告してから、本収集や再学習へ進む。

## 確認コマンド

Windowsでcommitし、通常のsync後、WSLで以下を実行する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q \
  tests/test_time_steering_pulse_v1.py tests/test_time_recovery_collection_v1.py \
  tests/test_time_recovery_training_v1.py
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

参照JSONに`steering_pulse`を指定した場合のみ有効。通常の収集launcherと同じ引数で起動し、
`--speed-policy aligned_gain4_v1 --separate-cpus`を付ける。
具体的なrun ID・root・参照hash・実測結果は方式確認後に追記する。
