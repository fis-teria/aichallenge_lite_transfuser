# 復帰収集の速度整合・実行記録

2026-09-14。固定目標5km/hを維持し、最初に外乱なしの教師PP走行を確認する。
旧収集の速度ゲイン1.0と評価側の4.0を明示的なprofileで区別する。

## 変更と確認する差

- `legacy_gain1_v1`は従来値。新しい`aligned_gain4_v1`はPPの速度ゲイン4.0 [1/s]。
- 実際に読み込まれたゲイン・外部目標速度・外部速度の有効化を、走行許可前に検査する。
- 目標は両方5/3.6 m/s。PPの速度上限は参照trajectoryも5/3.6 m/sであることを既存collectorが検査する。
- 収集の最終加速度は既存の±1 m/s^2、操舵・停止領域・時間監視を維持する。
- PPはodometryの前後速度を100Hzの計算に使い、collectorの最終発行は20Hz。
  評価側は選択したVelocityReportを使う。ゲインだけで入力時刻まで一致したとは扱わない。
- 新たに選択odometry速度、nominal加速度、実測操舵、最終指令の発行時刻・連番を記録する。
  発行時刻はROS側の境界であり、AWSIMへの物理適用時刻ではない。
- 新規campaign rootを使用できるようにし、過去のsource・参照・試行台帳を保全する。
  CPU割当はnodes 2–5、simulator/Autoware 0–1,6–19を起動時に設定し、走行許可前に照合する。

## 初回の有限試験

外乱なしの通常参照で2走行。目標5km/h、1周＋未来4秒、正常停止3秒。
各runは既存のsim 1800秒・wall 1860秒・outer 1980秒・bag 2GiBを上限とする。
対象角を安全に通過できなければ外乱試験へ移らず、教師制御を解析する。
同じ区間の実測速度中央値を評価側と比べ、差0.05m/s以内を暫定判定とする。

実行先は`graneple@192.168.3.10`、専用rootは
`/home/graneple/e2e_autonomous/time_recovery_speed_20260914`。
起動前にsource・CPP・入力のhash、通常参照、台帳の残り枠、空き容量を確認する。

```bash
PYTHONPATH=/home/graneple/e2e_autonomous/time_recovery_speed_20260914/source/src \
python3 /home/graneple/e2e_autonomous/time_recovery_speed_20260914/source/tools/run_time_recovery_awsim.py \
  --campaign-root /home/graneple/e2e_autonomous/time_recovery_speed_20260914 \
  --run-id codex-time-recovery-speedbase-r30 --side left \
  --speed-policy aligned_gain4_v1 --separate-cpus
```

上記IDはこの記録専用で、使用後に再利用しない。通常参照には介入phaseを設けない。
全ての未来教師は実測pose由来で、参照CSVを教師へ置換しない。

## 検証

Windowsでcommitし、`tools/sync_to_wsl.ps1 -CheckOnly`→通常sync後にnative WSLで実行する。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q tests/test_time_recovery_collection_v1.py tests/test_time_trial_v1.py
```

現段階は実装済み・WSLとAWSIMでの確認前。結果は本書へ追記する。
