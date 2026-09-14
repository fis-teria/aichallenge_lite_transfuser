# 外向き復帰教師の追加収集（2026-09-14）

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

実行結果・データ件数・採否は収集と監査が完了してから追記する。
