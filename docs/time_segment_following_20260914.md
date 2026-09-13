# 線分内の先読み位置選択と固定5km/h追従評価

ユーザー承認済みの[整合性調査の方針](time_lookahead_alignment_audit_20260914.md)を実施する。
Windows正本で実装し、native WSLでテスト・記録評価、AWSIMはgraneple@192.168.3.10で実施する。

## 実装と固定条件

- 新ポリシー`stopping_preview_segment_v1`を追加。旧ポリシーは過去記録の再現用として保持する。
- 最初に現行と同じ元の点を検索する。成立する点があれば位置・操舵・加速指令を保持する。
- 全点不成立のときだけ、元の隣接線分内の成立区間を求め、中点をfloat32で再検査する。
- raw予測・時刻順・後輪座標変換を保持。距離帯と0.3rad上限、速度目標5km/h、安全監視は維持。
- 選択方法、元の参照点番号、補間比率、残り時刻、観測からの時刻、距離・操舵余裕を指令詳細へ記録する。
- 診断で検証済みの線分境界計算をROS非依存の共通control moduleへ抽出する。
- checkpointは復帰学習epoch3、SHA `c2fdb6fd1daf525524fe44d3f793d84b482760aa1f9f1fa84d5315222ab9fbe0`。
- 新configは旧候補configからlookahead_policyのみ変更する。

## 検証手順と有限予算

1. WSLで全pytest、保存済み新旧試験の比較。既存成立指令は完全一致、無関係な拒否は維持すること。
2. 専用deploymentへ転送・ハッシュ確認、Humble build、ROS_DOMAIN_ID=93・network noneでsmoke。
   記録経路と停止した合成センサによる線分選択を、PP・応答補償・入力制限・clear scan監視まで通す。
3. 実行先の既存環境・資産・通常RVizを確認し、固定5km/h目標で1周を目標に試験。
   1試行600秒走行・720秒outer、最大2試行。同じ物理的失敗の繰り返しで設定は緩和しない。
4. 実記録をnative WSLへ戻し、選択点・操舵応答・先読み距離不足・停止理由を評価する。

```powershell
./tools/sync_to_wsl.ps1 -CheckOnly
./tools/sync_to_wsl.ps1
```

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
  .venv/bin/python tools/evaluate_time_segment_policy.py \
  --records ../runs/time_recovery_finetune_evidence_20260914 \
  --output ../runs/time_segment_policy_20260914
```

保存状態でのPP成立・仮の操舵指令列は、動いた後の予測や実scanの通過を保証しない。
ROS smokeの合成センサ試験と、AWSIMでモデルが出した予測による走行結果を区別する。

## 状態

実装sourceは`d990fd6e893841167d7974f8d5058b35185ffab5`。
native WSLで2,264 passed / 4 skipped / 64 warnings、記録比較もexit 0。
旧記録の成立1,956指令は位置・操舵・加速が完全一致し、応答補償・操舵マッピングのreplayも一致。
候補の103指令はすべて線分内選択でPP成立。元のraw予測は保持した。

証跡原本: `/home/thistle/e2e_autonomous/runs/time_segment_policy_evidence_20260914`と
`/home/thistle/e2e_autonomous/runs/time_segment_policy_20260914`。
AWSIM専用deployment `/home/graneple/e2e_autonomous/time_segment_following_20260914`で
source 737ファイル・install 216ファイルの一致、Humble build、isolated ROS smokeの成功を確認。
ROS smokeでは線分選択25指令が応答補償・操舵マッピング・clear scan監視まで到達した。

AWSIM `codex-time-segment01`を1試行実施。固定5km/h目標で発進し、約100.44m・80.62秒走行。
実測最高4.692km/h、区間0→1→2まで進み、`STOPPING_SWEEP_OCCUPIED`で終了。完走は未達。
最初の拒否時は先読み1.933m、PP要求タイヤ角0.132636radであり、PPは成立していた。
安全停止後の繰り返しを分離して、最初のscan拒否と操舵応答を追加解析する。

simulatorは監視faultでfreezeして終了。freeze前の物理的な停止確認は取れていない。
終了後のAWSIM/RViz/process/稼働containerは0、既存停止container114件と既存Git差分を保全。
RViz設定は事前のbytesへ復元し、scene/vehicle/DLL hashの一致を確認した。
74件の記録をSHA-256照合しnative WSLの
`/home/thistle/e2e_autonomous/runs/time_segment_awsim_evidence_20260914`へ保存した。
WSLで実指令replayは1,608件一致（うち操舵マッピング1,603件）、評価はexit 0。
