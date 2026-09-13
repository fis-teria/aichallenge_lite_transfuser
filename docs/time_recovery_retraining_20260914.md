# 復帰データ追加学習とAWSIM比較

ユーザー依頼: 採用した復帰データで再学習し、graneple@192.168.3.10でAWSIMテストを実施する。
この文書を実行状態の正本とする。WSL追加学習と新旧モデルのAWSIM比較は終了した。
候補はオフライン誤差が改善したが発進できず、走行用には不採用。停止理由のWSL再現、証跡保存、終了後確認まで完了した。

既存の学習入口は20周固定のscratch OFF/ON比較専用であり、復帰区間の除外と追加runの固定split、
既存checkpointからの追加学習を扱えない。データ変換と学習開始点のみを拡張する。
元の20周splitを内包し、元のtrain/validation/test割当とraw hashの非重複を強制する。
新形式以外の6/2/2制約は変えない。旧checkpoint/cache/rawと旧実験部署は保全し、出力は新規パスに限定する。
判定指標は通常走行・復帰holdoutの3秒誤差とAWSIM進捗/停止理由。ラベルshape/mask/履歴、split境界、
finetune初期重みをテストし、実センサ再構成の一致も確認する。安全監視やcontrollerは変更しない。

## 現在の実行状態

| 比較項目 | 追加学習前 | 追加学習後（epoch3） |
| --- | ---: | ---: |
| 通常走行holdout 4runの3秒位置誤差（run等重み） | 0.063942m | 0.053676m |
| 復帰holdout 2runの3秒位置誤差（run等重み） | 0.050488m | 0.035960m |
| 6run平均（checkpoint選定指標） | 0.059457m | 0.047770m |
| AWSIM記録自己位置の移動距離 | 123.464m | 発進なし |
| AWSIM終了理由 | CONTROL_STOPPING_SWEEP_OCCUPIED | PROGRESS_STALLED（PP先読み点なし） |
| 完走 | 未達 | 未達 |

誤差は観測未来教師との距離であり、AWSIM車線追従誤差ではない。
両試験とも設定目標は5km/h。旧モデルの実測最高は約4.70km/h、候補は停止状態のまま。
候補の記録自己位置の累積0.000237mは静止中の揺れであり、走行実績に数えない。

- 3epoch/4,281更新/136,938提示を完了。学習・検証・再ロードは2,556.77秒、前処理照合を含むコマンド全体は2,591.90秒。
  選定epoch3、保存時と再ロード時のvalidation予測は全件一致。
  checkpoint SHA `c2fdb6fd1daf525524fe44d3f793d84b482760aa1f9f1fa84d5315222ab9fbe0`。
- validationの3秒run-macro誤差: 通常4走行0.06394195→0.05367572m、復帰2走行0.05048828→0.03595959m。
  6走行平均0.05945739→0.04777034m。testは未使用。復帰データ追加と追加学習更新だけの効果を分離する実験ではない。
- 候補: `codex-time-recovery-model-candidate01`、source `54513331888f9cf3478db9d0d7b6f69ab278fc20`。
  `/home/graneple/e2e_autonomous/time_recovery_model_candidate_20260914`。
  baselineとsrc/ros2_ws/toolsの全ファイル一致。設定差はcheckpoint SHA/epochのみ。
  213module source/install一致、隔離ROSの実モデルPath6件一致・異常制動確認後に実行。
  `STOPPED_NO_LAP / PROGRESS_STALLED`、wall50.06秒。走行許可から5.15秒の全103指令が
  `STEERING_FEASIBLE_LOOKAHEAD_MISSING`で、正の加速指令0、実測停止確認あり。
  48個の走行中予測は現在のgeometry判定で全てRESOLVEDだが、PP先読み点の実行条件を満たさない。
  WSLで全103件の同じ拒否理由を再現。経路修正や監視緩和は行っていない。
- baselineのWSL評価は98.105秒/123.464m、section0→1→2、未完走。
  PP/記録制御1,958件を再現、最大差8.88e-16。scan admission全件再生ではない。
- 最終実行source `5451333` のWSL全体テスト: 2,227 passed / 4 skipped / 64 warnings、74.69秒。
  native runtime loaderでも候補重みと実センサcache入力から有限[30,2]出力を確認。

- 変換・学習source: `17777082d4d9c8cef0b2c7be482352a19fae7fa1`。WindowsからWSLへ同期済み。
- 前段source `6da2842` のWSL全体テスト: 2,227 passed / 4 skipped / 64 warnings、78.05秒。
  `1777708` はcacheに既存runtime契約のframe/points/dtを保持する1行の修正。
- 6runの原本照合・全332復帰候補の再教師化と採用一致・各run2件の実センサ/cache tensor一致が成功。
  学習復帰223件、検証復帰109件。通常データを含むunique train36,949件、validation12,346件。
  1epochの提示は45,646件、3epochで136,938提示/最大4,281更新。testは開封していない。
- cache: `/home/thistle/e2e_autonomous/datasets/cache/time_recovery_20260914`、
  identity `917df968ee8fb1951a2d150c2da82165aa351e7597e372054401601d02c09023`。
- 学習出力: `/home/thistle/e2e_autonomous/runs/time_recovery_finetune_20260914`。
  `status.json`/`result.json`はCOMPLETE。同じ出力へ学習を再起動しない。
- 実行ログ: `/home/thistle/e2e_autonomous/runs/time_recovery_finetune_evidence_20260914/train_1777708.log`。
  終了記録は同ディレクトリの`training_exit.json`、exit0。
- 旧モデル比較: `codex-time-recovery-model-base01`、専有deployment
  `/home/graneple/e2e_autonomous/time_recovery_model_baseline_20260914`。
  213moduleのsource/install一致、隔離ROSで実モデル経路6件一致・異常制動・publisher0を確認後に実行。
  144.05秒wallで`FAILED / CONTROL_STOPPING_SWEEP_OCCUPIED`、完走なし。
  発進から約98秒で監視作動。hostがfreeze終了したためfreeze前の実測停止確認は未成立。
  起動したcontainerは0、cleanup error0。速度目標と監視は前回のvehicle_model設定と同じ。
- 旧モデル記録58ファイル78,035,140bytesをnative WSLの上記evidence配下`baseline/`へ転送し全件一致。
  archive SHA `f00fdd628f211fefbdfd66b65bd285df332bea63fe7054c68b658f2d28174c4d`。
  内部のautoware.logリンクは解決先が同一archive内であることを確認。remote原本は保持。
  native WSLでの制御計算再生・定量評価を学習lock解放後に実施済み。
- 小さい証跡: `docs/evidence/time_recovery_finetune_20260914/`。
  通常RVizのmagenta経路を保存画像で目視確認した。XWDは宣言BGR24・bytes_per_lineを使ってPNG化。

## 発進拒否の診断

保存したraw [30,2]予測、観測時/制御時pose、速度、同一設定をWSLで元の制御関数へ再投入した。
候補103件の拒否理由は全件一致し、変更前モデルの発進直後104件は全件TRACKINGを再現した。
走行許可時に使う先読み距離1.0〜1.5mの候補点を調べると、次の違いがある。

- 候補: 各指令の最小必要操舵角の絶対値が0.300530〜0.301672rad。
  先読み点の採用上限0.300000radを全件でわずかに超え、採用可能点が0。
  初回は7点中0点が採用可能。3秒先端は約(1.850, -0.493)m。
- 旧モデル: 初回の最小必要角0.295700rad、7点中2点が採用可能。
  初回の3秒先端は約(1.765, -0.442)m。旧モデルも上限に近い出力だった。
- 現行のgeometry判定では候補の走行許可中48予測すべてRESOLVED。
  今回は、その後のPP先読み点選択で拒否されている。終了後のCLOCK_STALEは発進失敗の原因ではない。

ここでいう0.3radはPP点選択の採用条件であり、車両の絶対的な旋回限界を証明した数値ではない。
診断は実記録の再現であり、新旧モデルを同一センサtensorへ入力した比較ではない。
AWSIM各試験のcamera/LiDAR原本は収録しておらず、教師不足や学習側の根本原因はまだ確定していない。

次の優先課題は、発進・低速状態での経路とPP先読み点の成立率/操舵余裕を学習モデルの採用判定へ追加すること。
そのうえで、同一入力での新旧比較と発進教師の監査を行い、データ・学習目的・点選択のどこを直すか判断する。
3秒位置誤差だけのcheckpoint選定では、この上限付近の発進失敗を検出できなかった。
候補を走行用へ昇格させず、上限変更・経路補正・同条件の追加試行は行っていない。

## 保存・終了後確認

- native WSLに新旧試験記録を保存し、原本manifestの全file hashを照合。
  候補archive SHA `20f465228544a6733df59064f65eba6d3f68b982afd7c6e27c7d80fa81832973`。
- Windowsに学習結果JSON、評価JSON/図、通常RViz画像、実行/転送/テスト証跡のみをコピーしhash一致を確認。
  `training/transfer_verification.json`、`evaluation_transfer_verification.json`、
  `deployment/transfer_verification.json`を同梱。重み・raw bagはGitへ追加していない。
- 通常RVizの`Time model raw prediction`とmagenta経路を新旧とも画像で確認。
  候補の`rviz2`購読、Global Status: Okも確認。表示画像はXWDのBGR24/strideに従いPNG化したもの。
- AWSIM側の終了後確認: 起動中container 0、AWSIM/RViz process 0、既存停止container 114を保全。
  repo HEAD `4af395eee10f928c7fc7225760adfa04c4c07ff4`、既存変更155件を保持。
  scene/vehicle.yaml/Assembly-CSharp.dllのhash不変、RViz設定は試験前のbytesへ復元済み。
  空き22,055,682,048bytes（約20.54GiB）。旧/候補deploymentと記録原本は保持。
- WSLの学習・評価process 0。候補checkpoint hashを再照合し、空き約694.75GiB。
  最終全体pytestは2,227 passed / 4 skipped。skipはOSQP、jsonschema関連2件、任意の公式LiDAR package。
  診断script `496e189` は実記録103件/104件のsmokeで確認。追加の本番コード変更はない。

証跡入口:
[学習比較](evidence/time_recovery_finetune_20260914/training/comparison.json)、
[旧AWSIM評価](evidence/time_recovery_finetune_20260914/baseline_evaluation/summary.json)、
[候補AWSIM評価](evidence/time_recovery_finetune_20260914/candidate_evaluation/summary.json)、
[発進経路比較図](evidence/time_recovery_finetune_20260914/launch_diagnostic/launch_paths.png)、
[候補の通常RViz画像](evidence/time_recovery_finetune_20260914/candidate_normal_rviz.png)、
[最終AWSIM環境](evidence/time_recovery_finetune_20260914/deployment/candidate/final_environment.json)。

## 固定計画

- 起点: command OFF epoch10、SHA e857db4b67d3d9f6a77c2865cfc1fa53e9903e7a1d2d8a371407fbe009a1a44f。
- 元のtrain12周、validation4周、test4周の割当を保持。testは未使用。
- 復帰train: r19右20cm、r20左20cm、r21直線左40cm、r25右カーブ左40cm。
- 復帰validation: r22直線右40cm、r23右カーブ左20cm。1runをsplit間で分割しない。
- r24/r26の補助、部分周回、診断失敗は今回の教師に含めない。
- recovery開始150ms後から、因果的な入力と全30点の観測未来を持つアンカーのみ。
  50ms受信freeze、yaw異常を含む履歴の除外、phase maskを収集監査と再照合する。
- train復帰アンカーを40回提示し、通常走行に埋もれないよう約20%の提示割合とする。
  独立データ数は増えない。validationは繰り返さず、全6runの等重み3秒誤差でepochを選ぶ。
- AdamW lr3e-5、3epoch、batch32、seed42、float32/TF32無効。optimizer/RNGは新規開始。
  約136,938提示/4,281更新の有限予算（実測採用数で確定）、学習outer timeout 2時間。
- AWSIMはbaselineと候補各1周を目標、各720秒outer/600秒走行、最大4試行。
  同一の物理的失敗を2回繰り返したら打ち切り、記録して判断する。
- 目標固定5km/h、通常RVizのモデル経路、既存vehicle_model設定・指令制限・停止監視を維持。
  raw poseは推定自己位置、bag receiptは前処理完了時刻の実測ではない。

## 再現コマンド

Windows正本でcommit後、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`。
native WSL `/home/thistle/e2e_autonomous/e2e_lite_transfuser` で実行する。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_recovery.py prepare \
  --plan configs/time_path_p1/recovery_20260914.json \
  --cache ../datasets/cache/time_recovery_20260914
bash tools/with_wsl_training_lock.sh timeout --signal=TERM --kill-after=20s 7200s \
  env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_recovery.py train \
  --plan configs/time_path_p1/recovery_20260914.json \
  --cache ../datasets/cache/time_recovery_20260914 \
  --output ../runs/time_recovery_finetune_20260914

# 既存の出力は上書きせず、再評価時は新しい出力先を指定する。
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  docs/evidence/time_recovery_finetune_20260914/analyze_launch_rejection.py \
  --root ../runs/time_recovery_finetune_evidence_20260914 \
  --output ../runs/time_recovery_finetune_evidence_20260914/launch_diagnostic
```

AWSIMで実行したコマンド列は`deployment/{baseline,candidate}/*_exit.json`、
起動wrapperは同じdirectoryの`run_{baseline,candidate}.py`へ保存。
AWSIM開始・停止とnode起動は各`host_result.json`に記録している。

大きなデータ/cache/重みはnative WSLのF:上に保持する。Windows E:とGitには置かない。
AWSIMへはcommit済みsourceと選定重みを新規deploymentへ転送し、hash・install・asset・RViz接続を確認する。
完了判定は追加学習完了、候補のオフライン比較、AWSIM実行と結果分類、停止/環境後確認である。
完走できない場合も失敗を隠さず、offlineの改善と閉ループの成立を分けて記録する。
