# TimePath P1 学習準備

対象は[Astraレビュー](<Astra Pro/astra_2026_1006_review_ja.md>)のP1-prep。
実データ学習・20周原本の転送・AWSIM走行の保留は維持する。

## 接続した経路

`TimeEvent → TimeDataset → ModelBatchV3（入力のみ） → TimePath 30点 → 時間教師の損失 → optimizer → checkpoint`

- `assemble_time_inputs`をoffline/runtime双方が呼べる共通関数とした。実ROSノードへの接続は後続。
- 実camera取得時刻とavailable clockのfreezeを区別。freezeは各アンカーに指定する。
- 同run/epoch/clock、epoch範囲、到着cutで絞ってから重複解決。入力pose/velocity補間は両端到着後のみ。
- 教師はfull-runの未来poseを使う独立経路。欠損入力でも生成可能な教師と除外理由を保持する。
- 入力はRGB（既存ImageNet正規化）、2D LiDAR（範囲＋validity、角度で最近傍再配置）、ego SI値、過去final command。
- egoは `[longitudinal m/s, lateral m/s, yaw rate rad/s, actual steering rad]`。速度は必須、他はmaskで欠損を明示。位置・未来・教師指令を入力しない。
- LiDARの無効beamは既存契約どおりrangeチャンネル1.0＋validity 0。欠損の0値を実測と扱わない。
- 教師frameは`base_link_at_observation`。PPのrear axleへの実変換は未実装であり、同一と仮定しない。
- stop状態・意図・Safety・収集介入・horizon端を区別。状態閾値の実測校正は保留し、根拠なしはUNKNOWN。B0のstop probabilityはNone。
- 収集介入以降と、未来3秒に介入を含む巡航アンカーを除外する。

## 学習・評価の契約

- `train_time_epoch`は明示したアンカー数・optimizer更新数で必ず停止する。入力欠損も訪問数に含む。
- 損失はアンカー内の有効点L1座標平均、その後に有効アンカー平均。勾配蓄積もwindow全体の有効アンカー数で正規化する。
- 全無効microbatchは既存勾配を消さず、全無効windowはoptimizer/schedulerを進めない。
- freeze済み教師順序とcursorを維持し、checkpointはoptimizer window境界で保存する。中間勾配は保存しない。
- checkpointはモデル/前処理/履歴設定、optimizer/scheduler、Python/NumPy/Torch/CUDA RNG、epoch/step/cursor、manifest・source系譜を保存する。
- `inspect_time_checkpoint`で構成を検証してからモデルを構築し、`resume`で状態を復元する。`finetune`は元checkpoint identityで重みのみ読み、新実験のidentityを別途保存する。
- 評価は全アンカー数、教師支持数、raw誤差、採用条件付き誤差、入力欠損・非有限出力・棄却数、run平均/最悪runを保持する。支持0はNone/NOT_EVALUATED。
- horizonは0.5/1/2/3秒の点誤差。全点ADEは有効点のEuclidean誤差平均であり、学習L1とは異なる。
- baselineはゼロ移動と現在の実測縦横速度からの定速予測。未来教師から速度を推定しない。
- offlineの採用は「有効入力からの有限出力」。実controller採用率、実stale率、進行量、停止意図性能の証拠ではない。

## 分割と実験

20runを速度設定ごとに6/2/2で固定する。これは同コース・同条件内holdoutであり未知シナリオ汎化とは呼ばない。
raw bag ID/hashの重複を拒否する。移行先の原本hash照合前は`sources_verified=false`で学習APIが拒否する。
testは学習対象に渡せず、最終評価では明示的な`final_test=True`を要する。前処理のfitはtrainだけで行う。
OFF/ONは同seed・初期重み・分割・提示予算。初回比較はscratchを基準とし、継承元データ系譜不明はUNKNOWNとする。

## WSL実行

Windowsでコミットし、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`の順に同期する。
WSLのnative checkoutで以下を実行する。

```bash
cd /home/thistle/e2e_autonomous/e2e_lite_transfuser
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q \
  tests/test_time_dataset_p1.py tests/test_time_objective_p1.py \
  tests/test_time_metrics_p1.py tests/test_time_checkpoint_p1.py \
  tests/test_time_split_p1.py tests/test_time_pipeline_p1.py \
  tests/test_time_path_v1.py tests/test_time_backbone_p0.py \
  tests/test_time_teacher_p0.py tests/test_time_reference_p0.py
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
```

小型fixture内のoptimizer更新と再開比較はソフトウェア検証。実コーパス学習や学習済み候補モデルの作成は行わない。

分割/設定draftは以下。128アンカー・4更新は準備用の例示予算であり、本学習の予算決定ではない。

```bash
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/prepare_time_p1.py draft \
  --receipt ../runs/time_p1_preparation_88c4827/receipt.json --output-dir ../runs/time_p1_preparation_88c4827/draft \
  --seed 42 --max-anchors 128 --max-optimizer-steps 4
# SSD移行後だけ実行する原本の読み取り検証:
bash tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python tools/prepare_time_p1.py verify-sources \
  --split configs/time_path_p1/split_draft.json --raw-root /path/to/extracted/package \
  --output ../runs/time_p1/split_verified.json
```

## 残る実データ作業

20周原本の配置・hash照合、bag receiptと処理完了時刻の差の監査、実frame/取付姿勢の確認、介入時刻の抽出、
実教師の有効数・速度・停止ノイズの監査、大規模コーパスの逐次変換/cache、有限予算の学習実行は後続。
現在のDatasetはdecoded eventを受ける小型・共通ロジックであり、20GBをそのまま全RAMへ展開する運用は行わない。
生成した設定と分割draftは`configs/time_path_p1/`に保存した。例示の128アンカー・4更新を本学習予算として扱わない。
検証用receiptと出力はWSL worktreeの外に置く。WSLの`tmp/`は必ずしもignoreされておらず、クリーン状態を要求する検証を妨げるためである。

## 検証結果

実行commit: `88c48272e9bfa89e8b798c7e064029b27271f6cd`。WSL native checkout、共有lock経由。

- 関連回帰: **57 passed / 9 warnings / 10.59秒**。
- 全体: **1910 passed / 4 skipped / 62 warnings / 137.98秒**。
- skipは既存のOSQP、JSON Schema関連2件、任意official package。依存の追加インストールはしていない。
- 初回全体実行は確認用receiptがWSLで未追跡になり、既存clean-checkoutガードのテストが1件失敗した。
  receiptをworktree外へ移し、コード変更なしで全体を再実行して成功した。初回ログも保全。
- 20周分draft生成をWSLで実行。train 12 / validation 4 / test 4。OFF/ON設定差は`use_command_history`のみ。
- split hash: `0339b8cadc704b5b418279fd3c058008060fecd4268b30d7e3e6235aabab80b0`。

[検証JSON](evidence/time_path_p1/verification.json) / [全体ログ](evidence/time_path_p1/full_pytest.log)。
モデルの走行性能・実教師の品質・停止能力を確認した結果ではない。実データ学習とAWSIMは実行していない。
