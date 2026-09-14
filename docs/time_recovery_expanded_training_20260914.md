# 外向き復帰データ統合・同一更新予算の比較

追加した実測復帰データを既存の時間教師へ統合し、通常走行への影響と外向き復帰の予測を比較する。
編集・コミットはWindows正本、生成・学習・評価はnative WSLの共有worktree lock内で行う。
これは1seedのオフライン比較であり、新しいAWSIM走行の成功率ではない。

## 学習前に固定する条件

- 起点は旧TimePath command OFF epoch10、SHA256
  `e857db4b67d3d9f6a77c2865cfc1fa53e9903e7a1d2d8a371407fbe009a1a44f`。
  既存の復帰学習済み重みからさらに更新する方式にはせず、前回の対照と同じ起点にする。
- 既存の通常train 36,726件と復帰train 223件を保持し、今回のtrain 729件を追加。
  unique trainは37,678件、うち復帰952件。元の20周splitと旧復帰6runの割当を保持する。
- 通常アンカーの提示数と提示位置を前回の1epochに一致させ、復帰の8,920提示枠だけを
  旧・新の復帰952件へ割り当てる。復帰の各uniqueアンカーは9回または10回提示する。
  通常36,726＋復帰8,920＝45,646提示/epoch、3epochで136,938提示・4,281更新。
  反復によって独立データ数が増えたとは扱わない。
- AdamW lr=3e-5、weight_decay=1e-4、batch32、seed42、float32、TF32無効、勾配norm上限1.0。
  optimizer・scheduler・RNGは前回と同じ条件で新規開始。最大2時間の学習実行予算。
- checkpoint選定は前回と同じ通常validation 4run＋旧復帰validation 2runの
  run等重み3秒XY誤差。新しいvalidation r46/r47の186件は選定後の比較だけに使用する。
- 校正r36/r37・評価予約r48/r49は統合しない。元20周のtest 4runも引き続き未使用。
  未割当pilot、未監査の旧V4距離教師や公開データをこの実験へ追加しない。

対照は前回の既存データだけの3epoch学習結果を再利用する。
対照checkpointは`c2fdb6fd1daf525524fe44d3f793d84b482760aa1f9f1fa84d5315222ab9fbe0`。
前回とモデル・学習・cache読込・評価のコード、PyTorch版、初期重み、学習条件を照合する。
旧cache全ファイルのbyte一致、通常アンカー提示位置、同じ初期重みのvalidation結果の完全一致を
学習前に確認する。不一致なら対照が同条件とは扱わず、学習を開始しない。

## データの準備と評価

元の正常cacheはhardlinkで再利用する。旧6本＋追加10本については原本のhashを再確認し、
既存の教師生成器で全候補を再教師化して収集時のcausal auditと採用IDを照合する。
旧cacheとの完全一致を確認するため、前回の入力・教師を黙って更新しない。
新規出力は`datasets/cache/time_recovery_expanded_20260914`と
`runs/time_recovery_expanded_training_20260914`に限定し、旧成果物は保全する。

選定後、同一入力に対する旧・新モデルを通常4run、旧復帰2run、新復帰2runで比較する。
0.5/1/2/3秒XY誤差・ADE・run等重み・支持分母を併記し、
新復帰では両正常基準に対する外向き目標アンカー6件も別集計する。
PPの点選択は現行segment方針・目標5km/h・同じ車両設定で確認する。
保存した教師状態での制御計算と、モデル自身が状態を変える閉ループ走行は区別する。
観測時点のPP成立率は全validationで集計するが、実測速度が固定5km/h試験の運用範囲を
外れるアンカーは非該当として分母を別記する。新復帰2runでは、原本Odometry・velocityを
使って0/100/200ms経過後の姿勢・速度を与え、同じ予測の経過時間に対する感度を調べる。
この経過時間は仮定した条件であり、推論遅延の実測値ではない。
未来の状態は評価器だけに渡し、モデルの入力は元のfreeze以前のまま保持する。
全群の誤差・左右差・PP成立率を報告し、平均誤差だけを根拠に走行用へ昇格させない。

## 再現コマンド

Windowsでコミット後、`tools/sync_to_wsl.ps1 -CheckOnly`、`tools/sync_to_wsl.ps1`。
native WSLの`/home/thistle/e2e_autonomous/e2e_lite_transfuser`で以下を実行する。
出力は新規作成専用。同じ結果へ再起動しない。中断時のみ同一planで`--resume`を使う。

```bash
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python -m pytest -q
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_recovery_expansion.py prepare \
  --plan configs/time_path_p1/recovery_expanded_20260914.json \
  --cache ../datasets/cache/time_recovery_expanded_20260914
tools/with_wsl_training_lock.sh timeout --signal=TERM --kill-after=20s 7200s \
  env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/train_time_recovery_expansion.py train \
  --plan configs/time_path_p1/recovery_expanded_20260914.json \
  --cache ../datasets/cache/time_recovery_expanded_20260914 \
  --output ../runs/time_recovery_expanded_training_20260914
tools/with_wsl_training_lock.sh env PYTHONPATH=src OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 \
  .venv/bin/python -u tools/compare_time_recovery_expansion.py \
  --plan configs/time_path_p1/recovery_expanded_20260914.json \
  --cache ../datasets/cache/time_recovery_expanded_20260914 \
  --training ../runs/time_recovery_expanded_training_20260914 \
  --output ../runs/time_recovery_expanded_training_evidence_20260914/comparison
```

学習準備source `f7743b1`のWSL全体テストは2,331 passed / 4 skipped（100.68s）。
新しい比較処理のテスト・結果と未解決事項は、実行後に本書へ追記する。
