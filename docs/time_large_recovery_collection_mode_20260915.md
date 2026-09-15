# 実測横ずれを目標にする収集モード

通常の教師PPと横移動準備用PPを並行稼働させ、既存の収集ノードが最終指令を1系統に
選択する実装を追加した。実測横ずれ20/40/60cmを対象にし、初期値は1周1イベント。
本収集では結果を検証してから1→2→3…と増やせる。設定上限は1周6イベントだが、
候補数・40mの地点間隔・地図検査で収まらない計画は生成時に拒否する。
実走中に回数を自動で増やす機能は設けていない。

実装・合成入力の試験とAWSIMでの車両応答は別の検証である。
この実装作業で新規教師データやモデルの学習結果は作成していない。

## 動作

1. 通常の教師PPで目標5/3.6 m/sを維持する。各候補の準備開始前に、実測速度
   1.15～1.4 m/s、正常実走線から横5cm以内、向き1度以内を1秒確認する。
   前回の復帰区間や未来教師の確保時間中は開始しない。
2. 指定した開始位置の1m窓で、最終操舵の入力を準備用PPへ切り替える。
   準備経路は8mで左右へ移動し、2m程度、目標の横位置を保持する。
   通常PPも元の教師経路と現在の観測を使って動き続ける。
3. 予定復帰位置～その1m先で、目標横ずれ±5cm・向き±2度を0.25秒維持したら
   切替要求を記録する。要求より新しく、未来時刻でない通常PP指令を受信してから
   最終送信を通常PPへ切り替える。要求・解除時刻は実送信時刻で確定する。
4. 通常PPで10秒間の復帰区間を記録する。この間に横10cm以内・向き2度以内を
   1秒維持できたことを確認する。さらに3秒以上、通常走行を確保して次の候補へ進む。
5. 目標未達・準備限界・復帰未確認なら通常PPへ戻し、イベントを無効化する。
   その周では次の横移動を行わない。収集ノードの独立監視が停止要求を出した場合は
   従来どおり停止する。候補窓を通り過ぎた場合は、その地点をスキップして記録する。

初回の例は進捗50mで準備開始、58mから保持、60～61mで復帰要求。
準備中の専用限界は横75cm・向き12度・sim 12秒/wall 24秒・進捗62mまで。
これは新モードの中止条件であり、旧小外乱モードの25cm・4度条件を書き換えていない。
操舵0.5 rad、操舵変化0.8 rad/s、センサ期限、停止領域、判断全体100msの監視は共通。
操舵の単位はROS入力radであり、車体の向きや実タイヤ角とは別である。

公式の経路publisherはbest effortであり、受信側も同じQoSで最新1件を受ける。
新モード専用のDDS XMLでは受信バッファの最小値を1MBにする。実測メッセージサイズは
初回fixtureで通常約134kB、準備約155kBあり、同時配信の余裕を確保するためである。
ホストのsysctlや旧収集の128kB設定は変更しない。経路の期限は引き続き1.5秒。

準備用経路は通常完走の実測位置・向きを基準に作る。後方の元経路から滑らかに接続し、
最新の準備終了位置から物理的な経路長10m以上は横位置を保持する。
PPが復帰を先読みし、目標へ達する前に戻り始めることを防ぐための条件である。
その先の人工的な接続経路を復帰教師に使わない。実際の復帰操舵は通常PPが計算する。

## 複数イベントとデータの扱い

- 地点候補・左右・横ずれ量の組合せをseedで抽選し、走行順に並べる。
  `required_site_ids`で停止地点の手前S00などを固定対象にできる。
  追加10地点という収集全体の対象は複数周に分ける。1周へ全地点を詰め込まない。
- 復帰中の再外乱、前の未来3秒に次の横移動が混ざること、時計リセット後の再発火を
  防止する。監視に拒否された提案はイベント数や切替時刻を進めない。
- rawのCamera/LiDAR/egoと履歴は連続保存する。教師は確認済みの通常PP復帰区間に
  限定し、既存の因果入力・30点の実測未来検証を通す。
- `recovery_event_id`、`recovery_site_id`、`requested_recovery_offset_m`を採用アンカーへ
  付記する。requestedは目標値であり、そのアンカーで実測した横ずれの代わりではない。
  各帯の実測アンカー数を別途集計してから学習へ追加する。
- 完周と復帰イベント成功を別に判定する。run単位でtrain/validationを分け、既存の
  小外乱データとsealed testを保持する。

回数を増やす前に、全予定イベントの到達・復帰、正常完周・停止、bag閉鎖、WSLでの
全ファイル検証、Camera/LiDAR/履歴・未来教師の採用結果を確認する。
初回は20cm左右、次に40cm左右、最後に60cm左右の各1イベントを確認する。
本収集では2回、3回という順に次の計画ファイルの`event_cap`を増やす。
失敗やスキップがある場合は強度・回数を増やさず、その原因を調べる。

## 実行

Windowsでcommit後、既定の同期手順で同一commitをWSLへ同期する。
以下はnative WSLで行う。出力先は既存のディレクトリを再利用しない。

```sh
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/generate_time_large_recovery_reference.py \
  --inputs /home/thistle/e2e_autonomous/runs/time_recovery_collection_20260913/inputs \
  --normal-run /home/thistle/e2e_autonomous/raw/time_recovery_sites_20260915/codex-time-recovery-sites-normal-n03 \
  --normal-proof /home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915/selected_site_plan.json \
  --plan configs/data/large_recovery_pilot_20260915.json \
  --side left \
  --output /home/thistle/e2e_autonomous/runs/time_large_recovery_pilot_new/references

tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q
```

同梱planは左20cmを1回。`candidates`は`site_id`、`release_s_m`、`target_offset_m`、
`return_length_m`からなる。左右にはtargetの符号を使い、`--side`は出力スロット名とする。
出力は通常CSV、準備CSV、両方のSHAとguideを持つJSONの3点。
plan/source/mapのSHA確認、実測guideの範囲・shape確認、準備経路と候補復帰経路の地図検査を
済ませてからファイルを生成する。形状検査は実際の操舵・車体の復帰成功を証明しない。

実走先は`graneple@192.168.3.10`。従来の専用campaign構成（`source`、`inputs`、
`cpp_install`、`references`、テスト・配備のSHA記録）へ同一commitと3点を配置する。
完了済みの小外乱campaignを再利用しない。

```sh
python3 /home/graneple/e2e_autonomous/time_recovery_large_pilot_20260915/source/tools/run_time_recovery_awsim.py \
  --campaign-root /home/graneple/e2e_autonomous/time_recovery_large_pilot_20260915 \
  --run-id codex-time-recovery-large-pilot-left20-r01 --side left \
  --speed-policy aligned_gain4_v1 --separate-cpus
```

上記の実走campaignはこの実装作業では起動していない。30分上限、公式1周+未来確保後停止、
2runごとのWSL転送とhash/構造検証後の限定cleanupという運用を引き継ぐ。
通常RVizにはbaseline/reference/preparation/observedのPathと開始位置のMarkerArrayを表示する。
`PREP 60cm`などのマーカーは目標値付きの準備指令開始位置で、60cm到達の証明ではない。

転送後の既存auditコマンドで、新しいannotation schemaも検査できる。

```sh
tools/with_wsl_training_lock.sh env PYTHONPATH=src .venv/bin/python \
  tools/audit_time_recovery_collection.py --run /native/raw/run-id --output /native/runs/audit.json
```

`large_recovery_events`で切替と実測復帰、`all_planned_large_events_recovered`で予定件数の
成立を確認する。`target_band_recovery_samples`は送信ログ上の観測数であり、
採用Cameraアンカー数ではない。既存の因果auditとmaterializerで教師としての採否を確定する。

## 検証記録

関連テスト、全体pytest、保存済み正常データからの経路生成をnative WSLで実施する。
ROSの接続試験は公式イメージを`--network none`で隔離し、
`tools/smoke_time_large_recovery_ros.py`で合成観測・2系統PP・収集ノード・bag・Path・Markerを
通す。合成観測は操舵指令に従う車両モデルではないため、AWSIMでの復帰性能や教師取得には数えない。
結果の確定値と証拠は実行後に追記する。
