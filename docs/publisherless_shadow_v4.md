# Publisherless V4 shadow runner — 合成限定実装

## 結果と未完成境界

起点4af1e2650848cd6b25fb73120f1fac9b5596cd6a、Windows clean。
実行commit2914a21f8a81d07d8c191da68236cfdd352b1305。
今回追加したのは送信非接続のsession core、有限子process harness、入力専用node構築hook、合成tests。
**実AWSIMへ接続するrunner全体は未完成**。CLIはfixtureだけを許可し、実入力モードは
`LIVE_BLOCKED_PASSIVE_COMMAND_POSE_AND_ISOLATION_BINDINGS_REQUIRED` で起動前に拒否する。

ユーザーが承認した1回/120秒/40推論/送信0という試験方針に対して、上限と送信非接続を
独立に実装・検証した段階。実ROSの完成やAWSIM試験PASSとは扱わない。

## 変更

- `src/aic_transfuser_lite/runtime/publisherless_shadow_v4.py`
  - `ShadowSession`: adapter→V4 callback→raw20点→既存Bridge。既存送信runnerはimportしない。
  - `fixed_infer_factory`: 明示呼出し時だけ既存の厳密固定loaderへ接続。今回呼出していない。
  - 欠損commandを0で補わない。shadow出力をcommand履歴へ戻さない。
  - epoch変更で履歴とBridgeを無効化、重複候補を拒否、forward前に試行数を加算。
  - late forwardはplan化しない。最大40試行、最大200候補。
  - `control_tick`は観測受付とは別のAPI。ただし現段階の同一process内同期infer中には
    tickを実行できない。実transport版では別processの推論と独立周期の制御評価へ接続が必要。
  - `create_input_only_node`: 7種類の受動subscriptionだけを作る注入型hook。
    ROS middleware内部のparameter-event publisher等は未実測。全DDS publisherゼロは未証明。
    control/gear/mode publisherは新コードに存在しない。
- `tools/run_v4_publisherless_shadow.py`
  - fixture専用CLI。新規output必須、上書き禁止、自動retryなし。
  - 120秒のうち5秒を終了処理へ確保。所有childのみterminate→kill→reap確認。
  - 同期modelが止まっても親processが終了させる設計。OS停止不能なら成功扱いしない。
  - 合成fixtureの生成時だけ人工値を使用。実モードの承認期限や車両値を自動生成しない。
- `tests/test_publisherless_shadow_v4.py`: 11件。既存Bridge23件と合わせ34件。

## 入力の実在問題

既存 `SpatialInputV4.build()` はcommand bindingと利用可能な過去commandを要求する。
旧sim runnerはHOLD等を実際に送信して履歴を作っていた。送信ゼロの試験で同じ履歴を
捏造するとモデル入力契約を変えてしまう。
このため、新sessionは本当に受動取得したnominalがない場合に
`PASSIVE_COMMAND_MISSING` としてforwardを呼ばない。
静止していることだけからcommand=0とは推定しない。
現AWSIMが無送信状態でこの入力を提供できるかはUNKNOWN。

## 実行結果とコマンド

Windows commit→既定CheckOnly/sync→WSL lock。固定Dataset rootの存在確認を実施。
Dataset内容/raw/sensor/checkpointの読取なし。sim hostへの接続・変更なし。

```bash
bash tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q \
 tests/test_publisherless_shadow_v4.py tests/test_path_control_bridge.py \
 --junitxml=runs/publisherless_shadow_01/junit.xml

bash tools/with_wsl_training_lock.sh .venv/bin/python tools/run_v4_publisherless_shadow.py \
 --fixture --session-id synthetic-publisherless-01 \
 --output runs/publisherless_shadow_fixture_01 --wall-seconds 120 --forward-limit 40
```

限定tests: **34 passed / 0.41s**。
fixture process実行: exit0、child回収済み、wall約0.214s、人工出力callback40回。
これはV4モデルforward40回ではない。実モデルforward0、実scan0、AWSIM起動0、制御/gear/mode送信0。
fixtureログ中のFORWARD_STARTEDも人工callbackの試行であり、`fixture_only=true`で区別する。
上限到達後のcontrol_tickは指令無効として記録する。
trace.jsonl、summary.json、stdout/stderr、JUnitをWindows `tmp/publisherless_shadow_results/`へ保存。
旧live台帳の使用量・期限・active等は変更していない。今回はlive予算未消費。

ASTによる新コードのpublisher/client/action呼出し不在検査と、fake Nodeがpublisherを
作らない構築試験を実行。実ROS graphでの確認はNOT_RUN。
全pytest、ROS/AWSIM、実checkpoint推論、走行、pushはNOT_RUN。

## 実試験へ残る作業

1. 受動command sourceの実在確認。欠ける場合は入力契約変更を無断で行わず別設計判断。
2. 現simのpose生成経路と観測時刻を結合する入力受信部。任意のOdometry topicを仮定しない。
3. 車両Limitsと出典、current-container隔離の読取検証、実ROS内部endpoint監査。
4. 推論childと独立control tick、queue/失効/欠損を実transportへ接続。
5. 新しい有限試験枠を旧使用量へ追加する既存budget連携と、所有AWSIMの外側監視・終了。

これらを接続するまではlive CLIを開放しない。既存HOLD送信runnerを裏で使う回避も禁止。
