# PC10 Scenario Tool / コーン・箱・ポールの配置

## 導入先

- Host: `graneple@192.168.3.10`
- AWSIM checkout: `/home/graneple/git/autononous_ai/aichallenge-racingkart`
- 追加したツール: `scenario_tool/`
- 既存の `head_to_head/` は保全。
- ソース差分: [integrations/scenario_tool_native_objects](../integrations/scenario_tool_native_objects/README.md)
- 証跡: `/home/graneple/e2e_autonomous/scenario_tool_setup_20260918`
- 出力: checkout内の `output/scenario_tool/<run-id>/`

学習・V45教師データ収集は再開していない。

## GUIを開く

PC10のデスクトップ端末で:

```bash
cd /home/graneple/git/autononous_ai/aichallenge-racingkart
./scenario_tool/run.bash gui
```

ブラウザは `http://127.0.0.1:8030`。
「カート以外の障害物」でコーン・箱・ポールを選び「障害物を追加」を押す。
一覧で物体を選択し、マップをクリックまたはドラッグして配置し、向きを編集する。
「フォーム → YAML」「検証」「保存」でシナリオを保存できる。
`pc10_objects_smoke.yaml` には3種類と静止カート1台の配置例がある。

Windowsから編集画面を使う場合、PC10で上のGUIを起動したまま次の転送を開く:

```powershell
ssh -N -L 18030:127.0.0.1:8030 graneple@192.168.3.10
```

Windowsのブラウザで `http://127.0.0.1:18030` を開く。
AWSIM実行時はPC10デスクトップの `DISPLAY` / `XAUTHORITY` が必要。
GUIの公開先はlocalhostのままでよい。

## CLI

```bash
cd /home/graneple/git/autononous_ai/aichallenge-racingkart
./scenario_tool/run.bash validate --scenario scenarios/pc10_objects_smoke.yaml
./scenario_tool/run.bash run --scenario scenarios/pc10_objects_smoke.yaml --unattended --gpu
./scenario_tool/run.bash self-test
```

この例はPC10に既存のキャッシュ済みAutowareイメージを使う、5mの設置・起動確認。
E2EモデルやMPPI V45の回避性能を評価するシナリオではない。
`current_workspace` を選ぶ場合は現在のソースに対応したビルドが必要。
導入時のPC10はソースよりinstallが古く、既存のビルド検査で停止することを確認している。
別AWSIM試験と同時に起動しない。

## 検証範囲

- 座標校正: fit最大誤差 0.003292m、holdout最大誤差 約0.0087m。
  yawの実測確認は未完了。角度の座標変換はunit testで検証。
- GUI: 3物体読込、箱を追加、再検証、既存のegoイメージ/start_grid保持、JavaScript例外0。
  物体の向きを30度に編集し、`yaw_deg: 30` への反映を確認。
- 実走: `pc10-objects-smoke-r2-20260918`。3物体生成、5mの終了条件到達、
  monitor終了コード0、公式crash=0 / wall=0、ツール判定passed。
- カメラ85フレーム、LiDAR50scan、TF8本を取得。カメラにコーンの表示を確認。
  全物体のLiDAR識別や障害物回避性能までは未検証。
- 初回の物体runでmonitorがexit139となる終了処理不具合を発見して修正。
  そのrunの旧passed判定は採用せず、上記r2を正常終了の根拠とする。
- テスト・最終保全確認の結果は [validation JSON](pc10_scenario_tool_native_objects_validation.json) に記録。
  ツール単体350件中343件成功、7件は環境依存のスキップ。
  変更13ファイルの再適用と導入先SHA256一致、AWSIM本体を含む既存7ファイルの不変を確認。
- WSLの既存 `pytest -q`: 3,077 passed / 4 skipped（113.68s）。共有worktree lock下で実施。

## 制約

AWSIMに備わる標準形状を使う。任意メッシュやサイズ変更は対象外。
物体によっては接触時に動く。物体別の離隔・通過判定は未対応。
公式crash/wallカウントはAWSIM側の条件に依存し、全接触の検出を保証するものではない。
物体座標はシナリオ配置・検証用であり、E2Eの入力へ直接追加していない。
今回の確認は設置と起動の確認で、完走・回避・学習品質の評価ではない。
