# 独立レビュー依頼：Spatial Path V4 専用runtimeの実装と限定offline検証

Repository: https://github.com/fis-teria/aichallenge_lite_transfuser
Branch: codex/windows-wsl-training-sync
repo/固定実行版: 19c6be967ef5a14dca7800aafcda75d1e1d685fb
設計固定版: da8f5dc90e45f2bc80dce6651d84b509a9441042
synthetic exporter版: e00ab95afe2e145474507695fadf601cf1408e13
結果追記/梱包時HEAD: PACKAGE_MANIFEST.json参照。
checkpoint期待hash: 0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f

目的は、固定step500のV4を学習契約と整合する入力adapterへ組み込み、
同じforwardの未補正20点を制御せず記録する実装の確認です。
packageを主資料とし、実行版・結果追記版・梱包版、design-v2と実装用envelopeを区別してください。

最初にREADME、PACKAGE_MANIFEST、報告、execution manifest、input binding、resolved config、
code diff、生ログ/JUnit、代表入力・予測・全差分を読んでください。
数値結果: 2入力のbatch1 parity PASS、最大差2.384185791015625e-7m、rtol1e-5/atol1e-6。
最終実行2forward、タスク全体同じ2入力で4forward（上限6）、全state前後一致。
pytest26pass/1skip。完全Draft2020は未実施、既存Draft7の使用キーワード互換範囲と意味検証のみ。
rclpy不在、実node/live/制御/学習は未実施。standalone live bootstrapも未完了でmainは明示停止です。

許可は静的code/Schema/launchレビュー、添付hash/JSON identity計算、
allow_pickle=Falseの配列読取と独立差分計算です。
同梱codeのimport/validator/test/inferenceを自動実行する許可ではありません。
package外のcheckpoint/Dataset/raw/sensor/metadata/index、ROS graph、保存path/commandを追わず、
学習・実接続・走行・pushを行わないでください。

確認点:
- 版/hash/代表2入力ID/保存予測/新予測の結合と、未添付checkpointの検証限界。
- 4/4/10/10、9 tensors、padding/reset、commandの過去性/availability、sensor_dt意味。
  fake・packaged tensor・liveの証拠を混ぜない。
- strict restore、eval/inference_mode、state一致、forward回数、batch1/2差の扱い。
- 同一戻り値bits/NaN/−0、6状態、timing8/descriptor、母数/receipt/queue失敗、
  17窓・spacing・累積がdesign-v2と整合するか。
- V3 nodeやcontroller機能を取り込んでいないか。専用node/launchのendpoint/call経路とmock証拠。
- 実node未起動なのにgraph/real input parityを確認済みとしないか。
  teacher maskや20点finiteをruntime validityにしないか。
- command不明を有効0や自作値で埋めず、恒常欠測とwarm-upを分けているか。
- loggerだけで完了扱いしていないか。逆に、報告したlive bootstrap未完了とcore完成を分けているか。
- Schema互換検証を完全Draft2020合格と誤記していないか。
- wrapper grid phase/ego同期/全受信母数、callback例外・logger終了・保存失敗の未検証点を具体化できるか。

返答は(1)上記の独立判定項目ごとの結果、(2)重要度/file/line/根拠/影響/最小修正、
(3)確認済みとUNKNOWN/BLOCKED、(4)次の最小実装依頼に分けてください。
自己点検を独立レビュー合格と扱わず、残差再解析/S1/Ledger/MPC完成を開始条件に戻さないでください。
live shadowの承認は出さず、geometry教師採用/stop labels/motion permission/Safety/controller oracleは別gateのままです。

