# V4 shadow 配送・推論ボトルネック診断

## 対象と境界

2026-09-09。Windows正本 `E:/workspace/e2e_lite_transfuser`、branch
`codex/windows-wsl-training-sync`、参照HEAD
`803eac1529d6150e208529681abc0175d450d05a`。開始時working treeはclean。
保存済み `tmp/v4_pp_shadow_16` / `tmp/v4_pp_shadow_17` のログと現行コードを診断した。
今回、新規モデル推論、ROS起動、SSH接続、走行、設定・runtime変更、pushは行っていない。

## 結論

主要な改善対象はGPU forwardではなく、入力結合の時刻検査と先頭候補待ちである。
不適格な候補を一時的な入力不足と同様に待ち続け、後続cameraを止める構造がある。
モデルを軽量化する必要性や学習不足は、この診断からは確定できない。

## run17の時間内訳

以下は成功PLAN 366件のmonotonic時間。センサ取得時刻からの遅延ではない。

| 区間 | 平均 ms | p95 ms |
|---|---:|---:|
| camera受信 → 入力結合 | 110.51 | 270.27 |
| 入力結合 → 推論呼出し開始 | 27.70 | 84.34 |
| 推論呼出し | 33.15 | 59.01 |
| camera受信 → 結果生成 | 171.36 | 333.60 |

成功候補の採用入力が最後に到着したのはcamera受信から平均8.93msで、
その到着から入力結合までさらに平均101.58msあった。
別母集団である3766 batchのIPC送信→worker処理開始は平均2.04ms、p95 4.02ms。
このIPC区間は上表と重なり、加算してはならない。送信前の親側待ちも含まない。

推論呼出しには `freeze_batch`、device転送、モデルforward、
`SpatialRuntimeV4.snapshot` によるCPUへの結果取り出しを含む。
33.15msをCUDAカーネル単体の測定値とは扱わない。
入力結合後の27.70msも前処理、履歴処理等の合計であり、個別内訳は未計測。
結果生成後のqueue待ち・JSON保存・flush時間もUNKNOWN。

run16でもcamera受信→入力結合146.03ms、推論呼出し32.94msと同じ傾向だった。

## 拒否と先頭候補待ち

run17のJOIN_DEADLINEによる拒否は異なるcamera IDで580件。
期限前にtimestamp重複に関連する理由を持ったものは384件（66.2%）、
CAMERA_GRID_TOLERANCEのみは188件（32.4%）。合わせて572件（98.6%）。
JOIN_WAIT 3091イベントは再試行を含むため、frame数としては数えない。

`shadow_observation_join_v4.py` は各入力を64件のdequeへ追加し、
先頭cameraだけについて、各stream全体を時刻順にsortして結合を試す。
`ValueError` はJOIN_WAITとして同じcameraを残してreturnする。
候補は受信から300msのdeadlineまで後続候補をブロックし得る。

`synchronization_v3._validate_stamps` は渡された全履歴のstrict増加を要求する。
sort後のstrict増加エラーは同一timestampを意味する。
現在時刻に完全一致する入力があっても、古い重複が履歴に残るだけで拒否される。
同一値の古い重複を含む純粋合成配列で、この挙動を再現した。

`spatial_sim_adapter_v4.py` のcamera gridは初回cameraを位相とする100ms間隔、
許容差40ms。あるcameraがこの条件を満たさない事実は、その候補を待っても変わらない。
ただし現行joinはこれも300msまで待つ。許容差を広げればよいという結論ではない。

run17ではPLAN間隔が最大2.715 wall秒空いた。そのsource時刻範囲には
24件のcamera期限切れがあり、重複timestampのWAIT 75件とgridのWAIT 44件があった。
出力の長い途切れにも入力結合拒否が対応している。

## 重複の出典と未確定事項

独立observerのrun17 odometry記録4963件には54組の重複timestampがあった
（余剰54件）。ただしV4自身のJOIN_WAITには入力roleと問題のtimestamp対がない。
これだけで全384件をpose由来と断定できない。LiDAR、velocity、steering、poseの
どの履歴で失敗したか、同時刻の内容が同一か競合しているか、重複発生元はUNKNOWN。

成功PLANの入力到着時間を、失敗候補すべてに一般化してはならない。
全区間を細分化したprofilerではないため、GPU競合や保存時間の寄与をゼロとも主張しない。

## 次の修正候補（今回未実装）

1. 同時刻サンプルを「同一内容の再配送」と「競合内容」に分類する。
   role・timestamp・sourceを記録し、契約に基づく重複処理を決める。
   競合を無条件に捨てたり時刻を改変したりしない。
2. 入力到着で解消し得る不足と、固定cameraのgrid不適合等の確定拒否を分離する。
   後者は理由付きで即時終了し、先頭候補による後続の停止を避ける。
3. 親側送信前待ち、画像前処理、転送、forward、結果取り出し、保存を個別計測する。
   正常と拒否の両方を残し、同じ設定で改善前後を比較する。

## 再現

ローカル保存済みログに対する限定診断：

```powershell
C:/Python310/python.exe tmp/v4_bottleneck_audit.py
C:/Python310/python.exe tmp/v4_timestamp_repro.py
```

集計stdoutは `tmp/v4_bottleneck_audit.log`。
合成再現は `REPRODUCED: old identical duplicate rejects even exact current match` を出力した。
これらはtmpのローカル診断資料であり、Git配布物への同梱はしていない。
pytest全体、ROS、bag replay、新しいAWSIM試験はNOT_RUN。
