# 通常RVizへの表示と遠方経路の切り分け

通常Autoware RVizの既存設定へ標準Path displayを1個追加する。
`/visualization/v4_20/raw_path`、46点、観測時刻stamp、frame=lidar。
Vehicle root→LiDARの検証済み取付け並進だけを適用し、既存RVizがTFでmapへ描画する。
map/EKF/TF値はモデル入力・SLAM制御へ戻さない。専用RViz processは起動しない。
既存displayとFixed Frame=mapを維持し、Buffer Length=1で過去経路を重ねない。
期限切れは空Pathで消去。TF不足時に偽のmap→SLAM変換は生成しない。

Windows commit→既定CheckOnly/同期後、WSL lockで全pytest。
通常configへの追加は以下（元ファイルを.before-v4-20へ排他的backup）：

```bash
python3 tools/integrate_normal_rviz_v4.py EXISTING_AUTOWARE_CONFIG.rviz
```

V4 configの`slam_shadow.normal_rviz_path=true`を有効化する。
モデルは従来どおりraw全46点を保持。学習・重み・制御方針は本変更では変更しない。

## 遠方が乱れる原因の判断

モデルの保存raw自体に折り返しがあり、表示だけの乱れではない。
前回166受信PLANはrawと一致し、表示165件の剛体変換結果も最大差0m。
剛体変換は経路内部の点間距離・方向差・自己交差を変えない。

確認済みの構造的問題：

- 全1,786 trainのうち復帰1,020件（57.1%）は教師が最大1.3m。
  全1,786件が20m教師を持つわけではない。>10mは350件、20m地点は165件/7run。
- mask外はloss/gradientがゼロ。復帰入力に対する遠方出力は直接教師で拘束されない。
- runtimeは有効長/validity headを持たず、常に46点を表示する。
- path headは92個のXY座標を直接回帰し、損失はmask付き点誤差のみ。
  点順序、曲率、滑らかさ、自己交差、時間方向の連続性を保証する制約はない。
- 12 epochの開発validationでも遠方誤差1.058m、20m地点1.330m。
  遠方の評価は2runに偏り、AWSIMでの未見条件への一般化は未証明。

1.3m境界での乱れと教師支持の減少は整合するが、個々のAWSIM入力が復帰分布と
一致するかは未証明。教師不足だけが唯一の原因とは断定しない。
短距離教師で更新される共有特徴が遠方出力を変える可能性もある。
表示の平滑化だけで隠さず、有効長出力/短距離と長距離教師の扱い/経路形状制約を
分けて改善・検証する必要がある。単純なepoch増加は12→16で遠方誤差を悪化させた。

## 実施結果

実行commit `f30b8bd58d142f209ea6d4ececaa0b078f80dd60`。
WSLで全pytest 1838 passed / 4 skipped / 52 warnings、79.87秒。
Humble package build成功。通常Autoware configへ実適用し、元ファイルbackupを保存。
追加Path stanzaを取り除くと元ファイルと完全一致することを確認。

適用先：host `graneple@192.168.3.10` の
`/home/graneple/git/autononous_ai/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware.rviz`。
install configはこのsourceへのsymlinkであり、通常make devのRVizが読む対象。
既存map表示、車両、操作panel、Fixed Frameは維持。

停止状態の試験35/36を実施。Start要求なし、駆動権限file作成なし。
独自RViz起動行をrunnerから除去し、通常make devが起動する `/rviz2` だけを使用。
試験35では86 PLAN、標準Path 85件（全件46点）、受信XYと取付け並進後rawの最大差0m。
標準Pathの購読者に `/rviz2` を実確認した。
試験36では通常 `autoware.rviz` のウィンドウをXWDで取得し、画面上の
V4-20 display、Global Status: Ok、車両付近のピンク経路を確認した。
スクリーンショットは`tmp/v4_20_normal_rviz_35/normal_rviz.png`。
デコードはXWDの24bpp、BGR、bytes_per_lineに従い、画面内容の加工なし。

両試験ともhost/probe faultなし、所有sim終了後docker psは空。
既存wheel補助processには終了時rclpy二重shutdownのtracebackがあり、
子process全てがclean exitしたとは主張しない。表示中のデータ検証とは分けて保全。
新規学習・重み変更・追従制御変更はしていない。pushなし。

```bash
CARTOGRAPHER_BUILD_ROOT=/home/graneple/e2e_autonomous/cartographer_extrap_build_20260910 \
CARTOGRAPHER_TEST_PROJECT=codex-v4-20-normal-rviz-36 CARTOGRAPHER_V4_EXTRAP_COMPARE=1 \
V4_SLAM_SHADOW_ROOT=/home/graneple/e2e_autonomous/v4_20_normal_rviz_35 \
python3 /home/graneple/e2e_autonomous/cartographer_v4_20_normal_rviz_36/run_moving.py
```

既存outputの再利用は禁止。実行済みの試験は終了している。
今後もV4-20 configでnormal_rviz_pathを有効にし、専用RVizを追加起動しない。

教師支持はWSLで保存済みselection/teachers.npzから再集計した。

|教師grid|有効train anchor数|
|---|---:|
|0.1m|1786|
|1.1m|948|
|1.2m|722|
|1.3m|690|
|1.4m|677|
|20m|165|

復帰教師は上限1.3mで、全復帰anchorが1.3mまで有効という意味ではない。
診断/照合結果：WSL `/home/thistle/e2e_autonomous/runs/v4_20_normal_rviz_35/` の
`teacher_support.json`、`display_verification.json`、完全evidence。
同小JSONをWindows `tmp/v4_20_normal_rviz_35/`へ保存。
