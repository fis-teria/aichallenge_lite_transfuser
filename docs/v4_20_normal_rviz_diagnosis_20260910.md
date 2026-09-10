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
