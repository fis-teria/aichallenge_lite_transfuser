# Scenario Tool: カート以外の障害物

PC10 (`graneple@192.168.3.10`) に導入した Scenario Tool のソース差分。
AWSIM を変更せず、既存のネイティブ `objects` ローダーでコーン・箱・ポールを配置する。
導入場所と検証結果は [運用手順](../../docs/pc10_scenario_tool_native_objects_20260918.md) を参照。

## 対応する指定

```yaml
objects:
  - id: cone_1
    type: cone  # cone / box / pole
    pose:
      reference_s: 40.0       # reference line に沿った距離 [m]
      lateral_offset: 0.4    # reference line に対する横位置 [m]
      yaw_deg: 30.0          # map 平面の向き [deg]。省略時は経路接線
```

`reference_s` の代わりに `map_xy: [x, y]` または `awsim_xy: [x, z]` も指定できる。
距離の単位は m。`awsim_xy` に対する `yaw_deg` は Unity の向き [deg]。
map/reference から AWSIM への変換は各PCで計測した校正を使う。
最大32物体で、IDは自車・他車を含めて重複禁止。NaN/inf、複数の座標指定、不明な種類を拒否する。

形状とサイズは AWSIM の標準 prefab に従う。現行ローダーはサイズ指定を適用しないため、
`size` や `scale` は受け付けない。物体によっては接触時に動く。
GUIの記号は実寸の輪郭ではない。物体には車両番号・ROS domain・架空のV2X情報を割り当てない。

## 同じ版への再適用

`manifest.json` に元パッケージと変更前後の SHA256 を保存した。
異なる版へ無条件に上書きしない。PC10にはすでに適用済み。

元の `head_to_head-tool.tar.gz` の SHA256:

```text
415c40778cc35761d715436fa2fbd27a443d66c7a1073226bbf964d6b039bbbb
```

別環境で元の `scenario_tool` を展開した後、ソースと既存変更を確認して以下を実施する。
`patch` を置いたディレクトリには本ディレクトリの全ファイルを転送する。

```bash
cd /path/to/original/scenario_tool
git apply --no-index --check /path/to/patch/scenario_tool.patch
git apply --no-index /path/to/patch/scenario_tool.patch
./run.bash self-test
```

既存の AWSIM/車両設定/学習モデルは差分に含まない。
校正ファイルは別PCへ流用せず、対象PCで `./run.bash calibrate` を実施する。
本差分のテストはツール本体の `tests/test_native_objects.py` に含まれる。

## 判定と終了処理

物体生成数は当該runのAWSIMログで照合する。物体を含む `no_collision` は、
当該runの公式 `d1-result-details.json` の `crash` / `wall` カウントを使う。
ログ・カウント欠落は未判定とし、0件と仮定しない。物体別の最小距離や通過判定は未対応。

導入時に確認した終了時の不具合も修正した。監視executorを止めてspin threadをjoinしてから
ROSノードを破棄し、監視プロセスが異常終了したrunを成功扱いにしない。
公式結果はrun固有ディレクトリに出力し、監視ログはホストユーザー権限で書く。
