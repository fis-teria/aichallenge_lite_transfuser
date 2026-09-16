# 未充足のコーナー復帰条件の追加収集

前回の封印済み収集 `time_corner_gap_20260916` に残った28条件を対象に、
別ルート `time_corner_gap2_20260916` で最大12走行、1周最大3イベントの追加収集を行う。
実行先は graneple@192.168.3.10、ROS_DOMAIN_ID 1/2 とネットワーク名前空間を分離した2台。
AWSIM本体を変更せず、目標速度5km/h、通常RVizの経路・外乱マーカー表示を維持する。
取得上限速度超過は既存の `record_actual_v1` に従い記録する。

## 準備経路の補正

C02/C05/C07は前回の実測オーバーシュートに合わせ、準備経路だけを内側へ補正する。
C06の20cm条件は、前回の同一地点付近で nominal_path が約8cm、
measured_normal が約28cmの横ずれを生んだため、中間の準備経路を選択可能にする。

`LargeRecoverySite.preparation_origin="blended_normal"` は、元の指令経路と
通常走行の実測軌跡のx/yを `preparation_normal_fraction` (無次元、0..1、既定0.5)
で補間する。0はnominal_path、1はmeasured_normalと同じ準備位置になる。
既存2モードの幾何形状、測定用の通常走行座標系、目標の位置・横ずれ・向きは変更しない。
生成した経路には従来の車体占有チェックと実走時の停止領域監視を適用する。
この補間は収集の準備にのみ使用し、教師には通常PPへ切替後の実測3秒軌道を使う。

C03の60cmは、長い横ずれ生成経路が地図占有チェックを通らず、短い経路では
切替地点で連続1秒の開始条件が揃わなかった。そこで `preparation_delay_m`
(0..6m、既定0m) を追加し、準備コントローラへの切替後も前半は元の経路を進み、
後半の6m以上で横ずれを生成できるようにする。開始時の空間余裕・安定時間の条件は
維持し、地図チェック対象の区間や準備時間の制限も元の切替位置から計算する。
既定0mでは旧経路と同一。目標地点の移動や判定基準の緩和は行わない。

## 実行と検証

Windowsで変更をコミットし、`python tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py sync`
で同じコミットをWSLへ同期する。WSLのネイティブ作業ツリーで、
`tools/with_wsl_training_lock.sh .venv/bin/python -m pytest -q`
を実施する。同期・教師検証を同時に走らせない。

収集の操作スクリプトは `tmp/time_corner_gap2_20260916`。
`ops_corner.py setup`、`run_native_script.py adaptive_plan_v3.py ...`、
`start_monitor.py --pair N`、`collect_pair.py --pair N --left PLAN --right PLAN`、
`audit_pair.py --pair N` の順で、走行・転送照合・教師作成を行う。
2走行ごとにファイルハッシュ・ディレクトリ構成・SQLiteを検証してWSLへ移し、
確認済みの当該リモート生データのみ削除する。旧収集は保全する。

充足判定は従来どおり、完走した同一イベント内に有効教師60点以上と、
元の横ずれ±5cm・向き±1度の条件を満たす開始教師3点以上を要求する。
run単位でdomain 1をtrain、domain 2をvalidationへ割り当てる。
最終実測結果と未取得理由は収集後に追記する。
