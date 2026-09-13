# 時間基準モデルのROS接続とAWSIM確認

ユーザー依頼: 学習済み指令履歴OFFモデルをROSへ接続し、
`graneple@192.168.3.10`でAWSIMテストを実施する。
Windows正本で編集・コミットし、WSLでテスト後、指定ホストの専用ディレクトリへ配布する。
既存のdirtyな`/home/graneple/git/autononous_ai` checkoutと過去試行を保全する。

## 実装範囲と確認条件

既存V3 loaderは30点の時間モデルcheckpointを読めず、ROS履歴には学習時の50ms入力確定がない。
この二点を専用loaderと共通`assemble_time_inputs`を使うROS入力バッファで接続する。
既存モデル・学習契約は変更しない。教師・map poseはモデル入力に含めない。
画像受信から50ms後に使用可能な履歴だけを固定し、推論は購読処理を止めない別workerで行う。
標準`nav_msgs/Path`で通常RVizへ生30点を表示する。

必要な変更はruntime接続・明示的座標変換・試験用起動に限定する。
共通入力とのtensor一致、遅延到着除外、時計巻戻り、NaN/期限切れ、座標変換と
Pure Pursuit計算をWSLで検証し、HumbleでROSメッセージの往復を確認する。
元コードと重みは保持し、専用プロセス停止で切り戻せる。

制御は独立プロセスの50ms wall watchdog、唯一のcommand publisher確認、
sensor/plan期限、停止距離監視、操舵角・角速度制限を維持する。
最初の走行は最大10 sim秒、速度上限0.25m/s・過速度停止0.45m/s、
外側120 wall秒の有限試験。終了時は実測速度0.03m/s未満が1 sim秒継続したことを確認する。
安全な準備ができない場合はshadow/接続診断として記録し、走行成功としない。

## 現在の先行障害

2026-09-13の実行先確認:

- SSH接続可能、既存の実行中AWSIM/ROSコンテナなし。過去の停止済みコンテナは保全。
- CPUの既存Humble imageは起動可能、PyTorch `2.3.1+cu121`。
- GPU Docker起動は`/run/nvidia-persistenced/socket`不存在で失敗。
- `nvidia-smi`: `Failed to initialize NVML: Driver/library version mismatch`。
- 読込済みkernel moduleは`595.84`、NVML libraryは`595.91`。
- ホスト再起動・driver/service変更は未実施。ROS接続実装・CPU検証を先行する。

## 座標の根拠

現ホストAWSIM `AWSIM_Data/level1`のSHA-256は
`9ab2e1e8865c02885594e0bbdde372302530f047b5a2e89c455be6a18f3e090b`で既存抽出証拠と一致。
[既存の座標監査](spatial_path_v4_sim_continuation_gate.md)ではGoKart1のbase_linkは
root前後方向-0.485m、後輪中心は-0.484m。平面のrear axleはbase_link前方約0.001m。
wheelbaseは1.087mで現ホストvehicle_infoとも一致する。
これは静的設計値であり、動的な精度評価とは区別する。校正値は試験設定に明記し、既定で同一視しない。

## 現在の結果

ROS接続実装済み。共通入力一致・時刻/座標・制御のWSL限定テスト20 passed。
実装`fc381a2`の全体WSLテスト2,015 passed / 4 skipped / 63 warnings（128.44秒）。
Humble実checkpointのPath転送6件完全一致、合成oracleのshadow制御111件、
plan失効brake11件、clock停止brake7件、vehicle command publisherなし。
これは隔離合成ROS試験であり、AWSIM走行ではない。

GPU不整合はユーザーが画面側で再起動後に解消。GPU Docker CUDA=True、driver 595.91.07。
元boot ID `b3fd6d92-e22c-4145-8416-2e73ffb72cae`から
`89fc0b1b-6809-42c0-9a88-7ca1ebe23858`へ変更。
SSHからの再起動はsudo/対話認証で拒否され、アシスタントからの再起動は未実行。

trial01: 再起動後のXorg/Xwayland差でDISPLAY_AUTH_UNKNOWN、起動/駆動なし。
現デスクトップはXorg `:1`、認証パス`/run/user/1000/gdm/Xauthority`と確認して修正。
trial02: AWSIM実入力で121 plan、推論側rejected2、故障なし。Start helperは
`startup race-arm evidence did not become exact false`で失敗し、駆動認可なし。
先行原因はautoware-commandだけが旧DDS mountを参照する設定不一致。
次の試行では3サービスのDDS設定を統一し、公式Startの確認条件は保全する。
過去試行はすべて専有ディレクトリに保全し、所有した実行コンテナは停止・down済み。

隔離DDSはunicast peerとParticipantIndex autoを明示した。
[Cyclone DDS公式設定資料](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)に従い、
ホストsysctlを変更せず試験専用設定を使用する。
