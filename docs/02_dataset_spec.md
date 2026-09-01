# 02. Dataset仕様

## 1. データ利用方針

公開手動走行データは、通常走行・コース追従の事前学習に使う。
障害物回避、完全閉塞停止、回避後復帰が不足する場合は、追加手動走行、MPC、ルールベース教師で補う。

採用前に次を確認する。

- ライセンスまたは明示的な利用許諾
- 学習利用、競技利用、派生重み利用、再配布の範囲
- 収録環境・車両・センサ仕様
- topic、型、周期、時刻基準
- 完走、衝突、コースアウト、停止、追い越しの有無
- データに含まれる個人情報や第三者権利

## 2. Canonical dataset構造

```text
dataset_root/
├── index.csv
├── metadata.yaml
├── images/
│   ├── run_0001_000000.jpg
│   └── ...
├── lidar/
│   ├── run_0001_000000.npy
│   └── ...
└── optional/
    ├── occupancy/
    ├── debug/
    └── videos/
```

## 3. 1サンプルの意味

観測時刻`t_obs`に対し、将来軌道と速度を予測する。
手動操作の反応遅れを考慮し、直接制御ラベルには`label_shift_ms`を持たせる。

```text
observation: camera[t_obs], lidar[t_obs], ego[t_obs]
trajectory label: future pose[t_obs + 0.5 ... 3.0s]
direct control label: control[t_obs + delta_human]
```

`delta_human`は固定しない。0/100/200/300msなどを比較し、validationとclosed-loopで選ぶ。

## 4. index.csv列

必須列は `schemas/index_columns.md` を参照。
代表例：

```csv
sample_id,run_id,scenario_id,timestamp_ns,image_path,lidar_path,velocity_mps,steering_rad,heading_rate_rps,gear,wp_0_x,wp_0_y,wp_1_x,wp_1_y,wp_2_x,wp_2_y,wp_3_x,wp_3_y,wp_4_x,wp_4_y,wp_5_x,wp_5_y,target_speed_mps,stop_flag,behavior_mode,direct_steering_rad,direct_acceleration_mps2,collision,offtrack,quality_score
```

## 5. 同期ポリシー

- 基準時刻はCamera stampまたは明示したmaster clockとする。
- Camera/LiDAR/Ego/Controlごとにnearest-neighborまたはprevious-valueを使うか明示する。
- 将来情報を入力側へ混入させない。
- 許容差を超えたサンプルは無理に補完せず、invalidとして記録する。
- `camera_dt_ms`, `lidar_dt_ms`, `ego_dt_ms`, `control_dt_ms`を監査ログへ残す。

初期許容差候補：

| Stream | tolerance |
|---|---:|
| Camera | 50ms |
| LiDAR | 50ms |
| Ego state | 50ms |
| Direct control label | label shift適用後100ms |

公式環境の実周期に合わせて更新する。

## 6. LiDAR保存

`.npy`のfloat32配列を推奨する。

- shape: `[1080]`を初期想定
- 単位: m
- `NaN`, `inf`, 0, range外はraw値を残す場合でもvalid maskを保存する
- 学習時は`range_max`へ置換し、0〜1へ正規化する

## 7. Label

### future waypoints

- frame: ego/base_link
- x: 車両前方正
- y: 左正を推奨
- horizon: 3.0s
- point数: 6
- point時刻: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0sなど

### target speed

未来区間の代表速度とする。候補：

- horizon終端の速度
- horizon内の安全速度
- 直近0.5s平均速度

選択をconfigとmetadataに記録する。

### stop flag

停止ラベルは単なる`速度≈0`では不十分。
次の区別を持つ。

- intentional_stop: 障害物・閉塞・安全判断による停止
- incidental_zero_speed: 開始前、終了後、スタック、操作待ち
- invalid_stop: 衝突後、コースアウト後

### behavior mode

初期クラス：

- follow
- avoid_left
- avoid_right
- overtake
- return_to_center
- stop

modeが信頼できない場合、v0では補助Headを無効化してよい。

## 8. Quality gate

| 条件 | 処理 |
|---|---|
| image/lidar欠損 | 除外 |
| timestamp tolerance超過 | 除外またはlow quality |
| collision/offtrack後 | 原則除外 |
| 急激な手動修正 | quality低下、別bucket |
| steering/accel範囲外 | 除外・要確認 |
| 長時間静止 | stop理由により分類 |
| 重複フレーム | deduplicate |
| 同一runが複数splitへ混入 | 禁止 |

## 9. Split

frameランダムsplitは禁止。

推奨：

```text
train: run/scenario A, B, C
validation: 未使用run、既知scenarioの別seed
closed-loop test: 未見障害物位置、未見速度、未見run
```

driverが複数いる場合はdriver IDもsplit条件に含め、特定ドライバへの過適合を確認する。

## 10. 同梱MCAPのconverter

`tools/convert_mcap_dataset.py`は次のtopicを使用する。

| Role | Topic | 同期方法 |
|---|---|---|
| observation master | `/sensing/camera/image_raw` | 10 Hzへ間引き |
| observation LiDAR | `/sensing/lidar/scan` | Cameraにnearest |
| ego proxy | `/control/command/control_cmd` | Camera以前のlatest |
| teacher-only label | `/control/command/control_cmd` | future command列 |

保存画像はRGB JPEG、LiDARはfloat32 `[1080]`である。収録LaserScanは750点のため、
角度index上のnearestで1080点へresamplingする。NaN/infは補間せず保持し、
`DrivingDataset`のLiDAR preprocessingでinvalidとして処理する。

収録bagには実測odometry/poseがないため、以下は真値ではなくproxy labelである。

- `velocity_mps`、`steering_rad`: 観測時刻以前の最新制御指令
- `heading_rate_rps`: 指令速度・指令操舵・指定wheelbaseから算出
- future waypoints: 将来制御指令をkinematic bicycleで積分
- target speed: 観測から指定offset後の指令速度
- stop flag: target speedが閾値以下
- behavior mode: stopまたはfollowのみ

この制約は生成datasetの`metadata.yaml`にも記録する。特にbehavior modeは
avoidanceを識別できないため、このproxy datasetで学習する際はmode lossを0にする。
wheelbaseは暗黙の既定値を持たず、converter実行時にm単位で必須指定する。

同梱24 runのうち21 runは`speed`指令が常に0であり、`acceleration`指令だけが
変化する。実測速度topicがない以上、これらから速度・軌跡の教師真値を復元できない。
converterは既定で、最大指令速度が`--min-usable-commanded-speed-mps`未満のrunを
丸ごと除外する。現データでcanonical datasetへ採用されるのは、非ゼロ速度指令を持つ
`vehicle_x1_mpc_1`〜`3`の3 runである。除外runを学習へ戻すには、公式環境から実測
odometry/poseを追加取得するか、根拠を明記した別のteacher trajectoryが必要である。
