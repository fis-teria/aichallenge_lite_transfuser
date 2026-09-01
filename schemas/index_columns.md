# index.csv Contract

## 必須列

| Column | Type | Unit/Meaning |
|---|---|---|
| sample_id | str | dataset内で一意 |
| run_id | str | split単位 |
| scenario_id | str | scenario分類 |
| timestamp_ns | int | observation基準時刻 |
| image_path | str | index.csvからの相対path可 |
| lidar_path | str | float32 `.npy` |
| velocity_mps | float | longitudinal velocity |
| steering_rad | float | current steering tire angle |
| heading_rate_rps | float | yaw/heading rate、なければ0とmetadataに記載 |
| gear | float/int | normalized前のgear値 |
| target_speed_mps | float | 学習label |
| stop_flag | 0/1 | intentional stop |
| behavior_mode | int/str | class mappingはmetadata/config |

## Waypoint列

`num_waypoints=N`のとき：

```text
wp_0_x, wp_0_y, ... wp_{N-1}_x, wp_{N-1}_y
```

単位m、ego/base frame、x前方正、y左正を推奨する。

## Optional列

- direct_steering_rad
- direct_acceleration_mps2
- collision
- offtrack
- quality_score
- camera_dt_ms
- lidar_dt_ms
- ego_dt_ms
- control_dt_ms
- source_dataset
- driver_id
- weather_id
- obstacle_min_distance_m
- split

## 禁止事項

- 同じrunをtrain/validation/testへ分割しない。
- future poseやGT obstacleをmodel input列として扱わない。
- 不明な0値で欠損を隠さない。missing flagまたはmetadataに明記する。
