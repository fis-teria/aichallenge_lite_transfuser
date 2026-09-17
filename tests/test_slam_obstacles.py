import math
from pathlib import Path
import numpy as np
import pytest

from aic_transfuser_lite.runtime.slam_obstacles import (
    RecentOccupancy, SlamObstacleDetector, cartographer_stamp_ns, path_clearance,
    scan_geometry, slam_scan_pose,
)
from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose


def test_cartographer_scan_timestamp_quantization_is_not_a_pose_age_tolerance():
    pose = TimedBodyPose(1_000_000_000, 'sim', '0', 'time_slam_map', 'time_slam_lidar', 1., 2., .3)
    for delta in (-50, -27, 0, 27, 49):
        original = pose.stamp_ns+delta
        joined = slam_scan_pose([pose], original)
        assert joined.stamp_ns == original and joined.x_m == pose.x_m
    assert cartographer_stamp_ns(1_000_000_050) == 1_000_000_100
    for delta in (-51, 50, 10_000_000):
        with pytest.raises(ValueError, match='OBSERVATION_POSE_MISSING'):
            slam_scan_pose([pose], pose.stamp_ns+delta)
    with pytest.raises(ValueError, match='STAMP_CONTRACT'):
        cartographer_stamp_ns(-1)


def corridor_scan(box_x=None):
    angle = np.linspace(-1.5, 1.5, 750)
    c, s = np.cos(angle), np.sin(angle)
    ranges = np.minimum(18/c, 3/np.maximum(np.abs(s), 1e-9))
    if box_x is not None:
        front = box_x/c
        hit = np.abs(front*s) <= .6
        ranges[hit] = np.minimum(ranges[hit], front[hit])
    return ranges


def test_scan_contract_filters_bad_returns_and_preserves_infinity_free_rays():
    r = np.full(20, np.inf); r[0] = np.nan; r[2] = -1.; r[4] = 3.
    p, hit, ids = scan_geometry(r, -1., .1, 0., 25.)
    assert np.isfinite(p).all() and p.shape[1] == 2 and len(hit) == len(ids)
    assert 0 not in ids and 2 not in ids
    assert np.max(np.linalg.norm(p, axis=1)) <= 20.00001
    with pytest.raises(ValueError, match='MALFORMED'):
        scan_geometry(np.full(20, np.nan), 0., .1, 0., 25.)
    with pytest.raises(ValueError, match='CONTRACT'):
        scan_geometry(np.ones((3, 2)), 0., .1, 0., 25.)


def test_stationary_obstacle_is_not_absorbed_into_slam_background():
    core = SlamObstacleDetector(); path = np.column_stack((np.linspace(0., 10., 30), np.zeros(30)))
    for i in range(30):
        result = core.update(1_000_000_000+i*100_000_000, corridor_scan(5.), -1.5, 3/749,
                             0., 25., np.zeros(3), path)
        assert result['path_blocked'] is True
    candidates = [s for s in result['surfaces'] if s['path_overlap']]
    assert candidates and any(s['confirmed'] for s in candidates)
    assert result['nearest_path_obstacle_m'] == pytest.approx(6.65, abs=.1)
    assert result['motion_authority'] is False
    assert not result['semantic_vehicle_classification']
    # Same map, removed object: no ghost obstacle from stored occupied cells.
    result = core.update(4_100_000_000, corridor_scan(), -1.5, 3/749, 0., 25., np.zeros(3), path)
    assert result['path_blocked'] is False


def test_corridor_walls_remain_occupied_but_do_not_block_center_path():
    core = SlamObstacleDetector(); path = np.column_stack((np.linspace(0., 10., 30), np.zeros(30)))
    result = core.update(100, corridor_scan(), -1.5, 3/749, 0., 25., np.zeros(3), path)
    assert result['occupied_points'] > 100 and result['surfaces']
    assert result['path_valid'] and not result['path_blocked']
    assert np.any(core.grid.values == 100) and np.any(core.grid.values == 0) and np.any(core.grid.values == -1)
    unknown = core.update(200, corridor_scan(), -1.5, 3/749, 0., 25., np.zeros(3))
    assert unknown['path_blocked'] is None and unknown['path_valid'] is False


def test_occupancy_ray_clear_ttl_and_rolling_origin():
    grid = RecentOccupancy()
    grid.update(1., np.zeros(2), np.array([[3., 0.]]), np.array([True]))
    def value(x):
        ix, iy = np.floor(np.array([x, 0.])/grid.resolution_m).astype(int)-grid.origin
        return grid.values[iy, ix]
    assert value(3.) == 100
    grid.update(1.1, np.array([.4, 0.]), np.array([[8., 0.]]), np.array([True]))
    assert value(3.) == 0 and value(8.) == 100
    grid.update(4., np.array([.4, 0.]), np.empty((0, 2)), np.empty(0, dtype=bool))
    assert value(8.) == -1
    grid.update(4.1, np.array([100., 100.]), np.empty((0, 2)), np.empty(0, dtype=bool))
    assert np.all(grid.values == -1)


def test_pose_transform_and_path_geometry_are_metric_and_stamp_reset_is_explicit():
    point = np.array([[0., 5.], [4., 0.]])
    distance, arc = path_clearance(point, np.array([[0., 3.], [0., 6.]]))
    assert distance[0] == pytest.approx(0.) and arc[0] == pytest.approx(5.)
    assert distance[1] == pytest.approx(4.)
    core = SlamObstacleDetector()
    result = core.update(100, corridor_scan(5.), -1.5, 3/749, 0., 25., np.array([10., 20., math.pi/2]))
    np.testing.assert_allclose(result['base_pose_xyyaw'], [10., 18.35, math.pi/2])
    with pytest.raises(ValueError, match='STAMP'):
        core.update(99, corridor_scan(), -1.5, 3/749, 0., 25., np.zeros(3))
    with pytest.raises(ValueError): path_clearance(np.zeros((2, 3)), np.zeros((3, 2)))
    with pytest.raises(ValueError): path_clearance(np.zeros((2, 2)), np.full((3, 2), np.nan))


def test_slam_profile_uses_scan_and_odometry_without_global_inputs():
    root = Path(__file__).resolve().parents[1]
    config = (root/'ros2_ws/src/aic_e2e_runtime/config/time_slam_2d.lua').read_text()
    assert 'use_odometry = true' in config
    assert 'use_nav_sat = false' in config and 'use_imu_data = false' in config
    import yaml
    rviz = yaml.safe_load((root/'ros2_ws/src/aic_e2e_runtime/config/slam_obstacles.rviz').read_text())
    manager = rviz['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'time_slam_map'
    topics = {d['Topic']['Value'] for d in manager['Displays']}
    assert all(t.startswith('/time_path/slam/') for t in topics)
