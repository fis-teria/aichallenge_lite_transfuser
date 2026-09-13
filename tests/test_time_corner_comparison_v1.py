import math

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_clearance_v1 import PoseIndex
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import (
    RecordedLine, angle_delta, future_tracking_error, replay_observation_pose, world_points)


def pose(t, x=0., y=0., yaw=0., epoch="0"):
    return TimedBodyPose(t, "sim", epoch, "map", "base_link", x, y, yaw)


def line():
    return RecordedLine([pose(i*20_000_000, i*.02) for i in range(101)])


def test_projection_sign_progress_heading_and_rotation():
    for yaw in [0., .8, -2.1]:
        p = pose(0, 100., 200., yaw)
        xy = world_points(np.array([[i*.02, 0.] for i in range(101)]), p)
        reference = RecordedLine([pose(i*20_000_000, *point, yaw) for i, point in enumerate(xy)])
        query = world_points(np.array([[.75, -.4]]), p)[0]
        result = reference.project(query, yaw_hint_rad=yaw)
        assert result.left_m == pytest.approx(-.4)
        assert result.progress_m == pytest.approx(.75)
        assert result.stamp_ns == pytest.approx(750_000_000, abs=1)
        assert angle_delta(result.body_yaw_rad, yaw) == pytest.approx(0.)


def test_projection_rejects_wrong_direction_distance_and_endpoints():
    reference = line()
    with pytest.raises(ValueError, match="NO_MATCHING_SEGMENT"):
        reference.project(np.array([1., 0.]), yaw_hint_rad=math.pi)
    with pytest.raises(ValueError, match="REFERENCE_TOO_FAR"):
        reference.project(np.array([1., 4.]))
    with pytest.raises(ValueError, match="REFERENCE_ENDPOINT"):
        reference.project(np.array([3., .2]))
    with pytest.raises(ValueError, match="REFERENCE_ENDPOINT"):
        reference.project(np.array([-.5, .2]))


def test_progress_bounds_exclude_another_pass():
    reference = line()
    result = reference.project(np.array([1.2, .3]), progress_bounds_m=(1., 1.5))
    assert result.progress_m == pytest.approx(1.2)
    with pytest.raises(ValueError, match="INVALID_PROGRESS_BOUNDS"):
        reference.project(np.zeros(2), progress_bounds_m=(1., 0.))


def test_wrapped_body_yaw_and_conflicting_capture_stamps():
    reference = RecordedLine([pose(0, 0., yaw=3.13), pose(20_000_000, -.02, yaw=-3.13)])
    assert abs(reference.project(np.array([-.01, .1])).body_yaw_rad) == pytest.approx(math.pi)
    with pytest.raises(ValueError, match="NO_VALID_MOVING_SEGMENTS"):
        RecordedLine([pose(0), pose(20_000_000, .02), pose(20_000_000, .03)])
    with pytest.raises(ValueError, match="NO_VALID_MOVING_SEGMENTS"):
        RecordedLine([pose(0), pose(100_000_000, .1)])
    with pytest.raises(ValueError, match="POSE_IDENTITY_MISMATCH"):
        RecordedLine([pose(0), pose(20_000_000, .02, epoch="1")])


@pytest.mark.parametrize("bad", [np.array([0.]), np.array([0., np.nan])])
def test_invalid_query_shape_and_finite(bad):
    with pytest.raises(ValueError, match="POINT_SHAPE_OR_NONFINITE"):
        line().project(bad)


def test_future_separates_speed_error_from_geometric_following():
    raw = np.column_stack([np.arange(1, 31)*.1, np.zeros(30)])
    index = PoseIndex([pose(990_000_000, 1.49, .03), pose(1_010_000_000, 1.51, .03)])
    result = future_tracking_error(raw, pose(0), index, 1., before_ns=2_000_000_000)
    assert result["point_along_m"] == pytest.approx(.5)
    assert result["point_left_m"] == pytest.approx(.03)
    assert result["polyline_distance_m"] == pytest.approx(.03)
    assert not result["polyline_endpoint"]
    with pytest.raises(ValueError, match="FUTURE_AFTER_FIRST_FAULT"):
        future_tracking_error(raw, pose(0), index, 1., before_ns=999_999_999)


def test_future_does_not_bridge_missing_pose_or_change_epoch():
    raw = np.column_stack([np.arange(1, 31)*.1, np.zeros(30)])
    with pytest.raises(ValueError, match="OBSERVATION_POSE_MISSING"):
        future_tracking_error(raw, pose(0), PoseIndex([pose(0), pose(2_000_000_000, 2.)]), 1., before_ns=2_000_000_000)
    with pytest.raises(ValueError, match="POSE_IDENTITY_MISMATCH"):
        future_tracking_error(raw, pose(0), PoseIndex([pose(1_000_000_000, 1., epoch="1")]), 1., before_ns=2_000_000_000)


def test_future_uses_observation_frame_and_no_extrapolation():
    raw = np.column_stack([np.arange(1, 31)*.1, np.zeros(30)])
    observed = pose(0, 10., 20., math.pi/2)
    result = future_tracking_error(raw, observed, PoseIndex([pose(3_000_000_000, 10., 24., math.pi/2)]), 3., before_ns=3_000_000_000)
    assert result["point_distance_m"] == pytest.approx(1.)
    assert result["polyline_endpoint"]
    with pytest.raises(ValueError, match="FUTURE_SHAPE_OR_HORIZON"):
        future_tracking_error(raw[:29], observed, line().index, 1., before_ns=3_000_000_000)


def test_replay_teacher_observation_uses_declared_receipt_eligible_sources():
    sources = [(pose(0, 10., yaw=3.13), 1000), (pose(20_000_000, 10.02, yaw=-3.13), 1030)]
    actual = replay_observation_pose(sources, observation_ns=10_000_000, freeze_receipt_ns=1040)
    assert actual.x_m == pytest.approx(10.01)
    assert actual.yaw_rad == pytest.approx(math.pi)
    with pytest.raises(ValueError, match="NOT_AVAILABLE_AT_FREEZE"):
        replay_observation_pose(sources, observation_ns=10_000_000, freeze_receipt_ns=1020)
    with pytest.raises(ValueError, match="DUPLICATE_SOURCE_STAMP"):
        replay_observation_pose([(pose(0), 1000), (pose(0, 99.), 1001)], observation_ns=0, freeze_receipt_ns=1040)
    with pytest.raises(ValueError, match="OBSERVATION_POSE_MISSING"):
        replay_observation_pose(sources, observation_ns=100_000_000, freeze_receipt_ns=1040)


def test_teacher_anchor_adapter_reads_odometry_ids_not_display_pose():
    import importlib.util
    from pathlib import Path
    import sys
    from types import SimpleNamespace as NS
    tools_path = str(Path(__file__).parents[1]/"tools")
    sys.path.insert(0, tools_path)
    try:
        spec = importlib.util.spec_from_file_location("corner_comparison", Path(tools_path)/"compare_time_corner_tracking.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(tools_path)
    class FakeBag:
        def message(self, row_id):
            assert row_id == 42
            return NS(__msgtype__="nav_msgs/msg/Odometry", child_frame_id="base_link",
                header=NS(frame_id="map", stamp=NS(sec=1, nanosec=0)),
                pose=NS(pose=NS(position=NS(x=3., y=4.), orientation=NS(x=0., y=0., z=0., w=1.)))), 2000
    actual = module.anchor_observation(FakeBag(), {"observation_pose_row_ids": [42], "observation_ns": 1_000_000_000, "freeze_ns": 2100})
    assert (actual.x_m, actual.y_m) == (3., 4.)
