import math

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_corner_comparison_v1 import RecordedLine, world_points
from aic_transfuser_lite.evaluation.time_path_attribution_v1 import lateral_decomposition, reference_preview


def pose(t=0, x=0., y=0., yaw=0.):
    return TimedBodyPose(t, "sim", "0", "map", "base_link", x, y, yaw)


@pytest.mark.parametrize("yaw", [0., 1.2, -3.13])
def test_common_progress_separates_prediction_and_following_with_rotation(yaw):
    origin = pose(x=100., y=200., yaw=yaw)
    predicted = world_points(np.array([[0., .7], [2., .8], [4., .9]]), origin)
    actual = world_points(np.array([[1.5, .8]]), origin)[0]
    reference = world_points(np.array([[1.5, 0.]]), origin)[0]
    result = lateral_decomposition(predicted, np.array([0., 2., 4.]), actual, reference, yaw, 1.5)
    assert result["actual_left_m"] == pytest.approx(.8)
    assert result["prediction_left_m"] == pytest.approx(.775)
    assert result["following_left_m"] == pytest.approx(.025)
    assert abs(result["sum_error_m"]) < 1e-12


def test_progress_sampling_excludes_longitudinal_speed_error():
    result = lateral_decomposition(np.array([[0., -.2], [4., -.2]]), np.array([0., 4.]),
                                   np.array([3., -.21]), np.array([3., 0.]), 0., 3.)
    assert result["following_left_m"] == pytest.approx(-.01)
    assert result["matched_prediction_world_xy_m"] == pytest.approx([3., -.2])


@pytest.mark.parametrize("s,station,reason", [
    ([0., 0.], 0., "NOT_INCREASING"), ([2., 1.], 1.5, "NOT_INCREASING"),
    ([0., 1.], 1.001, "OUTSIDE"), ([0., 1.], -.001, "OUTSIDE"),
    ([0., float("nan")], .5, "NONFINITE"), ([0.], .5, "SHAPE")])
def test_decomposition_rejects_unsupported_progress_without_clipping(s, station, reason):
    with pytest.raises(ValueError, match=reason):
        lateral_decomposition(np.array([[0., 0.], [1., 0.]]), np.array(s),
                              np.array([.5, 0.]), np.array([.5, 0.]), 0., station)


@pytest.mark.parametrize("yaw", [0., .8, -2.])
def test_same_pp_geometry_points_back_towards_measured_line(yaw):
    origin = pose(x=100., y=200., yaw=yaw)
    xy = world_points(np.column_stack([np.arange(601)*.01, np.zeros(601)]), origin)
    reference = RecordedLine([pose(i*10_000_000, *p, yaw) for i, p in enumerate(xy)])
    current_xy = world_points(np.array([[1., .3]]), origin)[0]
    current = pose(50_000_000_000, *current_xy, yaw)
    result = reference_preview(reference, current, speed_mps=1.2, remaining_horizon_s=2.7,
                               rear_axle_offset_m=.001)
    assert -.3 <= result["required_tire_rad"] < 0.
    assert result["selection"]["xy_m"][1] == pytest.approx(-.3, abs=1e-6)
    assert result["selection"]["remaining_s"] <= 2.7
    assert "NOT_TIMEPLAN" in result["scope"]
    np.testing.assert_array_equal(reference.xy, xy)
    with pytest.raises(ValueError, match="FEASIBLE_LOOKAHEAD_MISSING"):
        reference_preview(reference, current, speed_mps=1.2, remaining_horizon_s=.1,
                          rear_axle_offset_m=.001)


def test_reference_preview_preserves_missing_future_and_units_contract():
    reference = RecordedLine([pose(i*10_000_000, i*.01) for i in range(101)])
    with pytest.raises(ValueError, match="OBSERVATION_POSE_MISSING"):
        reference_preview(reference, pose(x=.9), speed_mps=1., remaining_horizon_s=3.,
                          rear_axle_offset_m=.001)
    with pytest.raises(ValueError, match="REFERENCE_PREVIEW_CONTRACT"):
        reference_preview(reference, pose(x=.5), speed_mps=1., remaining_horizon_s=3.1,
                          rear_axle_offset_m=.001)
