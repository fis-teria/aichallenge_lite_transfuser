import json
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.control.time_reference_v1 import TimedBodyPose
from aic_transfuser_lite.evaluation.time_clearance_v1 import PoseIndex, project_to_polyline, scan_margin

spec = importlib.util.spec_from_file_location("compare_time_clearance", Path(__file__).parents[1]/"tools/compare_time_teacher_clearance.py")
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


def pose(t, x=0., yaw=0.):
    return TimedBodyPose(t, "sim", "0", "map", "base_link", x, 0., yaw)


def test_capture_interpolation_wrap_and_ambiguous_bracket():
    index = PoseIndex([pose(0, yaw=3.13), pose(20_000_000, 1., -3.13)])
    actual = index.at(10_000_000)
    assert actual.x_m == .5
    assert actual.yaw_rad == pytest.approx(np.pi)
    with pytest.raises(ValueError, match="OBSERVATION_POSE_MISSING"):
        index.at(21_000_000)
    duplicate = PoseIndex([pose(0), pose(20_000_000), pose(20_000_000, 2.)])
    with pytest.raises(ValueError, match="AMBIGUOUS_POSE_STAMP"):
        duplicate.at(10_000_000)
    with pytest.raises(ValueError, match="OBSERVATION_POSE_MISSING"):
        PoseIndex([pose(0), pose(200_000_000)]).at(100_000_000)


@pytest.mark.parametrize("point,sign", [([.5, 2.], 1), ([.5, -2.], -1)])
def test_polyline_left_sign_is_rotation_invariant(point, sign):
    path = np.array([[0., 0.], [0., 0.], [1., 0.]])
    for angle in [0., .7, -2.]:
        c, s = np.cos(angle), np.sin(angle)
        rotation = np.array([[c, s], [-s, c]])
        r = project_to_polyline(np.array(point) @ rotation, path @ rotation)
        assert r["distance_m"] == pytest.approx(2.)
        assert r["left_distance_m"] == pytest.approx(sign*2.)


@pytest.mark.parametrize("point,path", [([0], [[0, 0], [1, 0]]),
    ([0, 0], [[0, 0], [0, 0]]), ([np.nan, 0], [[0, 0], [1, 0]])])
def test_polyline_rejects_invalid_input(point, path):
    with pytest.raises(ValueError):
        project_to_polyline(point, path)


def test_exact_frozen_rejection_and_unknown_are_distinct():
    f = json.loads((Path(__file__).parent / "fixtures/time_path/turn16_side_margin_rejection.json").read_text())
    kwargs = dict(speed_mps=f["speed_mps"], measured_rad=f["measured_steer_rad"],
        issued_rad=f["issued_steer_rad"], previous_rad=f["previous_steer_rad"],
        yaw_rate_radps=f["motion_observation"]["heading_rate_radps"],
        lateral_mps=f["motion_observation"]["reported_lateral_mps"])
    r = scan_margin(f["scan"], f["scan_in_current_rear"], **kwargs)
    assert r["reason"] == "STOPPING_SWEEP_OCCUPIED"
    assert r["minimum_ray_margin_m"] == pytest.approx(-.004498958185470858, abs=1e-10)
    f["scan"]["ranges"][0] = float("nan")
    r = scan_margin(f["scan"], f["scan_in_current_rear"], **kwargs)
    assert r == {"reason": "SCAN_UNKNOWN", "minimum_ray_margin_m": None}
    f["scan"]["ranges"] = ["inf"]*len(f["scan"]["ranges"])
    r = scan_margin(f["scan"], f["scan_in_current_rear"], **kwargs)
    assert r["reason"] == "PASS"
    assert 0 < r["minimum_ray_margin_m"] < f["scan"]["range_max"]


def test_world_transform_keeps_left_positive_and_explicit_shape():
    p = TimedBodyPose(0, "sim", "0", "map", "base_link", 10., 20., np.pi/2)
    assert comparison.to_world(np.array([[1., 2.]]), p) == pytest.approx(np.array([[8., 21.]]))
    with pytest.raises(ValueError, match="WORLD_POINTS"):
        comparison.to_world(np.array([1., 2.]), p)


def test_following_uses_future_capture_and_restores_rear_offset(tmp_path):
    obs = [{"role": "pose", "stamp_ns": t, "epoch": "0", "frame": "map", "child_frame": "base_link",
            "position_xyz_m": [x, .2, 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
           for t, x in [(990_000_000, .99), (1_010_000_000, 1.01)]]
    (tmp_path/"vehicle_observations.jsonl").write_text("\n".join(json.dumps(r) for r in obs))
    from dataclasses import asdict
    row = {"sim_ns": 0, "details": {"current_pose": asdict(pose(0)),
        "reference_xy_rear_m": [[-comparison.REAR_OFFSET_M, 0.], [3.-comparison.REAR_OFFSET_M, 0.]]}}
    result = comparison.following_error(tmp_path, [row], 2_000_000_000)
    assert result["distance_m"]["count"] == 1
    assert result["distance_m"]["max"] == pytest.approx(.2)
    assert result["excluded"] == {}
