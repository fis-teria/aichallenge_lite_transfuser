from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/aic_lidar_v2x"))
from aic_lidar_v2x.core import (Config, Detection, Detector, Pose2, Reference, Scan,
    StaticMap, Tracker, clusters, fit_vehicle_box, interpolate_pose, scan_points, v2x_payload)
from aic_lidar_v2x.io import PoseHistory, load_map, planar_pose


def detection(x: float, y: float = 0.0) -> Detection:
    return Detection((x, y), (x, y), (0.4, 0.5), 10, "surface", 0.15)


def test_se2_and_shortest_yaw_interpolation():
    pose = Pose2(90000, 43000, math.pi / 2)
    local = np.array([[1., 2.], [-2., 3.]])
    np.testing.assert_allclose(pose.inverse_apply(pose.apply(local)), local, atol=1e-10)
    assert abs(interpolate_pose(Pose2(0, 0, 3.1), Pose2(0, 0, -3.1), .5).yaw_rad - math.pi) < 1e-10
    assert pose.compose(Pose2(1, 0, 0)).y_m == 43001
    with pytest.raises(ValueError):
        pose.apply(np.array([1., 2.]))


def test_scan_removes_invalid_and_no_returns_and_deskews():
    scan = Scan(2., np.array([np.nan, np.inf, -np.inf, 0., 2., 25., 26., 3.]), 0., .01, 0., 25., .01)
    xy, origins = scan_points(scan, Pose2(0, 0, 0), Pose2(.7, 0, 0), Config())
    assert xy.shape == origins.shape == (2, 2)
    np.testing.assert_allclose(origins[:, 0], [.4, .7])
    still, _ = scan_points(scan, Pose2(0, 0, 0), Pose2(.7, 0, 0), Config(motion_model="snapshot"))
    np.testing.assert_allclose(xy[:, 0] - still[:, 0], [.4, .7])


@pytest.mark.parametrize("kwargs", [{"cluster_gap_m": 0}, {"track_ttl_s": float('nan')},
    {"confirmation_scans": 2.5}, {"object_model": "unknown"}, {"motion_model": "latest"}])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs)


def test_map_rotation_unknown_boundary_and_wall_dilation():
    free = np.ones((7, 7), dtype=bool)
    free[3, 3] = False
    pose = Pose2(100, 200, math.pi / 2)
    grid = StaticMap(free, 1., pose, 1.)
    xy = pose.apply(np.array([[1.5, 1.5], [3.5, 3.5], [3.5, 2.5], [.5, .5], [-.1, 2.]]))
    assert grid.is_interior(xy).tolist() == [True, False, False, False, False]


def test_map_loader_ros_image_orientation(tmp_path):
    Image.fromarray(np.array([[0, 255], [255, 205]], dtype=np.uint8)).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text("image: map.pgm\nresolution: 1.0\norigin: [0, 0, 0]\nnegate: 0\nfree_thresh: 0.196\noccupied_thresh: 0.65\n")
    grid = load_map(tmp_path / "map.yaml", 0)
    assert grid.is_interior(np.array([[.5, .5], [1.5, .5], [.5, 1.5], [1.5, 1.5]])).tolist() == [True, False, False, True]


def test_clusters_do_not_merge_separate_objects():
    xy = np.array([[0., 0.], [.1, 0.], [2., 2.], [2.1, 2.], [5., 5.]])
    assert sorted(len(c) for c in clusters(xy, .15)) == [1, 2, 2]
    assert clusters(np.empty((0, 2)), .15) == []


def test_box_fit_corrects_surface_to_vehicle_centre():
    # A rectangular vehicle, with visible rear and right-side surfaces.
    centre = np.array([10., 1.])
    back = np.column_stack([np.full(20, 9.), np.linspace(.5, 1.5, 20)])
    side = np.column_stack([np.linspace(9., 11., 20), np.full(20, .5)])
    points = np.vstack([back, side])
    origins = np.zeros_like(points)
    fit, rmse, _ = fit_vehicle_box(points, origins, 0., 2., 1.)
    np.testing.assert_allclose(fit, centre, atol=.08)
    assert rmse < .05


def test_box_fit_reports_ambiguity_for_partial_face():
    points = np.column_stack([np.full(10, 9.), np.linspace(.9, 1.1, 10)])
    _, error, ambiguity = fit_vehicle_box(points, np.zeros_like(points), 0., 2., 1.)
    assert error < .05
    assert ambiguity > .25


def test_tracking_id_velocity_confirmation_dropout_and_ttl():
    tracker = Tracker(Config(confirmation_scans=2))
    assert tracker.update([detection(10)], 1.) == []
    a = tracker.update([detection(10.1)], 1.1)
    key = a[0].track_id
    assert 0 < a[0].velocity_xy_mps[0] < 1.01
    assert tracker.update([], 1.2) == []
    assert tracker.update([detection(10.3)], 1.3) == []
    assert tracker.update([detection(10.4)], 1.4)[0].track_id == key
    assert tracker.update([detection(10.4)], 2.) == []
    assert tracker.update([detection(10.4)], 2.1)[0].track_id != key
    with pytest.raises(ValueError, match="Non-increasing"):
        tracker.update([], 2.1)
    tracker.reset()
    assert tracker.update([detection(0)], .1) == []


def test_one_to_one_tracking_and_no_native_id_collision():
    tracker = Tracker(Config(confirmation_scans=1))
    initial = tracker.update([detection(10), detection(12)], 1.)
    keys = {t.track_id for t in initial}
    found = tracker.update([detection(12.1), detection(10.1)], 1.1)
    assert {t.track_id for t in found} == keys
    assert len(keys) == 2 and all(k.startswith("lidar_") for k in keys)


def test_v2x_uncertainty_is_std_metres_not_squared_and_stamp_is_observation():
    tracker = Tracker(Config(confirmation_scans=1))
    tracks = tracker.update([detection(10)], 1.)
    msg = v2x_payload(tracks, 1.2)
    assert msg["vehicles"][0]["covariance"]["x"] == .15
    assert msg["vehicles"][0]["stamp_s"] == 1.
    assert msg["frame_id"] == "map"
    with pytest.raises(ValueError):
        v2x_payload(tracks, .5)
    assert v2x_payload([], 1.)["vehicles"] == []


def test_detection_rejects_bad_sensor_not_empty_road():
    cfg = Config()
    detector = Detector(cfg, StaticMap(np.ones((100, 100)), .5, Pose2(-25, -25, 0), .25))
    with pytest.raises(ValueError, match="malformed"):
        detector.detect(Scan(1., np.full(750, np.nan), -1.5, .004, 0., 25.),
                        Pose2(0, 0, 0), Pose2(0, 0, 0), Pose2(0, 0, 0))
    detections, _ = detector.detect(Scan(1., np.full(750, np.inf), -1.5, .004, 0., 25.),
                                    Pose2(0, 0, 0), Pose2(0, 0, 0), Pose2(0, 0, 0))
    assert detections == []
    with pytest.raises(ValueError, match="reference"):
        Detector(replace(cfg, object_model="known_vehicle"), detector.wall_map)


def test_reference_uses_geometry_and_planar_tf_rejects_tilt():
    reference = Reference(np.array([[0., 0.], [0., 1.], [0., 1.], [1., 1.]]))
    assert reference.yaw_at(np.array([0., .5])) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError, match="roll/pitch"):
        planar_pose(0., 0., (math.sin(.1), 0., 0., math.cos(.1)))


def test_pose_history_never_uses_latest_or_future_unreceived_tf():
    history = PoseHistory()
    history.add(1., Pose2(0, 0, 0))
    with pytest.raises(LookupError):
        history.at(1.05)
    history.add(1.1, Pose2(.1, 0, 0))
    assert history.at(1.05).x_m == pytest.approx(.05)
    history.add(2., Pose2(1, 0, 0))
    with pytest.raises(LookupError, match="gap"):
        history.at(1.5)
    history.add(1.1, Pose2(.2, 0, 0))
    assert history.at(1.1).x_m == .2
