from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from aic_transfuser_lite.data.collection_reference_v3 import (
    CollectionCriteriaV3,
    CoverageRequirementV3,
    RoutePointV3,
    TeacherStateSampleV3,
    classify_collection_coverage_v3,
    load_collection_criteria_v3,
    load_route_reference_v3,
    project_to_route_v3,
    route_points_from_arrows,
    verify_route_reference_manifest_v3,
    write_route_reference_v3,
)
from tools.capture_raceline_reference_v3 import parse_marker_array


ROOT = Path(__file__).parents[1]


def _straight_route() -> tuple[RoutePointV3, ...]:
    return tuple(RoutePointV3(index, float(index), 0.0, 0.0) for index in range(3))


def _criteria() -> CollectionCriteriaV3:
    names = (
        "launch",
        "stop_approach",
        "offset_left_near",
        "offset_right_near",
        "recovery_left",
        "recovery_right",
    )
    return CollectionCriteriaV3(
        sample_rate_hz=10.0,
        stopped_speed_mps=0.05,
        moving_speed_mps=0.2,
        curve_curvature_inv_m=0.02,
        lateral_offset_min_m=0.25,
        lateral_offset_far_m=0.5,
        heading_error_min_rad=0.087,
        launch_horizon_sec=1.5,
        recovery_horizon_sec=3.0,
        recovery_improvement_m=0.15,
        episode_gap_sec=0.5,
        requirements={name: CoverageRequirementV3(1, 1, 1) for name in names},
    )


def test_arrow_reference_round_trip_and_signed_projection(tmp_path: Path) -> None:
    points = route_points_from_arrows(
        (
            (0, "map", 0.0, 0.0, 1.0, 0.0),
            (1, "map", 1.0, 0.0, 2.0, 0.0),
            (2, "map", 2.0, 0.0, 3.0, 0.0),
        )
    )
    destination = tmp_path / "reference.csv"
    manifest = write_route_reference_v3(
        destination,
        points,
        source_topic="/reference",
        source_type="visualization_msgs/msg/MarkerArray",
        captured_utc="2026-09-04T00:00:00+00:00",
    )

    loaded = load_route_reference_v3(destination)
    assert manifest.is_file()
    assert verify_route_reference_manifest_v3(destination)["point_count"] == 3
    assert project_to_route_v3(1.0, 0.4, 0.1, loaded).lateral_offset_m == pytest.approx(0.4)
    assert project_to_route_v3(1.0, -0.4, -0.1, loaded).lateral_offset_m == pytest.approx(-0.4)


def test_coverage_counts_launch_stop_and_bidirectional_recovery() -> None:
    values = (
        # left offset: stop -> launch -> recovery -> stop
        (0, 0.40, 0.0),
        (1, 0.40, 0.30),
        (2, 0.10, 0.30),
        (3, 0.10, 0.00),
        # right offset: stop -> launch -> recovery
        (10, -0.40, 0.0),
        (11, -0.40, 0.30),
        (12, -0.10, 0.30),
    )
    samples = tuple(
        TeacherStateSampleV3(
            run_id="run_1",
            scenario_id="scenario_1",
            timestamp_ns=index * 100_000_000,
            x_m=1.0,
            y_m=y_m,
            yaw_rad=0.0,
            speed_mps=speed_mps,
            yaw_rate_rps=0.0,
        )
        for index, y_m, speed_mps in values
    )

    report = classify_collection_coverage_v3(samples, _straight_route(), _criteria())

    assert report["overall_status"] == "PASS"
    assert report["buckets"]["launch"]["samples"] >= 2
    assert report["buckets"]["stop_approach"]["samples"] >= 1
    assert report["buckets"]["recovery_left"]["samples"] >= 1
    assert report["buckets"]["recovery_right"]["samples"] >= 1


def test_repository_criteria_is_valid_and_versioned() -> None:
    criteria = load_collection_criteria_v3(
        ROOT / "configs/data/recovery_collection_reference_v3.yaml"
    )
    assert criteria.sample_rate_hz == 10.0
    assert "recovery_left" in criteria.requirements
    assert criteria.lateral_offset_far_m > criteria.lateral_offset_min_m


def test_mixed_reference_frames_fail_closed() -> None:
    with pytest.raises(ValueError, match="mixes frames"):
        route_points_from_arrows(
            (
                (0, "map", 0.0, 0.0, 1.0, 0.0),
                (1, "odom", 1.0, 0.0, 2.0, 0.0),
                (2, "map", 2.0, 0.0, 3.0, 0.0),
            )
        )


def test_reference_hash_mismatch_fails_closed(tmp_path: Path) -> None:
    destination = tmp_path / "reference.csv"
    write_route_reference_v3(
        destination,
        _straight_route(),
        source_topic="/reference",
        source_type="visualization_msgs/msg/MarkerArray",
        captured_utc="2026-09-04T00:00:00+00:00",
    )
    destination.write_text(destination.read_text(encoding="utf-8") + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        verify_route_reference_manifest_v3(destination)


def test_sparse_curved_reference_uses_segment_projection() -> None:
    radius_m = 10.0
    points = tuple(
        RoutePointV3(
            point_id=index,
            x_m=radius_m * math.cos(angle),
            y_m=radius_m * math.sin(angle),
            heading_rad=angle + math.pi / 2.0,
        )
        for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0))
    )
    fraction = 0.5
    chord_x = (1.0 - fraction) * points[0].x_m + fraction * points[1].x_m
    chord_y = (1.0 - fraction) * points[0].y_m + fraction * points[1].y_m
    heading = 3.0 * math.pi / 4.0
    offset_m = 0.4
    query_x = chord_x - math.sin(heading) * offset_m
    query_y = chord_y + math.cos(heading) * offset_m

    projection = project_to_route_v3(query_x, query_y, heading + 0.1, points)

    assert projection.point_id == 0
    assert projection.lateral_offset_m == pytest.approx(offset_m, abs=1e-6)
    assert projection.heading_error_rad == pytest.approx(0.1, abs=1e-6)


def test_marker_parser_uses_only_heading_arrow_namespace() -> None:
    point = lambda x, y: SimpleNamespace(x=x, y=y)
    message = SimpleNamespace(
        markers=[
            SimpleNamespace(
                ns="ignored",
                type=0,
                id=99,
                header=SimpleNamespace(frame_id="map"),
                points=[point(0.0, 0.0), point(1.0, 0.0)],
            ),
            *[
                SimpleNamespace(
                    ns="heading_arrows",
                    type=0,
                    id=index,
                    header=SimpleNamespace(frame_id="map"),
                    points=[point(float(index), 0.0), point(float(index + 1), 0.0)],
                )
                for index in range(3)
            ],
        ]
    )

    assert [item.point_id for item in parse_marker_array(message)] == [0, 1, 2]
