from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

from aic_transfuser_lite.data.recovery_reference_v3 import (
    MpcReferencePointV3,
    OccupancyMapV3,
    RecoveryReferenceConfigV3,
    RecoverySegmentRequestV3,
    generate_recovery_reference_v3,
    load_mpc_reference_v3,
    load_occupancy_map_v3,
    load_recovery_reference_config_v3,
    render_official_mpc_recovery_config_v3,
    write_generated_recovery_reference_v3,
)


ROOT = Path(__file__).parents[1]


def _circle_points(count: int = 160, radius_m: float = 20.0) -> tuple[MpcReferencePointV3, ...]:
    angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    x = 30.0 + radius_m * np.cos(angle)
    y = 30.0 + radius_m * np.sin(angle)
    heading = angle + np.pi / 2.0
    segment = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate(([0.0], np.cumsum(segment)))
    return tuple(
        MpcReferencePointV3(
            s_m=float(s[index]),
            x_m=float(x[index]),
            y_m=float(y[index]),
            psi_rad=float(heading[index]),
            kappa_radpm=1.0 / radius_m,
            vx_mps=1.0,
            ax_mps2=0.0,
        )
        for index in range(count)
    )


def _map() -> OccupancyMapV3:
    return OccupancyMapV3(
        free=np.ones((700, 700), dtype=np.bool_),
        resolution_m_per_px=0.1,
        origin_x_m=0.0,
        origin_y_m=0.0,
    )


def _config() -> RecoveryReferenceConfigV3:
    return RecoveryReferenceConfigV3(
        approach_length_m=6.0,
        hold_length_m=3.0,
        recovery_length_m=4.0,
        minimum_segment_gap_m=8.0,
        minimum_center_clearance_m=1.4,
        geometry_curvature_threshold_inv_m=0.015,
        preferred_abs_curvature_inv_m=0.05,
        requests=(
            RecoverySegmentRequestV3("left_near", "left", 0.35, "left_curve"),
            RecoverySegmentRequestV3("right_far", "right", 0.55, "left_curve"),
        ),
    )


def test_generator_separates_approach_from_training_eligible_phases() -> None:
    generated = generate_recovery_reference_v3(_circle_points(), _map(), _config())

    assert len(generated.selected_segments) == 2
    assert len(generated.points) == 160
    assert [phase.phase for phase in generated.phases] == [
        "approach",
        "hold",
        "recovery",
        "approach",
        "hold",
        "recovery",
    ]
    assert [phase.training_eligible for phase in generated.phases] == [
        False,
        True,
        True,
        False,
        True,
        True,
    ]
    intervals = [
        (item["base_start_s_m"], item["base_end_s_m"])
        for item in generated.selected_segments
    ]
    assert intervals[0][1] + _config().minimum_segment_gap_m <= intervals[1][0]


def test_generated_offset_reaches_requested_hold_and_returns_smoothly() -> None:
    base = _circle_points()
    generated = generate_recovery_reference_v3(base, _map(), _config())
    first = generated.selected_segments[0]
    start = int(first["start_point_id"])
    end = int(first["end_point_id"])
    distance = np.asarray(
        [
            np.hypot(generated.points[i].x_m - base[i].x_m, generated.points[i].y_m - base[i].y_m)
            for i in range(start, end + 1)
        ]
    )

    assert distance.max() == pytest.approx(0.35, abs=0.01)
    assert distance[0] == pytest.approx(0.0, abs=1e-9)
    assert distance[-1] < 0.01
    assert all(np.isfinite(point.kappa_radpm) for point in generated.points)
    assert all(generated.points[i + 1].s_m > generated.points[i].s_m for i in range(159))


def test_occupied_footprint_fails_closed() -> None:
    blocked = OccupancyMapV3(
        free=np.zeros((700, 700), dtype=np.bool_),
        resolution_m_per_px=0.1,
        origin_x_m=0.0,
        origin_y_m=0.0,
    )
    with pytest.raises(ValueError, match="no safe, non-overlapping interval"):
        generate_recovery_reference_v3(_circle_points(), blocked, _config())


def test_repository_config_and_map_loader_are_versioned(tmp_path: Path) -> None:
    config = load_recovery_reference_config_v3(
        ROOT / "configs/data/recovery_reference_generator_v3.yaml"
    )
    assert config.approach_length_m == 10.0
    assert config.hold_length_m == 8.0
    assert config.recovery_length_m == 4.0
    assert {request.side for request in config.requests} == {"left", "right"}

    pixels = np.full((20, 30), 255, dtype=np.uint8)
    pixels[0, 0] = 0
    Image.fromarray(pixels).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "map.pgm",
                "resolution": 0.1,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
            }
        ),
        encoding="utf-8",
    )
    loaded = load_occupancy_map_v3(tmp_path / "map.yaml")
    assert loaded.free.shape == (20, 30)
    assert not bool(loaded.free[0, 0])


def test_writer_hash_binds_reference_and_intervals(tmp_path: Path) -> None:
    base = _circle_points()
    generated = generate_recovery_reference_v3(base, _map(), _config())
    base_csv = tmp_path / "base.csv"
    fields = ("s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2")
    with base_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for point in base:
            writer.writerow({name: getattr(point, name) for name in fields})
    map_yaml = tmp_path / "map.yaml"
    map_image = tmp_path / "map.pgm"
    Image.fromarray(np.full((4, 4), 255, dtype=np.uint8)).save(map_image)
    map_yaml.write_text("image: map.pgm\n", encoding="utf-8")
    config_yaml = tmp_path / "config.yaml"
    config_yaml.write_text("config: fixture\n", encoding="utf-8")

    intervals, manifest = write_generated_recovery_reference_v3(
        tmp_path / "recovery.csv",
        generated,
        base_reference_path=base_csv,
        occupancy_map_yaml=map_yaml,
        generator_config_path=config_yaml,
    )

    payload = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    assert payload["teacher_debug_only"] is True
    assert payload["training_policy"]["approach"] == "exclude"
    assert intervals.is_file()
    assert load_mpc_reference_v3(tmp_path / "recovery.csv")
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        write_generated_recovery_reference_v3(
            tmp_path / "recovery.csv",
            generated,
            base_reference_path=base_csv,
            occupancy_map_yaml=map_yaml,
            generator_config_path=config_yaml,
        )


def test_official_mpc_config_uses_external_reference_without_package_mutation(
    tmp_path: Path,
) -> None:
    source = tmp_path / "official.yaml"
    source.write_text(
        yaml.safe_dump(
            {
                "map": {"yaml_path": "env/map.yaml"},
                "reference_path": {
                    "csv_path": "env/base.csv",
                    "update_by_topic": True,
                    "circular": False,
                },
                "mpc": {"lateral_target_mode": "center_of_corridor"},
            }
        ),
        encoding="utf-8",
    )
    output = render_official_mpc_recovery_config_v3(
        source,
        tmp_path / "recovery.yaml",
        reference_container_path="/artifacts/recovery/reference.csv",
        package_share_container_path="/pkg/share/mpc",
    )

    rendered = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert rendered["reference_path"]["csv_path"] == "../../../artifacts/recovery/reference.csv"
    assert rendered["reference_path"]["circular"] is True
    assert rendered["reference_path"]["update_by_topic"] is False
    assert rendered["mpc"]["lateral_target_mode"] == "reference_path"
    assert yaml.safe_load(source.read_text(encoding="utf-8"))["reference_path"]["csv_path"] == "env/base.csv"
