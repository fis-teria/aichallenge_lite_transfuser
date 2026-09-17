from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from tools.prepare_avoidance_scenarios import (
    check_split_separation, finish_rule, prepare, scenario_document, select_pilot, validate_plan,
)


@pytest.fixture
def plan() -> dict:
    return json.loads((Path(__file__).parents[1] / "configs/collection/avoidance_static_v1.json").read_text())


def placement(site: str = "straight_a", split: str = "train", s_m: float = 70.) -> dict:
    return dict(site=site, split=split, context="straight", teacher_s_m=s_m,
                monitor_s_m=s_m, map_pose=[10., 20., math.pi / 2])


def test_plan_has_four_contexts_and_grouped_validation(plan):
    validate_plan(plan)
    assert len(plan["sites"]) == 16
    assert sum(s["split"] == "validation" for s in plan["sites"]) == 4
    assert {s["context"] for s in plan["sites"]} == {"straight", "entry", "apex", "exit"}
    assert [v * 3.6 for v in plan["teacher_speed_caps_mps"]] == pytest.approx([5., 8., 10.])


@pytest.mark.parametrize("field,value", [
    ("teacher_speed_caps_mps", [float("nan")]),
    ("teacher_speed_caps_mps", [0.]),
    ("teacher_speed_caps_mps", [5.]),  # m/s, not 5 km/h
    ("teacher_speed_caps_mps", [1., 1.000001]),
    ("station_offsets_m", [float("inf")]),
    ("station_offsets_m", [1., 1.]),
    ("sim_timeout_s", 999999),
    ("min_clearance_m", .1),
    ("finish_after_actor_m", 3.),
    ("pilot_limit", 0),
])
def test_invalid_units_values_and_budgets_rejected(plan, field, value):
    plan[field] = value
    with pytest.raises(ValueError):
        validate_plan(plan)


def test_related_group_cannot_cross_splits(plan):
    plan["sites"][4]["split"] = "validation"
    with pytest.raises(ValueError, match="group crosses"):
        validate_plan(plan)


def test_split_distance_checks_circular_seam():
    rows = [placement(s_m=98.), placement("other", "validation", 3.)]
    with pytest.raises(ValueError, match="nearby placements"):
        check_split_separation(rows, 100., 12.)
    rows[1]["teacher_s_m"] = 20.
    check_split_separation(rows, 100., 12.)


def test_finish_keeps_recovery_after_seam_instead_of_truncating_first_event():
    assert finish_rule(90., 100., 25.) == {"ego_laps_completed": 2}
    assert finish_rule(50., 100., 25.) == {"ego_reference_s_greater_than": 75.}
    assert finish_rule(10., 100., 25.) == {"ego_laps_completed": 1}
    with pytest.raises(ValueError):
        finish_rule(float("nan"), 100., 25.)


def test_h2h_boundary_converts_radians_to_degrees_without_changing_coordinates(plan):
    doc = scenario_document(plan, "case_a", [placement()], 332.)
    actor = doc["actors"][0]
    assert actor["pose"] == {"map_xy": [10., 20.], "yaw": 90.}
    assert actor["profile"] == "static_physical"
    assert doc["simulator"]["camera"] == doc["simulator"]["lidar"] == "gpu"
    assert doc["runtime"]["rosbag"] is True
    assert doc["expect"]["clearance"]["min_m"] == .3
    assert scenario_document(plan, "case_a", [placement()], 332.) == doc
    assert scenario_document(plan, "case_b", [placement()], 332.)["seed"] != doc["seed"]


def test_invalid_pose_shape_and_nonfinite_coordinates_rejected(plan):
    for pose in ([1., 2.], [1., 2., float("nan")]):
        p = placement()
        p["map_pose"] = pose
        with pytest.raises(ValueError, match="map_pose"):
            scenario_document(plan, "case", [p], 332.)


def test_multi_actor_run_keeps_split_and_complete_lap(plan):
    members = [placement("a"), placement("b", s_m=130.), placement("c", s_m=200.)]
    doc = scenario_document(plan, "multi", members, 332.)
    assert len(doc["actors"]) == 3
    assert doc["expect"]["finish"] == {"ego_laps_completed": 1}
    members[1]["split"] = "validation"
    with pytest.raises(ValueError, match="cannot mix"):
        scenario_document(plan, "mixed", members, 332.)


def test_pilot_balances_sites_omits_warned_and_multi_cases():
    rows = []
    for site in ("a", "b"):
        for ordinal in range(3):
            rows.append(dict(case_id=f"{site}{ordinal}", stage="single", warnings=[],
                             teacher_speed_cap_mps=5. / 3.6, placements=[placement(site)]))
    warning = deepcopy(rows[0]); warning.update(case_id="warning", warnings=["startup risk"])
    multi = deepcopy(rows[0]); multi.update(case_id="multi", stage="multi_after_singles")
    fast = deepcopy(rows[0]); fast.update(case_id="fast", teacher_speed_cap_mps=10. / 3.6)
    assert select_pilot([warning, multi, fast, *rows], 4) == ["a0", "b0", "a1", "b1"]


def test_existing_output_is_preserved_before_accessing_h2h(plan, tmp_path):
    output = tmp_path / "prior"
    output.mkdir()
    receipt = output / "keep.json"
    receipt.write_text('{"original":true}')
    with pytest.raises(FileExistsError):
        prepare(plan, repo=tmp_path, reference_csv=tmp_path / "absent", output=output)
    assert receipt.read_text() == '{"original":true}'


def test_reference_hash_mismatch_prevents_h2h_import_or_output_creation(plan, tmp_path):
    reference = tmp_path / "reference.csv"
    reference.write_text("changed teacher reference\n")
    output = tmp_path / "new"
    with pytest.raises(ValueError, match="reference differs"):
        prepare(plan, repo=tmp_path, reference_csv=reference, output=output)
    assert not output.exists()
