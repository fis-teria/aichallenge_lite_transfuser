"""Selection tests; actual inference reuses the tested frozen-fit evaluator."""
import math
from pathlib import Path
import runpy

import pytest


functions = runpy.run_path(str(Path(__file__).resolve().parents[1]/"tools/analyze_time_corner_learning.py"))
neighborhood = functions["neighborhood"]
population = functions["population"]
fit_indices = functions["fit_indices"]


def row(**kwargs):
    return dict(dict(anchor_id="a", run_id="run1", split="train", event_id=1, status="CORNER", eligible=True,
                     recovery=True, base_s_m=190., left_m=.4, heading_rad=.1, speed_mps=1.3), **kwargs)


def test_joint_neighborhood_does_not_treat_same_lateral_other_location_as_coverage():
    q = row()
    rows = [row(), row(base_s_m=100.), row(heading_rad=-.2), row(speed_mps=2.3), row(left_m=-.4)]
    assert neighborhood(rows, q, (3., .15, .09, .25)) == [0]


def test_heading_is_wrapped_and_tolerance_boundary_inclusive():
    q = row(heading_rad=math.pi-.01)
    assert neighborhood([row(base_s_m=193., heading_rad=-math.pi+.01)], q, (3., .15, .03, .25)) == [0]


@pytest.mark.parametrize("tolerance", [(3., .15, .09), (3., 0., .09, .25), (3., .15, float("nan"), .25)])
def test_bad_units_and_tolerances_fail(tolerance):
    with pytest.raises(ValueError):
        neighborhood([row()], row(), tolerance)


def test_invalid_geometry_cannot_count_as_coverage_but_supported_recovery_still_gets_fit():
    rows = [row(status="COURSE_TOO_FAR"), row(anchor_id="b", eligible=False, status="INPUT_OR_FULL_TEACHER_UNSUPPORTED"),
            row(anchor_id="c", recovery=False), row(anchor_id="d", status="OUTSIDE_CORNER", recovery=False)]
    assert neighborhood(rows, row(), (3., .15, .09, .25)) == [2]
    assert fit_indices(rows) == [0, 2]


def test_correlated_frames_and_repetitions_are_not_independent_events():
    rows = [row(), row(anchor_id="b"), row(anchor_id="c", event_id=2), row(anchor_id="d", run_id="run2", event_id=1)]
    result = population(rows, {"a": 10, "b": 2, "c": 1, "d": 1})
    assert (result["anchors"], result["runs"], result["recovery_events"], result["presentations_per_epoch"]) == (4, 2, 3, 14)


def test_validation_is_never_given_training_exposure_and_duplicate_ids_fail():
    with pytest.raises(ValueError):
        population([row(split="validation")], {"a": 1})
    with pytest.raises(ValueError):
        population([row(), row()])


def test_nonfinite_state_fails_instead_of_becoming_an_uncovered_state():
    with pytest.raises(ValueError):
        neighborhood([row(left_m=float("nan"))], row(), (3., .15, .09, .25))
