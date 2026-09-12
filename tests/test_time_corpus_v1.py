"""Reference-only corpus audit must agree with decoded P1 assembly."""
from dataclasses import replace

import numpy as np
import pytest

from aic_transfuser_lite.data.time_corpus_v1 import EventWindows, audit_anchor, intervention_from_probe
from aic_transfuser_lite.data.time_dataset_v1 import assemble_time_sample
from test_time_dataset_p1 import fixture, cfg, kwargs


def test_corpus_labels_and_eligibility_equal_full_assembly():
    events, anchor = fixture(steering=True)
    window = EventWindows(events).at(anchor)
    teacher, row = audit_anchor(window, anchor, config=cfg(), bounds=(0, 5_500_000_000),
                                freeze_ns=kwargs()["freeze_ns"], intervention_ns=None)
    full = assemble_time_sample(events, anchor, **kwargs())
    np.testing.assert_array_equal(teacher.xy_mask, full.teacher.xy_mask)
    np.testing.assert_allclose(teacher.xy_m, full.teacher.xy_m, rtol=0, atol=0)
    assert row["usable_full"] and full.inputs is not None
    assert row["history_row_ids"]["final_command"]


def test_corpus_intervention_and_missing_velocity_remain_in_denominator():
    events, anchor = fixture()
    events = [e for e in events if e.role != "velocity"]
    teacher, row = audit_anchor(events, anchor, config=cfg(), bounds=(0, 5_500_000_000),
                                freeze_ns=kwargs()["freeze_ns"], intervention_ns=4_000_000_000)
    assert not teacher.xy_mask.any()
    assert not row["usable_partial"]
    assert row["input_invalid_reason"] == "CURRENT_LONGITUDINAL_SPEED_MISSING"
    assert row["stop_reason"] == "COLLECTION_INTERVENTION"


def test_corpus_rejects_late_observation_pose_and_does_not_mix_epochs():
    events, anchor = fixture()
    events = [replace(e, available_ns=10_000_000_000) if e.role == "pose" else e for e in events]
    events += [replace(e, epoch="other", available_ns=0) for e in events if e.role == "pose"]
    teacher, row = audit_anchor(EventWindows(events).at(anchor), anchor, config=cfg(), bounds=(0, 5_500_000_000),
                                freeze_ns=kwargs()["freeze_ns"], intervention_ns=None)
    assert teacher is None
    assert row["teacher_reasons"] == ["OBSERVATION_POSE_MISSING"]
    assert row["input_eligible"]


def test_probe_requires_real_collection_stop_provenance():
    assert intervention_from_probe({"fault": None, "stop_confirmed": True, "brake_sim": 407.519990891}) == 407519990891
    with pytest.raises(ValueError):
        intervention_from_probe({"fault": "timeout", "stop_confirmed": True, "brake_sim": 407.5})
    with pytest.raises(ValueError):
        intervention_from_probe({"fault": None, "stop_confirmed": False, "brake_sim": 407.5})


def test_outside_epoch_reason_matches_full_assembly():
    events, anchor = fixture()
    bounds = (anchor.capture_ns + 25_000_000, 5_500_000_000)
    _, row = audit_anchor(events, anchor, config=cfg(), bounds=bounds,
                           freeze_ns=kwargs()['freeze_ns'], intervention_ns=None)
    full = assemble_time_sample(events, anchor, **{**kwargs(), 'epoch_start_ns': bounds[0]})
    assert row['input_invalid_reason'] == full.input_invalid_reason == 'ANCHOR_OUTSIDE_EPOCH'
    assert not row['input_eligible'] and full.inputs is None
