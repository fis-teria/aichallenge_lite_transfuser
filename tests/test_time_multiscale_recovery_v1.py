import pytest

from aic_transfuser_lite.data.time_multiscale_recovery_v1 import accepted_events, early_recovery_ids


def anchors():
    return [dict(anchor_id=str(i), run_id='run', split='train', recovery_event_id=e,
                 observation_ns=t) for i,(e,t) in enumerate([(1,100),(1,900),(1,1100),(2,5000),(2,5900)])]


def test_early_window_per_event_has_explicit_ns_boundary():
    assert early_recovery_ids(anchors(), duration_ns=1000) == ['0','1','3','4']


def test_validation_and_duplicates_never_enter_sampler():
    rows=anchors(); rows[0]['split']='validation'
    with pytest.raises(ValueError): early_recovery_ids(rows)
    with pytest.raises(ValueError): early_recovery_ids(anchors()+anchors()[:1])
    with pytest.raises(ValueError): early_recovery_ids(anchors(), duration_ns=0)


def test_failed_large_event_cannot_be_adopted():
    events=[dict(event_id=i, recovery_confirmed=True, completed=True, stable_at_end=True) for i in (1,2,3)]
    assert accepted_events(anchors(), events, large=True) == [1,2]
    events[1]['stable_at_end']=False
    with pytest.raises(ValueError): accepted_events(anchors(), events, large=True)


def test_unrepresented_failed_event_stays_excluded():
    events=[dict(event_id=1, recovery_confirmed=True), dict(event_id=2,recovery_confirmed=True),
            dict(event_id=3,recovery_confirmed=False)]
    assert accepted_events(anchors(), events, large=False) == [1,2]
    with pytest.raises(ValueError): accepted_events(anchors(), events+events[:1], large=False)
