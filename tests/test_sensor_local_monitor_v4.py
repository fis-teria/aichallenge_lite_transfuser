"""Artificial profiles only: no ROS, model, external map, sensor or dataset I/O."""
from dataclasses import asdict, replace
import inspect
import json
import math
import os
from pathlib import Path

import pytest

from aic_transfuser_lite.control import sensor_local_monitor_v4 as m


def fixture_request() -> m.Request:
    """Values are invented for finite geometry tests, NOT AWSIM calibration."""
    state = m.State('s0', 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.)
    profile = m.Profile('synthetic-v1', 'ARTIFICIAL_FIXTURE_ONLY', True,
                        'OPAQUE_MIN_WIDTH_V1', .3, True,
                        .4, .2, .2, .4, .8, 1., .5, 1.,
                        .01, .01, 0., .05, 8., .01, 4., 1., .5, .1,
                        500, 3000, 128, 4, 2_000_000)
    previous = m.Operation('previous-zero', 0., 0., 0., 2.)
    candidate = m.Operation('candidate-accelerate', .5, 0., .5, 2.)
    stop = m.StopPolicy('synthetic-stop-v1', 2., 1., 2., .01, .01, .01)
    scan = m.Scan('scan0', 'sim-synthetic', 'epoch0', 'local', 'tf0', 1., 1.,
                  0., 0., 0., -math.pi/2, math.pi/60, (3.,)*61,
                  .01, 5., .005, .005, .001, 0.)
    initial = m.InitialEvidence('initial0', 'sim-synthetic', 'epoch0', 'local', 'tf0',
                                profile.version, 1., 3., 'SYNTHETIC_CONFIRMED_REGION',
                                (-.4, -.4, .7, .4), state)
    return m.Request('sim-synthetic', 'epoch0', 'local', 'tf0', state, previous,
                     candidate, stop, profile, m.History('h0', (scan,)), initial)


def clean_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k: clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(v) for v in value]
    return value


@pytest.fixture
def check(request, tmp_path):
    counter = 0
    def save(record):
        nonlocal counter
        target = Path(os.environ.get('SENSOR_LOCAL_TRACE_DIR', str(tmp_path)))
        target.mkdir(parents=True, exist_ok=True)
        name = m.content_hash({'test': request.node.nodeid, 'call': counter})[:20]+'.json'
        counter += 1
        record.update(test=request.node.nodeid, synthetic_only=True)
        (target/name).write_text(json.dumps(clean_json(record), indent=2), encoding='utf8')
    def run(r, expected, reason=None, *, now=1., prior=None):
        result = m.evaluate(r, now_s=now, prior=prior)
        record = dict(test=request.node.nodeid, synthetic_only=True, now_s=now,
                      expected_status=expected, expected_reason=reason, input=asdict(r),
                      output=asdict(result), prior=None if prior is None else asdict(prior))
        save(record)
        assert result.status == expected, result.reason
        if reason is not None:
            assert result.reason == reason
        assert result.runtime_permission is False
        assert result.contact_telemetry == 'MISSING'
        return result
    def validate(d, r, now, alive, expected):
        actual = m.revalidate(d, r, now_s=now, monitor_alive=alive)
        save(dict(action='PRE_SEND_REVALIDATION_NO_PUBLISH', decision_hash=d.binding_hash,
                  input=asdict(r), now_s=now, monitor_alive=alive, expected=expected, actual=actual))
        assert actual == expected
    run.validate = validate
    return run


def test_map_independent_and_front_observed(check, monkeypatch):
    r = fixture_request()
    def forbidden(*args, **kwargs):
        raise AssertionError('core must not open files')
    with monkeypatch.context() as patch:
        patch.setattr('builtins.open', forbidden)
        a = m.evaluate(r, now_s=1.)
        patch.setenv('EXTERNAL_MAP', 'missing-or-changed-map')
        patch.setenv('SCENE_AABB', 'entire-course')
        b = m.evaluate(r, now_s=1.)
    assert a == b
    assert 'map' not in inspect.signature(m.evaluate).parameters
    check(r, 'LOCAL_CLEAR')


def test_candidate_accelerates_before_stopping(check):
    d = check(fixture_request(), 'LOCAL_CLEAR')
    assert d.travel_m > .05  # positive acceleration for .5s; cannot remain F0
    assert d.stopped and d.tubes[-1].speed_mps == 0
    assert d.tubes[-1].time_s > 1.5


def test_unobserved_front_band(check):
    r = fixture_request()
    scan = replace(r.history.scans[0], ranges_m=(math.inf,)*61)
    check(replace(r, history=m.History('no-return', (scan,))), 'UNKNOWN', 'UNOBSERVED_STOPPING_REGION')


def test_turn_side_rear_is_unknown(check):
    r = fixture_request()
    # Sensor moved to nose, frontal rays cannot certify newly swept side/rear.
    scan = replace(r.history.scans[0], x_m=.6)
    r = replace(r, initial=replace(r.initial, confirmed_box_m=(-.25,-.25,.65,.25)),
                candidate=replace(r.candidate, steering_rate_rps=.8),
                history=m.History('front-only', (scan,)))
    check(r, 'UNKNOWN', 'UNOBSERVED_STOPPING_REGION')


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf'), 5., 0.])
def test_invalid_rays_and_no_return_do_not_clear(check, value):
    r = fixture_request()
    scan = replace(r.history.scans[0], ranges_m=(value,)*61)
    check(replace(r, history=m.History('invalid', (scan,))), 'UNKNOWN')


def test_initial_hit_revokes_even_with_old_free(check):
    r = fixture_request()
    ranges = list(r.history.scans[0].ranges_m)
    ranges[30] = .3
    hit = replace(r.history.scans[0], id='new-hit', ranges_m=tuple(ranges))
    d = check(replace(r, history=m.History('h1', (*r.history.scans, hit))),
              'OBSTRUCTED', 'INITIAL_HIT_CONTRADICTION')
    assert any('REVOKED' in x for x in d.evidence_trace)


def test_stopping_region_hit(check):
    r = fixture_request()
    ranges = list(r.history.scans[0].ranges_m)
    ranges[30] = .78
    scan = replace(r.history.scans[0], ranges_m=tuple(ranges))
    check(replace(r, history=m.History('hit-ahead', (scan,))), 'OBSTRUCTED', 'STOPPING_REGION_HIT')


@pytest.mark.parametrize('change', ['calibration_source', 'minimum_obstacle_width_m', 'scan_height_confirmed', 'detection_contract'])
def test_missing_real_profile_unknown(check, change):
    r = fixture_request()
    check(replace(r, profile=replace(r.profile, **{change: None})), 'UNKNOWN', 'PROFILE_MISSING_OR_NONSTATIC')


def test_ray_gap_thin_obstacle_profile(check):
    r = fixture_request()
    r = replace(r, profile=replace(r.profile, minimum_obstacle_width_m=.001))
    check(r, 'UNKNOWN', 'UNOBSERVED_STOPPING_REGION')


def test_expired_free_and_unknown_cannot_renew_it(check):
    r = fixture_request()
    old = replace(r.history.scans[0], id='old', observed_s=0., received_s=0.)
    unknown = replace(r.history.scans[0], id='new-unknown', ranges_m=(math.inf,)*61)
    d = check(replace(r, history=m.History('h2', (old, unknown))), 'UNKNOWN')
    assert 'old:FREE_TTL_EXPIRED_HITS_RETAINED' in d.evidence_trace
    # Reprojection changes pose/receipt but NOT original observation time.
    moved = replace(old, x_m=.01, received_s=1.)
    check(replace(r, history=m.History('reprojected', (moved,))), 'STALE', 'ALL_SCANS_EXPIRED')


def test_expired_hit_not_erased_by_unknown(check):
    r = fixture_request()
    old = replace(r.history.scans[0], observed_s=0., received_s=0., ranges_m=(.3,)*61)
    unknown = replace(r.history.scans[0], id='unknown', ranges_m=(math.inf,)*61)
    check(replace(r, history=m.History('h3', (old, unknown))), 'OBSTRUCTED')


@pytest.mark.parametrize('change', ['static', 'epoch', 'tf', 'lost_history', 'teleport'])
def test_evidence_invalidation(check, change):
    r = fixture_request()
    if change == 'static':
        r = replace(r, profile=replace(r.profile, static_confirmed=False))
    elif change == 'epoch':
        r = replace(r, epoch='reset1')
    elif change == 'tf':
        r = replace(r, tf_version='changed')
    elif change == 'lost_history':
        r = replace(r, history=replace(r.history, complete=False))
    else:
        r = replace(r, state=replace(r.state, id='teleported', x_m=2.))
    check(r, 'UNKNOWN')


def test_initial_not_automatically_extended(check):
    r = fixture_request()
    first = check(r, 'LOCAL_CLEAR')
    shifted = replace(r, state=replace(r.state, id='new-state', time_s=1.1, x_m=.4),
                      previous=replace(r.candidate, duration_s=0.))
    check(shifted, 'UNKNOWN', 'INITIAL_STATE_OUTSIDE_CHECKED_TUBE', now=1.1, prior=first)


def test_initial_continuation_within_prior_tube(check):
    r = fixture_request()
    first = check(r, 'LOCAL_CLEAR')
    t = first.tubes[10]
    state = replace(r.state, id='s1', time_s=t.time_s, x_m=t.x_m, y_m=t.y_m,
                    yaw_rad=t.yaw_rad, speed_mps=t.speed_mps, steer_rad=t.steer_rad,
                    acceleration_mps2=.2)
    next_r = replace(r, state=state, previous=replace(r.candidate, duration_s=0.),
                     candidate=replace(r.candidate, id='c1', acceleration_mps2=0., duration_s=.1))
    d = check(next_r, 'LOCAL_CLEAR', now=t.time_s, prior=first)
    assert 'INITIAL_MAINTAINED_WITHIN_CHECKED_TUBE' in d.evidence_trace
    assert d.evidence_until_s <= first.evidence_until_s  # no proof TTL refresh


@pytest.mark.parametrize('budget', ['steps', 'cells', 'checks', 'horizon'])
def test_budget_is_not_stopped_success(check, budget):
    r = fixture_request()
    field, value = {'steps': ('max_steps', 1), 'cells': ('max_cells', 1),
                    'checks': ('max_checks', 1), 'horizon': ('horizon_s', .01)}[budget]
    d = check(replace(r, profile=replace(r.profile, **{field: value})), 'UNKNOWN')
    assert not d.stopped and d.stopping_clearance == 'UNVERIFIED'


@pytest.mark.parametrize('change', ['candidate', 'stop', 'epoch', 'hit', 'state', 'previous'])
def test_decision_content_binding(check, change):
    r = fixture_request()
    d = check(r, 'LOCAL_CLEAR')
    check.validate(d, r, 1.01, True, (True, 'MATCHED_CONDITIONAL_DECISION_NOT_PUBLISH'))
    if change == 'candidate':
        altered = replace(r, candidate=replace(r.candidate, steering_rate_rps=1e-9))
    elif change == 'stop':
        altered = replace(r, stop=replace(r.stop, version='other'))
    elif change == 'epoch':
        altered = replace(r, epoch='reset')
    elif change == 'state':
        altered = replace(r, state=replace(r.state, x_m=1e-9))
    elif change == 'previous':
        altered = replace(r, previous=replace(r.previous, duration_s=.01))
    else:
        scan = replace(r.history.scans[0], ranges_m=(.3,)*61)
        altered = replace(r, history=m.History('new-hit', (scan,)))
    check.validate(d, altered, 1.01, True, (False, 'BINDING_CHANGED'))
    check.validate(d, r, d.valid_until_s, True, (False, 'DECISION_EXPIRED'))
    check.validate(d, r, 1.01, False, (False, 'MONITOR_STOPPED'))


def test_sensor_error_does_not_inflate_vehicle_twice(check):
    r = fixture_request()
    low = check(r, 'LOCAL_CLEAR')
    # >1m uncertainty covers the origin of the newly swept cells, so their
    # whole-cell visibility cannot be certified even with a wide frontal FOV.
    scan = replace(r.history.scans[0], pose_error_m=1.5)
    high_r = replace(r, history=m.History('error-large', (scan,)))
    high = check(high_r, 'UNKNOWN')
    assert low.checked_cells == high.checked_cells  # uncertainty belongs only to sensor
    assert low.travel_m == high.travel_m


def test_state_error_grows_region_and_prevents_seed_renewal(check):
    r = fixture_request()
    baseline = check(r, 'LOCAL_CLEAR')
    changed = replace(r, state=replace(r.state, xy_error_m=.3))
    d = check(changed, 'UNKNOWN')
    assert set(baseline.checked_cells).issubset(d.checked_cells)


def test_missing_effective_braking_and_fault(check):
    r = fixture_request()
    check(replace(r, stop=replace(r.stop, effective_braking_mps2=None)), 'UNKNOWN')
    check(replace(r, stop=replace(r.stop, effective_braking_mps2=3.)), 'FAULT', 'INVALID_EFFECTIVE_BRAKING')
    check(replace(r, candidate=replace(r.candidate, acceleration_mps2=float('nan'))), 'FAULT')


def test_late_send_and_state(check):
    r = fixture_request()
    check(replace(r, candidate=replace(r.candidate, expires_s=1.1)), 'STALE', 'OPERATION_EXPIRED')
    check(r, 'STALE', 'STATE_STALE', now=1.6)


def test_speed_uncertainty_must_stop_too(check):
    r = fixture_request()
    state = replace(r.state, speed_error_mps=.02)
    r = replace(r, state=state, initial=replace(r.initial, anchor_state=state),
                candidate=replace(r.candidate, acceleration_mps2=0., duration_s=.01))
    d = check(r, 'LOCAL_CLEAR')
    assert d.tubes[-1].time_s > 1.04  # uncertain velocity cannot stop with zero brake time


def test_interval_sweep_not_only_pose_rectangles(check):
    r = fixture_request()
    d = check(r, 'LOCAL_CLEAR')
    # The origin tube's nonzero interval grows beyond the static rectangle.
    cells = m._footprint_cells(d.tubes[0], r.profile, 0., m._Checks(10000))
    assert set(d.checked_cells)-cells
    stationary = replace(r, candidate=replace(r.candidate, acceleration_mps2=0., duration_s=.01))
    check(stationary, 'LOCAL_CLEAR')


@pytest.mark.parametrize('field', ['steer_error_rad', 'acceleration_error_mps2'])
def test_unimplemented_state_error_does_not_get_zeroed(check, field):
    r = fixture_request()
    check(replace(r, state=replace(r.state, **{field: .01})),
          'UNKNOWN', 'STEER_OR_ACCEL_ERROR_MODEL_UNSUPPORTED')


def test_no_return_behind_hit_never_free():
    r = fixture_request()
    scan = replace(r.history.scans[0], ranges_m=(.4,)*61)
    assert not m._cell_free((20, 0), scan, r.profile, m._Checks(1000))
