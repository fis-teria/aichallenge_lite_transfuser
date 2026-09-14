from dataclasses import replace

import numpy as np
import pytest

from aic_transfuser_lite.data.time_steering_pulse_v1 import SteeringPulseConfig, SteeringPulseState
from aic_transfuser_lite.runtime.time_recovery_takeover_v1 import (
    RecoveryTakeoverState, propose_recovery_takeover, validate_recovery_reference,
)


CONFIG = SteeringPulseConfig(start_s_m=88., amplitude_rad=.1, duration_s=2., plateau_s=1.5)


def propose(state, sim_ns=10_200_000_000, observation_ns=10_150_000_000, **kwargs):
    values = dict(sim_ns=sim_ns, wall_ns=sim_ns, observation_ns=observation_ns,
                  s_m=90., speed_mps=1.27, lateral_m=.10, heading_rad=.04)
    values.update(kwargs)
    return propose_recovery_takeover(CONFIG, state, **values)


def released():
    return RecoveryTakeoverState(SteeringPulseState(stage='recovery', start_ns=8_000_000_000,
        start_wall_ns=8_000_000_000, zero_ns=10_000_000_000, approach_seen=True))


def test_first_causal_observation_latches_without_path_quality_input():
    state = released()
    pending, pulse, role = propose(state, observation_ns=10_149_999_999)
    assert role == 'TEACHER_BOOTSTRAP' and pulse == 0. and pending.takeover_ns is None
    taken, pulse, role = propose(state)
    assert role == 'E2E' and pulse == 0. and taken.takeover_ns == 10_200_000_000
    assert state.takeover_ns is None  # Pure proposal; controller commits explicitly.
    # No teacher fallback, even with old observation / invalid teacher state.
    assert propose(taken, observation_ns=1, speed_mps=float('nan')) == (taken, 0., 'E2E')


def test_unpublished_zero_does_not_trigger_takeover():
    state = RecoveryTakeoverState(SteeringPulseState(stage='active', approach_seen=True,
        start_ns=8_000_000_000, start_wall_ns=8_000_000_000))
    pending, pulse, role = propose(state)
    assert role == 'TEACHER_BOOTSTRAP' and pulse == 0.
    assert pending.pulse.zero_ns == 10_200_000_000
    assert state.pulse.zero_ns is None
    assert propose(state)[2] == 'TEACHER_BOOTSTRAP'


def test_takeover_is_bounded_and_cannot_reverse_clock():
    with pytest.raises(ValueError, match='TIMEOUT'):
        propose(released(), sim_ns=11_000_000_001, observation_ns=10_000_000_000)
    taken = replace(released(), takeover_ns=11_000_000_000)
    with pytest.raises(ValueError, match='CLOCK_RESET'):
        propose(taken)


@pytest.mark.parametrize('observation_ns', [-1, 11_000_000_000, 1.0])
def test_observation_clock_contract(observation_ns):
    with pytest.raises(ValueError, match='OBSERVATION_CLOCK'):
        propose(released(), observation_ns=observation_ns)


def test_missed_pulse_cannot_become_a_teacher_lap():
    with pytest.raises(ValueError, match='START_MISSED'):
        propose(RecoveryTakeoverState(SteeringPulseState(approach_seen=True)))


def reference():
    xy = np.column_stack([np.arange(30.), np.zeros(30)]).tolist()
    return dict(steering_pulse=dict(schema='measured_steering_pulse_v1', config=CONFIG.__dict__,
        nominal_guide=[[80., 0., 0.], [115., 0., 0.]]), intervals=[], signed_offset_m=0.,
        reference_xy_m=xy, baseline_xy_m=xy)


def test_reference_shapes_units_and_unmodified_geometry():
    cfg, baseline, guide = validate_recovery_reference(reference())
    assert cfg.amplitude_rad == .1 and baseline.shape == (30, 2) and guide.shape == (2, 3)
    value = reference(); value['signed_offset_m'] = .1
    with pytest.raises(ValueError, match='UNMODIFIED'):
        validate_recovery_reference(value)


@pytest.mark.parametrize('baseline', [[[0., 0.]], [[0., 0., 0.]]*30, [[float('nan'), 0.]]*30])
def test_invalid_baseline_rejected(baseline):
    value = reference(); value['baseline_xy_m'] = value['reference_xy_m'] = baseline
    with pytest.raises(ValueError, match='BASELINE_SHAPE_OR_FINITE'):
        validate_recovery_reference(value)


def test_nominal_guide_must_cover_whole_recovery_window():
    value = reference(); value['steering_pulse']['nominal_guide'][-1][0] = 100.
    with pytest.raises(ValueError, match='GUIDE_COVERAGE'):
        validate_recovery_reference(value)
