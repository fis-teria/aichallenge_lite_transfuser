"""One-way AWSIM recovery experiment; reference geometry is debug/teacher-only.

The caller commits pulse proposals only after a guarded command is published.
Model ownership latches before evaluating its path, including rejected paths.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping

import numpy as np

from aic_transfuser_lite.data.time_steering_pulse_v1 import (
    SteeringPulseConfig, SteeringPulseState, nominal_recovery_errors, propose_steering_pulse,
)


@dataclass(frozen=True)
class RecoveryTakeoverState:
    pulse: SteeringPulseState = SteeringPulseState()
    takeover_ns: int | None = None


def validate_recovery_reference(reference: Mapping[str, Any]) -> tuple[SteeringPulseConfig, np.ndarray, np.ndarray]:
    """Return pulse config, baseline [N,2] m and observed guide [M,3] m/m/rad."""
    spec = reference['steering_pulse']
    if (spec['schema'] != 'measured_steering_pulse_v1' or reference['intervals']
            or reference['signed_offset_m'] != 0.
            or reference['reference_xy_m'] != reference['baseline_xy_m']):
        raise ValueError('RECOVERY_UNMODIFIED_REFERENCE_REQUIRED')
    config = SteeringPulseConfig(**spec['config'])
    baseline = np.asarray(reference['baseline_xy_m'], dtype=float)
    guide = np.asarray(spec['nominal_guide'], dtype=float)
    if baseline.ndim != 2 or baseline.shape[1] != 2 or len(baseline) < 20 or not np.isfinite(baseline).all():
        raise ValueError('RECOVERY_BASELINE_SHAPE_OR_FINITE')
    nominal_recovery_errors(guide, s_m=config.start_s_m, offset_m=0., yaw_rad=0.)
    if (guide[0, 0] > config.start_s_m - 5.
            or guide[-1, 0] < config.start_s_m + config.start_window_m + 3. + 1.7*config.recovery_s):
        raise ValueError('RECOVERY_GUIDE_COVERAGE')
    return config, baseline, guide


def propose_recovery_takeover(config: SteeringPulseConfig, state: RecoveryTakeoverState, *,
                              sim_ns: int, wall_ns: int, observation_ns: int,
                              s_m: float, speed_mps: float, lateral_m: float,
                              heading_rad: float) -> tuple[RecoveryTakeoverState, float, str]:
    """Return proposed state, teacher perturbation rad, and command owner.

    Take the first causal model observation at least 150 ms after the published
    zero-pulse boundary. Path quality is deliberately not a selection input.
    Wait at most one simulated second after zero; never extend the disturbance.
    Once selected, a model remains the owner even if a later observation is bad.
    """
    if (type(sim_ns) is not int or type(observation_ns) is not int
            or not 0 <= observation_ns <= sim_ns):
        raise ValueError('RECOVERY_OBSERVATION_CLOCK')
    if state.takeover_ns is not None:
        if sim_ns < state.takeover_ns:
            raise ValueError('RECOVERY_CLOCK_RESET')
        return state, 0., 'E2E'
    # Use the last *published* zero boundary, not an uncommitted proposal.
    zero_ns = state.pulse.zero_ns
    if zero_ns is not None:
        if sim_ns - zero_ns > 1_000_000_000:
            raise ValueError('RECOVERY_CAUSAL_TAKEOVER_TIMEOUT')
        if observation_ns >= zero_ns + 150_000_000:
            return replace(state, takeover_ns=sim_ns), 0., 'E2E'
    decision = propose_steering_pulse(config, state.pulse, sim_ns=sim_ns, wall_ns=wall_ns,
        s_m=s_m, speed_mps=speed_mps, lateral_m=lateral_m, heading_rad=heading_rad)
    if decision.state.stage == 'skipped':
        raise ValueError('RECOVERY_PULSE_START_MISSED')
    return replace(state, pulse=decision.state), decision.perturbation_rad, 'TEACHER_BOOTSTRAP'
