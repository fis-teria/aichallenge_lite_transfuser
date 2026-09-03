from __future__ import annotations

from typing import Any

import numpy as np

from aic_transfuser_lite.control.executable_reference import (
    AuthoritativePlanV3,
    ExecutableReferenceConfigV3,
    build_executable_reference_v3,
)

from .plan_consistency import evaluate_plan_consistency_v3


PLAN_DIAGNOSTICS_FORMAT_V3 = "aic_plan_diagnostics_v3"


def plan_diagnostics_payload_v3(
    plan: AuthoritativePlanV3,
    *,
    current_speed_mps: float,
    reference_config: ExecutableReferenceConfigV3,
    speed_scale_mps: float = 1.0,
) -> dict[str, Any]:
    """Build a JSON-compatible, non-authoritative plan diagnostic record."""

    metrics = evaluate_plan_consistency_v3(
        plan,
        current_speed_mps=current_speed_mps,
        speed_scale_mps=speed_scale_mps,
    )
    decision = build_executable_reference_v3(
        plan,
        current_speed_mps=current_speed_mps,
        config=reference_config,
    )
    payload: dict[str, Any] = {
        "format": PLAN_DIAGNOSTICS_FORMAT_V3,
        "authority": "shadow_diagnostic_only",
        "observation_stamp_sec": float(plan.observation_stamp_sec),
        "frame_id": plan.frame_id,
        "current_speed_mps": float(current_speed_mps),
        "trajectory_xy_m": np.asarray(plan.trajectory_xy_m, dtype=np.float64).tolist(),
        "speed_profile_mps": np.asarray(
            plan.speed_profile_mps, dtype=np.float64
        ).tolist(),
        "waypoint_times_sec": np.asarray(
            plan.waypoint_times_sec, dtype=np.float64
        ).tolist(),
        "stop_probability": (
            None if plan.stop_probability is None else float(plan.stop_probability)
        ),
        "e_plan": {
            "segment_length_m": metrics.segment_length_m.tolist(),
            "geometric_speed_mps": metrics.geometric_speed_mps.tolist(),
            "trapezoidal_speed_mps": metrics.trapezoidal_speed_mps.tolist(),
            "speed_residual_mps": metrics.speed_residual_mps.tolist(),
            "mean_absolute_error_mps": metrics.mean_absolute_error_mps,
            "max_absolute_error_mps": metrics.max_absolute_error_mps,
            "normalized_huber_mean": metrics.normalized_huber_mean,
        },
        "decision": {
            "stop_required": decision.stop_required,
            "reasons": list(decision.reasons),
        },
    }
    if decision.reference is not None:
        reference = decision.reference
        payload["executable_reference"] = {
            "reference_id": reference.reference_id,
            "trajectory_xy_m": reference.trajectory_xy_m.tolist(),
            "arc_length_m": reference.arc_length_m.tolist(),
            "speed_mps": reference.speed_mps.tolist(),
            "time_from_observation_sec": (
                reference.time_from_observation_sec.tolist()
            ),
            "curvature_per_m": reference.curvature_per_m.tolist(),
            "transformations": list(reference.transformations),
        }
    else:
        payload["executable_reference"] = None
    return payload
