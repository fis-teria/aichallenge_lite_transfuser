from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Sequence

import numpy as np

from ..delay_estimation import (
    DelayEstimationConfig,
    DelayFitResult,
    estimate_steering_delay,
)


@dataclass(frozen=True)
class LateralCalibration:
    pure_delay_sec: float
    time_constant_sec: float
    gain: float
    bias_rad: float
    valid_speed_range_mps: tuple[float, float]
    nrmse: float
    yaw_rate_nrmse: float
    correlation_peak: float
    dynamic_sample_count: int
    total_sample_count: int
    excluded_sample_count: int
    individually_valid: bool
    validity_reasons: tuple[str, ...]
    source_method: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def fit_lateral_calibration(
    timestamps_sec: Sequence[float] | np.ndarray,
    command_steering_rad: Sequence[float] | np.ndarray,
    actual_steering_rad: Sequence[float] | np.ndarray,
    speed_mps: Sequence[float] | np.ndarray,
    yaw_rate_rps: Sequence[float] | np.ndarray,
    *,
    wheelbase_m: float,
    minimum_speed_mps: float = 0.1,
    max_abs_yaw_rate_rps: float = 5.0,
    maximum_steering_nrmse: float = 0.7,
    maximum_yaw_rate_nrmse: float = 0.8,
    config: DelayEstimationConfig | None = None,
    segment_ids: Sequence[str] | np.ndarray | None = None,
) -> LateralCalibration:
    """Filter declared applicability outliers, then call the V1 delay fitter."""

    values = [
        np.asarray(item, dtype=np.float64)
        for item in (
            timestamps_sec,
            command_steering_rad,
            actual_steering_rad,
            speed_mps,
            yaw_rate_rps,
        )
    ]
    if any(item.ndim != 1 for item in values):
        raise ValueError("lateral calibration inputs must be one-dimensional")
    if len(values[0]) < 3 or any(item.shape != values[0].shape for item in values[1:]):
        raise ValueError("lateral calibration inputs must have equal length >= 3")
    raw_segment_ids = None
    if segment_ids is not None:
        raw_segment_ids = np.asarray(segment_ids, dtype=object)
        if raw_segment_ids.ndim != 1 or raw_segment_ids.shape != values[0].shape:
            raise ValueError("segment_ids must match lateral calibration inputs")
    if not np.isfinite(max_abs_yaw_rate_rps) or max_abs_yaw_rate_rps <= 0.0:
        raise ValueError("max_abs_yaw_rate_rps must be finite and positive")
    if not np.isfinite(minimum_speed_mps) or minimum_speed_mps < 0.0:
        raise ValueError("minimum_speed_mps must be finite and non-negative")
    if (
        not np.isfinite(maximum_steering_nrmse)
        or not np.isfinite(maximum_yaw_rate_nrmse)
        or maximum_steering_nrmse <= 0.0
        or maximum_yaw_rate_nrmse <= 0.0
    ):
        raise ValueError("lateral NRMSE gates must be finite and positive")
    finite = np.logical_and.reduce([np.isfinite(item) for item in values])
    applicable = (
        finite
        & (np.abs(values[4]) <= max_abs_yaw_rate_rps)
        & (values[3] >= minimum_speed_mps)
    )
    if int(np.count_nonzero(applicable)) < 3:
        raise ValueError("insufficient applicable lateral calibration samples")
    selected = [item[applicable] for item in values]
    selected_segment_ids = (
        None if raw_segment_ids is None else raw_segment_ids[applicable]
    )
    fit: DelayFitResult = estimate_steering_delay(
        selected[0],
        selected[1],
        selected[2],
        selected[3],
        selected[4],
        wheelbase_m=wheelbase_m,
        config=config,
        segment_ids=selected_segment_ids,
    )
    if fit.time_constant_sec is None or fit.steering_nrmse is None:
        raise AssertionError("steering delay fitter did not separate first-order lag")
    reasons = list(fit.validity_reasons)
    if fit.steering_nrmse >= maximum_steering_nrmse:
        reasons.append(f"steering_nrmse>={maximum_steering_nrmse}")
    if fit.yaw_rate_nrmse >= maximum_yaw_rate_nrmse:
        reasons.append(f"yaw_rate_nrmse>={maximum_yaw_rate_nrmse}")
    return LateralCalibration(
        pure_delay_sec=fit.delay_sec,
        time_constant_sec=fit.time_constant_sec,
        gain=1.0,
        bias_rad=0.0,
        valid_speed_range_mps=(float(np.min(selected[3])), float(np.max(selected[3]))),
        nrmse=fit.steering_nrmse,
        yaw_rate_nrmse=fit.yaw_rate_nrmse,
        correlation_peak=fit.correlation_peak,
        dynamic_sample_count=fit.dynamic_sample_count,
        total_sample_count=fit.total_sample_count,
        excluded_sample_count=int(len(values[0]) - np.count_nonzero(applicable)),
        individually_valid=fit.individual_valid and not reasons,
        validity_reasons=tuple(reasons),
        source_method=fit.method,
    )
