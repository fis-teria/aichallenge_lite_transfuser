from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from statistics import median
from typing import Sequence

from .lateral import LateralCalibration
from .longitudinal import LongitudinalModeFit


@dataclass(frozen=True)
class CalibrationStabilityLimits:
    """Cross-run gates; time is seconds and errors keep each fit's SI normalization."""

    maximum_delay_span_sec: float
    maximum_lag_span_sec: float
    maximum_relative_gain_span: float
    maximum_bias_span: float
    minimum_correlation: float
    maximum_nrmse: float
    minimum_samples_per_cohort: int
    maximum_yaw_rate_nrmse: float | None = None

    def validate(self) -> None:
        numeric = (
            self.maximum_delay_span_sec,
            self.maximum_lag_span_sec,
            self.maximum_relative_gain_span,
            self.maximum_bias_span,
            self.minimum_correlation,
            self.maximum_nrmse,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("calibration stability limits must be finite")
        if any(value < 0.0 for value in numeric[:4]):
            raise ValueError("calibration stability spans must be non-negative")
        if not -1.0 <= self.minimum_correlation <= 1.0:
            raise ValueError("minimum_correlation must be in [-1,1]")
        if self.maximum_nrmse <= 0.0 or self.minimum_samples_per_cohort < 3:
            raise ValueError("NRMSE and sample limits are invalid")
        if self.maximum_yaw_rate_nrmse is not None and (
            not math.isfinite(self.maximum_yaw_rate_nrmse)
            or self.maximum_yaw_rate_nrmse <= 0.0
        ):
            raise ValueError("maximum_yaw_rate_nrmse must be finite and positive")


@dataclass(frozen=True)
class CalibrationStabilityResult:
    mode: str
    cohort_count: int
    delay_span_sec: float
    lag_span_sec: float
    relative_gain_span: float
    bias_span: float
    minimum_correlation: float
    maximum_nrmse: float
    maximum_yaw_rate_nrmse: float | None
    minimum_samples_per_cohort: int
    passed: bool
    reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def evaluate_lateral_stability(
    cohorts: Sequence[LateralCalibration],
    limits: CalibrationStabilityLimits,
) -> CalibrationStabilityResult:
    """Evaluate independently fitted steering cohorts without averaging them away."""

    if limits.maximum_yaw_rate_nrmse is None:
        raise ValueError("lateral stability requires a yaw-rate NRMSE limit")
    return _evaluate(
        "steering",
        cohorts,
        limits,
        biases=[item.bias_rad for item in cohorts],
        samples=[item.dynamic_sample_count for item in cohorts],
        yaw_nrmse=[item.yaw_rate_nrmse for item in cohorts],
    )


def evaluate_longitudinal_stability(
    cohorts: Sequence[LongitudinalModeFit],
    limits: CalibrationStabilityLimits,
    *,
    mode: str,
) -> CalibrationStabilityResult:
    """Evaluate drive or brake cohorts; mixing the two actuator modes is rejected."""

    if mode not in {"drive", "brake"}:
        raise ValueError("longitudinal stability mode must be drive or brake")
    if any(item.mode != mode for item in cohorts):
        raise ValueError(f"{mode} stability received a fit from another mode")
    return _evaluate(
        mode,
        cohorts,
        limits,
        biases=[item.bias_mps2 for item in cohorts],
        samples=[item.mode_sample_count for item in cohorts],
        yaw_nrmse=None,
    )


def _evaluate(
    mode: str,
    cohorts: Sequence[LateralCalibration] | Sequence[LongitudinalModeFit],
    limits: CalibrationStabilityLimits,
    *,
    biases: Sequence[float],
    samples: Sequence[int],
    yaw_nrmse: Sequence[float] | None,
) -> CalibrationStabilityResult:
    limits.validate()
    if len(cohorts) < 2:
        raise ValueError("cross-run stability requires at least two independent cohorts")
    if any(not item.individually_valid for item in cohorts):
        raise ValueError("cross-run stability requires individually valid cohort fits")
    vectors = [
        [item.pure_delay_sec for item in cohorts],
        [item.time_constant_sec for item in cohorts],
        [item.gain for item in cohorts],
        list(biases),
        [item.correlation_peak for item in cohorts],
        [item.nrmse for item in cohorts],
    ]
    if yaw_nrmse is not None:
        vectors.append(list(yaw_nrmse))
    if not all(math.isfinite(value) for vector in vectors for value in vector):
        raise ValueError("cohort fit metrics must be finite")
    delay_span = _span(vectors[0])
    lag_span = _span(vectors[1])
    gain_denominator = abs(median(vectors[2]))
    relative_gain_span = math.inf if gain_denominator <= 1e-12 else _span(vectors[2]) / gain_denominator
    bias_span = _span(vectors[3])
    minimum_correlation = min(vectors[4])
    maximum_nrmse = max(vectors[5])
    maximum_yaw = None if yaw_nrmse is None else max(yaw_nrmse)
    minimum_samples = min(samples)
    checks = (
        (delay_span <= limits.maximum_delay_span_sec, "delay_span"),
        (lag_span <= limits.maximum_lag_span_sec, "lag_span"),
        (relative_gain_span <= limits.maximum_relative_gain_span, "relative_gain_span"),
        (bias_span <= limits.maximum_bias_span, "bias_span"),
        (minimum_correlation >= limits.minimum_correlation, "correlation"),
        (maximum_nrmse <= limits.maximum_nrmse, "nrmse"),
        (minimum_samples >= limits.minimum_samples_per_cohort, "sample_count"),
    )
    reasons = [name for passed, name in checks if not passed]
    if maximum_yaw is not None and maximum_yaw > float(limits.maximum_yaw_rate_nrmse):
        reasons.append("yaw_rate_nrmse")
    return CalibrationStabilityResult(
        mode=mode,
        cohort_count=len(cohorts),
        delay_span_sec=delay_span,
        lag_span_sec=lag_span,
        relative_gain_span=relative_gain_span,
        bias_span=bias_span,
        minimum_correlation=minimum_correlation,
        maximum_nrmse=maximum_nrmse,
        maximum_yaw_rate_nrmse=maximum_yaw,
        minimum_samples_per_cohort=minimum_samples,
        passed=not reasons,
        reasons=tuple(reasons),
    )


def _span(values: Sequence[float]) -> float:
    return max(values) - min(values)
