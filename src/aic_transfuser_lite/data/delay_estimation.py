from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class DelayEstimationConfig:
    """Grid-search contract for steering response delay.

    Time is in seconds, steering is in radians, speed is in metres per second,
    and yaw rate is in radians per second. Inputs must already share one
    strictly increasing time grid.
    """

    tau_min_sec: float = 0.0
    tau_max_sec: float = 0.5
    tau_step_sec: float = 0.01
    time_constant_min_sec: float = 0.02
    time_constant_max_sec: float = 0.5
    time_constant_step_sec: float = 0.01
    minimum_dynamic_samples: int = 500
    minimum_correlation: float = 0.7
    minimum_dynamic_angle_rad: float = 0.02
    minimum_dynamic_rate_rps: float = 0.02

    def validate(self) -> None:
        if not 0.0 <= self.tau_min_sec <= self.tau_max_sec:
            raise ValueError("tau bounds must satisfy 0 <= min <= max")
        if self.tau_step_sec <= 0.0:
            raise ValueError("tau_step_sec must be positive")
        if not 0.0 < self.time_constant_min_sec <= self.time_constant_max_sec:
            raise ValueError("time constant bounds must satisfy 0 < min <= max")
        if self.time_constant_step_sec <= 0.0:
            raise ValueError("time_constant_step_sec must be positive")
        if self.minimum_dynamic_samples <= 0:
            raise ValueError("minimum_dynamic_samples must be positive")
        if not -1.0 <= self.minimum_correlation <= 1.0:
            raise ValueError("minimum_correlation must be in [-1, 1]")
        if self.minimum_dynamic_angle_rad < 0.0:
            raise ValueError("minimum_dynamic_angle_rad must be non-negative")
        if self.minimum_dynamic_rate_rps < 0.0:
            raise ValueError("minimum_dynamic_rate_rps must be non-negative")


@dataclass(frozen=True)
class DelayFitResult:
    method: str
    delay_sec: float
    time_constant_sec: float | None
    objective: float
    steering_nrmse: float | None
    yaw_rate_nrmse: float
    correlation_peak: float
    dynamic_sample_count: int
    total_sample_count: int
    individual_valid: bool
    validity_reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class DelayConsistencyAssessment:
    run_count: int
    individually_valid_run_count: int
    median_delay_sec: float | None
    max_deviation_sec: float | None
    required_run_count: int
    allowed_deviation_sec: float
    run_consistent: tuple[bool, ...]
    dataset_valid: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _as_finite_vector(name: str, values: Sequence[float] | np.ndarray) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 1:
        raise ValueError(f"{name} must be one-dimensional, got shape={result.shape}")
    if result.size < 3:
        raise ValueError(f"{name} must contain at least three samples")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains NaN or infinity")
    return result


def _validated_signals(
    timestamps_sec: Sequence[float] | np.ndarray,
    *signals: tuple[str, Sequence[float] | np.ndarray],
) -> tuple[np.ndarray, ...]:
    timestamps = _as_finite_vector("timestamps_sec", timestamps_sec)
    if np.any(np.diff(timestamps) <= 0.0):
        raise ValueError("timestamps_sec must be strictly increasing")
    vectors = [timestamps]
    for name, values in signals:
        vector = _as_finite_vector(name, values)
        if vector.shape != timestamps.shape:
            raise ValueError(
                f"{name} shape={vector.shape} does not match timestamps shape={timestamps.shape}"
            )
        vectors.append(vector)
    return tuple(vectors)


def validated_time_segment_slices(
    timestamps_sec: Sequence[float] | np.ndarray,
    segment_ids: Sequence[str] | np.ndarray | None,
) -> tuple[slice, ...]:
    """Return contiguous slices whose timestamps are strictly increasing.

    Separate calibration runs may reset ROS time. A segment identifier may
    appear in only one contiguous block so disjoint windows cannot share
    actuator state accidentally.
    """

    timestamps = _as_finite_vector("timestamps_sec", timestamps_sec)
    if segment_ids is None:
        if np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("timestamps_sec must be strictly increasing")
        return (slice(0, len(timestamps)),)
    identifiers = np.asarray(segment_ids, dtype=object)
    if identifiers.ndim != 1 or identifiers.shape != timestamps.shape:
        raise ValueError("segment_ids must match the one-dimensional timestamp shape")
    boundaries = np.flatnonzero(identifiers[1:] != identifiers[:-1]) + 1
    starts = np.concatenate(([0], boundaries))
    stops = np.concatenate((boundaries, [len(timestamps)]))
    slices: list[slice] = []
    seen: set[str] = set()
    for start, stop in zip(starts, stops, strict=True):
        identifier = str(identifiers[int(start)])
        if not identifier:
            raise ValueError("segment_ids must not contain empty identifiers")
        if identifier in seen:
            raise ValueError(f"segment_id {identifier!r} is not contiguous")
        seen.add(identifier)
        if int(stop) - int(start) < 3:
            raise ValueError(f"segment_id {identifier!r} must contain at least three samples")
        selected = timestamps[int(start) : int(stop)]
        if np.any(np.diff(selected) <= 0.0):
            raise ValueError(
                f"timestamps_sec must be strictly increasing within segment {identifier!r}"
            )
        slices.append(slice(int(start), int(stop)))
    return tuple(slices)


def _inclusive_grid(minimum: float, maximum: float, step: float) -> np.ndarray:
    count = int(math.floor((maximum - minimum) / step + 1e-9)) + 1
    values = minimum + np.arange(count, dtype=np.float64) * step
    if values[-1] < maximum - step * 1e-6:
        values = np.append(values, maximum)
    return values


def _shifted_hold(
    timestamps_sec: np.ndarray, signal: np.ndarray, delay_sec: float
) -> np.ndarray:
    query = timestamps_sec - delay_sec
    indices = np.searchsorted(timestamps_sec, query, side="right") - 1
    indices = np.clip(indices, 0, len(signal) - 1)
    return signal[indices]


def _first_order_response(
    timestamps_sec: np.ndarray,
    delayed_command_rad: np.ndarray,
    time_constant_sec: float,
    initial_steering_rad: float,
) -> np.ndarray:
    output = np.empty_like(delayed_command_rad)
    output[0] = initial_steering_rad
    for index in range(1, len(output)):
        dt_sec = timestamps_sec[index] - timestamps_sec[index - 1]
        alpha = 1.0 - math.exp(-dt_sec / time_constant_sec)
        output[index] = output[index - 1] + alpha * (
            delayed_command_rad[index] - output[index - 1]
        )
    return output


def _normalized_rmse(prediction: np.ndarray, target: np.ndarray) -> float:
    rmse = float(np.sqrt(np.mean(np.square(prediction - target))))
    scale = max(float(np.std(target)), 1e-6)
    return rmse / scale


def _correlation(left: np.ndarray, right: np.ndarray) -> float:
    left_centered = left - np.mean(left)
    right_centered = right - np.mean(right)
    denominator = float(
        np.sqrt(np.sum(np.square(left_centered)) * np.sum(np.square(right_centered)))
    )
    if denominator <= 1e-12:
        return 0.0
    return float(np.sum(left_centered * right_centered) / denominator)


def _dynamic_mask(
    timestamps_sec: np.ndarray,
    command_rad: np.ndarray,
    config: DelayEstimationConfig,
) -> np.ndarray:
    rate_rps = np.gradient(command_rad, timestamps_sec)
    centered = command_rad - float(np.median(command_rad))
    return (np.abs(centered) >= config.minimum_dynamic_angle_rad) | (
        np.abs(rate_rps) >= config.minimum_dynamic_rate_rps
    )


def _validity(
    dynamic_sample_count: int,
    correlation_peak: float,
    config: DelayEstimationConfig,
) -> tuple[bool, tuple[str, ...]]:
    reasons: list[str] = []
    if dynamic_sample_count < config.minimum_dynamic_samples:
        reasons.append(
            f"dynamic_sample_count<{config.minimum_dynamic_samples}"
        )
    if correlation_peak <= config.minimum_correlation:
        reasons.append(f"correlation_peak<={config.minimum_correlation}")
    return not reasons, tuple(reasons)


def estimate_steering_delay(
    timestamps_sec: Sequence[float] | np.ndarray,
    command_steering_rad: Sequence[float] | np.ndarray,
    actual_steering_rad: Sequence[float] | np.ndarray,
    speed_mps: Sequence[float] | np.ndarray,
    yaw_rate_rps: Sequence[float] | np.ndarray,
    *,
    wheelbase_m: float,
    config: DelayEstimationConfig | None = None,
    segment_ids: Sequence[str] | np.ndarray | None = None,
) -> DelayFitResult:
    """Fit pure delay and first-order steering response on one or more runs."""

    selected = config or DelayEstimationConfig()
    selected.validate()
    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive")
    if segment_ids is None:
        timestamps, command, actual, speed, yaw_rate = _validated_signals(
            timestamps_sec,
            ("command_steering_rad", command_steering_rad),
            ("actual_steering_rad", actual_steering_rad),
            ("speed_mps", speed_mps),
            ("yaw_rate_rps", yaw_rate_rps),
        )
    else:
        timestamps = _as_finite_vector("timestamps_sec", timestamps_sec)
        signals: list[np.ndarray] = []
        for name, values in (
            ("command_steering_rad", command_steering_rad),
            ("actual_steering_rad", actual_steering_rad),
            ("speed_mps", speed_mps),
            ("yaw_rate_rps", yaw_rate_rps),
        ):
            vector = _as_finite_vector(name, values)
            if vector.shape != timestamps.shape:
                raise ValueError(
                    f"{name} shape={vector.shape} does not match timestamps shape={timestamps.shape}"
                )
            signals.append(vector)
        command, actual, speed, yaw_rate = signals
    segments = validated_time_segment_slices(timestamps, segment_ids)
    dynamic = np.zeros(len(timestamps), dtype=np.bool_)
    for segment in segments:
        dynamic[segment] = _dynamic_mask(timestamps[segment], command[segment], selected)
    taus = _inclusive_grid(selected.tau_min_sec, selected.tau_max_sec, selected.tau_step_sec)
    constants = _inclusive_grid(
        selected.time_constant_min_sec,
        selected.time_constant_max_sec,
        selected.time_constant_step_sec,
    )
    best: tuple[float, float, float, float, float, np.ndarray] | None = None
    for delay_sec in taus:
        delayed_command = np.empty_like(command)
        valid_start = np.zeros(len(timestamps), dtype=np.bool_)
        for segment in segments:
            delayed_command[segment] = _shifted_hold(
                timestamps[segment], command[segment], float(delay_sec)
            )
            valid_start[segment] = (
                timestamps[segment] >= timestamps[segment.start] + float(delay_sec)
            )
        score_mask = dynamic & valid_start
        if int(np.count_nonzero(score_mask)) < 3:
            score_mask = valid_start
        for time_constant_sec in constants:
            predicted = np.empty_like(command)
            for segment in segments:
                predicted[segment] = _first_order_response(
                    timestamps[segment],
                    delayed_command[segment],
                    float(time_constant_sec),
                    float(actual[segment.start]),
                )
            predicted_yaw = speed * np.tan(predicted) / wheelbase_m
            steering_nrmse = _normalized_rmse(predicted[score_mask], actual[score_mask])
            yaw_nrmse = _normalized_rmse(predicted_yaw[score_mask], yaw_rate[score_mask])
            objective = 0.5 * (steering_nrmse + yaw_nrmse)
            candidate = (
                objective,
                float(delay_sec),
                float(time_constant_sec),
                steering_nrmse,
                yaw_nrmse,
                predicted,
            )
            if best is None or candidate[:3] < best[:3]:
                best = candidate
    if best is None:
        raise RuntimeError("Delay grid search produced no candidate")
    objective, delay_sec, time_constant_sec, steering_nrmse, yaw_nrmse, predicted = best
    valid_start = np.zeros(len(timestamps), dtype=np.bool_)
    for segment in segments:
        valid_start[segment] = timestamps[segment] >= timestamps[segment.start] + delay_sec
    score_mask = dynamic & valid_start
    if int(np.count_nonzero(score_mask)) < 3:
        score_mask = valid_start
    correlation = _correlation(predicted[score_mask], actual[score_mask])
    dynamic_count = int(np.count_nonzero(dynamic))
    valid, reasons = _validity(dynamic_count, correlation, selected)
    return DelayFitResult(
        method=(
            "command_to_actual_first_order_and_yaw"
            if segment_ids is None
            else "segmented_command_to_actual_first_order_and_yaw"
        ),
        delay_sec=delay_sec,
        time_constant_sec=time_constant_sec,
        objective=float(objective),
        steering_nrmse=float(steering_nrmse),
        yaw_rate_nrmse=float(yaw_nrmse),
        correlation_peak=correlation,
        dynamic_sample_count=dynamic_count,
        total_sample_count=int(len(timestamps)),
        individual_valid=valid,
        validity_reasons=reasons,
    )


def estimate_combined_yaw_delay(
    timestamps_sec: Sequence[float] | np.ndarray,
    command_steering_rad: Sequence[float] | np.ndarray,
    speed_mps: Sequence[float] | np.ndarray,
    yaw_rate_rps: Sequence[float] | np.ndarray,
    *,
    wheelbase_m: float,
    config: DelayEstimationConfig | None = None,
) -> DelayFitResult:
    """Estimate transport+actuator+vehicle delay when actual steering is absent."""

    selected = config or DelayEstimationConfig()
    selected.validate()
    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive")
    timestamps, command, speed, yaw_rate = _validated_signals(
        timestamps_sec,
        ("command_steering_rad", command_steering_rad),
        ("speed_mps", speed_mps),
        ("yaw_rate_rps", yaw_rate_rps),
    )
    dynamic = _dynamic_mask(timestamps, command, selected)
    best: tuple[float, float, float] | None = None
    for delay_sec in _inclusive_grid(
        selected.tau_min_sec, selected.tau_max_sec, selected.tau_step_sec
    ):
        delayed = _shifted_hold(timestamps, command, float(delay_sec))
        predicted_yaw = speed * np.tan(delayed) / wheelbase_m
        score_mask = dynamic & (timestamps >= timestamps[0] + float(delay_sec))
        if int(np.count_nonzero(score_mask)) < 3:
            score_mask = timestamps >= timestamps[0] + float(delay_sec)
        correlation = _correlation(predicted_yaw[score_mask], yaw_rate[score_mask])
        yaw_nrmse = _normalized_rmse(predicted_yaw[score_mask], yaw_rate[score_mask])
        candidate = (-correlation, yaw_nrmse, float(delay_sec))
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise RuntimeError("Yaw delay grid search produced no candidate")
    negative_correlation, yaw_nrmse, delay_sec = best
    correlation = -negative_correlation
    dynamic_count = int(np.count_nonzero(dynamic))
    valid, reasons = _validity(dynamic_count, correlation, selected)
    return DelayFitResult(
        method="command_to_yaw_combined_delay",
        delay_sec=delay_sec,
        time_constant_sec=None,
        objective=float(yaw_nrmse),
        steering_nrmse=None,
        yaw_rate_nrmse=float(yaw_nrmse),
        correlation_peak=float(correlation),
        dynamic_sample_count=dynamic_count,
        total_sample_count=int(len(timestamps)),
        individual_valid=valid,
        validity_reasons=reasons,
    )


def assess_delay_consistency(
    results: Sequence[DelayFitResult],
    *,
    minimum_runs: int = 5,
    max_deviation_sec: float = 0.10,
) -> DelayConsistencyAssessment:
    """Apply the across-run median-delay acceptance rule without mutating fits."""

    if minimum_runs <= 0:
        raise ValueError("minimum_runs must be positive")
    if max_deviation_sec < 0.0:
        raise ValueError("max_deviation_sec must be non-negative")
    valid_delays = [result.delay_sec for result in results if result.individual_valid]
    median = float(np.median(valid_delays)) if valid_delays else None
    consistency = tuple(
        bool(
            result.individual_valid
            and median is not None
            and abs(result.delay_sec - median) < max_deviation_sec
        )
        for result in results
    )
    deviations = (
        [abs(result.delay_sec - median) for result in results if result.individual_valid]
        if median is not None
        else []
    )
    dataset_valid = (
        len(results) >= minimum_runs
        and len(valid_delays) == len(results)
        and all(consistency)
    )
    return DelayConsistencyAssessment(
        run_count=len(results),
        individually_valid_run_count=len(valid_delays),
        median_delay_sec=median,
        max_deviation_sec=max(deviations) if deviations else None,
        required_run_count=minimum_runs,
        allowed_deviation_sec=max_deviation_sec,
        run_consistent=consistency,
        dataset_valid=dataset_valid,
    )
