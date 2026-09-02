from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Literal, Sequence

import numpy as np


LongitudinalMode = Literal["drive", "brake"]


@dataclass(frozen=True)
class LongitudinalFitConfig:
    delay_min_sec: float = 0.0
    delay_max_sec: float = 0.5
    delay_step_sec: float = 0.01
    time_constant_min_sec: float = 0.02
    time_constant_max_sec: float = 1.0
    time_constant_step_sec: float = 0.02
    mode_hysteresis_mps2: float = 0.1
    minimum_mode_samples: int = 150
    minimum_command_span_mps2: float = 0.25
    minimum_correlation: float = 0.3
    max_abs_actual_accel_mps2: float = 15.0

    def validate(self) -> None:
        numeric = (
            self.delay_min_sec,
            self.delay_max_sec,
            self.delay_step_sec,
            self.time_constant_min_sec,
            self.time_constant_max_sec,
            self.time_constant_step_sec,
            self.mode_hysteresis_mps2,
            self.minimum_command_span_mps2,
            self.minimum_correlation,
            self.max_abs_actual_accel_mps2,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("longitudinal fit configuration must be finite")
        if not 0.0 <= self.delay_min_sec <= self.delay_max_sec:
            raise ValueError("delay bounds must satisfy 0 <= min <= max")
        if self.delay_step_sec <= 0.0:
            raise ValueError("delay_step_sec must be positive")
        if not 0.0 < self.time_constant_min_sec <= self.time_constant_max_sec:
            raise ValueError("time constant bounds must be positive and ordered")
        if self.time_constant_step_sec <= 0.0:
            raise ValueError("time_constant_step_sec must be positive")
        if self.mode_hysteresis_mps2 < 0.0:
            raise ValueError("mode_hysteresis_mps2 must be non-negative")
        if self.minimum_mode_samples < 3:
            raise ValueError("minimum_mode_samples must be at least three")
        if self.minimum_command_span_mps2 < 0.0:
            raise ValueError("minimum_command_span_mps2 must be non-negative")
        if not -1.0 <= self.minimum_correlation <= 1.0:
            raise ValueError("minimum_correlation must be in [-1,1]")
        if self.max_abs_actual_accel_mps2 <= 0.0:
            raise ValueError("max_abs_actual_accel_mps2 must be positive")


@dataclass(frozen=True)
class LongitudinalModeFit:
    mode: LongitudinalMode
    pure_delay_sec: float
    time_constant_sec: float
    gain: float
    bias_mps2: float
    valid_speed_range_mps: tuple[float, float]
    command_range_mps2: tuple[float, float]
    rmse_mps2: float
    nrmse: float
    correlation_peak: float
    mode_sample_count: int
    total_sample_count: int
    excluded_actual_accel_count: int
    individually_valid: bool
    validity_reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class LongitudinalCalibration:
    drive: LongitudinalModeFit
    brake: LongitudinalModeFit


def _vectors(
    timestamps_sec: Sequence[float] | np.ndarray,
    command_accel_mps2: Sequence[float] | np.ndarray,
    actual_accel_mps2: Sequence[float] | np.ndarray,
    speed_mps: Sequence[float] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    values = tuple(
        np.asarray(item, dtype=np.float64)
        for item in (
            timestamps_sec,
            command_accel_mps2,
            actual_accel_mps2,
            speed_mps,
        )
    )
    if any(item.ndim != 1 for item in values):
        raise ValueError("longitudinal calibration inputs must be one-dimensional")
    if len(values[0]) < 3 or any(item.shape != values[0].shape for item in values[1:]):
        raise ValueError("longitudinal calibration inputs must have equal length >= 3")
    if not all(np.isfinite(item).all() for item in values):
        raise ValueError("longitudinal calibration inputs must be finite")
    if np.any(np.diff(values[0]) <= 0.0):
        raise ValueError("timestamps_sec must be strictly increasing")
    return values


def derive_actual_acceleration(
    timestamps_sec: Sequence[float] | np.ndarray,
    speed_mps: Sequence[float] | np.ndarray,
    *,
    smoothing_samples: int = 5,
) -> np.ndarray:
    """Differentiate measured speed and apply an edge-preserving moving mean."""

    timestamps = np.asarray(timestamps_sec, dtype=np.float64)
    speed = np.asarray(speed_mps, dtype=np.float64)
    if timestamps.ndim != 1 or speed.shape != timestamps.shape or len(speed) < 3:
        raise ValueError("timestamps and speed must be equal one-dimensional vectors")
    if not np.isfinite(timestamps).all() or not np.isfinite(speed).all():
        raise ValueError("timestamps and speed must be finite")
    if np.any(np.diff(timestamps) <= 0.0):
        raise ValueError("timestamps_sec must be strictly increasing")
    if smoothing_samples <= 0 or smoothing_samples % 2 == 0:
        raise ValueError("smoothing_samples must be a positive odd integer")
    acceleration = np.gradient(speed, timestamps)
    if smoothing_samples == 1:
        return acceleration
    half = smoothing_samples // 2
    padded = np.pad(acceleration, (half, half), mode="edge")
    kernel = np.full(smoothing_samples, 1.0 / smoothing_samples)
    return np.convolve(padded, kernel, mode="valid")


def _grid(minimum: float, maximum: float, step: float) -> np.ndarray:
    count = int(math.floor((maximum - minimum) / step + 1e-9)) + 1
    values = minimum + np.arange(count, dtype=np.float64) * step
    return np.append(values, maximum) if values[-1] < maximum - step * 1e-6 else values


def _delayed_hold(timestamps: np.ndarray, values: np.ndarray, delay_sec: float) -> np.ndarray:
    indices = np.searchsorted(timestamps, timestamps - delay_sec, side="right") - 1
    return values[np.clip(indices, 0, len(values) - 1)]


def _first_order(
    timestamps: np.ndarray, values: np.ndarray, time_constant_sec: float
) -> np.ndarray:
    filtered = np.empty_like(values)
    filtered[0] = values[0]
    for index in range(1, len(values)):
        alpha = 1.0 - math.exp(
            -(timestamps[index] - timestamps[index - 1]) / time_constant_sec
        )
        filtered[index] = filtered[index - 1] + alpha * (
            values[index] - filtered[index - 1]
        )
    return filtered


def _correlation(left: np.ndarray, right: np.ndarray) -> float:
    centered_left = left - np.mean(left)
    centered_right = right - np.mean(right)
    denominator = float(
        np.sqrt(np.sum(centered_left**2) * np.sum(centered_right**2))
    )
    return 0.0 if denominator <= 1e-12 else float(
        np.sum(centered_left * centered_right) / denominator
    )


def fit_longitudinal_mode(
    timestamps_sec: Sequence[float] | np.ndarray,
    command_accel_mps2: Sequence[float] | np.ndarray,
    actual_accel_mps2: Sequence[float] | np.ndarray,
    speed_mps: Sequence[float] | np.ndarray,
    *,
    mode: LongitudinalMode,
    config: LongitudinalFitConfig | None = None,
) -> LongitudinalModeFit:
    """Grid-fit pure delay and first-order lag, then linear gain and bias."""

    selected = config or LongitudinalFitConfig()
    selected.validate()
    if mode not in ("drive", "brake"):
        raise ValueError(f"unsupported longitudinal mode: {mode!r}")
    timestamps, command, actual, speed = _vectors(
        timestamps_sec, command_accel_mps2, actual_accel_mps2, speed_mps
    )
    mode_mask = (
        command > selected.mode_hysteresis_mps2
        if mode == "drive"
        else command < -selected.mode_hysteresis_mps2
    )
    plausible_actual = np.abs(actual) <= selected.max_abs_actual_accel_mps2
    excluded = int(np.count_nonzero(mode_mask & ~plausible_actual))
    best: tuple[float, float, float, float, float, float, float] | None = None
    for delay in _grid(
        selected.delay_min_sec, selected.delay_max_sec, selected.delay_step_sec
    ):
        delayed = _delayed_hold(timestamps, command, float(delay))
        valid_start = timestamps >= timestamps[0] + float(delay)
        mask = mode_mask & plausible_actual & valid_start
        if int(np.count_nonzero(mask)) < 3:
            continue
        for time_constant in _grid(
            selected.time_constant_min_sec,
            selected.time_constant_max_sec,
            selected.time_constant_step_sec,
        ):
            filtered = _first_order(timestamps, delayed, float(time_constant))
            design = np.column_stack((filtered[mask], np.ones(np.count_nonzero(mask))))
            gain, bias = np.linalg.lstsq(design, actual[mask], rcond=None)[0]
            prediction = gain * filtered[mask] + bias
            rmse = float(np.sqrt(np.mean((prediction - actual[mask]) ** 2)))
            scale = max(float(np.std(actual[mask])), 1e-6)
            nrmse = rmse / scale
            correlation = _correlation(prediction, actual[mask])
            candidate = (
                nrmse,
                -correlation,
                float(delay),
                float(time_constant),
                float(gain),
                float(bias),
                rmse,
            )
            if best is None or candidate[:4] < best[:4]:
                best = candidate
    if best is None:
        raise ValueError(f"insufficient {mode} samples for longitudinal fit")
    nrmse, negative_correlation, delay, time_constant, gain, bias, rmse = best
    fit_mask = mode_mask & plausible_actual & (timestamps >= timestamps[0] + delay)
    sample_count = int(np.count_nonzero(fit_mask))
    command_range = (
        float(np.min(command[fit_mask])),
        float(np.max(command[fit_mask])),
    )
    speed_range = (
        float(np.min(speed[fit_mask])),
        float(np.max(speed[fit_mask])),
    )
    correlation = -negative_correlation
    reasons: list[str] = []
    if sample_count < selected.minimum_mode_samples:
        reasons.append(f"mode_sample_count<{selected.minimum_mode_samples}")
    if command_range[1] - command_range[0] < selected.minimum_command_span_mps2:
        reasons.append(f"command_span<{selected.minimum_command_span_mps2}")
    if correlation <= selected.minimum_correlation:
        reasons.append(f"correlation_peak<={selected.minimum_correlation}")
    if gain <= 0.0:
        reasons.append("gain<=0")
    return LongitudinalModeFit(
        mode=mode,
        pure_delay_sec=delay,
        time_constant_sec=time_constant,
        gain=gain,
        bias_mps2=bias,
        valid_speed_range_mps=speed_range,
        command_range_mps2=command_range,
        rmse_mps2=rmse,
        nrmse=nrmse,
        correlation_peak=correlation,
        mode_sample_count=sample_count,
        total_sample_count=len(timestamps),
        excluded_actual_accel_count=excluded,
        individually_valid=not reasons,
        validity_reasons=tuple(reasons),
    )


def fit_longitudinal_calibration(
    timestamps_sec: Sequence[float] | np.ndarray,
    command_accel_mps2: Sequence[float] | np.ndarray,
    actual_accel_mps2: Sequence[float] | np.ndarray,
    speed_mps: Sequence[float] | np.ndarray,
    *,
    config: LongitudinalFitConfig | None = None,
) -> LongitudinalCalibration:
    return LongitudinalCalibration(
        drive=fit_longitudinal_mode(
            timestamps_sec,
            command_accel_mps2,
            actual_accel_mps2,
            speed_mps,
            mode="drive",
            config=config,
        ),
        brake=fit_longitudinal_mode(
            timestamps_sec,
            command_accel_mps2,
            actual_accel_mps2,
            speed_mps,
            mode="brake",
            config=config,
        ),
    )
