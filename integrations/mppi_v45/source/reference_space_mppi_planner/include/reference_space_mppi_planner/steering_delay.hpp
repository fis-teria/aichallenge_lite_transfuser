#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace reference_space_mppi_planner::mppi {

struct TimedSteeringCommand {
  double stamp_sec{0.0};
  double target_steering_rad{0.0};
  std::uint64_t input_sequence{0};
};

template <std::size_t Capacity> struct PendingSteeringTargets {
  bool valid{false};
  std::array<double, Capacity> targets_rad{};
  // Zero sequence identifies the fallback initial steering, not a received command.
  std::array<double, Capacity> source_stamps_sec{};
  std::array<std::uint64_t, Capacity> source_sequences{};
  std::size_t count{0U};
};

template <std::size_t HistoryCapacity, std::size_t OutputCapacity>
PendingSteeringTargets<OutputCapacity> samplePendingSteeringTargets(
    const std::array<TimedSteeringCommand, HistoryCapacity> &history,
    std::size_t history_count, double now_sec, double delay_sec, double dt_sec,
    double fallback_steering_rad) noexcept {
  PendingSteeringTargets<OutputCapacity> output;
  if (!std::isfinite(now_sec) || !std::isfinite(delay_sec) || delay_sec < 0.0 ||
      !std::isfinite(dt_sec) || dt_sec <= 0.0 ||
      !std::isfinite(fallback_steering_rad) ||
      history_count > HistoryCapacity) {
    return output;
  }
  const double bounded_steps =
      std::min(delay_sec / dt_sec, static_cast<double>(OutputCapacity));
  output.count = static_cast<std::size_t>(
      std::ceil(std::max(0.0, bounded_steps - 1.0e-12)));
  for (std::size_t step = 0U; step < output.count; ++step) {
    const double command_stamp =
        now_sec + static_cast<double>(step) * dt_sec - delay_sec;
    double target = fallback_steering_rad;
    for (std::size_t index = 0U; index < history_count; ++index) {
      const auto &sample = history[index];
      if (!std::isfinite(sample.stamp_sec) ||
          !std::isfinite(sample.target_steering_rad)) {
        return PendingSteeringTargets<OutputCapacity>{};
      }
      if (sample.stamp_sec > command_stamp + 1.0e-9) {
        break;
      }
      target = sample.target_steering_rad;
      output.source_stamps_sec[step] = sample.stamp_sec;
      output.source_sequences[step] = sample.input_sequence;
    }
    output.targets_rad[step] = target;
  }
  output.valid = true;
  return output;
}

template <std::size_t Capacity> class SteeringCommandDelayLine {
public:
  static_assert(Capacity > 0U, "steering delay line must have capacity");

  struct Output {
    bool valid{false};
    double delayed_target_steering_rad{0.0};
    std::size_t delay_steps{0U};
  };

  void reset(double initial_steering_rad) noexcept {
    initial_steering_rad_ = initial_steering_rad;
    pending_target_count_ = 0U;
    command_count_ = 0U;
  }

  void reset(double initial_steering_rad, const double *pending_targets_rad,
             std::size_t pending_target_count) noexcept {
    reset(initial_steering_rad);
    if (pending_targets_rad == nullptr) {
      return;
    }
    pending_target_count_ = std::min(pending_target_count, Capacity);
    std::copy_n(pending_targets_rad, pending_target_count_,
                pending_targets_rad_.begin());
  }

  Output pushAndSelect(double commanded_steering_rad, double delay_sec,
                       double dt_sec) noexcept {
    Output output;
    if (!std::isfinite(initial_steering_rad_) ||
        !std::isfinite(commanded_steering_rad) || !std::isfinite(delay_sec) ||
        delay_sec < 0.0 || !std::isfinite(dt_sec) || dt_sec <= 0.0) {
      return output;
    }

    const double delay_steps =
        std::min(delay_sec / dt_sec, static_cast<double>(Capacity));
    output.delay_steps = static_cast<std::size_t>(
        std::ceil(std::max(0.0, delay_steps - 1.0e-12)));
    if (command_count_ < output.delay_steps) {
      output.delayed_target_steering_rad =
          command_count_ < pending_target_count_
              ? pending_targets_rad_[command_count_]
              : initial_steering_rad_;
    } else if (output.delay_steps == 0U) {
      output.delayed_target_steering_rad = commanded_steering_rad;
    } else {
      output.delayed_target_steering_rad =
          commands_[(command_count_ - output.delay_steps) % Capacity];
    }
    if (!std::isfinite(output.delayed_target_steering_rad)) {
      return Output{};
    }
    // Retain only the delay window, so a long maneuver uses the same model
    // without increasing every short-horizon request's fixed storage.
    commands_[command_count_ % Capacity] = commanded_steering_rad;
    ++command_count_;
    output.valid = true;
    return output;
  }

private:
  std::array<double, Capacity> commands_{};
  std::array<double, Capacity> pending_targets_rad_{};
  double initial_steering_rad_{0.0};
  std::size_t pending_target_count_{0U};
  std::size_t command_count_{0U};
};

} // namespace reference_space_mppi_planner::mppi
