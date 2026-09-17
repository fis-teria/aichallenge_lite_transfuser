#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace simple_pure_pursuit {

struct WallEdgeTrackingSpeedConfig {
  bool enabled{true};
  double lateral_velocity_preview_sec{0.20};
  double minimum_speed_mps{0.50};
  double deceleration_mps2{3.0};
  double release_reserve_ratio{0.75};
  double release_duration_sec{0.15};
};

struct WallEdgeTrackingSpeedState {
  bool deficit_latched{false};
  std::uint64_t latched_geometry_revision{0U};
  std::uint64_t release_geometry_revision{0U};
  double release_elapsed_sec{0.0};
};

struct WallEdgeTrackingSpeedResult {
  bool active{false};
  double tracking_reserve_consumption_m{0.0};
  double usable_corridor_reserve_m{0.0};
  double projected_lateral_error_m{0.0};
  double wall_tracking_reserve_consumption_m{0.0};
  double opponent_tracking_reserve_consumption_m{0.0};
  double usable_wall_reserve_m{0.0};
  double usable_opponent_reserve_m{0.0};
  double reserve_ratio{1.0};
  double speed_cap_mps{0.0};
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
};

inline WallEdgeTrackingSpeedResult finishWallEdgeTrackingSpeed(
    const WallEdgeTrackingSpeedConfig &config, const bool profile_active,
    const bool inputs_valid, const double current_speed_mps,
    const double requested_speed_mps, const double speed_proportional_gain,
    const double requested_acceleration_mps2,
    const std::uint64_t geometry_revision, const double elapsed_sec,
    WallEdgeTrackingSpeedState *state,
    WallEdgeTrackingSpeedResult result) noexcept {
  if (!inputs_valid) {
    if (state != nullptr && (!profile_active || !config.enabled)) {
      *state = WallEdgeTrackingSpeedState{};
    }
    return result;
  }

  if (result.usable_corridor_reserve_m <= 0.0) {
    state->deficit_latched = true;
    state->latched_geometry_revision = geometry_revision;
    state->release_geometry_revision = 0U;
    state->release_elapsed_sec = 0.0;
  } else if (state->deficit_latched) {
    const bool replacement_geometry =
        geometry_revision != 0U &&
        geometry_revision != state->latched_geometry_revision;
    const bool release_reserve_available =
        result.reserve_ratio + 1.0e-9 >= config.release_reserve_ratio;
    if (replacement_geometry && release_reserve_available) {
      if (state->release_geometry_revision != 0U &&
          geometry_revision < state->release_geometry_revision) {
        // A producer restart or out-of-order identity cannot satisfy release.
        state->release_geometry_revision = 0U;
        state->release_elapsed_sec = 0.0;
      } else {
        if (state->release_geometry_revision == 0U) {
          state->release_geometry_revision = geometry_revision;
          state->release_elapsed_sec = 0.0;
        }
        // Planner may publish a newly validated revision every cycle. Healthy
        // reserve is accumulated across monotonically newer revisions.
        state->release_elapsed_sec += elapsed_sec;
      }
      if (state->release_elapsed_sec + 1.0e-9 >= config.release_duration_sec) {
        *state = WallEdgeTrackingSpeedState{};
      }
    } else {
      state->release_geometry_revision = 0U;
      state->release_elapsed_sec = 0.0;
    }
  }
  const double minimum_speed_mps =
      std::min(config.minimum_speed_mps, requested_speed_mps);
  result.speed_cap_mps =
      state->deficit_latched
          ? minimum_speed_mps
          : minimum_speed_mps + (requested_speed_mps - minimum_speed_mps) *
                                    std::sqrt(result.reserve_ratio);
  result.speed_mps = std::min(requested_speed_mps, result.speed_cap_mps);
  result.active = result.speed_mps + 1.0e-9 < requested_speed_mps;
  if (result.active) {
    const double current_forward_speed_mps = std::abs(current_speed_mps);
    const double cap_tracking_acceleration_mps2 =
        std::max(-config.deceleration_mps2,
                 speed_proportional_gain *
                     (result.speed_cap_mps - current_forward_speed_mps));
    result.acceleration_mps2 =
        std::min(requested_acceleration_mps2, cap_tracking_acceleration_mps2);
  }
  return result;
}

// A wall-edge candidate is statically safe at publication time, but PP alone
// observes the delayed control pose and lateral motion while executing it.
// Consume the candidate's symmetric wall--opponent reserve with that live
// tracking tube and reduce only longitudinal authority. Steering is never
// clamped or rejected here.
inline WallEdgeTrackingSpeedResult applyWallEdgeTrackingSpeed(
    const WallEdgeTrackingSpeedConfig &config, const bool wall_edge_profile,
    const double corridor_reserve_m, const double desired_control_reserve_m,
    const double lateral_error_m, const double lateral_velocity_mps,
    const double current_speed_mps, const double requested_speed_mps,
    const double speed_proportional_gain,
    const double requested_acceleration_mps2,
    const std::uint64_t geometry_revision, const double elapsed_sec,
    WallEdgeTrackingSpeedState *state) noexcept {
  WallEdgeTrackingSpeedResult result;
  result.speed_cap_mps = requested_speed_mps;
  result.speed_mps = requested_speed_mps;
  result.acceleration_mps2 = requested_acceleration_mps2;

  const bool inputs_valid =
      config.enabled && wall_edge_profile &&
      std::isfinite(config.lateral_velocity_preview_sec) &&
      config.lateral_velocity_preview_sec >= 0.0 &&
      std::isfinite(config.minimum_speed_mps) &&
      config.minimum_speed_mps >= 0.0 &&
      std::isfinite(config.deceleration_mps2) &&
      config.deceleration_mps2 >= 0.0 && std::isfinite(corridor_reserve_m) &&
      std::isfinite(config.release_reserve_ratio) &&
      config.release_reserve_ratio >= 0.0 &&
      config.release_reserve_ratio <= 1.0 &&
      std::isfinite(config.release_duration_sec) &&
      config.release_duration_sec >= 0.0 && std::isfinite(elapsed_sec) &&
      elapsed_sec >= 0.0 && state != nullptr && corridor_reserve_m >= 0.0 &&
      std::isfinite(desired_control_reserve_m) &&
      desired_control_reserve_m > 0.0 && std::isfinite(lateral_error_m) &&
      std::isfinite(lateral_velocity_mps) && std::isfinite(current_speed_mps) &&
      std::isfinite(requested_speed_mps) && requested_speed_mps >= 0.0 &&
      std::isfinite(speed_proportional_gain) &&
      speed_proportional_gain >= 0.0 &&
      std::isfinite(requested_acceleration_mps2);
  if (!inputs_valid) {
    return finishWallEdgeTrackingSpeed(
        config, wall_edge_profile, false, current_speed_mps,
        requested_speed_mps, speed_proportional_gain,
        requested_acceleration_mps2, geometry_revision, elapsed_sec, state,
        result);
  }

  result.tracking_reserve_consumption_m =
      std::abs(lateral_error_m) +
      std::abs(lateral_velocity_mps) * config.lateral_velocity_preview_sec;
  result.usable_corridor_reserve_m =
      corridor_reserve_m - result.tracking_reserve_consumption_m;
  result.reserve_ratio = std::clamp(
      result.usable_corridor_reserve_m / desired_control_reserve_m, 0.0, 1.0);
  return finishWallEdgeTrackingSpeed(
      config, wall_edge_profile, true, current_speed_mps, requested_speed_mps,
      speed_proportional_gain, requested_acceleration_mps2, geometry_revision,
      elapsed_sec, state, result);
}

// Evaluate the two physical sides of a committed pass corridor separately.
// corridor_side points from the trajectory toward the wall; its opposite
// points toward the passed opponent. A tracking error which moves away from
// one boundary must not consume that boundary's reserve.
inline WallEdgeTrackingSpeedResult applyDirectionalCorridorTrackingSpeed(
    const WallEdgeTrackingSpeedConfig &config,
    const bool corridor_clearance_profile, const int corridor_side,
    const double wall_clearance_reserve_m,
    const double opponent_clearance_reserve_m,
    const double desired_control_reserve_m, const double lateral_error_m,
    const double lateral_velocity_mps, const double current_speed_mps,
    const double requested_speed_mps, const double speed_proportional_gain,
    const double requested_acceleration_mps2,
    const std::uint64_t geometry_revision, const double elapsed_sec,
    WallEdgeTrackingSpeedState *state) noexcept {
  WallEdgeTrackingSpeedResult result;
  result.speed_cap_mps = requested_speed_mps;
  result.speed_mps = requested_speed_mps;
  result.acceleration_mps2 = requested_acceleration_mps2;

  const bool inputs_valid =
      config.enabled && corridor_clearance_profile &&
      (corridor_side == -1 || corridor_side == 1) &&
      std::isfinite(config.lateral_velocity_preview_sec) &&
      config.lateral_velocity_preview_sec >= 0.0 &&
      std::isfinite(config.minimum_speed_mps) &&
      config.minimum_speed_mps >= 0.0 &&
      std::isfinite(config.deceleration_mps2) &&
      config.deceleration_mps2 >= 0.0 &&
      std::isfinite(config.release_reserve_ratio) &&
      config.release_reserve_ratio >= 0.0 &&
      config.release_reserve_ratio <= 1.0 &&
      std::isfinite(config.release_duration_sec) &&
      config.release_duration_sec >= 0.0 && std::isfinite(elapsed_sec) &&
      elapsed_sec >= 0.0 && state != nullptr &&
      std::isfinite(wall_clearance_reserve_m) &&
      wall_clearance_reserve_m >= 0.0 &&
      std::isfinite(opponent_clearance_reserve_m) &&
      opponent_clearance_reserve_m >= 0.0 &&
      std::isfinite(desired_control_reserve_m) &&
      desired_control_reserve_m > 0.0 && std::isfinite(lateral_error_m) &&
      std::isfinite(lateral_velocity_mps) && std::isfinite(current_speed_mps) &&
      std::isfinite(requested_speed_mps) && requested_speed_mps >= 0.0 &&
      std::isfinite(speed_proportional_gain) &&
      speed_proportional_gain >= 0.0 &&
      std::isfinite(requested_acceleration_mps2);
  if (!inputs_valid) {
    return finishWallEdgeTrackingSpeed(
        config, corridor_clearance_profile, false, current_speed_mps,
        requested_speed_mps, speed_proportional_gain,
        requested_acceleration_mps2, geometry_revision, elapsed_sec, state,
        result);
  }

  result.projected_lateral_error_m =
      lateral_error_m +
      lateral_velocity_mps * config.lateral_velocity_preview_sec;
  const double side = static_cast<double>(corridor_side);
  const double current_toward_wall_m = side * lateral_error_m;
  const double projected_toward_wall_m =
      side * result.projected_lateral_error_m;
  // Observe the full linear preview interval. This retains the current
  // footprint when it is already near a boundary while allowing inward motion
  // to consume only the opponent-side reserve.
  result.wall_tracking_reserve_consumption_m =
      std::max({0.0, current_toward_wall_m, projected_toward_wall_m});
  result.opponent_tracking_reserve_consumption_m =
      std::max({0.0, -current_toward_wall_m, -projected_toward_wall_m});
  result.tracking_reserve_consumption_m =
      std::max(result.wall_tracking_reserve_consumption_m,
               result.opponent_tracking_reserve_consumption_m);
  result.usable_wall_reserve_m =
      wall_clearance_reserve_m - result.wall_tracking_reserve_consumption_m;
  result.usable_opponent_reserve_m =
      opponent_clearance_reserve_m -
      result.opponent_tracking_reserve_consumption_m;
  result.usable_corridor_reserve_m =
      std::min(result.usable_wall_reserve_m, result.usable_opponent_reserve_m);

  const auto remaining_ratio = [](const double reserve_m,
                                  const double consumption_m) {
    if (reserve_m <= 1.0e-9) {
      return 0.0;
    }
    return std::clamp((reserve_m - consumption_m) / reserve_m, 0.0, 1.0);
  };
  const double wall_ratio = remaining_ratio(
      wall_clearance_reserve_m, result.wall_tracking_reserve_consumption_m);
  const double opponent_ratio =
      remaining_ratio(opponent_clearance_reserve_m,
                      result.opponent_tracking_reserve_consumption_m);
  result.reserve_ratio = std::min(wall_ratio, opponent_ratio);

  return finishWallEdgeTrackingSpeed(
      config, corridor_clearance_profile, true, current_speed_mps,
      requested_speed_mps, speed_proportional_gain, requested_acceleration_mps2,
      geometry_revision, elapsed_sec, state, result);
}

// Compatibility overload for stateless callers and focused arithmetic tests.
// Runtime control uses the stateful overload above.
inline WallEdgeTrackingSpeedResult applyWallEdgeTrackingSpeed(
    const WallEdgeTrackingSpeedConfig &config, const bool wall_edge_profile,
    const double corridor_reserve_m, const double desired_control_reserve_m,
    const double lateral_error_m, const double lateral_velocity_mps,
    const double current_speed_mps, const double requested_speed_mps,
    const double speed_proportional_gain,
    const double requested_acceleration_mps2) noexcept {
  WallEdgeTrackingSpeedState state;
  return applyWallEdgeTrackingSpeed(
      config, wall_edge_profile, corridor_reserve_m, desired_control_reserve_m,
      lateral_error_m, lateral_velocity_mps, current_speed_mps,
      requested_speed_mps, speed_proportional_gain, requested_acceleration_mps2,
      1U, 0.0, &state);
}

} // namespace simple_pure_pursuit
