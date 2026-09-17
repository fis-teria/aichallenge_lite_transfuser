#pragma once

#include <cmath>
#include <string_view>

namespace reference_space_mppi_planner::target_selection {

inline bool pathFootprintsConflict(double centerline_distance_m,
                                   double ego_half_width_m,
                                   double target_half_width_m,
                                   double selection_margin_m,
                                   double lateral_uncertainty_m) noexcept {
  if (!std::isfinite(centerline_distance_m) ||
      !std::isfinite(ego_half_width_m) || !std::isfinite(target_half_width_m) ||
      !std::isfinite(selection_margin_m) ||
      !std::isfinite(lateral_uncertainty_m) || centerline_distance_m < 0.0 ||
      ego_half_width_m < 0.0 || target_half_width_m < 0.0 ||
      selection_margin_m < 0.0 || lateral_uncertainty_m < 0.0) {
    return false;
  }
  return centerline_distance_m <= ego_half_width_m + target_half_width_m +
                                      selection_margin_m +
                                      lateral_uncertainty_m;
}

inline bool
committedTargetRetainsPriority(double relative_s_m,
                               double pass_clear_distance_m,
                               double maximum_forward_distance_m) noexcept {
  return std::isfinite(relative_s_m) && std::isfinite(pass_clear_distance_m) &&
         std::isfinite(maximum_forward_distance_m) &&
         pass_clear_distance_m >= 0.0 && maximum_forward_distance_m >= 0.0 &&
         relative_s_m >= -pass_clear_distance_m &&
         relative_s_m <= maximum_forward_distance_m;
}

inline int targetScopedPreferredSide(std::string_view committed_target_id,
                                     std::string_view selected_target_id,
                                     int committed_side) noexcept {
  if (!committed_target_id.empty() &&
      committed_target_id == selected_target_id) {
    return committed_side;
  }
  // A new opponent starts a new MPPI decision. Carrying the ego's current
  // lane across target hand-off biases the next pass before either side has
  // been costed and can make the kart follow a laterally moving opponent all
  // the way to a wall. Hysteresis is target-scoped: only the same opponent
  // retains its committed side.
  return 0;
}

inline bool targetRequiresFollow(double relative_s_m) noexcept {
  return std::isfinite(relative_s_m) && relative_s_m >= 0.0;
}

} // namespace reference_space_mppi_planner::target_selection
