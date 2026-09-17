#pragma once

#include <string_view>

namespace simple_pure_pursuit {

enum class SplitTrajectoryInput {
  kNone,
  kBaseline,
  kManeuver,
};

inline SplitTrajectoryInput
splitTrajectoryInputForPathSource(const std::string_view path_source) noexcept {
  if (path_source == "CMA_REFERENCE_FREE_RUN" ||
      path_source == "CMA_REFERENCE_CURVE" ||
      path_source == "CMA_REFERENCE_SPEED_CAPPED") {
    return SplitTrajectoryInput::kBaseline;
  }
  if (path_source == "AVOID_CONNECTOR_CMA" ||
      path_source == "AVOID_REFERENCE" || path_source == "RETURN_REFERENCE" ||
      path_source == "POST_AVOID_REFERENCE" ||
      path_source == "POST_RECOVERY_REFERENCE") {
    return SplitTrajectoryInput::kManeuver;
  }
  return SplitTrajectoryInput::kNone;
}

} // namespace simple_pure_pursuit
