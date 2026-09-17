#pragma once

#include <string_view>

namespace simple_pure_pursuit
{

inline bool isCurvatureFeedforwardManeuverMode(const std::string_view mode) noexcept
{
  // PREPARE is BASE_REFERENCE geometry with a longitudinal deceleration
  // policy. Curvature feed-forward starts only after a hard-validated PASS
  // trajectory owns the same published generation.
  return mode == "OVERTAKE_LEFT" || mode == "OVERTAKE_RIGHT" ||
         mode == "SIDE_BY_SIDE_KEEP";
}

}  // namespace simple_pure_pursuit
