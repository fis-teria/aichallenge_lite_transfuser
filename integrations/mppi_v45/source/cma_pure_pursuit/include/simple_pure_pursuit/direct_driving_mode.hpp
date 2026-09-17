#pragma once

#include <string>

namespace simple_pure_pursuit {

// Translate the adopted avoidance/overtaking geometry's side
// into the existing controller policies without adding a controller FSM.
inline std::string directTrackingMode(const std::string &mode, int corridor_side) {
  if (mode != "AVOID" && mode != "OVERTAKE") return mode;
  if (corridor_side == 0) return "MERGE_BACK";
  return corridor_side > 0 ? "OVERTAKE_LEFT" : "OVERTAKE_RIGHT";
}

}  // namespace simple_pure_pursuit
