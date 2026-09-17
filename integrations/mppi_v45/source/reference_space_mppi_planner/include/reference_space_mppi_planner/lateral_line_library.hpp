#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace reference_space_mppi_planner::lateral_line_library {

constexpr std::size_t kLineCount = 5U;

struct SideInterval {
  int side{0};
  bool feasible{false};
  double minimum_d_m{0.0};
  double maximum_d_m{0.0};
};

struct Line {
  int side{0};
  double d_m{0.0};
};

struct Library {
  std::array<Line, kLineCount> lines{};
  std::size_t count{0U};
};

inline bool valid(const SideInterval &interval) noexcept {
  return interval.feasible && (interval.side == 1 || interval.side == -1) &&
         std::isfinite(interval.minimum_d_m) &&
         std::isfinite(interval.maximum_d_m) &&
         interval.minimum_d_m <= interval.maximum_d_m;
}

inline Library wallEdges(const SideInterval &left, const SideInterval &right) noexcept {
  Library output;
  if(valid(left)) output.lines[output.count++]={1,left.maximum_d_m};
  if(valid(right)) output.lines[output.count++]={-1,right.minimum_d_m};
  return output;
}

// Build five complete lateral-line hypotheses from the physically usable
// intervals. When only one side is open, all five lines cover that side rather
// than wasting slots on the blocked side or the opponent-centred reference.
// With two open sides, the preferred (or wider) side receives the midpoint.
inline Library make(const SideInterval &left, const SideInterval &right,
                    int preferred_side, bool include_inner_boundary = true) noexcept {
  Library output;
  const bool have_left = valid(left);
  const bool have_right = valid(right);
  if (!have_left && !have_right) {
    return output;
  }

  const auto append = [&](const SideInterval &interval,
                          std::size_t line_count) {
    if (line_count == 0U)
      return;
    for (std::size_t index = 0U;
         index < line_count && output.count < output.lines.size(); ++index) {
      const double ratio = line_count == 1U
                               ? 0.5
                               : static_cast<double>(!include_inner_boundary && interval.side < 0
                                         ? index + 1U : index) /
                                     static_cast<double>(include_inner_boundary
                                         ? line_count - 1U : line_count);
      // Road-relative sides share d=0. Excluding that inner boundary keeps
      // all five hypotheses lateral instead of duplicating the base route.
      // Store lines left-to-right in RViz: positive side outer-to-inner,
      // negative side inner-to-outer.
      const double d_m = interval.maximum_d_m -
                         ratio * (interval.maximum_d_m - interval.minimum_d_m);
      output.lines[output.count++] = Line{interval.side, d_m};
    }
  };

  if (have_left && !have_right) {
    append(left, kLineCount);
    return output;
  }
  if (!have_left && have_right) {
    append(right, kLineCount);
    return output;
  }

  const double left_width = left.maximum_d_m - left.minimum_d_m;
  const double right_width = right.maximum_d_m - right.minimum_d_m;
  const int enriched_side = preferred_side == 1 || preferred_side == -1
                                ? preferred_side
                                : (left_width >= right_width ? 1 : -1);
  append(left, enriched_side == 1 ? 3U : 2U);
  append(right, enriched_side == -1 ? 3U : 2U);
  return output;
}

} // namespace reference_space_mppi_planner::lateral_line_library
