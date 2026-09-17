#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace reference_space_mppi_planner {

struct PreparationBoostConfig {
  bool enabled{false};
  double start_waypoint{0.0}, end_waypoint{60.0};
  std::array<std::int64_t, 2> first_lap{{1, 5}}, last_lap{{4, 6}};
};

// Each race has two independent, non-overlapping lap allocations. A missed
// early allocation does not become a second use in the late allocation.
struct PreparationBoostPolicy {
  std::array<bool, 2> fired{{false, false}};
  bool ready_seen{false}, race_started{false}, status_received{false};
  bool is_boosting{false};
  int lap{0}, remaining{0};
  std::string vehicle_state;

  void observeState(const std::string &state) {
    if (state == vehicle_state) return;
    if (state == "spawned" || state == "grounded") {
      fired.fill(false);
      ready_seen = race_started = status_received = false;
      lap = remaining = 0;
      is_boosting = false;
    } else if (state == "finish") {
      ready_seen = race_started = false;
    } else if (state == "ready") {
      if (!ready_seen) fired.fill(false);
      race_started = race_started || vehicle_state == "start";
      ready_seen = true;
    } else if (state == "start") {
      race_started = ready_seen;
    }
    vehicle_state = state;
  }

  void observeStatus(int current_lap, int uses_left, bool active) {
    lap = current_lap;
    remaining = uses_left;
    is_boosting = active;
    status_received = true;
  }

  std::optional<std::size_t> select(
      const PreparationBoostConfig &config, bool preparation_ready,
      double waypoint) const {
    if (!config.enabled || !preparation_ready || !race_started ||
        !status_received || remaining <= 0 || is_boosting ||
        !std::isfinite(waypoint) || waypoint < config.start_waypoint ||
        waypoint > config.end_waypoint) return {};
    for (std::size_t slot = 0; slot < fired.size(); ++slot) {
      if (!fired[slot] && lap >= config.first_lap[slot] &&
          lap <= config.last_lap[slot]) return slot;
    }
    return {};
  }
};

}  // namespace reference_space_mppi_planner
