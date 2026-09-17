#ifndef SIMPLE_TRAJECTORY_GENERATOR__RACE_REFERENCE_HPP_
#define SIMPLE_TRAJECTORY_GENERATOR__RACE_REFERENCE_HPP_

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>

namespace simple_trajectory_generator {

enum class RaceReference { kLap1, kNormal, kLeader };

inline RaceReference selectRaceReference(int current_lap,
                                         std::optional<int> estimated_rank) {
  if (current_lap <= 1) return RaceReference::kLap1;
  return estimated_rank == 1 ? RaceReference::kLeader : RaceReference::kNormal;
}

// All cars must use the same fixed course projection, including after the
// driving reference changes. Unwrapping preserves the order across the seam
// and when the leader laps another car. This estimates progress, not penalties
// or the simulator's official classification.
class RaceProgressRanking {
public:
  explicit RaceProgressRanking(double length_m) : length_m_(length_m) {}

  void reset() { tracks_.clear(); }

  void update(const std::string &id, double s_m, double stamp_sec,
              double initial_anchor_s_m) {
    if (id.empty() || !std::isfinite(s_m) || !std::isfinite(stamp_sec) ||
        !std::isfinite(initial_anchor_s_m) || length_m_ <= 0.0) return;
    auto found = tracks_.find(id);
    if (found == tracks_.end()) {
      tracks_.emplace(id, Track{s_m, initial_anchor_s_m +
          std::remainder(s_m - initial_anchor_s_m, length_m_), stamp_sec});
      return;
    }
    auto &track = found->second;
    if (stamp_sec <= track.stamp_sec) return;
    track.progress_m += std::remainder(s_m - track.s_m, length_m_);
    track.s_m = s_m;
    track.stamp_sec = stamp_sec;
  }

  std::optional<int> rank(const std::string &own_id, double now_sec,
                          double maximum_age_sec) const {
    const auto own = tracks_.find(own_id);
    if (own == tracks_.end()) return std::nullopt;
    int result = 1;
    for (const auto &[id, track] : tracks_) {
      const double age = now_sec - track.stamp_sec;
      if (age < 0.0 || age > maximum_age_sec) return std::nullopt;
      if (id != own_id && track.progress_m > own->second.progress_m) ++result;
    }
    return result;
  }

  std::optional<std::string> leader(double now_sec, double maximum_age_sec) const {
    std::optional<std::string> result;
    double progress = -std::numeric_limits<double>::infinity();
    for (const auto &[id, track] : tracks_) {
      const double age = now_sec - track.stamp_sec;
      if (age < 0.0 || age > maximum_age_sec) return std::nullopt;
      if (track.progress_m > progress ||
          (track.progress_m == progress && result && id < *result)) {
        progress = track.progress_m;
        result = id;
      }
    }
    return result;
  }

 private:
  struct Track { double s_m; double progress_m; double stamp_sec; };
  double length_m_;
  std::unordered_map<std::string, Track> tracks_;
};

} // namespace simple_trajectory_generator

#endif // SIMPLE_TRAJECTORY_GENERATOR__RACE_REFERENCE_HPP_
