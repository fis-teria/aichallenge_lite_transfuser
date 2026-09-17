#pragma once

#include <cstddef>
#include <cstdint>

namespace reference_space_mppi_planner {

inline bool generationWithinLag(std::uint64_t planned_generation,
                                std::uint64_t latest_generation,
                                std::size_t maximum_lag) noexcept {
  if (latest_generation < planned_generation) {
    return false;
  }
  return latest_generation - planned_generation <= maximum_lag;
}

// A proposal revalidated at the current vehicle/opponent state has a new
// validity epoch. Owner, semantics, execution revision and deadline are
// checked separately in the publication transaction.
inline bool generationAdoptable(std::uint64_t planned, std::uint64_t latest,
                                std::size_t maximum_lag, bool revalidated) noexcept {
  return planned <= latest && (revalidated || generationWithinLag(planned,latest,maximum_lag));
}

} // namespace reference_space_mppi_planner
