#pragma once
#include <cstdint>

namespace reference_space_mppi_planner {
// Validation belongs to the execution revision it observed, not just a path owner.
struct ExecutionRevisionCheck {
  std::uint64_t validated_revision;
  bool valid{true};
  bool needsValidation(std::uint64_t live) const { return live != validated_revision; }
  void recordValidation(std::uint64_t revision, bool accepted) {
    validated_revision = revision;
    valid = accepted;
  }
  bool canCommit(std::uint64_t live) const {
    return valid && live == validated_revision;
  }
};
}
