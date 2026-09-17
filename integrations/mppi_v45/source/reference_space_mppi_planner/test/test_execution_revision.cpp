#include "reference_space_mppi_planner/execution_revision.hpp"
#include <stdexcept>
using reference_space_mppi_planner::ExecutionRevisionCheck;
void require(bool b) { if (!b) throw std::runtime_error("execution revision contract"); }
int main() {
  ExecutionRevisionCheck pending{0};
  require(pending.canCommit(0)); // Ordinary hold heartbeats do not change revision.
  require(!pending.canCommit(1)); // Recovery published while worker ran.
  require(pending.needsValidation(1));
  pending.recordValidation(1, false);
  require(!pending.canCommit(1));
  pending.recordValidation(1, true);
  require(pending.canCommit(1)); // A fresh valid proposal can leave recovery.
  require(!pending.canCommit(2)); // Another recovery during validation.
  pending.recordValidation(2, true);
  require(pending.canCommit(2));
}
