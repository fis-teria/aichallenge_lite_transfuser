#pragma once

#include <cstdint>
#include <mutex>

namespace reference_space_mppi_planner {

// Output order is independent of optimizer snapshot generation. A heartbeat
// must not make a still-current asynchronous optimum look out of order to PP.
class SequencedOutput {
public:
  template <class Command, class Publish>
  void publish(Command command, Publish publish) {
    std::lock_guard<std::mutex> lock(mutex_);
    command.generation = ++generation_;
    // Preserve the snapshot stamp and lease: sequencing is not revalidation.
    publish(command);
  }

private:
  std::mutex mutex_;
  std::uint64_t generation_{0U};
};

} // namespace reference_space_mppi_planner
