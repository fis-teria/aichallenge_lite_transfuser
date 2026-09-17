#pragma once

#include "reference_space_mppi_planner/reference_space_mppi.hpp"

#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace reference_space_mppi_planner::mppi {

// Six slots cover three timed passing goals on both existing wall lines.
// Keeping the capacity in the core avoids allocating in the 20 Hz planning
// loop and gives each line an independent receding-horizon warm start.
constexpr std::size_t kMaximumBatchCandidateCount = 6U;

struct BatchScratch {
  std::array<Scratch, kMaximumBatchCandidateCount> candidates{};
};

struct BatchPlanResult {
  std::array<PlanResult, kMaximumBatchCandidateCount> candidates{};
  std::size_t candidate_count{0U};
  // End-to-end latency observed by the caller.
  double elapsed_ms{0.0};
  // Sum of candidate times, retained to quantify parallel efficiency.
  double candidate_elapsed_sum_ms{0.0};
};

// Nav2-style synchronous batch boundary for the existing reference-space
// optimizer. Candidate slots execute concurrently on persistent workers, then
// join before the caller performs side selection. This is intentionally a
// fixed-capacity bridge toward a future SoA rollout/critic implementation.
class ReferenceSpaceMppiBatchOptimizer {
public:
  explicit ReferenceSpaceMppiBatchOptimizer(Config config = Config{},
                                            bool parallel_enabled = true);
  ~ReferenceSpaceMppiBatchOptimizer();

  ReferenceSpaceMppiBatchOptimizer(const ReferenceSpaceMppiBatchOptimizer &) =
      delete;
  ReferenceSpaceMppiBatchOptimizer &
  operator=(const ReferenceSpaceMppiBatchOptimizer &) = delete;

  void reset();
  // Synchronous reuse of the planner workers. Each callback owns one slot;
  // callers reduce results only after this returns. Exceptions rethrow after
  // every active worker has joined, in slot order.
  void executeCandidates(std::size_t candidate_count,
                         const std::function<void(std::size_t)> &task);
  void accept(std::size_t slot, const PlanRequest &request, const PlanResult &result);
  BatchPlanResult
  plan(const std::array<const PlanRequest *, kMaximumBatchCandidateCount>
           &requests,
       const std::array<std::size_t, kMaximumBatchCandidateCount>
           &warm_start_slots,
       std::size_t candidate_count, BatchScratch *scratch);

private:
  void workerLoop(std::size_t slot);

  bool parallel_enabled_{true};
  std::array<std::unique_ptr<ReferenceSpaceMppiPlanner>,
             kMaximumBatchCandidateCount>
      planners_{};
  std::array<std::thread, kMaximumBatchCandidateCount> workers_{};

  // Only one synchronous batch may be active. This also serializes reset()
  // against planning without exposing the worker protocol to callers.
  std::mutex invocation_mutex_;
  std::mutex worker_mutex_;
  std::condition_variable start_cv_;
  std::condition_variable complete_cv_;
  bool stopping_{false};
  std::uint64_t epoch_{0U};
  std::size_t active_candidate_count_{0U};
  std::size_t completed_candidate_count_{0U};
  const std::array<const PlanRequest *, kMaximumBatchCandidateCount>
      *active_requests_{nullptr};
  const std::array<std::size_t, kMaximumBatchCandidateCount>
      *active_warm_start_slots_{nullptr};
  BatchScratch *active_scratch_{nullptr};
  BatchPlanResult *active_result_{nullptr};
  const std::function<void(std::size_t)> *active_task_{nullptr};
  std::array<std::exception_ptr, kMaximumBatchCandidateCount> task_errors_{};
};

} // namespace reference_space_mppi_planner::mppi
