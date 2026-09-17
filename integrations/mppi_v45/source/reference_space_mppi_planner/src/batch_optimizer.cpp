#include "reference_space_mppi_planner/batch_optimizer.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace reference_space_mppi_planner::mppi {

ReferenceSpaceMppiBatchOptimizer::ReferenceSpaceMppiBatchOptimizer(
    Config config, bool parallel_enabled)
    : parallel_enabled_(parallel_enabled) {
  for (auto &planner : planners_) {
    planner = std::make_unique<ReferenceSpaceMppiPlanner>(config);
  }
  if (parallel_enabled_) {
    for (std::size_t slot = 0U; slot < workers_.size(); ++slot) {
      workers_[slot] = std::thread([this, slot]() { workerLoop(slot); });
    }
  }
}

ReferenceSpaceMppiBatchOptimizer::~ReferenceSpaceMppiBatchOptimizer() {
  if (!parallel_enabled_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    stopping_ = true;
  }
  start_cv_.notify_all();
  for (auto &worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

void ReferenceSpaceMppiBatchOptimizer::reset() {
  std::lock_guard<std::mutex> invocation_lock(invocation_mutex_);
  for (auto &planner : planners_) {
    planner->reset();
  }
}

void ReferenceSpaceMppiBatchOptimizer::accept(std::size_t slot,
    const PlanRequest &request,const PlanResult &result) {
  std::lock_guard<std::mutex> lock(invocation_mutex_);
  if(slot<planners_.size()) planners_[slot]->accept(request,result);
}

BatchPlanResult ReferenceSpaceMppiBatchOptimizer::plan(
    const std::array<const PlanRequest *, kMaximumBatchCandidateCount>
        &requests,
    const std::array<std::size_t, kMaximumBatchCandidateCount>
        &warm_start_slots,
    std::size_t candidate_count, BatchScratch *scratch) {
  const auto started = std::chrono::steady_clock::now();
  BatchPlanResult result;
  result.candidate_count =
      candidate_count <= kMaximumBatchCandidateCount ? candidate_count : 0U;
  const auto finish = [&]() {
    for (std::size_t index = 0U; index < result.candidate_count; ++index) {
      result.candidate_elapsed_sum_ms += result.candidates[index].elapsed_ms;
    }
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - started)
                            .count();
    return result;
  };
  if (scratch == nullptr || candidate_count == 0U ||
      candidate_count > kMaximumBatchCandidateCount) {
    return finish();
  }
  for (std::size_t index = 0U; index < candidate_count; ++index) {
    if (requests[index] == nullptr ||
        warm_start_slots[index] >= kMaximumBatchCandidateCount) {
      return finish();
    }
    for (std::size_t earlier = 0U; earlier < index; ++earlier) {
      if (warm_start_slots[earlier] == warm_start_slots[index]) {
        return finish();
      }
    }
  }

  std::lock_guard<std::mutex> invocation_lock(invocation_mutex_);
  if (!parallel_enabled_ || candidate_count == 1U) {
    for (std::size_t index = 0U; index < candidate_count; ++index) {
      try {
        result.candidates[index] = planners_[warm_start_slots[index]]->plan(
            *requests[index], &scratch->candidates[index]);
      } catch (...) {
        PlanResult failed;
        failed.generation = requests[index]->generation;
        failed.reject_reason = RejectReason::INVALID_INPUT;
        result.candidates[index] = std::move(failed);
      }
    }
    return finish();
  }

  {
    std::lock_guard<std::mutex> worker_lock(worker_mutex_);
    active_requests_ = &requests;
    active_warm_start_slots_ = &warm_start_slots;
    active_scratch_ = scratch;
    active_result_ = &result;
    active_candidate_count_ = candidate_count;
    completed_candidate_count_ = 0U;
    ++epoch_;
  }
  start_cv_.notify_all();
  {
    std::unique_lock<std::mutex> worker_lock(worker_mutex_);
    complete_cv_.wait(worker_lock, [this, candidate_count]() {
      return completed_candidate_count_ == candidate_count;
    });
    active_requests_ = nullptr;
    active_warm_start_slots_ = nullptr;
    active_scratch_ = nullptr;
    active_result_ = nullptr;
    active_candidate_count_ = 0U;
  }
  return finish();
}

void ReferenceSpaceMppiBatchOptimizer::executeCandidates(
    std::size_t candidate_count,
    const std::function<void(std::size_t)> &task) {
  if (candidate_count > kMaximumBatchCandidateCount) {
    throw std::invalid_argument("candidate count exceeds worker capacity");
  }
  std::lock_guard<std::mutex> invocation_lock(invocation_mutex_);
  task_errors_.fill(nullptr);
  if (!parallel_enabled_ || candidate_count <= 1U) {
    for (std::size_t slot = 0; slot < candidate_count; ++slot) {
      try { task(slot); }
      catch (...) { task_errors_[slot] = std::current_exception(); }
    }
  } else {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      active_task_ = &task;
      active_candidate_count_ = candidate_count;
      completed_candidate_count_ = 0U;
      ++epoch_;
    }
    start_cv_.notify_all();
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      complete_cv_.wait(lock, [this, candidate_count]() {
        return completed_candidate_count_ == candidate_count;
      });
      active_task_ = nullptr;
      active_candidate_count_ = 0U;
    }
  }
  for (std::size_t slot = 0; slot < candidate_count; ++slot) {
    if (task_errors_[slot]) std::rethrow_exception(task_errors_[slot]);
  }
}

void ReferenceSpaceMppiBatchOptimizer::workerLoop(std::size_t slot) {
  std::uint64_t observed_epoch = 0U;
  while (true) {
    const std::array<const PlanRequest *, kMaximumBatchCandidateCount>
        *requests = nullptr;
    const std::array<std::size_t, kMaximumBatchCandidateCount>
        *warm_start_slots = nullptr;
    BatchScratch *scratch = nullptr;
    BatchPlanResult *result = nullptr;
    const std::function<void(std::size_t)> *task = nullptr;
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      start_cv_.wait(lock, [this, observed_epoch]() {
        return stopping_ || epoch_ != observed_epoch;
      });
      if (stopping_) {
        return;
      }
      observed_epoch = epoch_;
      if (slot >= active_candidate_count_) {
        continue;
      }
      requests = active_requests_;
      warm_start_slots = active_warm_start_slots_;
      scratch = active_scratch_;
      result = active_result_;
      task = active_task_;
    }

    try {
      if (task) {
        (*task)(slot);
      } else {
        result->candidates[slot] = planners_[(*warm_start_slots)[slot]]->plan(
            *(*requests)[slot], &scratch->candidates[slot]);
      }
    } catch (...) {
      if (task) {
        task_errors_[slot] = std::current_exception();
      } else {
        // A throwing environment validator must invalidate only its candidate;
        // all workers still rendezvous so the planning thread cannot deadlock.
        PlanResult failed;
        failed.generation = (*requests)[slot]->generation;
        failed.reject_reason = RejectReason::INVALID_INPUT;
        result->candidates[slot] = std::move(failed);
      }
    }

    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      ++completed_candidate_count_;
      if (completed_candidate_count_ == active_candidate_count_) {
        complete_cv_.notify_one();
      }
    }
  }
}

} // namespace reference_space_mppi_planner::mppi
