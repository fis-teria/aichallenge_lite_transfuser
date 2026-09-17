#include "simple_trajectory_generator/dual_reference.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using simple_trajectory_generator::buildCircularReference;
using simple_trajectory_generator::buildLocalCircularTrajectory;
using simple_trajectory_generator::CircularReferenceConfig;
using simple_trajectory_generator::LapProgressConfig;
using simple_trajectory_generator::LapProgressTracker;
using simple_trajectory_generator::projectToCircularReference;

TrajectoryPoint point(const double x_m, const double y_m) {
  TrajectoryPoint value;
  value.pose.position.x = x_m;
  value.pose.position.y = y_m;
  value.pose.orientation.w = 1.0;
  value.longitudinal_velocity_mps = 1.0F;
  return value;
}

Trajectory squareReference() {
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points = {
      point(0.0, 0.0),
      point(10.0, 0.0),
      point(10.0, 10.0),
      point(0.0, 10.0),
  };
  return trajectory;
}

TEST(DualReference, RotatesToAnchorAndDensifiesClosingSegment) {
  const auto result = buildCircularReference(
      squareReference(), CircularReferenceConfig{10.0, 1.0, 1000U},
      10.0, 10.0, true);

  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_FALSE(result.reference.trajectory.points.empty());
  const auto &first = result.reference.trajectory.points.front();
  EXPECT_NEAR(first.pose.position.x, 10.0, 1.0e-9);
  EXPECT_NEAR(first.pose.position.y, 10.0, 1.0e-9);
  EXPECT_NEAR(result.reference.length_m, 40.0, 1.0e-6);
  for (const auto &entry : result.reference.trajectory.points) {
    EXPECT_FLOAT_EQ(entry.longitudinal_velocity_mps, 10.0F);
  }
}

TEST(DualReference, BuildsLocalTrajectoryAcrossCircularSeam) {
  const auto result = buildCircularReference(
      squareReference(), CircularReferenceConfig{10.0, 1.0, 1000U},
      0.0, 0.0, false);
  ASSERT_TRUE(result.valid) << result.reason;
  const auto projection = projectToCircularReference(
      result.reference, 0.0, 1.0);
  ASSERT_TRUE(projection.valid);

  const auto local = buildLocalCircularTrajectory(
      result.reference, projection.nearest_index, 2.0, 6.0, 100U);
  ASSERT_GT(local.points.size(), 4U);
  bool contains_start_side = false;
  bool contains_end_side = false;
  for (const auto &entry : local.points) {
    contains_start_side = contains_start_side ||
                          (entry.pose.position.y < 0.1 &&
                           entry.pose.position.x < 2.1);
    contains_end_side = contains_end_side ||
                        (entry.pose.position.x < 0.1 &&
                         entry.pose.position.y > 0.9);
  }
  EXPECT_TRUE(contains_start_side);
  EXPECT_TRUE(contains_end_side);
}

TEST(LapProgress, CompletesOnlyAfterForwardCoverageAndSeamCrossing) {
  LapProgressConfig config;
  config.path_length_m = 100.0;
  config.start_window_m = 5.0;
  config.end_window_m = 8.0;
  config.minimum_coverage_ratio = 0.95;
  config.maximum_forward_step_m = 10.0;
  LapProgressTracker tracker(config);

  double time_sec = 1.0;
  EXPECT_FALSE(tracker.update(1.0, time_sec).lap_complete);
  for (double s_m = 6.0; s_m <= 96.0; s_m += 5.0) {
    time_sec += 0.1;
    EXPECT_FALSE(tracker.update(s_m, time_sec).lap_complete);
  }
  time_sec += 0.1;
  const auto completed = tracker.update(1.5, time_sec);
  EXPECT_TRUE(completed.accepted);
  EXPECT_TRUE(completed.seam_crossed);
  EXPECT_TRUE(completed.lap_complete);
}

TEST(LapProgress, CreditsSpawnProgressAndCompletesAtFirstReferenceZero) {
  LapProgressConfig config;
  config.path_length_m = 100.0;
  config.start_window_m = 5.0;
  config.end_window_m = 8.0;
  config.minimum_coverage_ratio = 0.95;
  config.maximum_forward_step_m = 10.0;
  LapProgressTracker tracker(config);

  double time_sec = 1.0;
  const auto initialized = tracker.update(30.0, time_sec);
  EXPECT_TRUE(initialized.accepted);
  EXPECT_TRUE(initialized.start_armed);
  EXPECT_EQ(initialized.reason, "initial_progress_credited");
  EXPECT_DOUBLE_EQ(initialized.accumulated_forward_m, 30.0);

  for (double s_m = 35.0; s_m <= 95.0; s_m += 5.0) {
    time_sec += 0.1;
    EXPECT_FALSE(tracker.update(s_m, time_sec).lap_complete);
  }
  time_sec += 0.1;
  const auto completed = tracker.update(1.0, time_sec);
  EXPECT_TRUE(completed.accepted);
  EXPECT_TRUE(completed.seam_crossed);
  EXPECT_TRUE(completed.lap_complete);
  EXPECT_GE(completed.accumulated_forward_m, 95.0);
}

TEST(LapProgress, ReportsADeferredSwitchRetryAtTheNextForwardSeam) {
  LapProgressConfig config;
  config.path_length_m = 100.0;
  config.start_window_m = 5.0;
  config.end_window_m = 8.0;
  config.minimum_coverage_ratio = 0.95;
  config.maximum_forward_step_m = 10.0;
  LapProgressTracker tracker(config);

  double time_sec = 1.0;
  EXPECT_TRUE(tracker.update(1.0, time_sec).accepted);
  for (double s_m = 6.0; s_m <= 96.0; s_m += 5.0) {
    time_sec += 0.1;
    EXPECT_FALSE(tracker.update(s_m, time_sec).lap_complete);
  }
  time_sec += 0.1;
  const auto first_seam = tracker.update(1.5, time_sec);
  ASSERT_TRUE(first_seam.lap_complete);
  ASSERT_TRUE(first_seam.seam_crossed);

  time_sec += 0.1;
  const auto after_first_seam = tracker.update(6.5, time_sec);
  EXPECT_TRUE(after_first_seam.accepted);
  EXPECT_TRUE(after_first_seam.lap_complete);
  EXPECT_FALSE(after_first_seam.seam_crossed);
  EXPECT_EQ(after_first_seam.reason, "lap_already_complete");

  for (double s_m = 11.5; s_m <= 96.5; s_m += 5.0) {
    time_sec += 0.1;
    const auto progress = tracker.update(s_m, time_sec);
    EXPECT_TRUE(progress.accepted);
    EXPECT_FALSE(progress.seam_crossed);
  }
  time_sec += 0.1;
  const auto retry_seam = tracker.update(2.0, time_sec);
  EXPECT_TRUE(retry_seam.accepted);
  EXPECT_TRUE(retry_seam.lap_complete);
  EXPECT_TRUE(retry_seam.seam_crossed);
  EXPECT_EQ(retry_seam.reason, "subsequent_seam_crossed");
}

TEST(LapProgress, RejectsJumpAndReverseAsCompletionEvidence) {
  LapProgressConfig config;
  config.path_length_m = 100.0;
  config.maximum_forward_step_m = 5.0;
  LapProgressTracker tracker(config);

  EXPECT_TRUE(tracker.update(1.0, 1.0).accepted);
  EXPECT_FALSE(tracker.update(96.0, 1.1).accepted);
  const auto reverse = tracker.update(99.0, 1.2);
  EXPECT_FALSE(reverse.accepted);
  EXPECT_FALSE(reverse.lap_complete);
  const auto backward_seam = tracker.update(2.0, 1.3);
  EXPECT_TRUE(backward_seam.accepted);
  EXPECT_FALSE(backward_seam.lap_complete);
}

TEST(LapProgress, RejectsNonMonotonicMeasurementTime) {
  LapProgressConfig config;
  config.path_length_m = 100.0;
  LapProgressTracker tracker(config);
  EXPECT_TRUE(tracker.update(1.0, 2.0).accepted);
  const auto duplicate = tracker.update(2.0, 2.0);
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_EQ(duplicate.reason, "non_monotonic_time");
}

} // namespace
