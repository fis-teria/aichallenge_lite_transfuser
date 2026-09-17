#include "reference_space_mppi_planner/collection_motion.hpp"
#include "reference_space_mppi_planner/recent_motion_prediction.hpp"
#include <gtest/gtest.h>

namespace reference_space_mppi_planner::opponent_prediction {

TEST(CollectionMotion, StationaryJitterDoesNotBecomeTravelledDistance) {
  std::vector<PositionObservation> history;
  for (int i=0; i<=12; ++i)
    history.push_back({10.+i*.05, 20.+(i%2 ? .05 : -.05), 3., 0.});
  const auto original=fitPhysicalSpeed(history);
  ASSERT_TRUE(original); EXPECT_GT((*original)[0], 1.38);
  const auto fit=fitCollectionMotion(history);
  ASSERT_TRUE(fit); EXPECT_DOUBLE_EQ(fit->vx_mps, 0.); EXPECT_DOUBLE_EQ(fit->vy_mps, 0.);
  EXPECT_NEAR(fit->x_m, 20., .051); EXPECT_DOUBLE_EQ(fit->y_m, 3.);
}

TEST(CollectionMotion, ThreeKmhMotionKeepsVelocityAndLatestEpochPosition) {
  std::vector<PositionObservation> history;
  for (int i=0; i<=12; ++i) {
    const double t=i*.05;
    history.push_back({10.+t, 20.+t*3./3.6*.8, 3.+t*3./3.6*.6, 0.});
  }
  const auto fit=fitCollectionMotion(history);
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->vx_mps, 3./3.6*.8, 1e-10);
  EXPECT_NEAR(fit->vy_mps, 3./3.6*.6, 1e-10);
  EXPECT_NEAR(fit->x_m, history.back().x, 1e-10);
  EXPECT_NEAR(fit->y_m, history.back().y, 1e-10);
}

TEST(CollectionMotion, IsolatedOutlierDoesNotLaunchStationaryObstacle) {
  std::vector<PositionObservation> history;
  for (int i=0; i<=12; ++i) history.push_back({10.+i*.05, 20., 3., 0.});
  history.back().x+=2.; history.back().y-=1.;
  const auto fit=fitCollectionMotion(history);
  ASSERT_TRUE(fit); EXPECT_DOUBLE_EQ(fit->x_m, 20.); EXPECT_DOUBLE_EQ(fit->y_m, 3.);
  EXPECT_DOUBLE_EQ(fit->vx_mps, 0.); EXPECT_DOUBLE_EQ(fit->vy_mps, 0.);
}

TEST(CollectionMotion, SparseObservationsStillRecognizeThreeKmhMovement) {
  const auto fit=fitCollectionMotion({{10.,0.,0.,0.},
      {10.25,.25*3./3.6,0.,0.},{10.5,.5*3./3.6,0.,0.}});
  ASSERT_TRUE(fit); EXPECT_NEAR(fit->vx_mps,3./3.6,1e-10);
  EXPECT_NEAR(fit->x_m,.5*3./3.6,1e-10);
}

TEST(CollectionMotion, StartsAndStopsWithinTheFiniteHistoryWindow) {
  std::vector<PositionObservation> history;
  for (int i=0; i<=60; ++i) {
    const double t=i*.05;
    history.push_back({10.+t, 20.+std::clamp(t-1.,0.,1.)*3./3.6, 0.,0.});
    const auto fit=fitCollectionMotion(history); ASSERT_TRUE(fit);
    if (t>.7 && t<1.) { EXPECT_DOUBLE_EQ(fit->vx_mps,0.); }
    if (t>1.7 && t<2.) { EXPECT_NEAR(fit->vx_mps,3./3.6,1e-8); }
    if (t>2.7) { EXPECT_DOUBLE_EQ(fit->vx_mps,0.); }
  }
}

TEST(CollectionMotion, SlowTurnRemainsMoving) {
  std::vector<PositionObservation> history;
  for (int i=0; i<=12; ++i) {
    const double t=i*.05, angle=t*(3./3.6)/5.;
    history.push_back({10.+t, 5.*std::sin(angle), 5.*(1.-std::cos(angle)),0.});
  }
  const auto fit=fitCollectionMotion(history); ASSERT_TRUE(fit);
  EXPECT_NEAR(std::hypot(fit->vx_mps,fit->vy_mps),3./3.6,.02);
  EXPECT_NEAR(fit->x_m,history.back().x,.03); EXPECT_NEAR(fit->y_m,history.back().y,.03);
}

TEST(CollectionMotion, InvalidOrReversedTimeIsRejectedAndWarmupIsExplicit) {
  EXPECT_FALSE(fitCollectionMotion({}));
  EXPECT_FALSE(fitCollectionMotion({{1.,NAN,0.,0.}}));
  EXPECT_FALSE(fitCollectionMotion({{1.,0.,0.,0.},{1.,0.,0.,0.}}));
  EXPECT_FALSE(fitCollectionMotion({{2.,0.,0.,0.},{1.,0.,0.,0.}}));
  const auto warmup=fitCollectionMotion({{1.,2.,3.,0.}});
  ASSERT_TRUE(warmup); EXPECT_DOUBLE_EQ(warmup->x_m,2.); EXPECT_DOUBLE_EQ(warmup->vx_mps,0.);
}

} // namespace reference_space_mppi_planner::opponent_prediction
