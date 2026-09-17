#include "simple_pure_pursuit/reference_return.hpp"
#include <gtest/gtest.h>

namespace simple_pure_pursuit {
TEST(ReferenceReturn, CoarseRouteJoinsFromMeasuredPositionHeadingAndCurvature) {
  for(double side:{-1.,1.}) {
    autoware_auto_planning_msgs::msg::Trajectory route;
    for(int x=-2;x<=50;++x) {
      autoware_auto_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x=x;p.pose.position.y=2.*side;p.pose.orientation.w=1.;
      p.longitudinal_velocity_mps=4.;route.points.push_back(p);
    }
    const ReferenceReturnState ego{0.,0.,.1*side,.08*side};
    const auto result=makeReferenceReturn(route,ego,{1.087,.3141592654});
    ASSERT_TRUE(result);const auto &points=result->points;
    ASSERT_GT(points.size(),100U);
    EXPECT_DOUBLE_EQ(points.front().pose.position.x,ego.x_m);
    EXPECT_DOUBLE_EQ(points.front().pose.position.y,ego.y_m);
    const auto &a=points[0].pose.position,&b=points[1].pose.position,&c=points[2].pose.position;
    EXPECT_NEAR(std::atan2(b.y-a.y,b.x-a.x),ego.yaw_rad,.02);
    EXPECT_NEAR(reference_connection::curvature({a.x,a.y},{b.x,b.y},{c.x,c.y}),
        std::tan(ego.steering_rad)/1.087,.02);
    EXPECT_NEAR(points.back().pose.position.x,50.,1e-9);
    EXPECT_NEAR(points.back().pose.position.y,2.*side,1e-9);
    for(const auto &p:points) EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps,4.);
  }
}
TEST(ReferenceReturn, ImpossibleGeometryDoesNotReturnRawRoute) {
  autoware_auto_planning_msgs::msg::Trajectory route;
  for(int i=0;i<3;++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x=.1*i;p.pose.position.y=10.;p.pose.orientation.w=1.;route.points.push_back(p);
  }
  EXPECT_FALSE(makeReferenceReturn(route,{0.,0.,0.,0.},{1.087,.3141592654}));
}
}  // namespace simple_pure_pursuit
