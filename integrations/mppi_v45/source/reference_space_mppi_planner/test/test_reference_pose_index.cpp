#include "reference_space_mppi_planner/reference_pose_index.hpp"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
#include <thread>

namespace reference_space_mppi_planner {
namespace {
TEST(OpponentPoseCache, ExactTimesSnapshotIsolationAndCapacity) {
  OpponentPoseCache cache(2), next_snapshot;
  int computations = 0;
  const auto compute = [&] {
    ++computations;
    return OpponentPoseCache::Poses{std::array<double, 3>{1, 2, 3},
                                    std::nullopt};
  };
  const auto first = cache.at(.1, compute);
  EXPECT_EQ(first, cache.at(.1, compute));
  EXPECT_EQ(computations, 1);
  cache.at(std::nextafter(.1, 1.0), compute);
  EXPECT_EQ(computations, 2);
  EXPECT_EQ(*first, *cache.at(.2, compute));
  cache.at(.2, compute);
  EXPECT_EQ(computations, 4);
  next_snapshot.at(.1, compute);
  EXPECT_EQ(computations, 5);
}

TEST(OpponentPoseCache, SharedAcrossFiveWorkers) {
  OpponentPoseCache cache;
  std::vector<std::thread> workers;
  for (int w = 0; w < 5; ++w) {
    workers.emplace_back([&] {
      for (int i = 0; i < 1000; ++i) {
        const double time = (i % 60) * .05;
        const auto result = cache.at(time, [=] {
          return OpponentPoseCache::Poses{std::array<double, 3>{time, 2, 3}};
        });
        ASSERT_TRUE((*result)[0]);
        EXPECT_DOUBLE_EQ((*(*result)[0])[0], time);
      }
    });
  }
  for (auto &worker : workers)
    worker.join();
}

using Point = std::array<double, 2>;
using Points = std::vector<Point>;
auto position = [](const Point &p) { return p; };

TEST(WorldOccupancy, ContinuousInteriorContactBendAndWrap) {
  ReferencePoseIndex straight(Points{{0,0},{20,0}}, position);
  // Endpoints are clear; the continuous uncertainty interval is not.
  EXPECT_TRUE(straight.occupancyOverlap({10,0,0}, 5,15,0,0,0,1,.5,0,0).value());
  EXPECT_TRUE(straight.occupancyOverlap({10,1,0}, 10,10,0,0,0,1,.5,0,0).value());
  EXPECT_FALSE(straight.occupancyOverlap({10,1.01,0}, 10,10,0,0,0,1,.5,0,0).value());
  ReferencePoseIndex bend(Points{{0,0},{10,0},{10,10}}, position);
  EXPECT_TRUE(bend.occupancyOverlap({10,5,0}, 14,16,0,0,0,1,.5,0,0).value());
  EXPECT_FALSE(bend.occupancyOverlap({5,5,0}, 8,16,0,0,0,1,.5,0,0).value());
  ReferencePoseIndex loop(Points{{0,0},{10,0},{10,10},{0,10},{0,0}}, position);
  EXPECT_TRUE(loop.occupancyOverlap({0,0,0}, 39,41,0,0,0,1,.5,0,0).value());
  EXPECT_TRUE(loop.occupancyOverlap({0,5,0}, -100,100,0,0,0,1,.5,0,0).value());
  EXPECT_FALSE(straight.occupancyOverlap({10,0,0}, 15,5,0,0,0,1,.5,0,0).has_value());
}

TEST(WorldOccupancy, DisplayPolygonAndDecisionAgreeAcrossBend) {
  ReferencePoseIndex index(Points{{0,0},{10,0},{10,10}},position);
  for(double yaw:{-.8,0.,.7}) for(double relative:{-.3,.4})
    for(int x=0;x<15;++x) for(int y=-4;y<12;++y) {
      bool inside=false;
      ASSERT_TRUE(index.visitOccupancy(7,17,-.3,.5,[&](const auto &tile) {
        const auto polygon=ReferencePoseIndex::forbiddenPolygon(tile,yaw,relative,1.1,.65,3.,1.5);
        ASSERT_GE(polygon.size(),4U);
        bool hit=true;
        for(std::size_t i=1;i<polygon.size();++i) {
          const auto &a=polygon[i-1], &b=polygon[i];
          if((b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]) < -1e-9) hit=false;
        }
        inside=inside||hit;
      }));
      EXPECT_EQ(inside,index.occupancyOverlap({double(x),double(y),yaw},7,17,-.3,.5,relative,1.1,.65,3.,1.5).value());
    }
}

// Previous linear implementation, retained as an independent numerical oracle.
std::optional<std::array<double, 3>> linear(const Points &p, double s,
                                            double d) {
  if (p.size() < 2)
    return std::nullopt;
  double length = 0;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    double ds = std::hypot(p[i + 1][0] - p[i][0], p[i + 1][1] - p[i][1]);
    if (std::isfinite(ds))
      length += ds;
  }
  if (!std::isfinite(length) || length < .001)
    return std::nullopt;
  if (std::hypot(p.front()[0] - p.back()[0], p.front()[1] - p.back()[1]) < 5) {
    s = std::fmod(s, length);
    if (s < 0)
      s += length;
  } else
    s = std::clamp(s, 0.0, length);
  double accumulated = 0;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    const double dx = p[i + 1][0] - p[i][0], dy = p[i + 1][1] - p[i][1];
    const double ds = std::hypot(dx, dy);
    if (!std::isfinite(ds) || ds < .0001)
      continue;
    if (s <= accumulated + ds || i + 2 == p.size()) {
      const double ratio = std::clamp((s - accumulated) / ds, 0.0, 1.0);
      const double yaw = std::atan2(dy, dx);
      return std::array<double, 3>{p[i][0] + ratio * dx - std::sin(yaw) * d,
                                   p[i][1] + ratio * dy + std::cos(yaw) * d,
                                   yaw};
    }
    accumulated += ds;
  }
  return std::nullopt;
}

void compare(const Points &points) {
  const ReferencePoseIndex index(points, position);
  for (int n = -3000; n <= 30000; ++n) {
    const double s = n * .125, d = (n % 17) * .125;
    const auto expected = linear(points, s, d), actual = index.pose(s, d);
    ASSERT_EQ(expected.has_value(), actual.has_value()) << s;
    if (expected) {
      for (std::size_t k = 0; k < 3; ++k) {
        EXPECT_DOUBLE_EQ((*expected)[k], (*actual)[k]) << s;
      }
    }
  }
}

TEST(ReferencePoseIndex, MatchesOpenClosedAndDegenerateSegments) {
  compare({{0, 0}, {10, 0}, {10, 10}, {20, 10}});
  compare({{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}});
  compare({{0, 0}, {0, 0}, {.00001, 0}, {10, 0}, {10, 0}});
  compare({});
  compare({{0, 0}});
  compare({{0, 0}, {0, 0}});
}

TEST(ReferencePoseIndex, ExactVertexUsesIncomingSegment) {
  ReferencePoseIndex index(Points{{0, 0}, {10, 0}, {10, 10}}, position);
  const auto at = index.pose(10, 1);
  ASSERT_TRUE(at);
  EXPECT_DOUBLE_EQ((*at)[0], 10);
  EXPECT_DOUBLE_EQ((*at)[1], 1);
  EXPECT_DOUBLE_EQ((*at)[2], 0);
}

TEST(ReferencePoseIndex, ImmutableQueriesOnFiveWorkers) {
  const Points points{{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
  const ReferencePoseIndex index(points, position);
  std::vector<std::thread> workers;
  for (int w = 0; w < 5; ++w)
    workers.emplace_back([&] {
      for (int i = 0; i < 1000; ++i)
        EXPECT_EQ(index.pose(i * .1, .4), linear(points, i * .1, .4));
    });
  for (auto &worker : workers)
    worker.join();
}

TEST(ReferencePoseIndex, RecordedCourseEquivalenceAndQueryBenchmark) {
  const char *path = std::getenv("MPPI_REFERENCE_BENCHMARK_CSV");
  if (!path)
    GTEST_SKIP()
        << "Set MPPI_REFERENCE_BENCHMARK_CSV for recorded-course benchmark";
  std::ifstream input(path);
  ASSERT_TRUE(input.good());
  Points points;
  std::string line;
  std::getline(input, line);
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    double s, x, y;
    if (row >> s >> x >> y)
      points.push_back({x, y});
  }
  ASSERT_GT(points.size(), 2U);
  compare(points);
  const ReferencePoseIndex index(points, position);
  auto measure = [&](auto query) {
    volatile double checksum = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 100000; ++i) {
      const auto pose = query(i * .017, .4);
      if (pose)
        checksum += (*pose)[0];
    }
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start)
        .count();
  };
  const double old_ms =
      measure([&](double s, double d) { return linear(points, s, d); });
  const double new_ms =
      measure([&](double s, double d) { return index.pose(s, d); });
  std::cout << "reference_points=" << points.size()
            << " queries=100000 linear_ms=" << old_ms
            << " indexed_ms=" << new_ms << '\n';
}
} // namespace
} // namespace reference_space_mppi_planner
