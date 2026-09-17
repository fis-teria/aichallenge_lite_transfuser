#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cstdint>
#include <array>
#include <string>
#include <vector>

namespace reference_space_mppi_planner {

struct WallFootprintPose {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

// Convex body projection expressed about the same anchor as WallFootprintPose
// (base_link for the planner). Precompute separating axes once per shape.
class ConvexWallFootprint {
public:
  struct Axis {
    double x, y, minimum, maximum;
  };
  explicit ConvexWallFootprint(const std::vector<double> &xy);
  const std::vector<std::array<double, 2>> &vertices() const { return vertices_; }
  const std::vector<Axis> &axes() const { return axes_; }
  double radius() const { return radius_; }

private:
  std::vector<std::array<double, 2>> vertices_;
  std::vector<Axis> axes_;
  double radius_{0.0};
};

// One immutable occupancy snapshot and its occupied/unknown-cell prefix sum.
// Keep the map alive with the index; a new map requires a new index.
class OccupancyGridWallIndex {
public:
  explicit OccupancyGridWallIndex(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);
  bool rectangleFree(const nav_msgs::msg::OccupancyGrid &map,
                     int minimum_x, int minimum_y,
                     int maximum_x, int maximum_y) const;

private:
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<std::uint64_t> prefix_;
};

bool loadStaticWallMap(const std::string &yaml_path,
                       const std::string &frame_id,
                       nav_msgs::msg::OccupancyGrid *map,
                       std::string *error = nullptr);

// Unknown cells and positions outside the map are treated as walls. The
// rectangle is checked against every overlapped occupancy-grid cell rather
// than a sparse set of points, so a thin wall cannot fall between samples.
bool occupancyGridFootprintFree(const nav_msgs::msg::OccupancyGrid &map,
                                const WallFootprintPose &pose,
                                double front_extent_m, double rear_extent_m,
                                double half_width_m,
                                const OccupancyGridWallIndex *index = nullptr);

bool occupancyGridFootprintFree(const nav_msgs::msg::OccupancyGrid &map,
                                const WallFootprintPose &pose,
                                const ConvexWallFootprint &footprint,
                                const OccupancyGridWallIndex *index = nullptr);

// Sweeps the footprint between path poses. max_step_m controls both linear
// and corner-rotation sampling and must be positive.
bool occupancyGridFootprintPathFree(const nav_msgs::msg::OccupancyGrid &map,
                                    const std::vector<WallFootprintPose> &path,
                                    double front_extent_m, double rear_extent_m,
                                    double half_width_m, double max_step_m,
                                    const OccupancyGridWallIndex *index = nullptr);

bool occupancyGridFootprintPathFree(const nav_msgs::msg::OccupancyGrid &map,
                                    const std::vector<WallFootprintPose> &path,
                                    const ConvexWallFootprint &footprint,
                                    double max_step_m,
                                    const OccupancyGridWallIndex *index = nullptr);

} // namespace reference_space_mppi_planner
