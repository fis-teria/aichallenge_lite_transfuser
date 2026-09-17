#include "reference_space_mppi_planner/static_wall_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace reference_space_mppi_planner {
namespace {

constexpr double kOverlapTolerance = 1.0e-9;

void fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
}

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n\"'");
  const auto last = value.find_last_not_of(" \t\r\n\"'");
  return first == std::string::npos ? std::string{}
                                    : value.substr(first, last - first + 1U);
}

bool pgmToken(std::istream &stream, std::string *token) {
  while (stream >> *token) {
    if (!token->empty() && token->front() == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      continue;
    }
    return true;
  }
  return false;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool intervalsOverlap(double center_delta, double first_radius,
                      double second_radius) {
  return std::abs(center_delta) <=
         first_radius + second_radius + kOverlapTolerance;
}

} // namespace

ConvexWallFootprint::ConvexWallFootprint(const std::vector<double> &xy) {
  if (xy.size() < 6U || xy.size() % 2U != 0U)
    throw std::invalid_argument("wall footprint requires at least three xy pairs");
  for (std::size_t i = 0; i < xy.size(); i += 2U) {
    if (!std::isfinite(xy[i]) || !std::isfinite(xy[i + 1U]))
      throw std::invalid_argument("wall footprint coordinates must be finite");
    vertices_.push_back({xy[i], xy[i + 1U]});
    radius_ = std::max(radius_, std::hypot(xy[i], xy[i + 1U]));
  }
  double winding = 0.0;
  for (std::size_t i = 0; i < vertices_.size(); ++i) {
    const auto &a = vertices_[i];
    const auto &b = vertices_[(i + 1U) % vertices_.size()];
    const double dx = b[0] - a[0], dy = b[1] - a[1];
    const double length = std::hypot(dx, dy);
    if (length <= kOverlapTolerance)
      throw std::invalid_argument("wall footprint has a duplicate edge vertex");
    Axis axis{-dy / length, dx / length,
              std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()};
    const double anchor = axis.x * a[0] + axis.y * a[1];
    for (const auto &p : vertices_) {
      const double projection = axis.x * p[0] + axis.y * p[1];
      axis.minimum = std::min(axis.minimum, projection);
      axis.maximum = std::max(axis.maximum, projection);
      const double side = projection - anchor;
      if (std::abs(side) > kOverlapTolerance) {
        if (winding != 0.0 && side * winding < 0.0)
          throw std::invalid_argument("wall footprint must be convex and ordered");
        winding = side;
      }
    }
    axes_.push_back(axis);
  }
  if (winding == 0.0)
    throw std::invalid_argument("wall footprint has zero area");
}

OccupancyGridWallIndex::OccupancyGridWallIndex(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) : map_(std::move(map)) {
  if (!map_ || map_->info.width == 0U || map_->info.height == 0U ||
      map_->data.size() < static_cast<std::size_t>(map_->info.width) * map_->info.height)
    return;
  const std::size_t stride = static_cast<std::size_t>(map_->info.width) + 1U;
  prefix_.assign(stride * (static_cast<std::size_t>(map_->info.height) + 1U), 0U);
  for (std::size_t y = 0; y < map_->info.height; ++y) {
    std::uint64_t row = 0U;
    for (std::size_t x = 0; x < map_->info.width; ++x) {
      const auto cell = map_->data[y * map_->info.width + x];
      row += cell < 0 || cell >= 50;
      prefix_[(y + 1U) * stride + x + 1U] = prefix_[y * stride + x + 1U] + row;
    }
  }
}

bool OccupancyGridWallIndex::rectangleFree(
    const nav_msgs::msg::OccupancyGrid &map, int minimum_x, int minimum_y,
    int maximum_x, int maximum_y) const {
  if (&map != map_.get() || prefix_.empty() || minimum_x < 0 || minimum_y < 0 ||
      maximum_x < minimum_x || maximum_y < minimum_y ||
      static_cast<std::uint32_t>(maximum_x) >= map.info.width ||
      static_cast<std::uint32_t>(maximum_y) >= map.info.height)
    return false;
  const std::size_t stride = static_cast<std::size_t>(map.info.width) + 1U;
  const auto at = [&](std::size_t x, std::size_t y) { return prefix_[y * stride + x]; };
  return at(maximum_x + 1U, maximum_y + 1U) + at(minimum_x, minimum_y) ==
         at(minimum_x, maximum_y + 1U) + at(maximum_x + 1U, minimum_y);
}

bool loadStaticWallMap(const std::string &yaml_path,
                       const std::string &frame_id,
                       nav_msgs::msg::OccupancyGrid *map, std::string *error) {
  if (map == nullptr) {
    fail(error, "wall_map_output_is_null");
    return false;
  }
  std::ifstream yaml(yaml_path);
  if (!yaml) {
    fail(error, "wall_map_yaml_open_failed");
    return false;
  }

  std::string image_name;
  double resolution = 0.0;
  double occupied_threshold = 0.65;
  double free_threshold = 0.196;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  int negate = 0;
  bool reading_origin = false;
  int origin_index = 0;
  std::string line;
  try {
    while (std::getline(yaml, line)) {
      const auto colon = line.find(':');
      if (colon != std::string::npos) {
        const std::string key = trim(line.substr(0U, colon));
        const std::string value = trim(line.substr(colon + 1U));
        reading_origin = key == "origin";
        if (key == "image")
          image_name = value;
        else if (key == "resolution")
          resolution = std::stod(value);
        else if (key == "occupied_thresh")
          occupied_threshold = std::stod(value);
        else if (key == "free_thresh")
          free_threshold = std::stod(value);
        else if (key == "negate")
          negate = std::stoi(value);
        continue;
      }
      if (reading_origin) {
        const auto dash = line.find('-');
        if (dash != std::string::npos && origin_index < 3) {
          const double value = std::stod(trim(line.substr(dash + 1U)));
          if (origin_index++ == 0)
            origin_x = value;
          else if (origin_index == 2)
            origin_y = value;
          else
            origin_yaw = value;
        }
      }
    }
  } catch (const std::exception &) {
    fail(error, "wall_map_yaml_value_invalid");
    return false;
  }
  if (image_name.empty() || !std::isfinite(resolution) || resolution <= 0.0 ||
      origin_index != 3 || !std::isfinite(origin_yaw) ||
      !(free_threshold < occupied_threshold)) {
    fail(error, "wall_map_yaml_invalid");
    return false;
  }

  const auto pgm_path =
      std::filesystem::path(yaml_path).parent_path() / image_name;
  std::ifstream pgm(pgm_path, std::ios::binary);
  std::string magic;
  std::string token;
  if (!pgmToken(pgm, &magic) || (magic != "P5" && magic != "P2") ||
      !pgmToken(pgm, &token)) {
    fail(error, "wall_map_pgm_header_invalid");
    return false;
  }

  std::size_t width = 0U;
  std::size_t height = 0U;
  int maximum_value = 0;
  try {
    width = static_cast<std::size_t>(std::stoull(token));
    if (!pgmToken(pgm, &token))
      throw std::runtime_error("height_missing");
    height = static_cast<std::size_t>(std::stoull(token));
    if (!pgmToken(pgm, &token))
      throw std::runtime_error("maximum_value_missing");
    maximum_value = std::stoi(token);
  } catch (const std::exception &) {
    fail(error, "wall_map_pgm_header_invalid");
    return false;
  }
  if (width == 0U || height == 0U || width > 100000000U / height ||
      maximum_value <= 0 || maximum_value > 255) {
    fail(error, "wall_map_pgm_dimensions_invalid");
    return false;
  }

  std::vector<unsigned char> pixels(width * height);
  if (magic == "P5") {
    pgm.get();
    pgm.read(reinterpret_cast<char *>(pixels.data()),
             static_cast<std::streamsize>(pixels.size()));
    if (pgm.gcount() != static_cast<std::streamsize>(pixels.size())) {
      fail(error, "wall_map_pgm_truncated");
      return false;
    }
  } else {
    try {
      for (auto &pixel : pixels) {
        if (!pgmToken(pgm, &token))
          throw std::runtime_error("pixel_missing");
        pixel = static_cast<unsigned char>(std::stoi(token));
      }
    } catch (const std::exception &) {
      fail(error, "wall_map_pgm_truncated");
      return false;
    }
  }

  nav_msgs::msg::OccupancyGrid loaded;
  loaded.header.frame_id = frame_id;
  loaded.info.resolution = static_cast<float>(resolution);
  loaded.info.width = static_cast<std::uint32_t>(width);
  loaded.info.height = static_cast<std::uint32_t>(height);
  loaded.info.origin.position.x = origin_x;
  loaded.info.origin.position.y = origin_y;
  loaded.info.origin.orientation.z = std::sin(0.5 * origin_yaw);
  loaded.info.origin.orientation.w = std::cos(0.5 * origin_yaw);
  loaded.data.assign(width * height, -1);
  for (std::size_t image_y = 0U; image_y < height; ++image_y) {
    const std::size_t cell_y = height - 1U - image_y;
    for (std::size_t x = 0U; x < width; ++x) {
      const double normalized =
          static_cast<double>(pixels[image_y * width + x]) / maximum_value;
      const double occupancy = negate != 0 ? normalized : 1.0 - normalized;
      loaded.data[cell_y * width + x] =
          occupancy > occupied_threshold ? static_cast<std::int8_t>(100)
          : occupancy < free_threshold   ? static_cast<std::int8_t>(0)
                                         : static_cast<std::int8_t>(-1);
    }
  }
  *map = std::move(loaded);
  return true;
}

bool occupancyGridFootprintFree(const nav_msgs::msg::OccupancyGrid &map,
                                const WallFootprintPose &pose,
                                double front_extent_m, double rear_extent_m,
                                double half_width_m,
                                const OccupancyGridWallIndex *index) {
  const double resolution = static_cast<double>(map.info.resolution);
  if (!std::isfinite(resolution) || resolution <= 0.0 || map.info.width == 0U ||
      map.info.height == 0U ||
      map.data.size() < static_cast<std::size_t>(map.info.width) *
                            static_cast<std::size_t>(map.info.height) ||
      !std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
      !std::isfinite(pose.yaw_rad) || !std::isfinite(front_extent_m) ||
      !std::isfinite(rear_extent_m) || !std::isfinite(half_width_m) ||
      front_extent_m < 0.0 || rear_extent_m < 0.0 || half_width_m < 0.0) {
    return false;
  }

  const auto &origin = map.info.origin;
  const double map_yaw = yawFromQuaternion(origin.orientation);
  const double map_cosine = std::cos(map_yaw);
  const double map_sine = std::sin(map_yaw);
  const double world_dx = pose.x_m - origin.position.x;
  const double world_dy = pose.y_m - origin.position.y;
  const double pose_x = map_cosine * world_dx + map_sine * world_dy;
  const double pose_y = -map_sine * world_dx + map_cosine * world_dy;
  const double relative_yaw = pose.yaw_rad - map_yaw;
  const double forward_x = std::cos(relative_yaw);
  const double forward_y = std::sin(relative_yaw);
  const double lateral_x = -forward_y;
  const double lateral_y = forward_x;
  const double half_length_m = 0.5 * (front_extent_m + rear_extent_m);
  const double center_offset_m = 0.5 * (front_extent_m - rear_extent_m);
  const double rectangle_x = pose_x + center_offset_m * forward_x;
  const double rectangle_y = pose_y + center_offset_m * forward_y;
  const double extent_x =
      half_length_m * std::abs(forward_x) + half_width_m * std::abs(lateral_x);
  const double extent_y =
      half_length_m * std::abs(forward_y) + half_width_m * std::abs(lateral_y);

  const int minimum_x =
      static_cast<int>(std::floor((rectangle_x - extent_x) / resolution));
  const int maximum_x =
      static_cast<int>(std::floor((rectangle_x + extent_x) / resolution));
  const int minimum_y =
      static_cast<int>(std::floor((rectangle_y - extent_y) / resolution));
  const int maximum_y =
      static_cast<int>(std::floor((rectangle_y + extent_y) / resolution));
  if (minimum_x < 0 || minimum_y < 0 ||
      maximum_x >= static_cast<int>(map.info.width) ||
      maximum_y >= static_cast<int>(map.info.height)) {
    return false;
  }

  // An empty enclosing rectangle proves the footprint free. Otherwise retain
  // the exact cell-overlap checks, including thin walls and unknown cells.
  if (index && index->rectangleFree(map, minimum_x, minimum_y, maximum_x, maximum_y))
    return true;
  const double cell_half = 0.5 * resolution;
  for (int cell_y = minimum_y; cell_y <= maximum_y; ++cell_y) {
    for (int cell_x = minimum_x; cell_x <= maximum_x; ++cell_x) {
      const auto index = static_cast<std::size_t>(cell_y) * map.info.width +
                         static_cast<std::size_t>(cell_x);
      if (map.data[index] >= 0 && map.data[index] < 50) {
        continue;
      }
      const double delta_x =
          (static_cast<double>(cell_x) + 0.5) * resolution - rectangle_x;
      const double delta_y =
          (static_cast<double>(cell_y) + 0.5) * resolution - rectangle_y;
      const bool overlaps_map_x =
          intervalsOverlap(delta_x, extent_x, cell_half);
      const bool overlaps_map_y =
          intervalsOverlap(delta_y, extent_y, cell_half);
      const bool overlaps_forward = intervalsOverlap(
          delta_x * forward_x + delta_y * forward_y, half_length_m,
          cell_half * (std::abs(forward_x) + std::abs(forward_y)));
      const bool overlaps_lateral = intervalsOverlap(
          delta_x * lateral_x + delta_y * lateral_y, half_width_m,
          cell_half * (std::abs(lateral_x) + std::abs(lateral_y)));
      if (overlaps_map_x && overlaps_map_y && overlaps_forward &&
          overlaps_lateral) {
        return false;
      }
    }
  }
  return true;
}

bool occupancyGridFootprintPathFree(const nav_msgs::msg::OccupancyGrid &map,
                                    const std::vector<WallFootprintPose> &path,
                                    double front_extent_m, double rear_extent_m,
                                    double half_width_m, double max_step_m,
                                    const OccupancyGridWallIndex *wall_index) {
  if (path.empty() || !std::isfinite(max_step_m) || max_step_m <= 0.0) {
    return false;
  }
  if (!occupancyGridFootprintFree(map, path.front(), front_extent_m,
                                  rear_extent_m, half_width_m, wall_index)) {
    return false;
  }
  const double corner_radius_m =
      std::hypot(std::max(front_extent_m, rear_extent_m), half_width_m);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const auto &previous = path[index - 1U];
    const auto &current = path[index];
    const double dx = current.x_m - previous.x_m;
    const double dy = current.y_m - previous.y_m;
    const double yaw_delta =
        std::remainder(current.yaw_rad - previous.yaw_rad, 2.0 * M_PI);
    const double swept_distance_m =
        std::hypot(dx, dy) + corner_radius_m * std::abs(yaw_delta);
    if (!std::isfinite(swept_distance_m)) {
      return false;
    }
    const std::size_t subdivisions = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(swept_distance_m / max_step_m)));
    for (std::size_t subdivision = 1U; subdivision <= subdivisions;
         ++subdivision) {
      const double ratio =
          static_cast<double>(subdivision) / static_cast<double>(subdivisions);
      const WallFootprintPose interpolated{
          previous.x_m + ratio * dx, previous.y_m + ratio * dy,
          previous.yaw_rad + ratio * yaw_delta};
      if (!occupancyGridFootprintFree(map, interpolated, front_extent_m,
                                      rear_extent_m, half_width_m, wall_index)) {
        return false;
      }
    }
  }
  return true;
}

bool occupancyGridFootprintFree(const nav_msgs::msg::OccupancyGrid &map,
                                const WallFootprintPose &pose,
                                const ConvexWallFootprint &footprint,
                                const OccupancyGridWallIndex *index) {
  const double resolution = map.info.resolution;
  if (!std::isfinite(resolution) || resolution <= 0.0 || map.info.width == 0U ||
      map.info.height == 0U || map.data.size() <
          static_cast<std::size_t>(map.info.width) * map.info.height ||
      !std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
      !std::isfinite(pose.yaw_rad))
    return false;
  const auto &origin = map.info.origin;
  const double map_yaw = yawFromQuaternion(origin.orientation);
  const double mc = std::cos(map_yaw), ms = std::sin(map_yaw);
  const double dx = pose.x_m - origin.position.x;
  const double dy = pose.y_m - origin.position.y;
  const double px = mc * dx + ms * dy, py = -ms * dx + mc * dy;
  const double c = std::cos(pose.yaw_rad - map_yaw);
  const double s = std::sin(pose.yaw_rad - map_yaw);
  double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
  double max_x = -min_x, max_y = -min_x;
  for (const auto &v : footprint.vertices()) {
    const double x = px + c * v[0] - s * v[1];
    const double y = py + s * v[0] + c * v[1];
    min_x = std::min(min_x, x); max_x = std::max(max_x, x);
    min_y = std::min(min_y, y); max_y = std::max(max_y, y);
  }
  const int x0 = static_cast<int>(std::floor((min_x - kOverlapTolerance) / resolution));
  const int x1 = static_cast<int>(std::floor((max_x + kOverlapTolerance) / resolution));
  const int y0 = static_cast<int>(std::floor((min_y - kOverlapTolerance) / resolution));
  const int y1 = static_cast<int>(std::floor((max_y + kOverlapTolerance) / resolution));
  if (x0 < 0 || y0 < 0 || x1 >= static_cast<int>(map.info.width) ||
      y1 >= static_cast<int>(map.info.height))
    return false;
  if (index && index->rectangleFree(map, x0, y0, x1, y1))
    return true;
  const double half = 0.5 * resolution;
  struct ProjectedAxis { double x, y, minimum, maximum; };
  std::vector<ProjectedAxis> axes;
  axes.reserve(footprint.axes().size());
  for (const auto &a : footprint.axes()) {
    const double ax = c * a.x - s * a.y, ay = s * a.x + c * a.y;
    const double radius = half * (std::abs(ax) + std::abs(ay));
    axes.push_back({ax, ay, a.minimum - radius - kOverlapTolerance,
                   a.maximum + radius + kOverlapTolerance});
  }
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const auto value = map.data[static_cast<std::size_t>(y) * map.info.width + x];
      if (value >= 0 && value < 50) continue;
      const double cx = (x + 0.5) * resolution;
      const double cy = (y + 0.5) * resolution;
      if (cx + half < min_x - kOverlapTolerance ||
          cx - half > max_x + kOverlapTolerance ||
          cy + half < min_y - kOverlapTolerance ||
          cy - half > max_y + kOverlapTolerance) continue;
      bool overlap = true;
      for (const auto &a : axes) {
        const double projection = a.x * (cx - px) + a.y * (cy - py);
        if (projection < a.minimum || projection > a.maximum) {
          overlap = false;
          break;
        }
      }
      if (overlap) return false;
    }
  }
  return true;
}

bool occupancyGridFootprintPathFree(const nav_msgs::msg::OccupancyGrid &map,
                                    const std::vector<WallFootprintPose> &path,
                                    const ConvexWallFootprint &footprint,
                                    double max_step_m,
                                    const OccupancyGridWallIndex *index) {
  if (path.empty() || !std::isfinite(max_step_m) || max_step_m <= 0.0 ||
      !occupancyGridFootprintFree(map, path.front(), footprint, index))
    return false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto &a = path[i - 1U], &b = path[i];
    const double dx = b.x_m - a.x_m, dy = b.y_m - a.y_m;
    const double yaw = std::remainder(b.yaw_rad - a.yaw_rad, 2.0 * M_PI);
    const double distance = std::hypot(dx, dy) + footprint.radius() * std::abs(yaw);
    if (!std::isfinite(distance)) return false;
    const auto n = std::max<std::size_t>(1U, std::ceil(distance / max_step_m));
    for (std::size_t j = 1; j <= n; ++j) {
      const double f = static_cast<double>(j) / n;
      if (!occupancyGridFootprintFree(map,
              {a.x_m + f * dx, a.y_m + f * dy, a.yaw_rad + f * yaw}, footprint, index))
        return false;
    }
  }
  return true;
}

} // namespace reference_space_mppi_planner
