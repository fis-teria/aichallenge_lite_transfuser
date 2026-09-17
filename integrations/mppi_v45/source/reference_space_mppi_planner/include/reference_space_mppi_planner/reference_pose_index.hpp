#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace reference_space_mppi_planner {

// One instance per immutable opponent snapshot. Exact time keys preserve the
// swept validator's sample times, including distinct adjacent floating values.
class OpponentPoseCache {
public:
  using Poses = std::vector<std::optional<std::array<double, 3U>>>;
  explicit OpponentPoseCache(std::size_t capacity = 16384U)
      : capacity_(capacity) {}

  template <class Compute>
  std::shared_ptr<const Poses> at(double time, Compute compute) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = poses_.find(time);
      if (found != poses_.end())
        return found->second;
    }
    auto result = std::make_shared<const Poses>(compute());
    std::lock_guard<std::mutex> lock(mutex_);
    if (poses_.size() < capacity_) {
      return poses_.emplace(time, result).first->second;
    }
    return result;
  }

private:
  std::size_t capacity_;
  std::mutex mutex_;
  std::map<double, std::shared_ptr<const Poses>> poses_;
};

// Immutable index: preserve the legacy arc-length and degenerate-segment
// semantics, while avoiding a full course scan for every collision query.
class ReferencePoseIndex {
public:
  template <class Points, class Position>
  ReferencePoseIndex(const Points &points, Position position) {
    if (points.size() < 2U)
      return;
    const auto first = position(points.front());
    const auto last = position(points.back());
    closed_ = std::hypot(first[0] - last[0], first[1] - last[1]) < 5.0;
    segments_.reserve(points.size() - 1U);
    double searchable_length = 0.0;
    for (std::size_t i = 0; i + 1U < points.size(); ++i) {
      const auto a = position(points[i]);
      const auto b = position(points[i + 1U]);
      const double dx = b[0] - a[0], dy = b[1] - a[1];
      const double length = std::hypot(dx, dy);
      if (!std::isfinite(length)) {
        occupancy_valid_=false;
        continue;
      }
      length_ += length;
      if (length < 1.0e-4)
        continue;
      const double yaw = std::atan2(dy, dx);
      segments_.push_back({a[0], a[1], dx, dy, length, searchable_length,
                           searchable_length + length, yaw, std::sin(yaw),
                           std::cos(yaw), i + 2U == points.size()});
      searchable_length += length;
    }
    if (!segments_.empty()) {
      projection_nodes_.reserve(2U * segments_.size());
      buildProjectionIndex(0U, segments_.size());
    }
  }

  // Exact nearest-segment projection; the immutable bounding tree avoids a
  // whole-course scan for each predicted vehicle corner. Equal distances use
  // the earliest segment, independently of tree traversal order.
  std::optional<double> projectStation(double x, double y) const {
    if (!std::isfinite(x) || !std::isfinite(y) || projection_nodes_.empty() ||
        !occupancy_valid_ || !std::isfinite(length_) || length_ < 1.0e-3 ||
        segments_.back().end != length_)
      return std::nullopt;
    double distance = std::numeric_limits<double>::infinity(), station = 0.0;
    std::size_t segment = segments_.size();
    projectNode(0U, x, y, distance, station, segment);
    return std::isfinite(distance) ? std::optional<double>(station) : std::nullopt;
  }

  double stationDifference(double station, double origin) const {
    const double delta = station - origin;
    return closed_ ? std::remainder(delta, length_) : delta;
  }

  std::optional<std::array<double, 3U>> pose(double s, double d) const {
    if (!std::isfinite(s) || !std::isfinite(d) || !std::isfinite(length_) ||
        length_ < 1.0e-3 || segments_.empty())
      return std::nullopt;
    if (closed_) {
      s = std::fmod(s, length_);
      if (s < 0.0)
        s += length_;
    } else {
      s = std::clamp(s, 0.0, length_);
    }
    auto it = std::lower_bound(segments_.begin(), segments_.end(), s,
                               [](const Segment &segment, double value) {
                                 return segment.end < value;
                               });
    if (it == segments_.end()) {
      if (!segments_.back().last)
        return std::nullopt;
      it = segments_.end() - 1;
    }
    const double ratio = std::clamp((s - it->start) / it->length, 0.0, 1.0);
    return std::array<double, 3U>{it->x + ratio * it->dx - it->sine * d,
                                  it->y + ratio * it->dy + it->cosine * d,
                                  it->yaw};
  }

  struct OccupancyTile {
    double x, y, yaw, half_s, half_d;
  };

  // Configuration-space polygon: ego reference point versus both bodies.
  // Used for visualization and as an independent vertex-based SAT oracle.
  static std::vector<std::array<double,2>> forbiddenPolygon(
      const OccupancyTile &tile,double ego_yaw,double relative_yaw,
      double hl,double hw,double inflation_s,double inflation_d) {
    using P=std::array<double,2>;
    const double c=std::cos(tile.yaw), s=std::sin(tile.yaw);
    const double e=ego_yaw-tile.yaw, o=relative_yaw;
    const double hs=tile.half_s+std::max(0.0,inflation_s-
        hl*(std::abs(std::cos(e))+std::abs(std::cos(o)))-hw*(std::abs(std::sin(e))+std::abs(std::sin(o))));
    const double hd=tile.half_d+std::max(0.0,inflation_d-
        hl*(std::abs(std::sin(e))+std::abs(std::sin(o)))-hw*(std::abs(std::cos(e))+std::abs(std::cos(o))));
    const std::array<P,6> generators{{{hs*c,hs*s},{-hd*s,hd*c},
        {hl*std::cos(ego_yaw),hl*std::sin(ego_yaw)},
        {-hw*std::sin(ego_yaw),hw*std::cos(ego_yaw)},
        {hl*std::cos(tile.yaw+o),hl*std::sin(tile.yaw+o)},
        {-hw*std::sin(tile.yaw+o),hw*std::cos(tile.yaw+o)}}};
    std::vector<P> vertices;
    for(unsigned mask=0;mask<64;++mask) {
      P p{tile.x,tile.y};
      for(unsigned k=0;k<6;++k) for(unsigned j=0;j<2;++j)
        p[j]+=((mask>>k)&1?1:-1)*generators[k][j];
      vertices.push_back(p);
    }
    std::sort(vertices.begin(),vertices.end());
    vertices.erase(std::unique(vertices.begin(),vertices.end()),vertices.end());
    const auto cross=[](P a,P b,P c) {return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);};
    std::vector<P> hull;
    for(const auto &p:vertices) {
      while(hull.size()>1 && cross(hull[hull.size()-2],hull.back(),p)<=0) hull.pop_back();
      hull.push_back(p);
    }
    const auto lower=hull.size();
    for(auto it=vertices.rbegin()+1;it!=vertices.rend();++it) {
      while(hull.size()>lower && cross(hull[hull.size()-2],hull.back(),*it)<=0) hull.pop_back();
      hull.push_back(*it);
    }
    return hull; // Closed CCW ring.
  }

  // The union of these translation rectangles is the continuous s/d set in
  // the SAME piecewise Reference model as pose(). Include both closures at
  // corners; never connect different segment normals with a fictitious hull.
  template <class Visit>
  bool visitOccupancy(double low, double high, double dlow, double dhigh,
                      Visit visit) const {
    return visitOccupancyFrames(low,high,dlow,dhigh,
        [&](const OccupancyTile &tile,double,double) { visit(tile); });
  }

  // Exact fixed-time SAT for a body translated over each tile. Inflation is
  // only the configured excess beyond the two projected bodies, not another
  // copy of the vehicle dimensions. Touching counts as overlap.
  std::optional<bool> occupancyOverlap(const std::array<double,3> &ego,
      double low,double high,double dlow,double dhigh,double relative_yaw,
      double half_length,double half_width,double inflation_s,double inflation_d) const {
    for(double v : {ego[0],ego[1],ego[2],relative_yaw,half_length,half_width,
                     inflation_s,inflation_d}) if(!std::isfinite(v)) return std::nullopt;
    if(half_length<=0 || half_width<=0 || inflation_s<0 || inflation_d<0)
      return std::nullopt;
    const std::array<double,2> e{std::cos(ego[2]),std::sin(ego[2])};
    bool overlap=false, arithmetic_valid=true;
    const bool valid=visitOccupancyFrames(low,high,dlow,dhigh,
        [&](const OccupancyTile &tile,double cosine,double sine) {
      if(overlap) return;
      const double oy=tile.yaw+relative_yaw;
      const std::array<double,2> o{std::cos(oy),std::sin(oy)}, r{cosine,sine};
      const auto radius=[&](const std::array<double,2> &axis,const std::array<double,2> &heading) {
        return half_length*std::abs(axis[0]*heading[0]+axis[1]*heading[1])+
               half_width*std::abs(-axis[0]*heading[1]+axis[1]*heading[0]);
      };
      const std::array<double,2> n{-r[1],r[0]};
      const double hs=tile.half_s+std::max(0.0,inflation_s-radius(r,e)-radius(r,o));
      const double hd=tile.half_d+std::max(0.0,inflation_d-radius(n,e)-radius(n,o));
      if(!std::isfinite(tile.x) || !std::isfinite(tile.y) ||
          !std::isfinite(hs) || !std::isfinite(hd)) {arithmetic_valid=false;return;}
      for(const auto &axis:std::array<std::array<double,2>,6>{e,{-e[1],e[0]},o,{-o[1],o[0]},r,n}) {
        const double support=radius(axis,e)+radius(axis,o)+
            hs*std::abs(axis[0]*r[0]+axis[1]*r[1])+hd*std::abs(axis[0]*n[0]+axis[1]*n[1]);
        if(!std::isfinite(support)) {arithmetic_valid=false;return;}
        if(std::abs((ego[0]-tile.x)*axis[0]+(ego[1]-tile.y)*axis[1])>support+1e-9) return;
      }
      overlap=true;
    });
    return valid && arithmetic_valid?std::optional<bool>(overlap):std::nullopt;
  }

private:
  // Segment headings are immutable and already prepared by the constructor.
  template <class Visit>
  bool visitOccupancyFrames(double low,double high,double dlow,double dhigh,
                            Visit visit) const {
    if (!std::isfinite(low) || !std::isfinite(high) ||
        !std::isfinite(dlow) || !std::isfinite(dhigh) || low > high ||
        dlow > dhigh || segments_.empty() || length_ < .001 ||
        !std::isfinite(length_) || !occupancy_valid_ ||
        !std::isfinite(dhigh-dlow) || segments_.back().end != length_)
      return false;
    const auto range = [&](double a, double b) {
      auto it = std::lower_bound(segments_.begin(), segments_.end(), a,
          [](const Segment &seg, double s) { return seg.end < s; });
      for (; it != segments_.end() && it->start <= b; ++it) {
        const double l = std::max(a, it->start), h = std::min(b, it->end);
        const double s = l + (h-l)*.5, d = dlow + (dhigh-dlow)*.5;
        visit(OccupancyTile{it->x + (s-it->start)*it->cosine-it->sine*d,
                            it->y + (s-it->start)*it->sine+it->cosine*d,
                            it->yaw, (h-l)*.5, (dhigh-dlow)*.5},
              it->cosine,it->sine);
      }
    };
    if (!closed_) range(std::clamp(low,0.0,length_), std::clamp(high,0.0,length_));
    else if (high-low >= length_) range(0,length_);
    else {
      double a=std::fmod(low,length_); if(a<0) a+=length_;
      const double b=a+(high-low);
      if(b>=length_) {range(a,length_); range(0,b-length_);}
      else {range(a,b); if(a==0) range(length_,length_);}
    }
    return true;
  }

  struct Segment {
    double x, y, dx, dy, length, start, end, yaw, sine, cosine;
    bool last;
  };
  struct ProjectionNode {
    double xmin, xmax, ymin, ymax;
    std::size_t begin, end, left{0U}, right{0U};
  };
  std::size_t buildProjectionIndex(std::size_t begin, std::size_t end) {
    const auto index = projection_nodes_.size();
    projection_nodes_.push_back({0, 0, 0, 0, begin, end});
    if (end - begin <= 8U) {
      const auto &first = segments_[begin];
      auto &node = projection_nodes_[index];
      node.xmin = node.xmax = first.x;
      node.ymin = node.ymax = first.y;
      for (auto i = begin; i < end; ++i) {
        const auto &s = segments_[i];
        node.xmin = std::min({node.xmin, s.x, s.x + s.dx});
        node.xmax = std::max({node.xmax, s.x, s.x + s.dx});
        node.ymin = std::min({node.ymin, s.y, s.y + s.dy});
        node.ymax = std::max({node.ymax, s.y, s.y + s.dy});
      }
    } else {
      const auto middle = begin + (end - begin) / 2U;
      const auto left = buildProjectionIndex(begin, middle);
      const auto right = buildProjectionIndex(middle, end);
      const auto &a = projection_nodes_[left], &b = projection_nodes_[right];
      projection_nodes_[index] = {std::min(a.xmin, b.xmin), std::max(a.xmax, b.xmax),
          std::min(a.ymin, b.ymin), std::max(a.ymax, b.ymax), begin, end, left, right};
    }
    return index;
  }
  double projectionDistance(std::size_t index, double x, double y) const {
    const auto &n = projection_nodes_[index];
    const double dx = x - std::clamp(x, n.xmin, n.xmax);
    const double dy = y - std::clamp(y, n.ymin, n.ymax);
    return dx * dx + dy * dy;
  }
  void projectNode(std::size_t index, double x, double y, double &distance,
                   double &station, std::size_t &segment) const {
    if (projectionDistance(index, x, y) > distance) return;
    const auto &node = projection_nodes_[index];
    if (node.end - node.begin > 8U) {
      auto a = node.left, b = node.right;
      if (projectionDistance(b, x, y) < projectionDistance(a, x, y)) std::swap(a, b);
      projectNode(a, x, y, distance, station, segment);
      projectNode(b, x, y, distance, station, segment);
      return;
    }
    for (auto i = node.begin; i < node.end; ++i) {
      const auto &s = segments_[i];
      const double u = std::clamp(((x-s.x)*s.dx + (y-s.y)*s.dy) /
                                 (s.length*s.length), 0.0, 1.0);
      const double dx = x - (s.x + u*s.dx), dy = y - (s.y + u*s.dy);
      const double candidate_distance = dx*dx + dy*dy;
      if (candidate_distance < distance || (candidate_distance == distance && i < segment)) {
        distance = candidate_distance;
        station = s.start + u*s.length;
        segment = i;
      }
    }
  }
  std::vector<Segment> segments_;
  std::vector<ProjectionNode> projection_nodes_;
  double length_{0.0};
  bool closed_{false};
  bool occupancy_valid_{true};
};

} // namespace reference_space_mppi_planner
