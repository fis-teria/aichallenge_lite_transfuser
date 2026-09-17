#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace simple_pure_pursuit {

struct DelayReferencePoint { double x, y; };
struct DelayReferenceProjection {
  bool valid{false};
  std::size_t lower{0};
  double s{0.0}, distance{INFINITY}, tx{1.0}, ty{0.0};
};
struct DelayLeader {
  std::string id;
  double speed_mps{0.0}, forward_m{0.0};
};

// Only the measured Reference and short V2X velocity history are needed while
// the planner is busy. No candidate generation or opponent rollout runs here.
class ReferenceDelayFallback {
 public:
  void setReference(std::vector<DelayReferencePoint> points) {
    points_ = std::move(points);
    arc_.assign(points_.size(), 0.0);
    for (std::size_t i = 1; i < points_.size(); ++i)
      arc_[i] = arc_[i-1] + std::hypot(points_[i].x-points_[i-1].x,
                                     points_[i].y-points_[i-1].y);
    closed_ = points_.size() >= 3 &&
        std::hypot(points_.back().x-points_.front().x,
                   points_.back().y-points_.front().y) < 5.0;
  }
  bool valid() const { return points_.size() >= 3 && arc_.back() > 0.0; }
  bool closed() const { return closed_; }

  DelayReferenceProjection project(double x, double y) const {
    DelayReferenceProjection best;
    if (!std::isfinite(x) || !std::isfinite(y)) return best;
    for (std::size_t i = 1; i < points_.size(); ++i) {
      const auto &a = points_[i-1]; const auto &b = points_[i];
      const double dx = b.x-a.x, dy = b.y-a.y, length = arc_[i]-arc_[i-1];
      if (!(length > 1e-6)) continue;
      const double u = std::clamp(((x-a.x)*dx+(y-a.y)*dy)/(length*length),0.0,1.0);
      const double distance = std::hypot(x-a.x-u*dx,y-a.y-u*dy);
      if (distance < best.distance)
        best = {true,i-1,arc_[i-1]+u*length,distance,dx/length,dy/length};
    }
    return best;
  }

  void observe(const std::string &id, double x, double y, double stamp,
               double receive, double uncertainty) {
    if (id.empty() || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(stamp) || !std::isfinite(receive)) return;
    auto it = vehicles_.find(id);
    if (it != vehicles_.end() && stamp <= it->second.stamp) return;
    Vehicle next{x,y,stamp,receive,0.0,0.0,std::max(0.0,uncertainty)};
    if (it != vehicles_.end()) {
      const auto &previous = it->second;
      const double dt = stamp-previous.stamp;
      next.vx = previous.vx; next.vy = previous.vy;
      if (dt > 1e-3 && dt <= 1.0 && std::hypot(x-previous.x,y-previous.y) <= 10.0) {
        const double alpha = std::clamp(dt/(0.30+dt),0.05,1.0);
        next.vx += alpha*((x-previous.x)/dt-next.vx);
        next.vy += alpha*((y-previous.y)/dt-next.vy);
      }
    }
    vehicles_[id] = next;
  }

  std::optional<DelayLeader> leader(double x, double y, double now,
      const std::string &own_id, double maximum_forward_m = 40.0,
      double conflict_width_m = 1.45, double timeout_sec = 0.5) const {
    const auto ego = project(x,y);
    if (!ego.valid || !valid()) return std::nullopt;
    std::optional<DelayLeader> best;
    for (const auto &[id,v] : vehicles_) {
      if (id == own_id || now-v.receive < 0.0 || now-v.receive > timeout_sec ||
          now-v.stamp < 0.0 || now-v.stamp > timeout_sec) continue;
      const auto p = project(v.x,v.y);
      if (!p.valid || p.distance > conflict_width_m+v.uncertainty) continue;
      double forward = p.s-ego.s;
      if (closed_) forward = std::remainder(forward,arc_.back());
      if (forward <= 0.0 || forward > maximum_forward_m) continue;
      if (!best || forward < best->forward_m)
        best = DelayLeader{id,std::max(0.0,v.vx*p.tx+v.vy*p.ty),forward};
    }
    return best;
  }
  void clearVehicles() { vehicles_.clear(); }

 private:
  struct Vehicle { double x,y,stamp,receive,vx,vy,uncertainty; };
  std::vector<DelayReferencePoint> points_;
  std::vector<double> arc_;
  std::map<std::string,Vehicle> vehicles_;
  bool closed_{false};
};

}  // namespace simple_pure_pursuit
