#pragma once

#include "reference_space_mppi_planner/prior_lap_prediction.hpp"
#include <complex>

namespace reference_space_mppi_planner::opponent_prediction {

struct MotionPersistence {
  double acceleration_sec{0.6};
  double yaw_rate_sec{0.6};
};

struct RecentMotion {
  double x, y, speed, yaw, acceleration, yaw_rate;
};

// Durations start at the observed source epoch. Aligning an old observation
// must not restart its acceleration or turn assumption at every planning tick.
inline RecentMotion advanceRecentMotion(RecentMotion state, double from,
    double to, const MotionPersistence &persistence) {
  while (from < to) {
    const double a = from < persistence.acceleration_sec ? state.acceleration : 0.;
    const double w = from < persistence.yaw_rate_sec ? state.yaw_rate : 0.;
    double end = to;
    if (from < persistence.acceleration_sec) end = std::min(end, persistence.acceleration_sec);
    if (from < persistence.yaw_rate_sec) end = std::min(end, persistence.yaw_rate_sec);
    if (state.speed <= 0. && a <= 0.) return state;
    const double stop = a < 0. ? from-state.speed/a : std::numeric_limits<double>::infinity();
    end = std::min(end,stop);
    const double dt = end-from;
    std::complex<double> delta;
    if (std::abs(w) < 1e-5) {
      delta = std::polar(state.speed*dt+.5*a*dt*dt, state.yaw);
    } else {
      const std::complex<double> iw(0.,w), e = std::exp(iw*dt);
      delta = std::polar(1.,state.yaw) *
          (state.speed*(e-1.)/iw + a*(e*dt/iw+(e-1.)/(w*w)));
    }
    state.x += delta.real(); state.y += delta.imag();
    state.yaw += w*dt;
    state.speed = std::max(0., state.speed+a*dt);
    if (end == stop) { state.speed = 0.; return state; }
    from = end;
  }
  return state;
}

// A displacement measures average scalar speed at the interval midpoint.
// Fit its trend independently of the Cartesian curve: differentiating an XY
// quadratic creates a fictitious tangential acceleration on constant-speed turns.
inline std::optional<std::array<double,2>> fitPhysicalSpeed(
    const std::vector<PositionObservation> &history) {
  if (history.size() < 3U) return {};
  const double stamp=history.back().stamp;
  const PositionObservation *previous=nullptr;
  double weight=0., time=0., speed=0., time2=0., time_speed=0.;
  std::size_t count=0;
  for (const auto &p:history) {
    if (p.stamp < stamp-.6-1e-8 || (previous && p.stamp<=previous->stamp)) continue;
    if (previous) {
      const double dt=p.stamp-previous->stamp;
      const double t=(p.stamp+previous->stamp)*.5-stamp;
      const double v=std::hypot(p.x-previous->x,p.y-previous->y)/dt;
      weight+=dt; time+=dt*t; speed+=dt*v;
      time2+=dt*t*t; time_speed+=dt*t*v; ++count;
    }
    previous=&p;
  }
  const double denominator=weight*time2-time*time;
  if (count<2U || denominator<=1e-12) return {};
  const double a=(weight*time_speed-time*speed)/denominator;
  const double v=(speed-a*time)/weight;
  if (!std::isfinite(v) || !std::isfinite(a)) return {};
  return std::array<double,2>{std::max(0.,v),a};
}

inline std::optional<RecentMotion> fitRecentMotion(
    const std::vector<PositionObservation> &history) {
  std::array<double,4> fitted{};
  std::array<double,2> acceleration{};
  if (!fitRecent(history, fitted, &acceleration) ||
      !std::isfinite(acceleration[0]) || !std::isfinite(acceleration[1])) return {};
  const auto scalar=fitPhysicalSpeed(history);
  if (!scalar) return {};
  const double cartesian_speed = std::hypot(fitted[2],fitted[3]);
  RecentMotion state{fitted[0],fitted[1],(*scalar)[0],std::atan2(fitted[3],fitted[2]),0.,0.};
  if (state.speed >= .2 && cartesian_speed >= .2) {
    state.acceleration = (*scalar)[1];
    state.yaw_rate = (fitted[2]*acceleration[1]-fitted[3]*acceleration[0])/
        (cartesian_speed*cartesian_speed);
  } else {
    state.speed = 0.;
  }
  return state;
}

inline std::shared_ptr<const Prediction> buildRecentMotionPrediction(
    const std::vector<PositionObservation> &history, double start_stamp,
    double horizon, const ReferencePoseIndex &world,
    const MotionPersistence &persistence) {
  if (!std::isfinite(start_stamp) || !std::isfinite(horizon) || horizon <= 0. ||
      !std::isfinite(persistence.acceleration_sec) || persistence.acceleration_sec < 0. ||
      !std::isfinite(persistence.yaw_rate_sec) || persistence.yaw_rate_sec < 0.) return {};
  const auto fitted = fitRecentMotion(history);
  if (!fitted) return {};
  const double age = start_stamp-history.back().stamp;
  if (age < 0.) return {};
  auto state = *fitted;
  state = advanceRecentMotion(state, 0., age, persistence);
  auto prediction = std::make_shared<Prediction>();
  prediction->source_stamp = history.back().stamp;
  prediction->vx = state.speed*std::cos(state.yaw);
  prediction->vy = state.speed*std::sin(state.yaw);
  const auto steps = static_cast<std::size_t>(std::ceil(horizon/.0125));
  prediction->points.reserve(steps+1);
  double previous = age;
  for (std::size_t i=0; i<=steps; ++i) {
    const double t = horizon*static_cast<double>(i)/steps;
    state = advanceRecentMotion(state, previous, age+t, persistence);
    previous = age+t;
    const auto station = world.projectStation(state.x,state.y);
    if (!station) return {};
    const auto pose = world.pose(*station,0.);
    if (!pose) return {};
    const double d = -(state.x-(*pose)[0])*std::sin((*pose)[2])+
        (state.y-(*pose)[1])*std::cos((*pose)[2]);
    const double s = prediction->points.empty() ? *station :
        prediction->points.back().global_s+world.stationDifference(*station,prediction->points.back().global_s);
    prediction->points.push_back({t,s,d,std::remainder(state.yaw-(*pose)[2],2.*std::acos(-1.))});
  }
  return prediction;
}

} // namespace reference_space_mppi_planner::opponent_prediction
