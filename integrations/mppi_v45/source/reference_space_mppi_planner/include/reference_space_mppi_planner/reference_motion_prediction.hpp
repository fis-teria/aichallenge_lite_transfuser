#pragma once

#include "reference_space_mppi_planner/recent_motion_prediction.hpp"

namespace reference_space_mppi_planner::opponent_prediction {

struct ReferenceMotion {
  // along/lateral are physical velocity components at the source epoch;
  // acceleration is scalar tangential acceleration, all in metres/seconds.
  double stamp, s, d, along, lateral;
  double acceleration{0.};
};

inline std::optional<double> referenceDistanceScale(
    const ReferencePoseIndex &world, double s, double d) {
  // Average the polyline's heading derivative over one metre. Taking a
  // derivative inside a single straight segment would miss all bend geometry.
  const auto before=world.pose(s-.5,0.), after=world.pose(s+.5,0.);
  if (!before || !after) return {};
  const double curvature=std::remainder((*after)[2]-(*before)[2],2.*std::acos(-1.));
  const double scale=1.-curvature*d;
  return scale>0. && std::isfinite(scale) ? std::optional<double>(scale) : std::nullopt;
}

inline std::optional<ReferenceMotion> fitReferenceMotionFromHistory(
    const std::vector<PositionObservation> &history, const ReferencePoseIndex &world) {
  const auto physical = fitRecentMotion(history);
  if (!physical) return {};
  std::vector<PositionObservation> road;
  for (const auto &p : history) {
    if (p.stamp < history.back().stamp-.6-1e-8) continue;
    const auto station = world.projectStation(p.x,p.y);
    if (!station) return {};
    const auto pose = world.pose(*station,0.);
    if (!pose) return {};
    const double s = road.empty() ? *station :
        road.back().x+world.stationDifference(*station,road.back().x);
    const double d = -(p.x-(*pose)[0])*std::sin((*pose)[2])+
        (p.y-(*pose)[1])*std::cos((*pose)[2]);
    road.push_back({p.stamp,s,d,0.});
  }
  std::array<double,4> fitted{};
  if (!fitRecent(road,fitted)) return {};
  const auto scale=referenceDistanceScale(world,fitted[0],fitted[1]);
  if (!scale) return {};
  const double yaw=std::atan2(fitted[3],std::max(0.,fitted[2])*(*scale));
  return ReferenceMotion{history.back().stamp,fitted[0],fitted[1],
      physical->speed*std::cos(yaw),physical->speed*std::sin(yaw),physical->acceleration};
}

inline std::optional<ReferenceMotion> fitReferenceMotion(
    const std::vector<PositionObservation> &history, const ReferencePoseIndex &world) {
  auto state=fitReferenceMotionFromHistory(history,world);
  if (!state) return {};
  // Position and lateral motion retain the 0.6 s fit. A shorter acceleration
  // fit responds to braking onsets and releases when the estimates disagree.
  const auto first=std::find_if(history.begin(),history.end(),
      [&](const auto &p){return p.stamp>=history.back().stamp-.3-1e-8;});
  const std::vector<PositionObservation> recent(first,history.end());
  const auto short_fit=fitReferenceMotionFromHistory(recent,world);
  if (short_fit) {
    const double change=short_fit->acceleration-state->acceleration;
    // Blend continuously over 0.5-1.5 m/s^2 of disagreement; small changes
    // should not replace the stable acceleration estimate wholesale.
    const double weight=std::clamp(std::abs(change)-.5,0.,1.);
    state->acceleration+=weight*change;
  }
  return state;
}

inline std::optional<ReferenceMotion> referenceMotion(
    const PositionObservation &observation, double vx, double vy,
    const ReferencePoseIndex &world) {
  const auto s = world.projectStation(observation.x, observation.y);
  if (!s || !std::isfinite(vx) || !std::isfinite(vy)) return {};
  const auto pose = world.pose(*s, 0.);
  if (!pose) return {};
  const double c=std::cos((*pose)[2]), sn=std::sin((*pose)[2]);
  return ReferenceMotion{observation.stamp, *s,
      -(observation.x-(*pose)[0])*sn+(observation.y-(*pose)[1])*c,
      vx*c+vy*sn, -vx*sn+vy*c};
}

struct ReferenceMotionSample {
  PredictionPoint point;
  double along, lateral;
};

struct PhysicalMotionSample {
  double distance, speed, yaw;
};

inline PhysicalMotionSample physicalMotionAt(const ReferenceMotion &state,
    double time, double lateral_persistence, double acceleration_duration) {
  const double initial=std::hypot(state.along,state.lateral);
  const double until_stop=state.acceleration<0. ?
      initial/-state.acceleration : std::numeric_limits<double>::infinity();
  const double active=std::min({time,acceleration_duration,until_stop});
  const double speed=std::max(0.,initial+state.acceleration*active);
  const double moving=until_stop<=acceleration_duration ? std::min(time,until_stop) : time;
  const double lateral=lateral_persistence>0. ?
      state.lateral*std::exp(-moving/lateral_persistence) : 0.;
  return {initial*active+.5*state.acceleration*active*active+speed*(time-active),
      speed,std::atan2(lateral,state.along)};
}

// Both issued learner forecasts and planning snapshots use this integration.
// The grid is anchored at the source epoch; a fractional query never moves the
// grid origin, so delayed and undelayed forecasts agree at the same source time.
class ReferenceMotionIntegrator {
public:
  ReferenceMotionIntegrator(const ReferenceMotion &state, const ReferencePoseIndex &world,
      double lateral_persistence, double acceleration_duration)
      : state_(state), world_(world), lateral_(lateral_persistence),
        acceleration_(acceleration_duration), frame_{state.s,state.d,{}} {}

  std::optional<ReferenceMotionSample> at(double time) {
    if (!valid_ || !std::isfinite(time) || time<0. || time<step_*interval_-1e-10) return {};
    while ((step_+1)*interval_<=time+1e-10) {
      if (!prepareNext()) return {};
      frame_=*next_; next_.reset(); ++step_;
    }
    Frame sample=frame_;
    if (time>step_*interval_+1e-10) {
      if (!prepareNext()) return {};
      // A fractional query uses the same full source-grid step to decide the
      // switch as a longer query. Query order cannot restart or move it.
      if (next_->continuation) {
        sample.continuation=next_->continuation;
        if (!advanceCartesian(sample,time)) return {};
      } else {
        const double a=physicalMotionAt(state_,step_*interval_,lateral_,acceleration_).distance;
        const double b=physicalMotionAt(state_,(step_+1)*interval_,lateral_,acceleration_).distance;
        const double distance=physicalMotionAt(state_,time,lateral_,acceleration_).distance;
        const double u=b>a ? (distance-a)/(b-a) : 0.;
        sample.s+=u*(next_->s-frame_.s);
        sample.d+=u*(next_->d-frame_.d);
      }
    }
    const auto motion=physicalMotionAt(state_,time,lateral_,acceleration_);
    double yaw=motion.yaw;
    if (sample.continuation) {
      const auto pose=world_.pose(sample.s,sample.d);
      if (!pose) return {};
      yaw=std::remainder(sample.continuation->yaw-(*pose)[2],2.*std::acos(-1.));
    }
    return ReferenceMotionSample{{time,sample.s,sample.d,yaw},
        motion.speed*std::cos(yaw),motion.speed*std::sin(yaw)};
  }

private:
  struct Continuation {double x,y,yaw,distance;};
  struct Frame {double s,d;std::optional<Continuation> continuation;};

  bool prepareNext() {
    if (next_) return true;
    Frame next=frame_;
    const double from=step_*interval_, to=(step_+1)*interval_;
    if (!next.continuation && !advanceRoad(next.s,next.d,from,to)) {
      const auto pose=world_.pose(frame_.s,frame_.d);
      if (!pose) {valid_=false;return false;}
      const auto motion=physicalMotionAt(state_,from,lateral_,acceleration_);
      // Road coordinates have ceased to be usable. Continue from the last
      // valid position and physical heading, preserving the source clock.
      next.continuation=Continuation{(*pose)[0],(*pose)[1],(*pose)[2]+motion.yaw,motion.distance};
    }
    if (next.continuation && !advanceCartesian(next,to)) {valid_=false;return false;}
    next_=next;
    return true;
  }

  bool advanceCartesian(Frame &frame,double time) const {
    const auto &origin=*frame.continuation;
    const double distance=physicalMotionAt(state_,time,lateral_,acceleration_).distance-origin.distance;
    const double x=origin.x+distance*std::cos(origin.yaw);
    const double y=origin.y+distance*std::sin(origin.yaw);
    const auto station=world_.projectStation(x,y);
    if (!station) return false;
    const auto pose=world_.pose(*station,0.);
    if (!pose) return false;
    frame.s+=world_.stationDifference(*station,frame.s);
    frame.d=-(x-(*pose)[0])*std::sin((*pose)[2])+(y-(*pose)[1])*std::cos((*pose)[2]);
    return true;
  }

  bool advanceRoad(double &s,double &d,double from,double to) const {
    if (state_.acceleration<0.) {
      const double stop=std::hypot(state_.along,state_.lateral)/-state_.acceleration;
      if (stop<=acceleration_) to=std::min(to,stop);
    }
    if (to<=from) return true;
    const auto a=physicalMotionAt(state_,from,lateral_,acceleration_);
    const auto b=physicalMotionAt(state_,to,lateral_,acceleration_);
    const double distance=b.distance-a.distance;
    if (distance<=0.) return true;
    const auto middle=physicalMotionAt(state_,(from+to)*.5,lateral_,acceleration_);
    const double along=distance*std::cos(middle.yaw), lateral=distance*std::sin(middle.yaw);
    const auto first=referenceDistanceScale(world_,s,d);
    if (!first) return false;
    const auto metric=referenceDistanceScale(world_,s+.5*along/(*first),d+.5*lateral);
    if (!metric) return false;
    s+=along/(*metric); d+=lateral;
    return true;
  }

  static constexpr double interval_=.0125;
  const ReferenceMotion &state_;
  const ReferencePoseIndex &world_;
  double lateral_, acceleration_;
  Frame frame_;
  std::optional<Frame> next_;
  std::size_t step_{0};
  bool valid_{true};
};

inline std::optional<PredictionPoint> referenceMotionAt(const ReferenceMotion &state,
    double time, const ReferencePoseIndex &world, double lateral_persistence,
    double acceleration_duration = MotionPersistence{}.acceleration_sec) {
  ReferenceMotionIntegrator integrator(state,world,lateral_persistence,acceleration_duration);
  const auto sample=integrator.at(time);
  return sample ? std::optional<PredictionPoint>(sample->point) : std::nullopt;
}

inline std::shared_ptr<const Prediction> buildReferenceMotionPrediction(
    const ReferenceMotion &state, double start_stamp, double horizon,
    const ReferencePoseIndex &world, double lateral_persistence,
    double acceleration_duration = MotionPersistence{}.acceleration_sec) {
  const double age=start_stamp-state.stamp;
  if (!std::isfinite(age) || age < 0. || !std::isfinite(horizon) || horizon <= 0. ||
      !std::isfinite(lateral_persistence) || lateral_persistence < 0. ||
      !std::isfinite(acceleration_duration) || acceleration_duration < 0.) return {};
  auto result=std::make_shared<Prediction>();
  result->source_stamp=state.stamp;
  ReferenceMotionIntegrator integrator(state,world,lateral_persistence,acceleration_duration);
  const auto first=integrator.at(age);
  if (!first) return {};
  const auto pose=world.pose(first->point.global_s,first->point.d);
  if (!pose) return {};
  const double c=std::cos((*pose)[2]),sn=std::sin((*pose)[2]);
  result->vx=first->along*c-first->lateral*sn;
  result->vy=first->along*sn+first->lateral*c;
  const auto steps=static_cast<std::size_t>(std::ceil(horizon/.0125));
  result->points.reserve(steps+1);
  result->points.push_back(first->point);
  result->points.back().time=0.;
  for (std::size_t i=1; i<=steps; ++i) {
    const double t=horizon*static_cast<double>(i)/steps;
    const auto sample=integrator.at(age+t);
    if (!sample) return {};
    auto p=sample->point; p.time=t;
    result->points.push_back(p);
  }
  return result;
}

} // namespace reference_space_mppi_planner::opponent_prediction
