#pragma once

#include "reference_space_mppi_planner/recent_motion_prediction.hpp"
#include "reference_space_mppi_planner/reference_motion_prediction.hpp"
#include <deque>

namespace reference_space_mppi_planner::opponent_prediction {

// Each opponent owns a learner. Only matured, previously issued forecasts
// enter the scores; planning reads the selected durations from its snapshot.
class OnlineMotionLearner {
public:
  static constexpr std::array<double, 5> durations{0., .3, .6, 1.2, 2.4};
  static constexpr std::array<double, 3> horizons{.5, 1., 2.};
  // A zero duration discards a moving opponent's fitted lateral velocity at
  // the source epoch. Every road candidate must begin with that same motion.
  static constexpr std::array<double, 5> lateral_durations{.3, .6, 1.2, 2.4, 9.};
  static constexpr std::size_t motion_model_count = durations.size()*durations.size();
  static constexpr std::size_t model_count =
      motion_model_count+durations.size()*lateral_durations.size();
  static constexpr std::size_t initial_model = 2*durations.size()+2;
  static constexpr std::size_t initial_reference_model =
      motion_model_count+2*lateral_durations.size()+1;
  using Losses = std::array<double, model_count>;

  struct Summary {
    std::size_t model{initial_model};
    MotionPersistence persistence{};
    bool follows_reference{false};
    double lateral_persistence_sec{0.6};
    std::array<std::size_t, horizons.size()> samples{};
    double mean_squared_error{0.};
  };

  static MotionPersistence persistence(std::size_t model) {
    return {durations.at(model/durations.size()), durations.at(model%durations.size())};
  }

  void observe(const std::vector<PositionObservation> &history,
      const ReferencePoseIndex *world = nullptr) {
    if (history.empty()) return;
    const auto &now = history.back();
    if (!std::isfinite(now.stamp) || !std::isfinite(now.x) || !std::isfinite(now.y)) return;
    if (previous_ && now.stamp <= previous_->stamp) return;
    // A long observation gap supplies no trajectory labels across the gap.
    if (previous_ && now.stamp-previous_->stamp > 1.) *this = OnlineMotionLearner{};
    if (previous_) {
      for (auto &forecast : pending_) {
        while (forecast.next < horizons.size() &&
               forecast.stamp+horizons[forecast.next] <= now.stamp) {
          const auto h = forecast.next++;
          const double target = forecast.stamp+horizons[h];
          if (target < previous_->stamp) continue;
          const double u = (target-previous_->stamp)/(now.stamp-previous_->stamp);
          const double x = previous_->x+u*(now.x-previous_->x);
          const double y = previous_->y+u*(now.y-previous_->y);
          const double alpha = samples_[h] == 0 ? 1. :
              -std::expm1(-std::log(2.)*(target-last_scored_[h])/3.);
          const auto context=forecast.acceleration_context;
          const double road_alpha=-std::expm1(
              -std::log(2.)*(target-reference_last_scored_[context][h])/3.);
          for (std::size_t m=0; m<model_count; ++m) {
            if (!forecast.valid[m]) continue;
            const auto &p = forecast.xy[h][m];
            const double error = (p[0]-x)*(p[0]-x)+(p[1]-y)*(p[1]-y);
            losses_[h][m] += (model_samples_[h][m] ? alpha : 1.)*(error-losses_[h][m]);
            ++model_samples_[h][m];
            if(m>=motion_model_count) {
              auto &loss=reference_losses_[context][h][m];
              auto &count=reference_samples_[context][h][m];
              loss+=(count ? road_alpha : 1.)*(error-loss);
              ++count;
            }
          }
          ++samples_[h];
          last_scored_[h] = target;
          reference_last_scored_[context][h]=target;
        }
      }
      while (!pending_.empty() && pending_.front().next == horizons.size()) pending_.pop_front();
    }
    previous_ = now;
    if (now.stamp-last_issued_ < .1-1e-8) return;
    const auto state = fitRecentMotion(history);
    if (!state) return;
    Forecast forecast{};
    forecast.stamp = now.stamp;
    for (std::size_t m=0; m<motion_model_count; ++m) {
      forecast.valid[m] = true;
      for (std::size_t h=0; h<horizons.size(); ++h) {
        const auto p = advanceRecentMotion(*state, 0., horizons[h], persistence(m));
        forecast.xy[h][m] = {p.x, p.y};
      }
    }
    const auto reference = world ? fitReferenceMotion(history,*world) : std::nullopt;
    reference_available_ = reference.has_value();
    if (world && reference) {
      // Acceleration and braking have different persistence histories.
      // Select only from forecasts issued with the currently observed sign.
      reference_context_ = reference->acceleration < 0. ? 0U : 1U;
      forecast.acceleration_context=reference_context_;
      for (std::size_t m=motion_model_count; m<model_count; ++m) {
        forecast.valid[m] = true;
        const auto road_model=m-motion_model_count;
        ReferenceMotionIntegrator integrator(*reference,*world,
            lateral_durations[road_model%lateral_durations.size()],
            durations[road_model/lateral_durations.size()]);
        for (std::size_t h=0; h<horizons.size(); ++h) {
          const auto p=integrator.at(horizons[h]);
          if (!p) {forecast.valid[m]=false;break;}
          const auto xy=world->pose(p->point.global_s,p->point.d);
          if (!xy) {forecast.valid[m]=false;break;}
          forecast.xy[h][m]={(*xy)[0],(*xy)[1]};
        }
      }
    }
    pending_.push_back(std::move(forecast));
    last_issued_ = now.stamp;
  }

  Summary summary() const {
    Summary result;
    result.samples = samples_;
    if (reference_available_) result.model=initial_reference_model;
    Losses aggregate{};
    std::array<std::size_t,model_count> available{};
    for (std::size_t h=0; h<horizons.size(); ++h) {
      for (std::size_t m=0; m<model_count; ++m) {
        const bool road=m>=motion_model_count;
        const auto count=road ? reference_samples_[reference_context_][h][m] : model_samples_[h][m];
        if (count == 0) continue;
        ++available[m];
        // Compare velocity-scale errors so the oldest 2 s outcomes do not
        // dominate the more recent 0.5 s observations solely by horizon.
        aggregate[m] += road ? reference_losses_[reference_context_][h][m] /
            (horizons[h]*horizons[h]) : losses_[h][m];
      }
    }
    for (std::size_t m=0; m<model_count; ++m)
      aggregate[m] = available[m] ? aggregate[m]/available[m] : std::numeric_limits<double>::infinity();
    // Equal scores keep the initial model, including straight-line motion
    // where several turn durations cannot yet be distinguished.
    // With a road fit, learn lateral persistence along that road. A Cartesian
    // model's older low loss must not discard newly observed braking or the bend.
    const std::size_t begin = reference_available_ ? motion_model_count : 0;
    const std::size_t end = reference_available_ ? model_count : motion_model_count;
    for (std::size_t m=begin; m<end; ++m)
      if (aggregate[m] < aggregate[result.model]) result.model = m;
    result.follows_reference = result.model >= motion_model_count;
    if (result.follows_reference) {
      const auto road_model=result.model-motion_model_count;
      result.lateral_persistence_sec=lateral_durations[road_model%lateral_durations.size()];
      result.persistence.acceleration_sec=durations[road_model/lateral_durations.size()];
    }
    else result.persistence = persistence(result.model);
    if (available[result.model]) {
      if(result.follows_reference) {
        for(std::size_t h=0;h<horizons.size();++h)
          if(reference_samples_[reference_context_][h][result.model])
            result.mean_squared_error+=reference_losses_[reference_context_][h][result.model];
        result.mean_squared_error/=available[result.model];
      } else result.mean_squared_error=aggregate[result.model];
    }
    return result;
  }

  std::size_t pendingCount() const { return pending_.size(); }

private:
  struct Forecast {
    double stamp{0.};
    std::array<std::array<std::array<double, 2>, model_count>, horizons.size()> xy{};
    std::size_t next{0};
    std::array<bool,model_count> valid{};
    std::size_t acceleration_context{0};
  };
  std::optional<PositionObservation> previous_{};
  std::deque<Forecast> pending_{};
  std::array<Losses, horizons.size()> losses_{};
  std::array<std::array<std::size_t,model_count>,horizons.size()> model_samples_{};
  std::array<std::size_t, horizons.size()> samples_{};
  std::array<double, horizons.size()> last_scored_{};
  double last_issued_{-std::numeric_limits<double>::infinity()};
  bool reference_available_{false};
  std::size_t reference_context_{0};
  std::array<std::array<Losses,horizons.size()>,2> reference_losses_{};
  std::array<std::array<std::array<std::size_t,model_count>,horizons.size()>,2> reference_samples_{};
  std::array<std::array<double,horizons.size()>,2> reference_last_scored_{};
};

} // namespace reference_space_mppi_planner::opponent_prediction
