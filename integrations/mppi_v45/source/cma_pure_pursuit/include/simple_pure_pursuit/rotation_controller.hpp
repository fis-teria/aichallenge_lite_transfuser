#pragma once

#include "simple_pure_pursuit/rotation_recovery.hpp"

namespace simple_pure_pursuit {

template<class PointAt>
double rotationSignedCurvature(std::size_t count, std::size_t nearest, PointAt point) {
  if (count < 3) return 0.;
  const std::size_t first = nearest > 1 ? nearest - 1 : 0;
  const std::size_t last = std::min(count - 1, nearest + 4);
  const auto a = point(first), b = point((first + last) / 2), c = point(last);
  const double denominator = std::hypot(b.first-a.first,b.second-a.second) *
      std::hypot(c.first-b.first,c.second-b.second) *
      std::hypot(a.first-c.first,a.second-c.second);
  return denominator <= 1.e-6 ? 0. :
      2.*((b.first-a.first)*(c.second-a.second) -
          (b.second-a.second)*(c.first-a.first))/denominator;
}

struct RotationControllerParameters {
  bool rotation_gate_enabled{false};
  double rotation_gate_min_curvature{0.06};
  double rotation_gate_min_steering_angle{0.18};
  double rotation_gate_yaw_rate_error_threshold{0.30};
  double rotation_gate_yaw_rate_error_release_ratio{0.60};
  double rotation_gate_max_lateral_error{1.00};
  double rotation_gate_min_duration_sec{0.05};
  double rotation_gate_max_duration_sec{0.20};
  double rotation_gate_cooldown_sec{0.50};
  double rotation_gate_countersteer_gain{0.35};
  double rotation_gate_max_countersteer_rad{0.12};
  bool rotation_prediction_enabled{false};
  double rotation_prediction_lead_time_sec{0.15};
  double rotation_prediction_yaw_acceleration_filter_time_constant_sec{0.08};
  double rotation_prediction_max_yaw_acceleration_radps2{6.0};
  double rotation_prediction_entry_threshold_radps{0.18};
  double rotation_prediction_min_yaw_acceleration_radps2{0.30};
  double rotation_prediction_slip_angle_threshold_rad{0.04};
  double rotation_prediction_slip_yaw_rate_gain{2.0};
  double pp_control_delay_sec{0.20};
  double steering_time_constant_sec{0.30};
};

struct RotationControllerState {
  bool active{false};
  double started_sec{0.};
  double cooldown_until_sec{0.};
  bool yaw_acceleration_initialized{false};
  double previous_yaw_rate_radps{0.};
  double filtered_yaw_acceleration_radps2{0.};
};

struct RotationControllerInput {
  double stamp_sec, dt_sec, signed_curvature_1pm, reference_yaw_rate_radps;
  double measured_yaw_rate_radps, commanded_yaw_rate_radps;
  double slip_angle_rad, nominal_tire_angle_rad, measured_tire_angle_rad;
  double absolute_lateral_error_m;
  bool slip_angle_valid, steering_fresh;
};

struct RotationControllerResult {
  PredictiveRotationRecoveryResult recovery;
  double signed_yaw_rate_excess_radps{0.};
  bool entry_condition{false};
};

// Both the live controller and each independent rollout advance this state.
// Parameters are supplied by the live controller, including dynamic updates.
template<class Parameters>
RotationControllerResult stepRotationController(
    const Parameters &p, const RotationControllerInput &in,
    RotationControllerState &state) {
  if (!state.yaw_acceleration_initialized) {
    state.previous_yaw_rate_radps=in.measured_yaw_rate_radps;
    state.filtered_yaw_acceleration_radps2=0.;
    state.yaw_acceleration_initialized=true;
  } else {
    const double raw=std::clamp(
        (in.measured_yaw_rate_radps-state.previous_yaw_rate_radps)/in.dt_sec,
        -p.rotation_prediction_max_yaw_acceleration_radps2,
        p.rotation_prediction_max_yaw_acceleration_radps2);
    const double alpha=1.-std::exp(-in.dt_sec/
        p.rotation_prediction_yaw_acceleration_filter_time_constant_sec);
    state.filtered_yaw_acceleration_radps2+=alpha*(raw-state.filtered_yaw_acceleration_radps2);
    state.previous_yaw_rate_radps=in.measured_yaw_rate_radps;
  }
  const PredictiveRotationRecoveryConfig config{
      p.rotation_prediction_enabled?p.pp_control_delay_sec+p.rotation_prediction_lead_time_sec:0.,
      p.rotation_prediction_enabled?1.-std::exp(-p.rotation_prediction_lead_time_sec/p.steering_time_constant_sec):0.,
      p.rotation_prediction_max_yaw_acceleration_radps2,
      p.rotation_gate_yaw_rate_error_threshold,
      p.rotation_prediction_enabled?p.rotation_prediction_entry_threshold_radps:p.rotation_gate_yaw_rate_error_threshold,
      p.rotation_prediction_min_yaw_acceleration_radps2,
      p.rotation_prediction_slip_angle_threshold_rad,
      p.rotation_prediction_enabled?p.rotation_prediction_slip_yaw_rate_gain:0.,
      p.rotation_gate_countersteer_gain,p.rotation_gate_max_countersteer_rad};
  const auto observation=computePredictiveRotationRecovery(false,
      in.signed_curvature_1pm,in.reference_yaw_rate_radps,in.measured_yaw_rate_radps,
      state.filtered_yaw_acceleration_radps2,in.commanded_yaw_rate_radps,
      in.slip_angle_rad,in.slip_angle_valid,in.nominal_tire_angle_rad,config);
  const double tire_angle=std::max(in.steering_fresh?std::abs(in.measured_tire_angle_rad):0.,
      std::abs(in.nominal_tire_angle_rad));
  const bool safe=in.steering_fresh&&in.absolute_lateral_error_m<=p.rotation_gate_max_lateral_error;
  const bool entry=p.rotation_gate_enabled&&safe&&
      std::abs(in.signed_curvature_1pm)>=p.rotation_gate_min_curvature&&
      tire_angle>=p.rotation_gate_min_steering_angle&&
      (observation.signed_yaw_rate_excess_radps>=p.rotation_gate_yaw_rate_error_threshold||
       (p.rotation_prediction_enabled&&observation.predictive_trigger));
  const double threshold=(p.rotation_prediction_enabled?
      p.rotation_prediction_entry_threshold_radps:p.rotation_gate_yaw_rate_error_threshold)*
      p.rotation_gate_yaw_rate_error_release_ratio;
  const bool hold=safe&&std::abs(in.signed_curvature_1pm)>=.8*p.rotation_gate_min_curvature&&
      tire_angle>=.8*p.rotation_gate_min_steering_angle&&observation.risk_yaw_rate_excess_radps>=threshold;
  if(state.active) {
    const double elapsed=in.stamp_sec-state.started_sec;
    const bool minimum_hold=elapsed<p.rotation_gate_min_duration_sec&&safe;
    if(elapsed>=p.rotation_gate_max_duration_sec||(!minimum_hold&&!hold)) {
      state.active=false;state.cooldown_until_sec=in.stamp_sec+p.rotation_gate_cooldown_sec;
    }
  }
  if(!state.active&&entry&&in.stamp_sec>=state.cooldown_until_sec) {
    state.active=true;state.started_sec=in.stamp_sec;
  }
  return {computePredictiveRotationRecovery(state.active,in.signed_curvature_1pm,
      in.reference_yaw_rate_radps,in.measured_yaw_rate_radps,state.filtered_yaw_acceleration_radps2,
      in.commanded_yaw_rate_radps,in.slip_angle_rad,in.slip_angle_valid,in.nominal_tire_angle_rad,config),
      observation.signed_yaw_rate_excess_radps,entry};
}

struct RotationPredictionSnapshot {
  bool valid{false};
  double stamp_sec{0.};
  double slip_angle_rad{0.};
  bool slip_angle_valid{false};
  RotationControllerParameters parameters;
  RotationControllerState state;
};

} // namespace simple_pure_pursuit
