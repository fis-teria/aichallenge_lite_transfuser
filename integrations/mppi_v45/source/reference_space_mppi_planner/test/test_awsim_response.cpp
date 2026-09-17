#include "reference_space_mppi_planner/awsim_vehicle_response.hpp"
#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include <cassert>
#include <cmath>
using namespace reference_space_mppi_planner::mppi;
int main() {
  const double tire=awsimSteeringResponse(0.,.3,.05,.02);
  assert(std::abs(tire-1.0471975511965976*.05)<1e-12);
  double fine=0.;
  for(int i=0;i<10;++i)fine=awsimSteeringResponse(fine,.3,.005,.02);
  assert(std::abs(fine-tire)<1e-12);
  AwsimLongitudinalResponse drive;
  const double a=drive.acceleration(2.,5.,0.);
  assert(a>.8 && a<.86);
  assert(drive.acceleration(-2.,5.,.05)==a);
  assert(std::abs(drive.acceleration(-2.,5.,.1)+2.52)<1e-12);
  assert(std::abs(drive.acceleration(0.,5.,.2)+.52)<1e-12);
  AwsimLongitudinalResponse high_speed;
  assert(std::abs(high_speed.acceleration(2.,10.,0.)+.3)<1e-12);
  Config c;c.enabled=true;c.shadow_only=false;c.horizon_steps=10;
  c.awsim_vehicle_response_enabled=true;c.steering_control_delay_sec=.07;c.steering_time_constant_sec=.02;
  PlanRequest r;r.valid=true;r.side=1;r.phase=Phase::OVERTAKE;r.sample_staged_speed=true;
  r.ego.speed_mps=1.;r.ego.yaw_rate_radps=.5;r.base_reference_count=101;
  r.nominal={0,4,4,4,1,0,1};r.bounds.minimum={0,4,4,4,.1,0,1};r.bounds.maximum={0,4,4,4,1,0,1};
  TemporaryReference path;path.count=101;
  for(size_t i=0;i<path.count;++i) {
    auto &b=r.base_reference[i];b.s_m=b.x_m=.4*i;b.speed_mps=8.;
    b.minimum_d_m=-10.;b.maximum_d_m=10.;b.active_d_valid=true;
    auto &p=path.points[i];p.x_m=p.s_m=p.source_s_m=.4*i;p.speed_mps=p.uncapped_speed_mps=1.;
  }
  const auto result=ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(path,r);
  assert(result.valid);
  assert(std::abs(result.predicted_rollout[0].speed_mps-.98)<1e-9);
  assert(std::abs(result.predicted_rollout[0].yaw_rad-.5*.1*(1-std::exp(-.05/.1)))<1e-9);
  for(size_t i=0;i<path.count;++i)
    path.points[i].speed_mps=path.points[i].uncapped_speed_mps=3.;
  r.stamp_sec=10.;
  const auto propulsion=ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(path,r);
  assert(propulsion.valid);
  r.rotation_prediction.valid=true;
  r.rotation_prediction.stamp_sec=10.;
  r.rotation_prediction.state.active=true;
  r.rotation_prediction.state.started_sec=10.;
  const auto held=ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(path,r);
  assert(held.valid);
  assert(held.predicted_rollout[0].speed_mps<propulsion.predicted_rollout[0].speed_mps);
  assert(r.rotation_prediction.state.active);
  const auto repeated=ReferenceSpaceMppiPlanner(c).evaluateExecutionReference(path,r);
  assert(repeated.predicted_rollout[0].speed_mps==held.predicted_rollout[0].speed_mps);
}
