#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include <iostream>
using namespace reference_space_mppi_planner::mppi;
int main() {
  Config c;c.enabled=true;c.shadow_only=false;c.collision_only_rejection=true;
  c.horizon_steps=60;c.dt_sec=.05;c.cost_curvature_weight=.2;
  ReferenceSpaceMppiPlanner planner(c);
  PlanRequest r;r.valid=true;r.sample_staged_speed=true;r.side=1;r.phase=Phase::OVERTAKE;
  r.ego.speed_mps=1.;r.base_reference_count=121;
  r.curvature_evaluation_distance_m=5.;
  r.nominal={0,8,8,8,1,0,1};r.bounds.minimum={0,8,8,8,.1,0,1};r.bounds.maximum={0,8,8,8,1,0,1};
  TemporaryReference straight;straight.count=121;
  for(size_t i=0;i<121;++i) {
    auto &b=r.base_reference[i];b.x_m=b.s_m=.5*i;b.speed_mps=1.;b.minimum_d_m=-20.;b.maximum_d_m=20.;
    auto &p=straight.points[i];p.x_m=p.s_m=.5*i;p.speed_mps=p.uncapped_speed_mps=1.;
  }
  auto tail=straight;
  for(size_t i=0;i<tail.count;++i) {
    const double u=std::clamp((tail.points[i].x_m-40.)/8.,0.,1.);
    tail.points[i].y_m=2*u*u*u*(10+u*(-15+6*u));
  }
  updateExecutionGeometry(tail);
  const auto a=planner.evaluateExecutionReference(straight,r);
  const auto b=planner.evaluateExecutionReference(tail,r);
  std::cout<<"straight="<<a.cost_terms[3]<<" distant_tail="<<b.cost_terms[3]<<std::endl;
  if(!a.valid || !b.valid || b.field_terminal_arc_m>4. || std::abs(a.cost_terms[3]-b.cost_terms[3])>1e-12)return 1;
  auto curved=straight;
  for(size_t i=0;i<curved.count;++i) {
    const double u=std::clamp(curved.points[i].x_m/8.,0.,1.);
    curved.points[i].y_m=2*u*u*u*(10+u*(-15+6*u));
  }
  updateExecutionGeometry(curved);
  const auto near=planner.evaluateExecutionReference(curved,r);
  if(!near.valid || near.cost_terms[3]<=a.cost_terms[3])return 2;
  r.path_constraint_validator=[](const TemporaryReference &p) {
    return p.points[p.count-1].y_m>1.?RejectReason::WALL:RejectReason::NONE;
  };
  if(planner.evaluateExecutionReference(tail,r).valid)return 3;
  std::cout<<"Reached curvature is penalized; unreached wall remains rejected\n";
}
