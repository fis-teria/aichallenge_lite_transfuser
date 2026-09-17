#include "reference_space_mppi_planner/execution_trajectory.hpp"
#include <cstdlib>
#include <iostream>
using namespace reference_space_mppi_planner::mppi;
void near(double a,double b) {
  if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>1e-10*(1+std::abs(b))) {
    std::cerr<<a<<" != "<<b<<'\n';std::exit(1);
  }
}
int main() {
  Config config;config.control_lateral_std_m=.2;
  config.control_lateral_reference_spacing_m=60./7.;
  config.cost_control_change_weight=.15;config.cost_control_smoothness_weight=.1;
  for(double lateral_cost_scale:{0.,1.})
  for(int history_mode=0;history_mode<2;++history_mode)
  for(int varying=0;varying<2;++varying)
  for(int mode=0;mode<4;++mode)
  for(double cutoff:{-1.,0.,13.17,30.,60.,61.}) {
    config.history_lateral_cost_scale=lateral_cost_scale;
    PlanRequest request;request.base_reference_count=3;
    request.bounds.minimum.speed_scale=0.;request.bounds.maximum.speed_scale=1.;
    for(std::size_t i=0;i<3;++i) {
      auto &b=request.base_reference[i];b.s_m=i*30.;b.speed_mps=10.+varying*i*3.;
      b.active_d_valid=b.active_speed_valid=true;b.active_d_m=0.;b.active_speed_mps=10.;
    }
    auto history=std::make_shared<TemporaryReference>();history->count=121;
    TemporaryReference path;path.count=121;
    for(std::size_t i=0;i<path.count;++i) {
      auto &p=path.points[i];p.x_m=p.s_m=.5*i;
      p.y_m=p.d_m=(mode&1)?.2:0.;p.speed_mps=10.;
      if((mode&2)&&p.x_m>40.)p.speed_mps+=.5*std::min(p.x_m-40.,60.-p.x_m);
      auto &h=history->points[i];h.x_m=h.s_m=p.x_m;h.speed_mps=10.;
    }
    if(history_mode)request.execution_history_field=history;
    Evaluation diagnostic;diagnostic.field_terminal_arc_m=cutoff;
    const auto plain=executionFieldCosts(path,request,config);
    const auto costs=executionFieldCosts(path,request,config,&diagnostic);
    near(costs[0],plain[0]);near(costs[1],plain[1]);
    const auto &s=diagnostic.field_cost_split;
    near(costs[0],s[4]+s[5]);near(costs[1],s[0]+s[1]+s[2]+s[3]);
    near(diagnostic.field_history_coverage_m,60.);near(diagnostic.field_gradient_coverage_m,60.);
    if(!(mode&1)||lateral_cost_scale==0.)near(s[0]+s[1],0.);
    if(lateral_cost_scale==0.) {
      auto unchanged_lateral=path;
      for(std::size_t i=0;i<path.count;++i)
        unchanged_lateral.points[i].y_m=unchanged_lateral.points[i].d_m=0.;
      const auto speed_only=executionFieldCosts(unchanged_lateral,request,config);
      near(costs[0],speed_only[0]);near(costs[1],speed_only[1]);
    }
    if(!(mode&2)){near(s[2]+s[3],0.);near(s[4]+s[5],0.);}
    if(cutoff<0.){near(s[0],0.);near(s[2],0.);near(s[4],0.);}
    if(cutoff>=60.){near(s[1],0.);near(s[3],0.);near(s[5],0.);}
    if(cutoff==30.&&!varying) {
      // The quadrature node at the cutoff belongs to the inside region,
      // including its weight from the interval immediately after the cutoff.
      const double boundary_weight=.15*.5/6./60.;
      near(s[0],(mode&1)?lateral_cost_scale*(.075+boundary_weight):0.);
      near(s[1],(mode&1)?lateral_cost_scale*(.075-boundary_weight):0.);
      near(s[2],0.);near(s[4],0.);
      near(s[3],(mode&2)?1./240.:0.);
    }
#ifdef MPPI_COMPARE_LEGACY
    const auto legacy=legacyExecutionFieldCosts(path,request,config);
    if(lateral_cost_scale==1. && costs!=legacy){std::cerr<<"Legacy cost changed\n";return 1;}
#endif
  }
  std::cout<<"192 field attribution cases passed\n";
}
