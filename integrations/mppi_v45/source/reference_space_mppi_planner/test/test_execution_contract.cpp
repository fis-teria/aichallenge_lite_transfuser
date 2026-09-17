// Standalone regression tests: use the production MPPI core, no ROS/GTest mocks.
#include "reference_space_mppi_planner/reference_space_mppi.hpp"
#include "reference_space_mppi_planner/arrival_time.hpp"
#include "reference_space_mppi_planner/batch_optimizer.hpp"
#include <cmath>
#include <functional>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace reference_space_mppi_planner::mppi {
struct PlannerEvaluationTestAccess {
  static Evaluation evaluate(const ReferenceSpaceMppiPlanner &p,
      const PlanRequest &r, const ReferenceControlSequence *c, TemporaryReference *out) {
    return p.evaluate(r.nominal,r,c,out);
  }
  static bool generate(const ReferenceSpaceMppiPlanner &p,
      const PlanRequest &r, const ReferenceControlSequence *c, TemporaryReference *out) {
    Evaluation e;
    return p.generateReference(r.nominal,r,c,out,&e);
  }
  static void replaceWarmControls(ReferenceSpaceMppiPlanner &p,
      const ReferenceControlSequence &c) { p.warm_control_sequence_=c; }
  static ReferenceControlSequence warm(const ReferenceSpaceMppiPlanner &p,
      const PlanRequest &r) { return p.controlSequenceMean(r); }
};
}
using namespace reference_space_mppi_planner::mppi;
namespace {
void require(bool value,const std::string &message) {
  if (!value) throw std::runtime_error(message);
}
void near(double actual,double expected,double tolerance,const std::string &message) {
  require(std::isfinite(actual) && std::abs(actual-expected)<=tolerance,
    message+" actual="+std::to_string(actual)+" expected="+std::to_string(expected));
}
Config config() {
  Config c;
  c.enabled=true; c.shadow_only=false; c.collision_only_rejection=true;
  c.horizon_steps=60; c.sample_count=10; c.minimum_valid_count=1; c.minimum_valid_ratio=0;
  c.maximum_acceleration_mps2=2; c.maximum_deceleration_mps2=2;
  c.speed_proportional_gain=3; c.lookahead_gain=.2;
  c.lookahead_min_distance_m=2; c.curvature_lookahead_min_distance_m=2;
  c.continuous_preview_interpolation_enabled=true;
  c.actual_lookahead_distance_blend=1;
  c.control_maximum_speed_scale_adjustment=1;
  c.sigma.fill(0); c.control_lateral_std_m=0; c.control_speed_scale_std=0;
  return c;
}
PlanRequest request(double speed=.664) {
  PlanRequest r;
  r.valid=true; r.side=1; r.phase=Phase::OVERTAKE;
  r.generation=1; r.semantic_key=42; r.stamp_sec=1;
  r.ego.speed_mps=speed; r.anchor_s_m=0;
  r.nominal={0,4,4,4,1,0,1};
  r.bounds.minimum={0,4,4,4,.1,0,1};
  r.bounds.maximum={0,4,4,4,1,0,1};
  r.base_reference_count=101;
  for(std::size_t i=0;i<r.base_reference_count;++i) {
    auto &b=r.base_reference[i]; b.s_m=.2*i; b.x_m=b.s_m;
    b.speed_mps=10; b.minimum_d_m=-10; b.maximum_d_m=10;
    b.active_d_valid=true;
  }
  return r;
}
Evaluation run(const Config &c,const PlanRequest &r) {
  ReferenceSpaceMppiPlanner planner(c); TemporaryReference out;
  return PlannerEvaluationTestAccess::evaluate(planner,r,nullptr,&out);
}
struct Case {const char *name; std::function<void()> body;};
std::vector<Case> cases() {
  return {
    {"terminal_tracking_does_not_target_unreached_tail",[]{
      auto c=config(); c.cost_terminal_lateral_weight=2;
      auto r=request(1); r.sample_staged_speed=true;
      ReferenceSpaceMppiPlanner planner(c); TemporaryReference straight;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&straight),"generate failed");
      for(std::size_t i=0;i<straight.count;++i) straight.points[i].speed_mps=1;
      auto tail=straight;
      for(std::size_t i=0;i<tail.count;++i) {
        auto &p=tail.points[i];
        if(p.s_m>15) p.y_m=p.d_m=(p.s_m-15)/5;
      }
      const auto a=planner.validateReference(straight,r);
      const auto b=planner.validateReference(tail,r);
      require(a.valid && b.valid,"safe references rejected");
      near(a.progress_m,b.progress_m,1e-12,"unreached tail changed rollout");
      near(b.cost_terms[7],a.cost_terms[7],1e-12,"unreached tail changed terminal tracking");
      near(b.cost_terms[7],0,1e-12,"perfect tracking charged terminal penalty");
    }},
    {"terminal_tracking_does_not_penalize_error_within_tolerance",[]{
      auto c=config(); c.cost_terminal_lateral_weight=2;
      auto r=request(1); r.sample_staged_speed=true;
      ReferenceSpaceMppiPlanner planner(c); TemporaryReference path;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&path),"generate failed");
      for(std::size_t i=0;i<path.count;++i) {
        path.points[i].y_m=path.points[i].d_m=.4;
        path.points[i].speed_mps=1;
      }
      const auto e=planner.validateReference(path,r);
      require(e.valid,"local tracking example invalid");
      const auto &last=e.predicted_rollout[e.predicted_rollout_count-1];
      const double error=std::abs(last.y_m-.4);
      require(error>1e-6 && error<c.maximum_cross_track_error_m,
          "fixture needs nonzero local tracking error within tolerance");
      near(e.cost_terms[7],0,1e-10,"tracking error within tolerance was penalized");
    }},
    {"rollout_safe_candidate_not_rejected_by_fictitious_arrival",[]{
      auto c=config(); auto r=request();
      auto empty=run(c,r); require(empty.valid,"empty scene invalid");
      r.dynamic_obstacle_count=1;
      r.dynamic_obstacles[0].s_m=empty.progress_m+2.25;
      auto e=run(c,r);
      require(e.valid,"safe actual rollout rejected; progress="+std::to_string(empty.progress_m)+
        " reject_s="+std::to_string(e.reject_s_m)+" reason="+ReferenceSpaceMppiPlanner::toString(e.reject_reason));
    }},
    {"stationary_ego_detects_crossing_between_rollout_steps",[]{
      auto c=config(); c.dt_sec=.5; c.horizon_steps=6;
      auto r=request(0); for(auto &b:r.base_reference) b.speed_mps=0;
      r.dynamic_obstacle_count=1;
      r.dynamic_obstacles[0].s_m=0; r.dynamic_obstacles[0].d_m=-2;
      r.dynamic_obstacles[0].lateral_speed_mps=8;
      auto e=run(c,r);
      require(!e.valid && e.reject_reason==RejectReason::COLLISION,
        "moving opponent crosses stationary ego at t=.25 but sample endpoints are clear");
    }},
    {"nonfinite_arrival_input_is_not_silently_clamped",[]{
      auto e=segmentArrival(1,2,std::numeric_limits<double>::quiet_NaN(),2,2);
      require(!std::isfinite(e.time_sec),"NaN target converted into a valid braking arrival");
    }},
    {"longitudinal_knots_do_not_inherit_lateral_frozen_prefix",[]{
      auto c=config(); auto r=request(); r.anchor_s_m=2;
      r.sample_control_sequence=true; r.sample_staged_speed=true;
      ReferenceControlSequence control; control.count=c.control_knot_count;
      control.speed_scale_adjustment[0]=-.9;
      TemporaryReference out; ReferenceSpaceMppiPlanner planner(c);
      require(PlannerEvaluationTestAccess::generate(planner,r,&control,&out),"generate failed");
      require(out.points[2].speed_mps>out.points[0].speed_mps+.5,
        "acceleration field frozen until lateral anchor instead of ego station");
    }},
    {"warm_transport_uses_measured_progress_not_speed_times_dt",[]{
      auto c=config(); auto r=request(4); r.sample_control_sequence=true;
      ReferenceSpaceMppiPlanner planner(c); Scratch scratch;
      require(planner.plan(r,&scratch).valid,"initial planning failed");
      ReferenceControlSequence control; control.count=c.control_knot_count;
      for(std::size_t i=0;i<control.count;++i) control.speed_scale_adjustment[i]=-.7+.1*i;
      PlannerEvaluationTestAccess::replaceWarmControls(planner,control);
      r.stamp_sec+=.1; ++r.generation; // Same measured pose: no spatial progress.
      auto warm=PlannerEvaluationTestAccess::warm(planner,r);
      near(warm.speed_scale_adjustment[2],control.speed_scale_adjustment[2],1e-12,
        "unconsumed speed field moved despite unchanged measured pose");
    }},
    {"weighted_update_uses_projected_proposal_delta",[]{
      auto c=config(); c.sample_count=16; c.sigma[4]=10;
      auto r=request(1); r.nominal.speed_scale=.5;
      ReferenceSpaceMppiPlanner planner(c); Scratch scratch;
      require(planner.plan(r,&scratch).valid,"planning failed");
      for(std::size_t i=0;i<c.sample_count;++i) if(scratch.valid[i]) {
        require(scratch.noise[i][4]>=-.4-1e-12 && scratch.noise[i][4]<=.5+1e-12,
          "weighted delta refers to unclipped noise, not the evaluated speed proposal");
      }
    }},
    {"duplicate_deterministic_proposals_do_not_multiply_mass",[]{
      auto c=config(); auto r=request(1);
      r.sample_control_sequence=true; r.sample_staged_speed=true;
      r.minimum_speed_mps=10;
      r.bounds.minimum.speed_scale=1;
      ReferenceSpaceMppiPlanner planner(c); Scratch scratch;
      auto out=planner.plan(r,&scratch);
      require(out.valid,"planning failed");
      require(out.valid_sample_count==1,"identical deterministic paths counted "+
        std::to_string(out.valid_sample_count)+" times");
    }},
    {"nonfinite_dynamic_obstacle_is_invalid_input",[]{
      auto c=config(); auto r=request(); r.dynamic_obstacle_count=1;
      r.dynamic_obstacles[0].s_m=100;
      r.dynamic_obstacles[0].d_m=std::numeric_limits<double>::quiet_NaN();
      ReferenceSpaceMppiPlanner planner(c); Scratch scratch;
      auto e=planner.plan(r,&scratch);
      require(!e.valid && (e.reject_reason==RejectReason::INVALID_INPUT ||
          e.reject_reason==RejectReason::NON_FINITE),"NaN obstacle silently ignored");
    }},
    {"real_forward_collision_remains_rejected",[]{
      auto c=config(); auto r=request(); r.dynamic_obstacle_count=1;
      r.dynamic_obstacles[0].s_m=3;
      auto e=run(c,r);
      require(!e.valid && e.reject_reason==RejectReason::COLLISION,"real collision accepted");
    }},
    {"environment_wall_rejection_remains_hard",[]{
      auto c=config(); auto r=request();
      r.rollout_constraint_validator=[](const RolloutState &,const RolloutState &b){
        return b.x_m>1?RejectReason::WALL:RejectReason::NONE;};
      auto e=run(c,r);
      require(!e.valid && e.reject_reason==RejectReason::WALL,"wall callback bypassed");
    }}
#ifdef MPPI_EXECUTION_CONTRACT_V2
    ,{"semantic_change_does_not_charge_previous_target_history",[]{
      auto c=config();auto r=request(1);r.sample_control_sequence=true;r.sample_staged_speed=true;
      ReferenceSpaceMppiPlanner planner(c);Scratch scratch;require(planner.plan(r,&scratch).valid,"initial failed");
      ReferenceControlSequence control;control.count=c.control_knot_count;
      ++r.semantic_key;TemporaryReference ref;
      auto e=PlannerEvaluationTestAccess::evaluate(planner,r,&control,&ref);
      require(e.valid,"new target invalid");near(e.cost_terms[9],0,0,"old target history charged");
    }},
    {"cross_mode_history_uses_common_published_physical_field",[]{
      auto c=config();auto r=request(1);r.sample_control_sequence=r.sample_staged_speed=true;
      ReferenceSpaceMppiPlanner with_cache(c),without_cache(c);Scratch scratch;
      require(with_cache.plan(r,&scratch).valid,"initial plan failed");
      for(std::size_t i=0;i<r.base_reference_count;++i) {
        auto &b=r.base_reference[i];b.active_speed_valid=true;b.active_speed_mps=2;b.active_d_valid=true;b.active_d_m=0;
      }
      ReferenceControlSequence control;control.count=c.control_knot_count;TemporaryReference a,b;
      const auto ea=PlannerEvaluationTestAccess::evaluate(with_cache,r,&control,&a);
      const auto eb=PlannerEvaluationTestAccess::evaluate(without_cache,r,&control,&b);
      require(ea.valid && eb.valid,"candidate invalid");
      require(ea.cost_terms[9]>0,"history discrepancy not measured");
      near(ea.cost_terms[9],eb.cost_terms[9],1e-12,"mode-local history biased comparison");
    }},
    {"cartesian_entry_removes_nonuniform_frenet_normal_snap",[]{
      auto c=config();auto r=request(.8);r.cartesian_measured_prefix=true;
      r.ego.y_m=1.5;r.ego.d_m=1.5;r.anchor_d_m=1.5;r.anchor_s_m=.5;
      for(std::size_t i=0;i<r.base_reference_count;++i) {
        auto &b=r.base_reference[i];b.active_d_m=1.5;
        b.s_m=b.x_m=i==0?0:.001+(i-1)*.2;
        b.yaw_rad=i==0?0:.28;
      }
      ReferenceSpaceMppiPlanner planner(c);TemporaryReference ref;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&ref),"entry construction failed");
      near(ref.points[0].x_m,r.ego.x_m,1e-12,"start x moved");
      near(ref.points[0].y_m,r.ego.y_m,1e-12,"start y moved");
      require(std::hypot(ref.points[1].x_m-ref.points[0].x_m,ref.points[1].y_m-ref.points[0].y_m)<.002,
              "0.001m Reference segment creates a 0.4m Cartesian jump");
      near(ref.points[0].yaw_rad,r.ego.yaw_rad,1e-12,"start heading changed");
    }},
    {"cartesian_conditioning_is_before_wall_validation",[]{
      auto c=config();auto r=request(1);r.cartesian_measured_prefix=true;
      r.ego.y_m=1.5;r.ego.d_m=1.5;r.anchor_d_m=1.5;
      for(auto &b:r.base_reference)b.active_d_m=1.5;
      bool checked=false;
      r.path_constraint_validator=[&](const TemporaryReference &ref) {
        checked=true;near(ref.points[0].y_m,1.5,1e-12,"wall checked an unconditioned path");
        return RejectReason::WALL;
      };
      auto e=run(c,r);require(checked && !e.valid && e.reject_reason==RejectReason::WALL,"conditioned wall bypassed");
    }},
    {"cartesian_entry_matches_actual_nonzero_heading",[]{
      auto c=config();auto r=request(1);r.cartesian_measured_prefix=true;r.ego.yaw_rad=.12;
      r.base_reference[1].s_m=r.base_reference[1].x_m=.0001;
      ReferenceSpaceMppiPlanner planner(c);TemporaryReference ref;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&ref),"generate failed");
      near(std::atan2(ref.points[1].y_m-ref.points[0].y_m,ref.points[1].x_m-ref.points[0].x_m),.12,1e-5,"entry tangent not measured");
    }},
    {"same_reference_same_clock_for_planner_and_hold",[]{
      auto c=config(); auto r=request(); ReferenceSpaceMppiPlanner planner(c);
      TemporaryReference reference;
      const auto evaluation=PlannerEvaluationTestAccess::evaluate(planner,r,nullptr,&reference);
      require(evaluation.valid,"generated candidate invalid");
      const auto hold=planner.validateReference(reference,r);
      require(hold.valid,"same held Reference rejected");
      near(hold.progress_m,evaluation.progress_m,1e-12,"planner/hold progress mismatch");
      near(hold.cost,evaluation.cost,1e-12,"planner/hold costs differ");
    }},
    {"nonuniform_reference_s_is_not_cartesian_path_arc",[]{
      auto c=config(); auto r=request(); ReferenceSpaceMppiPlanner planner(c);
      TemporaryReference reference;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&reference),"generate failed");
      const auto a=planner.validateReference(reference,r);
      for(std::size_t i=0;i<reference.count;++i) reference.points[i].s_m=2.0*i+.01*i*i;
      const auto b=planner.validateReference(reference,r);
      require(a.valid && b.valid,"relabelled path incorrectly invalid");
      near(a.progress_m,b.progress_m,1e-12,"speed clock depends on unrelated Reference station");
    }},
    {"hold_revalidation_does_not_mutate_supplied_geometry_or_speed",[]{
      auto c=config(); auto r=request(); ReferenceSpaceMppiPlanner planner(c); TemporaryReference ref;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&ref),"generate failed");
      const auto copy=ref; planner.validateReference(ref,r);
      for(std::size_t i=0;i<ref.count;++i) {
        near(ref.points[i].x_m,copy.points[i].x_m,0,"x mutated");
        near(ref.points[i].y_m,copy.points[i].y_m,0,"y mutated");
        near(ref.points[i].speed_mps,copy.points[i].speed_mps,0,"speed mutated");
      }
    }},
    {"negative_or_nonfinite_reference_speed_rejected",[]{
      auto c=config(); auto r=request(); ReferenceSpaceMppiPlanner planner(c); TemporaryReference ref;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&ref),"generate failed");
      ref.points[3].speed_mps=-1; require(!planner.validateReference(ref,r).valid,"reverse silently abs-clamped");
      ref.points[3].speed_mps=std::numeric_limits<double>::quiet_NaN();
      require(!planner.validateReference(ref,r).valid,"NaN speed accepted");
    }},
    {"speed_match_is_proposal_not_mandatory_floor",[]{
      auto c=config(); auto r=request(); r.preferred_matching_speed_mps=.8;
      r.minimum_speed_mps=0; r.nominal.speed_scale=0; r.bounds.minimum.speed_scale=0;
      ReferenceSpaceMppiPlanner planner(c); TemporaryReference ref;
      require(PlannerEvaluationTestAccess::generate(planner,r,nullptr,&ref),"zero target not representable");
      for(std::size_t i=0;i<ref.count;++i) near(ref.points[i].speed_mps,0,0,"forced matching speed");
    }},
    {"crossing_rejection_records_actual_clock_and_envelope",[]{
      auto c=config(); c.dt_sec=.5;c.horizon_steps=6;auto r=request(0);
      for(auto &b:r.base_reference)b.speed_mps=0;
      r.dynamic_obstacle_count=1;r.dynamic_obstacles[0].d_m=-2;r.dynamic_obstacles[0].lateral_speed_mps=8;
      const auto e=run(c,r);require(!e.valid,"crossing accepted");
      require(std::string(e.reject_stage)=="execution_sweep","wrong reject stage");
      require(e.reject_time_sec>0 && e.reject_time_sec<.5,"not an intermediate-time witness");
      require(e.reject_obstacle_index==0,"wrong opponent index");
      require(e.reject_d_m>=e.reject_obstacle_envelope[2] && e.reject_d_m<=e.reject_obstacle_envelope[3],
              "witness outside recorded envelope");
    }},
    {"zero_target_does_not_mean_instantaneous_stop",[]{
      auto c=config();auto r=request(5);r.nominal.speed_scale=0;r.bounds.minimum.speed_scale=0;
      r.dynamic_obstacle_count=1;r.dynamic_obstacles[0].s_m=3;
      const auto e=run(c,r);require(!e.valid && e.reject_reason==RejectReason::COLLISION,
                                   "braking infeasibility hidden by zero target");
    }},
    {"planning_without_publication_does_not_commit_warm",[]{
      auto c=config();auto r=request(1);r.sample_control_sequence=true;r.defer_warm_update=true;
      ReferenceSpaceMppiPlanner planner(c); Scratch scratch;
      auto out=planner.plan(r,&scratch);require(out.valid,"plan invalid");
      auto w=PlannerEvaluationTestAccess::warm(planner,r);
      for(std::size_t i=0;i<w.count;++i) near(w.speed_scale_adjustment[i],0,0,"speculative warm committed");
      out.selected_control_sequence.count=c.control_knot_count;
      out.selected_control_sequence.speed_scale_adjustment.fill(-.4);
      planner.accept(r,out);
      w=PlannerEvaluationTestAccess::warm(planner,r);near(w.speed_scale_adjustment[2],-.4,1e-12,"commit not retained");
    }},
    {"invalid_or_out_of_order_accept_does_not_replace_warm",[]{
      auto c=config();auto r=request(1);r.sample_control_sequence=true;r.defer_warm_update=true;
      ReferenceSpaceMppiPlanner planner(c);Scratch scratch;auto out=planner.plan(r,&scratch);
      require(out.valid,"plan invalid");out.selected_control_sequence.count=c.control_knot_count;
      out.selected_control_sequence.speed_scale_adjustment.fill(-.4);planner.accept(r,out);
      auto changed=out;changed.selected_control_sequence.speed_scale_adjustment.fill(-.8);
      changed.valid=false;planner.accept(r,changed);
      near(PlannerEvaluationTestAccess::warm(planner,r).speed_scale_adjustment[2],-.4,1e-12,"invalid committed");
      changed.valid=true;auto old=r;old.stamp_sec-=.1;planner.accept(old,changed);
      near(PlannerEvaluationTestAccess::warm(planner,r).speed_scale_adjustment[2],-.4,1e-12,"older committed");
    }},
    {"measured_progress_transports_speed_even_if_reported_velocity_is_zero",[]{
      auto c=config();auto r=request(0);r.sample_control_sequence=true;
      ReferenceSpaceMppiPlanner planner(c);Scratch scratch;require(planner.plan(r,&scratch).valid,"plan invalid");
      ReferenceControlSequence ctrl;ctrl.count=c.control_knot_count;
      for(std::size_t i=0;i<ctrl.count;++i)ctrl.speed_scale_adjustment[i]=-.7+.1*i;
      PlannerEvaluationTestAccess::replaceWarmControls(planner,ctrl);
      auto advanced=r; advanced.stamp_sec+=.1;advanced.ego.x_m=1;
      for(std::size_t i=0;i<advanced.base_reference_count;++i)advanced.base_reference[i].x_m+=1;
      auto w=PlannerEvaluationTestAccess::warm(planner,advanced);
      require(w.speed_scale_adjustment[0]>ctrl.speed_scale_adjustment[0],"actual progress ignored");
    }},
    {"semantic_change_discards_old_speed_sequence",[]{
      auto c=config();auto r=request(1);r.sample_control_sequence=true;
      ReferenceSpaceMppiPlanner planner(c);Scratch scratch;require(planner.plan(r,&scratch).valid,"plan invalid");
      ReferenceControlSequence ctrl;ctrl.count=c.control_knot_count;ctrl.speed_scale_adjustment.fill(-.8);
      PlannerEvaluationTestAccess::replaceWarmControls(planner,ctrl);++r.semantic_key;
      const auto w=PlannerEvaluationTestAccess::warm(planner,r);
      near(w.speed_scale_adjustment[2],0,0,"old target speed seed retained");
    }},
    {"world_fixed_published_speed_is_shared_across_modes",[]{
      auto c=config();auto r=request(1);r.sample_control_sequence=r.sample_staged_speed=true;
      for(std::size_t i=0;i<r.base_reference_count;++i) {
        r.base_reference[i].active_speed_valid=true;
        r.base_reference[i].active_speed_mps=1+.4*r.base_reference[i].s_m;
      }
      ReferenceSpaceMppiPlanner planner(c);const auto w=PlannerEvaluationTestAccess::warm(planner,r);
      for(std::size_t i=0;i<w.count;++i) {
        const double u=double(i)/(w.count-1), station=20*u*u;
        near(r.nominal.speed_scale+w.speed_scale_adjustment[i],(1+.4*station)/10,1e-12,"wrong absolute speed field");
      }
    }},
    {"serial_and_parallel_batch_use_same_core_contract",[]{
      auto c=config();auto r=request(1);r.defer_warm_update=true;
      ReferenceSpaceMppiBatchOptimizer serial(c,false),parallel(c,true);
      std::array<const PlanRequest *,kMaximumBatchCandidateCount> requests{&r,&r};
      std::array<std::size_t,kMaximumBatchCandidateCount> slots{0,1};
      auto scratch=std::make_unique<BatchScratch>();
      const auto a=serial.plan(requests,slots,2,scratch.get());
      const auto b=parallel.plan(requests,slots,2,scratch.get());
      require(a.candidate_count==2 && b.candidate_count==2,"batch size changed");
      for(std::size_t i=0;i<2;++i) {
        require(a.candidates[i].valid && b.candidates[i].valid,"batch invalid");
        near(a.candidates[i].selected_evaluation.cost,b.candidates[i].selected_evaluation.cost,1e-12,"parallel changed result");
      }
    }}
#endif
  };
}
std::string escape(const std::string &s) {
  std::string out; for(char c:s) {if(c=='"'||c=='\\')out+='\\';
    if(c=='\n')out+="\\n";else out+=c;}return out;
}
}
int main(int argc,char **argv) {
  int failed=0; std::string json="["; bool first=true;
  for(const auto &test:cases()) {
    std::string error;
    try{test.body();}catch(const std::exception &e){error=e.what();++failed;}
    std::cout<<(error.empty()?"PASS ":"FAIL ")<<test.name;
    if(!error.empty()) { std::cout<<": "<<error; }
    std::cout<<'\n';
    if(!first) { json+=","; }
    first=false;
    json+="{\"name\":\""+std::string(test.name)+"\",\"passed\":"+
      (error.empty()?"true":"false")+",\"error\":\""+escape(error)+"\"}";
  }
  json+="]\n";
  if(argc>1){std::ofstream out(argv[1]);out<<json;}
  std::cout<<"TOTAL="<<cases().size()<<" FAILED="<<failed<<'\n';
  return failed?1:0;
}
