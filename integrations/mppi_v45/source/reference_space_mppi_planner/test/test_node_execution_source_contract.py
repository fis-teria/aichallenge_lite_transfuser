"""Source-boundary regressions; these do not replace ROS/AWSIM execution tests."""

from pathlib import Path
import unittest
import subprocess
import tempfile
import re
import xml.etree.ElementTree as ET


SOURCE = (Path(__file__).resolve().parents[1] / "src/reference_space_mppi_node.cpp").read_text()


class NodeExecutionSourceContract(unittest.TestCase):
    def test_progress_reward_is_configured_in_each_runtime_reference_profile(self):
        config = Path(__file__).resolve().parents[1] / "config"
        profiles = [config / "reference_space_mppi.param.yaml",
                    *sorted((config / "cma_20260913").glob("*.param.yaml"))]
        self.assertEqual(len(profiles), 3)
        for profile in profiles:
            with self.subTest(profile=profile.name):
                value = re.search(r"^\s+brain\.cost_progress_weight:\s*([\d.]+)",
                                  profile.read_text(), re.MULTILINE)
                self.assertIsNotNone(value)
                self.assertEqual(float(value.group(1)), 1.25)

    def test_new_shapes_are_speed_optimized_before_execution_selection(self):
        body = SOURCE.split('auto execution_request = batch.candidates[0].request;', 1)[1]
        new = body.split('const double new_validation_ms', 1)[0]
        self.assertIn('optimizeExecutionSpeed(execution_evaluator, reference,', new)
        self.assertIn('execution_evaluations[i] = optimized.evaluation;', new)
        self.assertIn('optimized.reference.points[j].speed_mps', new)
        self.assertLess(new.index('optimized.evaluation'), new.index('side_candidates[i].cost'))

    def test_execution_ownership_uses_geometry_speed_and_driving_revision(self):
        # Execute the production commit predicate with controlled concurrent owners.
        start = SOURCE.index('        const std::optional<std::uint64_t> snapshot_source = batch.brain_hold ?')
        end = SOURCE.index('        execution_current =', start)
        program = r'''
#include <optional>
#include <string>
#include <cstdint>
struct Command { std::uint64_t generation; };
struct Batch {
  std::optional<Command> brain_hold;
  std::uint64_t speed_generation, driving_revision;
  std::uint64_t reference_revision{0};
  std::optional<Command> ordinary_hold;
};
struct Fsm { std::uint64_t value; std::uint64_t revision() const { return value; } };
bool check(const Batch &batch, const Fsm &driving_fsm_,
           std::optional<Command> active_brain_command_, std::uint64_t active_brain_speed_generation_,
           std::uint64_t reference_revision_=0, std::optional<Command> ordinary_hold_={},
           std::optional<Command> reference_transition_={}) {
bool owner_current = true;
''' + SOURCE[start:end] + r'''
return owner_current;
}
int main() {
if (!check({{}, 0, 0}, {0}, {}, 0)) return 1;
if (check({{}, 0, 0}, {1}, {}, 0)) return 2;
if (!check({Command{2157}, 2189, 1}, {1}, Command{2157}, 2189)) return 3;
if (check({Command{2157}, 2189, 1}, {1}, Command{2157}, 2190)) return 4;
if (check({Command{2157}, 2189, 1}, {1}, Command{2200}, 2189)) return 5;
if (check({{}, 0, 1}, {1}, Command{2200}, 0)) return 6;
if (check({Command{2157}, 2189, 1}, {2}, Command{2157}, 2189)) return 7;
if (check({{}, 0, 0, 1}, {0}, {}, 0, 2)) return 8;
if (check({{}, 0, 0, 2, Command{100}}, {0}, {}, 0, 2, Command{101})) return 9;
if (!check({{}, 0, 0, 2, Command{100}}, {0}, {}, 0, 2, {}, Command{100})) return 10;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'owner.cpp'
            binary = Path(directory) / 'owner'
            source.write_text(program)
            subprocess.run(['c++', '-std=c++17', str(source), '-o', str(binary)], check=True)
            self.assertEqual(subprocess.run([str(binary)]).returncode, 0)

    def test_recovery_invalidates_pending_execution_validation(self):
        heartbeat = SOURCE.split('bool publishHeartbeatCommand(', 1)[1].split('BrainInputs captureBrainInputs(', 1)[0]
        self.assertIn('++execution_revision_', heartbeat)
        self.assertIn('batch.execution_revision=inputs.execution_revision;', SOURCE)
        self.assertIn('execution_current = revision_check.canCommit(execution_revision_);', SOURCE)
        self.assertIn('semantic_current && owner_current && execution_current', SOURCE)

    def test_changed_execution_is_revalidated_without_publication_lock(self):
        validation = SOURCE.index('ExecutionRevisionCheck revision_check')
        capture = SOURCE.index('captureBrainInputs()', validation)
        check = SOURCE.index('validateBrainCommand(*proposal, validator)', capture)
        lock = SOURCE.index('std::unique_lock<std::mutex> authority_lock', check)
        self.assertLess(check, lock)
        self.assertIn('makeBrainCommandValidator(live, *live.odometry, 0.0)', SOURCE[capture:lock])

    def test_final_validation_and_commit_exclude_heartbeat_decision(self):
        worker = SOURCE.split('void workerLoop()', 1)[1]
        lock = worker.index('decision_lock(decision_mutex_)')
        validation = worker.index('ExecutionRevisionCheck revision_check')
        capture = worker.index('captureBrainInputs()', validation)
        check = worker.index('validateBrainCommand(*proposal, validator)', capture)
        commit = worker.index('revision_check.canCommit(execution_revision_)', check)
        self.assertLess(lock, validation)
        self.assertNotIn('decision_lock.unlock()', worker[lock:commit])
        # Unchanged plan identity does not waive current-state validation.
        self.assertNotIn('needsValidation', worker[validation:check])

    def test_clamped_projection_is_used_as_local_origin(self):
        body = SOURCE.split("const double local_distance_m = localDistance(request.ego.speed_mps);",1)[1].split("if(!populateDynamicObstacles(request,inputs",1)[0]
        self.assertIn("previous.x = ego_projection.x_m;",body)
        self.assertIn("previous.y = ego_projection.y_m;",body)
        self.assertNotIn("ego_projection.tangent_y * ego_projection.d_m",body)
        self.assertIn("request.ego.s_m = local_ego.s;",body)
        self.assertIn("request.ego.d_m = local_ego.d;",body)
        self.assertIn("request.anchor_d_m = local_ego.d;",body)

    def test_free_run_uses_shared_nonrotating_connection(self):
        body = SOURCE.split('makeBrainBaseCommand(const Command &source,',1)[1].split('selectFollowVehicle(',1)[0]
        self.assertIn('mppi::executionConnectionTranslation(',body)
        self.assertNotIn('reference_yaw_at',body)
        helper = (Path(__file__).resolve().parents[1]/'include/reference_space_mppi_planner/execution_trajectory.hpp').read_text()
        self.assertIn('const auto translation = executionConnectionTranslation(',helper)

    def test_adoption_diagnostics_distinguish_not_evaluated_from_rejected(self):
        worker = SOURCE.split("void workerLoop()", 1)[1].split("double wallPathCheckStep", 1)[0]
        self.assertIn('execution_evaluated[i] = true;', worker)
        self.assertIn('"not_evaluated"', worker)
        self.assertIn('raw_reason:', worker)

    def test_adoption_result_is_propagated_to_markers_and_cost_breakdown(self):
        worker = SOURCE.split("void workerLoop()", 1)[1].split("double wallPathCheckStep", 1)[0]
        self.assertIn('certified[i] = execution_evaluations[i].valid;', worker)
        self.assertIn('const auto &display_evaluation =', SOURCE)
        self.assertIn('const auto &summary_evaluation =', worker)

    def test_continuation_is_compared_in_same_snapshot_before_publication(self):
        worker = SOURCE.split("void workerLoop()", 1)[1].split("double wallPathCheckStep", 1)[0]
        self.assertIn("auto execution_request = batch.candidates[0].request;", worker)
        self.assertIn("const auto &effective_brain_hold = batch.brain_hold;", worker)
        self.assertIn("auto reference = temporaryReferenceFromCommand(*execution_commands[i]);", worker)
        self.assertIn("executionRequestForCandidate(execution_request,batch.candidates[i].request)", worker)
        self.assertIn("reference, candidate_request);", worker)
        self.assertIn("executionRequestForCandidate(execution_request,work.request)", worker)
        self.assertIn("bindPreparationConnection(adopted_request,config_,execution_evaluations[index]", worker)
        self.assertIn("temporaryReferenceFromCommand(refreshed), execution_request", worker)
        self.assertIn("selectExecutionCandidate(", worker)
        self.assertIn('(effective_brain_hold->corridor_side == 0)', worker)
        self.assertIn("execution_request.phase == mppi::Phase::MERGE", worker)
        self.assertIn("batch.driving_revision == driving_fsm_.revision()", SOURCE)

    def test_continuation_has_same_authority_gate_and_does_not_recommit_source(self):
        branch = SOURCE.split("if (continue_execution && current && unexpired && !shadow_only_)", 1)[1].split(
            "if (selected_index < batch.candidate_count", 1)[0]
        self.assertIn("publishOutput(hold)", branch)
        self.assertIn("MPPI_EXECUTION_APPLIED", branch)
        self.assertNotIn("active_brain_command_ =", branch)
        self.assertNotIn("batch_optimizer_->accept", branch)
        self.assertNotIn("continuedBrainHold(", SOURCE)

    def test_newer_execution_owner_cannot_be_overwritten_by_an_old_snapshot(self):
        receive = SOURCE.split('void receiveBrainCommand(', 1)[1].split('\n  void ', 1)[0]
        preparation, queue = receive.split('decision_lock(decision_mutex_)', 1)
        self.assertIn('recoverBrainCommandSpeed(', preparation)
        self.assertIn('work_mutex_', queue)
        heartbeat = SOURCE.split('bool publishHeartbeatCommand(', 1)[1].split('void receiveSteeringReport', 1)[0]
        self.assertLess(heartbeat.index('heartbeatPlanContent('), heartbeat.index('decision_lock(decision_mutex_)'))
        self.assertLess(heartbeat.index('heartbeatSnapshotCurrent('), heartbeat.index('if (commit) commit()'))
        self.assertIn("owner_current = snapshot_source == live_source &&", SOURCE)
        self.assertIn("batch.speed_generation == live_speed_generation", SOURCE)
        self.assertIn("generation_current && semantic_current && owner_current", SOURCE)

    def test_speed_update_keeps_shape_source_and_authority_gate(self):
        branch = SOURCE.split("if(retime_execution && current && unexpired && !shadow_only_)",1)[1].split(
            "if (continue_execution",1)[0]
        self.assertIn("publishOutput(command)",branch)
        self.assertIn("mppi::executionSpeedProfile(retimed.reference)",branch)
        self.assertIn("active_brain_speed_generation_=generation",branch)
        self.assertNotIn("active_brain_command_ =",branch)
        self.assertNotIn("batch_optimizer_->accept",branch)
        self.assertIn("mppi::optimizeExecutionSpeed(execution_evaluator,reference,execution_request,hold_evaluation,mppi::ExecutionSpeedSearch::Full,",
                      "".join(SOURCE.split()))

    def test_history_uses_remaining_shape_and_same_projection_as_evaluation(self):
        body = SOURCE.split("makeBrainWork(",1)[1]
        self.assertIn("mppi::setExecutionHistory",body)
        self.assertNotIn("offsetAtNormal(",body)
        self.assertIn("active_brain_speed_profile_.reset()",SOURCE)

    def test_world_occupancy_shared_by_candidates_hold_and_display(self):
        populate=SOURCE.split("bool populateDynamicObstacles(",1)[1].split("// Build a dynamics-only",1)[0]
        self.assertIn("request.world_reference=",populate)
        self.assertIn("o.global_reference_s_m=projection.s_m",populate)
        self.assertIn("populateDynamicObstacles(r,inputs",SOURCE)
        self.assertIn("populateDynamicObstacles(request,inputs",SOURCE)
        display=SOURCE.split("void publishDecisionSnapshot(",1)[1].split("void publishCandidate(",1)[0]
        self.assertIn("work.request.world_reference->visitOccupancy",display)
        self.assertIn("ReferencePoseIndex::forbiddenPolygon",display)
        self.assertNotIn('"full_forbidden_region"',display)

    def test_local_geometry_is_diagnostic_only_and_static_ids_are_not_dynamic(self):
        package = Path(__file__).resolve().parents[1]
        core = (package / "src/reference_space_mppi.cpp").read_text()
        self.assertNotIn("localEnvelopeDiagnostic(", core)
        self.assertIn("evaluation.reject_dynamic_obstacle &&", SOURCE)
        self.assertIn("ev.reject_dynamic_obstacle &&", SOURCE)
        body = SOURCE.split("std::ostringstream witness;", 1)[1].split(
            'RCLCPP_INFO(get_logger(), "[MPPI_COLLISION_WITNESS]', 1)[0]
        self.assertIn("local_forward_overlap=", body)
        self.assertNotIn("return", body)
        self.assertNotIn(".valid =", body)

    def test_witness_initial_state_is_evaluated_batch_input(self):
        body = SOURCE.split("const auto &opponent=batch.opponents[obstacle_index];", 1)[1].split(
            'RCLCPP_INFO(get_logger(), "[MPPI_COLLISION_WITNESS]', 1)[0]
        self.assertIn("const auto &initial = request.dynamic_obstacles[obstacle_index];", body)
        for field in ("s_m", "d_m", "longitudinal_speed_mps", "lateral_speed_mps",
                      "longitudinal_acceleration_bound_mps2", "longitudinal_uncertainty_m",
                      "lateral_uncertainty_m", "heading_relative_to_reference_rad"):
            self.assertIn("initial." + field, body)
        self.assertIn("opponent.stamp_sec", body)
        self.assertNotIn("now()", body)
        self.assertNotIn("opponents_", body)

    def test_cma_geometry_switch_and_window_match_launch(self):
        for field in ("compare_cma_delay_compensation", "compare_cma_preview_feedforward",
                      "cma_lookahead_curvature_enabled"):
            self.assertIn(f"config.{field} = cma_prediction_geometry;", SOURCE)
        package = Path(__file__).resolve().parents[1]
        launch = package.parent / "aichallenge_submit_launch/launch/control/cma_pure_pursuit.launch.xml"
        value = next(float(arg.attrib["default"]) for arg in ET.parse(launch).getroot().findall("arg")
                     if arg.attrib["name"] == "curvature_speed_preview_distance")
        yaml = (package / "config/reference_space_mppi.param.yaml").read_text()
        self.assertEqual(float(re.search(r"prediction.cma_curvature_preview_distance_m: ([\d.]+)", yaml)[1]), value)

    def test_candidate_snapshot_includes_clear_and_all_lines(self):
        body = SOURCE.split("visualization_msgs::msg::MarkerArray candidate_markers;", 1)[1].split(
            "std::size_t selected_index", 1
        )[0]
        self.assertIn("Marker::DELETEALL", body)
        self.assertIn("index < batch.candidate_count", body)
        self.assertLess(body.index("publishCandidate("), body.index("candidates_pub_->publish("))
        self.assertEqual(body.count("candidates_pub_->publish("), 1)

    def test_decision_snapshot_is_bounded_and_read_only(self):
        body = SOURCE.split("void publishDecisionSnapshot(", 1)[1].split("void publishCandidate(", 1)[0]
        self.assertIn("Marker::DELETEALL", body)
        self.assertIn("from_seconds(1.5)", body)
        self.assertIn('"/s"+std::to_string(slot)', body)
        self.assertIn('"/p"+std::to_string(profile)', body)
        self.assertIn("work.request.generation", body)
        self.assertIn("tail UNVERIFIED", body)
        self.assertIn("ENVELOPE_ONLY_HERE", body)
        self.assertEqual(body.count("decision_pub_->publish("), 1)
        for forbidden in ("output_pub_", "input_mutex_", "plan(", "validateReference(", "now().seconds()", "now().to_msg()"):
            self.assertNotIn(forbidden, body)
        self.assertIn('"brain.decision_markers_enabled", false', SOURCE)

    def test_sample_markers_use_packed_count_and_original_ids(self):
        self.assertIn("sample_slot < result.visualized_sample_count", SOURCE)
        self.assertIn("result.visualized_sample_indices[sample_slot]", SOURCE)


if __name__ == "__main__":
    unittest.main()
