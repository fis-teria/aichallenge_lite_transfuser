#!/usr/bin/env python3
"""Static guardrails for the default-off State Lattice live source."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


SUBMIT_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_ROOT = SUBMIT_ROOT / "aichallenge_submit_launch" / "launch"
PLANNER_ROOT = SUBMIT_ROOT / "overtake_planner"
PLANNER_NODE = PLANNER_ROOT / "src" / "overtake_planner_node.cpp"
PLANNER_CORE = PLANNER_ROOT / "src" / "overtake_planner_core.cpp"
PLANNER_PARAMS = PLANNER_ROOT / "config" / "overtake_planner.param.yaml"
ROUTE_RESOLVER = PLANNER_ROOT / "scripts" / "resolve_overtake_control_route"
STATE_LATTICE_ROOT = SUBMIT_ROOT / "state_lattice_overtake_planner"
STATE_LATTICE_NODE = (
    STATE_LATTICE_ROOT / "src" / "state_lattice_overtake_planner_node.cpp"
)
STATE_LATTICE_PARAMS = (
    STATE_LATTICE_ROOT / "config" / "state_lattice_overtake_planner.param.yaml"
)
MPPI_ROOT = SUBMIT_ROOT / "reference_space_mppi_planner"
MPPI_NODE = MPPI_ROOT / "src" / "reference_space_mppi_node.cpp"
MPPI_CORE = MPPI_ROOT / "src" / "reference_space_mppi.cpp"
MPPI_PARAMS = MPPI_ROOT / "config" / "reference_space_mppi.param.yaml"
AWSIM_MODE_LAUNCH = (
    SUBMIT_ROOT.parent
    / "aichallenge_system"
    / "aichallenge_system_launch"
    / "launch"
    / "mode"
    / "awsim.launch.xml"
)
RVIZ_CONFIG_ROOT = (
    SUBMIT_ROOT.parent
    / "aichallenge_system"
    / "aichallenge_system_launch"
    / "config"
)
SYSTEM_LAUNCH = (
    SUBMIT_ROOT.parent
    / "aichallenge_system"
    / "aichallenge_system_launch"
    / "launch"
    / "aichallenge_system.launch.xml"
)
AICHALLENGE_ROOT = next(
    (
        parent
        for parent in SUBMIT_ROOT.parents
        if (parent / "run_autoware.bash").is_file()
    ),
    None,
)
RUN_AUTOWARE = (
    AICHALLENGE_ROOT / "run_autoware.bash" if AICHALLENGE_ROOT else None
)
RUN_EVALUATION = (
    AICHALLENGE_ROOT / "run_evaluation.bash" if AICHALLENGE_ROOT else None
)
EVALUATION_LAUNCH = SYSTEM_LAUNCH.with_name("evaluation.launch.xml")
DOCKER_COMPOSE = (
    AICHALLENGE_ROOT.parent / "docker-compose.yml"
    if AICHALLENGE_ROOT is not None
    else None
)

EXPECTED_AUTHORITY_PUBLISHERS = {
    "/overtake/reference_override": "std_msgs::msg::Float32MultiArray",
    "/overtake/safety_constraint": (
        "multi_purpose_mpc_ros_msgs::msg::SafetyConstraint"
    ),
    "/overtake/plan": "multi_purpose_mpc_ros_msgs::msg::OvertakePlan",
}

V2_EVIDENCE_TOPICS = (
    "/planning/overtake/state_lattice/v2_proposal",
    "/debug/overtake/state_lattice/v2_binding_status",
    "/control/overtake/state_lattice/v2_base_attestation",
)
V2_FORBIDDEN_AUTHORITY_CONSUMER_ROOTS = (
    SUBMIT_ROOT / "hybrid_control_mux",
    SUBMIT_ROOT / "multi_purpose_mpc_ros",
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def braced_block(source: str, marker: str) -> str:
    start = source.index(marker)
    open_brace = source.index("{", start)
    depth = 0
    for index in range(open_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace + 1 : index]
    raise AssertionError(f"unterminated block after {marker!r}")


class StateLatticeNonLiveContractTest(unittest.TestCase):
    def test_mppi_is_a_dedicated_always_live_control_method_entry(self) -> None:
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        reference_args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in reference.findall("arg")
        }
        self.assertEqual(reference_args["control_method"], "mppi")

        submission = ET.fromstring(
            read(LAUNCH_ROOT / "aichallenge_submit.launch.xml")
        )
        submission_args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in submission.findall("arg")
        }
        self.assertEqual(submission_args["control_method"], "mppi")

        mppi_group = next(
            group
            for group in reference.findall("group")
            if "control_method)' == 'mppi'" in group.attrib.get("if", "")
        )
        entry_include = mppi_group.find("include")
        self.assertIsNotNone(entry_include)
        self.assertIn(
            "/launch/control/mppi.launch.xml",
            entry_include.attrib.get("file", ""),
        )
        entry_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in entry_include.findall("arg")
        }
        self.assertEqual(entry_values["simulation"], "$(var simulation)")
        self.assertEqual(entry_values["use_sim_time"], "$(var use_sim_time)")
        self.assertEqual(
            entry_values["pure_pursuit_speed_mps"],
            "$(var effective_reference_execution_speed_cap_mps)",
        )

        mppi_entry = ET.fromstring(
            read(LAUNCH_ROOT / "control/mppi.launch.xml")
        )
        includes = mppi_entry.findall("include")
        self.assertEqual(len(includes), 2)
        mppi_include = next(
            include
            for include in includes
            if "reference_space_mppi.launch.xml"
            in include.attrib.get("file", "")
        )
        mppi_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in mppi_include.findall("arg")
        }
        self.assertEqual(mppi_values["enabled"], "true")
        self.assertEqual(mppi_values["shadow_only"], "false")
        self.assertEqual(mppi_values["brain_mode"], "true")
        self.assertEqual(mppi_values["internal_timer_enabled"], "true")
        self.assertEqual(mppi_values["static_wall_map_enabled"], "true")
        self.assertEqual(
            mppi_values["input_trajectory_command"],
            "/mppi/unused/upstream_trajectory_command",
        )
        self.assertEqual(
            mppi_values["input_control_command"],
            "/control/command/control_cmd",
        )

        cma_include = next(
            include
            for include in includes
            if "cma_pure_pursuit.launch.xml" in include.attrib.get("file", "")
        )
        cma_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in cma_include.findall("arg")
        }
        self.assertEqual(cma_values["use_atomic_direct_trajectory_command"], "true")
        self.assertEqual(cma_values["output_control_cmd"], "/mppi/internal/cma_control_cmd")
        self.assertEqual(cma_values["recovery_service_enabled"], "true")
        self.assertEqual(mppi_values["recovery_service_enabled"], "true")
        recovery = mppi_entry.find("node[@pkg='mppi_recovery_controller']")
        self.assertIsNotNone(recovery)
        recovery_remaps = {r.attrib["from"]: r.attrib["to"] for r in recovery.findall("remap")}
        self.assertEqual(recovery_remaps["input/auto_cmd"], cma_values["output_control_cmd"])
        self.assertEqual(recovery_remaps["output/control_cmd"], "/control/command/control_cmd")
        self.assertEqual(recovery_remaps["output/gear_cmd"], "/control/command/gear_cmd")
        self.assertEqual(
            cma_values["steering_command_passthrough_enabled"], "false"
        )
        self.assertEqual(
            cma_values["steering_command_to_tire_angle_ratio"], "0.60"
        )
        self.assertEqual(
            cma_values["hard_steering_angle_limit_rad"],
            "0.5235987755982988",
        )
        self.assertEqual(
            cma_values["maximum_tire_steering_angle_rad"],
            "0.3141592653589793",
        )
        self.assertEqual(
            cma_values["steering_demand_acceleration_hold_enabled"], "true"
        )
        self.assertEqual(
            cma_values["steering_acceleration_hold_minimum_speed_mps"], "3.0"
        )
        gentle_acceleration = "steering_acceleration_hold_maximum_acceleration_mps2"
        prediction_params = yaml.safe_load(MPPI_PARAMS.read_text())["/**"]["ros__parameters"]
        self.assertGreater(float(cma_values[gentle_acceleration]), 0.0)
        self.assertEqual(
            float(cma_values[gentle_acceleration]), prediction_params[gentle_acceleration]
        )
        cma_launch = ET.parse(LAUNCH_ROOT / "control/cma_pure_pursuit.launch.xml").getroot()
        self.assertEqual(
            cma_launch.find(f".//param[@name='{gentle_acceleration}']").attrib["value"],
            f"$(var {gentle_acceleration})",
        )
        self.assertEqual(cma_values["lateral_error_speed_gate_enabled"], "false")
        self.assertEqual(cma_values["steering_tracking_speed_gate_enabled"], "false")
        self.assertEqual(cma_values["wall_edge_tracking_speed_enabled"], "false")
        self.assertEqual(cma_values["speed_proportional_gain"], "3.0")
        self.assertEqual(cma_values["longitudinal_acceleration_limit"], "2.0")
        self.assertEqual(cma_values["wheel_base"], "1.087")
        self.assertEqual(cma_values["lookahead_gain"], "0.20")
        self.assertEqual(cma_values["lookahead_min_distance"], "2.0")
        self.assertEqual(
            cma_values["curvature_lookahead_min_distance"], "2.0"
        )
        self.assertEqual(
            cma_values["continuous_preview_interpolation_enabled"], "true"
        )
        self.assertEqual(cma_values["actual_lookahead_distance_blend"], "1.0")
        self.assertEqual(
            cma_values["curvature_feedforward_maneuver_only"], "false"
        )
        self.assertEqual(cma_values["diagnostic_trace_enabled"], "true")
        self.assertEqual(
            cma_values["diagnostic_trace_maneuver_only"], "false"
        )
        self.assertEqual(cma_values["rotation_gate_enabled"], "true")
        self.assertEqual(cma_values["rotation_gate_countersteer_gain"], "0.35")
        self.assertEqual(
            cma_values["rotation_gate_max_countersteer_rad"], "0.12"
        )
        self.assertEqual(cma_values["rotation_prediction_enabled"], "true")
        self.assertEqual(
            cma_values["rotation_prediction_lead_time_sec"], "0.15"
        )
        self.assertEqual(
            cma_values["rotation_prediction_entry_threshold_radps"], "0.18"
        )
        self.assertEqual(
            cma_values["rotation_prediction_slip_angle_threshold_rad"], "0.04"
        )

        source = read(LAUNCH_ROOT / "control/mppi.launch.xml")
        self.assertNotIn("state_lattice_overtake_planner", source)
        self.assertNotIn("state_lattice_pure_pursuit_direct.launch.xml", source)
        self.assertNotIn("boost_command_bridge", source)
        self.assertNotIn('exec="boost_commander"', source)
        self.assertNotIn("/boost_commander/command", source)

    def test_mppi_constraints_are_integrated_into_sample_evaluation(self) -> None:
        parameters = yaml.safe_load(read(MPPI_PARAMS))["/**"]["ros__parameters"]
        self.assertFalse(parameters["brain.enable_safe_stop"])
        self.assertTrue(parameters["collision_only_rejection"])
        self.assertGreater(parameters["brain.minimum_rolling_speed_mps"], 0.0)
        self.assertGreaterEqual(parameters["brain.max_track_offset_m"], 4.0)
        self.assertEqual(parameters["brain.overtake_speed_mps"], 10.0)
        self.assertEqual(parameters["brain.cruise_speed_mps"], 10.0)
        self.assertEqual(parameters["maximum_acceleration_mps2"], 2.0)
        self.assertEqual(parameters["wheel_base_m"], 1.087)
        self.assertEqual(parameters["lookahead_gain"], 0.20)
        self.assertEqual(parameters["lookahead_min_distance_m"], 2.0)
        self.assertEqual(
            parameters["curvature_lookahead_min_distance_m"], 2.0
        )
        self.assertTrue(parameters["continuous_preview_interpolation_enabled"])
        self.assertEqual(parameters["actual_lookahead_distance_blend"], 1.0)
        self.assertEqual(parameters["steering_control_delay_sec"], 0.20)
        self.assertGreaterEqual(
            parameters["brain.overtake_speed_mps"],
            parameters["brain.cruise_speed_mps"],
        )
        self.assertGreater(parameters["brain.target_path_conflict_margin_m"], 0.0)
        self.assertGreater(
            parameters["brain.minimum_passing_speed_mps"],
            parameters["brain.minimum_rolling_speed_mps"],
        )
        self.assertEqual(parameters["brain.minimum_passing_speed_mps"], 6.0)
        self.assertGreater(
            parameters["brain.minimum_proximity_speed_mps"],
            parameters["brain.minimum_rolling_speed_mps"],
        )
        self.assertEqual(
            parameters["brain.minimum_proximity_speed_mps"],
            parameters["brain.overtake_speed_mps"],
        )
        self.assertGreaterEqual(parameters["brain.pass_offset_m"], 2.2)
        self.assertEqual(parameters["brain.min_pass_offset_m"], 0.0)
        self.assertLess(
            parameters["brain.min_pass_offset_m"],
            parameters["brain.pass_offset_m"],
        )
        self.assertEqual(
            parameters["brain.hold_minimum_opponent_clearance_m"], 0.0
        )
        self.assertGreater(
            parameters["brain.hold_revalidation_horizon_sec"], 0.0
        )
        self.assertEqual(
            parameters["brain.hold_revalidation_horizon_sec"], 1.5
        )
        self.assertGreater(parameters["brain.minimum_passing_advantage_mps"], 0.0)
        self.assertEqual(parameters["brain.hard_clearance_m"], 0.0)

        node_source = read(MPPI_NODE)
        worker = braced_block(node_source, "void workerLoop()")
        self.assertIn("candidate_reject_s > current_reject_s", worker)
        self.assertIn("certified[index] = results[index].valid", worker)
        self.assertIn(
            "batch.candidates[0U].request.path_constraint_validator",
            worker,
        )
        self.assertNotIn("brainCandidateCertified", worker)
        self.assertNotIn("brainHoldTrackable", worker)
        self.assertNotIn("rollingLateralHold", worker)
        self.assertNotIn("brain:fallback_safety_gate", worker)
        self.assertIn("kBrainTemplateLineCount = 5U", node_source)
        self.assertIn("work.request.nominal_only = true", node_source)
        self.assertIn(
            "work.request.cost_preferred_pass_separation_m", node_source
        )
        self.assertIn("lateral_line_library::make", node_source)
        self.assertIn("for (std::size_t slot = 0U; slot < library.count", node_source)
        self.assertIn("append_template(source, line.d_m", node_source)
        self.assertIn("[MPPI_TEMPLATE_SELECTION]", worker)
        self.assertIn(
            "batch.brain_owned && batch.brain_hold.has_value()",
            worker,
        )
        self.assertIn("publishAppliedCommand(guarded)", worker)
        self.assertIn("const auto guarded =", worker)
        self.assertIn("active_brain_command_ = refined", worker)
        self.assertNotIn("active_brain_command_ = hold", worker)
        self.assertIn("safe_refreshed_hold", worker)
        self.assertIn("hold_reject_reason", worker)
        self.assertIn('"brain:abort_opponent_clearance"', worker)
        self.assertIn('"brain:abort_wall_clearance"', worker)
        self.assertIn("batch.brain_hold.has_value()", worker)
        self.assertIn("selected_index = batch.candidate_count", worker)
        self.assertIn(
            'publishStatus(guarded, &diagnostic_result, "brain:hold_last_valid")',
            worker,
        )

        brain_receive = braced_block(node_source, "void receiveBrainCommand(")
        self.assertIn("publish_committed_hold", brain_receive)
        self.assertIn("makeBrainPathConstraintValidator", brain_receive)
        self.assertIn("validateBrainCommand", brain_receive)
        self.assertIn('"brain:hold_committed_target_unavailable"', brain_receive)
        self.assertIn('"brain:hold_during_retry"', brain_receive)
        self.assertIn("merge_complete", brain_receive)
        self.assertNotIn("active_brain_command_ = hold", brain_receive)

        refreshed_hold = braced_block(node_source, "Command refreshedBrainHold(")
        self.assertIn("projectOnTrajectory", refreshed_hold)
        self.assertIn("anchor.pose = odometry.pose.pose", refreshed_hold)
        self.assertIn("connector_length_m", refreshed_hold)
        self.assertIn("Mppi::quinticBlend", refreshed_hold)
        self.assertIn("translation_x", refreshed_hold)
        self.assertIn("translation_y", refreshed_hold)
        self.assertNotIn("path_yaw_at", refreshed_hold)
        self.assertIn("hold.trajectory.points = std::move(remaining)", refreshed_hold)
        self.assertIn("continuedBrainHold", refreshed_hold)

        continued_hold = braced_block(node_source, "Command continuedBrainHold(")
        self.assertIn("anchor.pose = odometry.pose.pose", continued_hold)
        self.assertIn("global_projection.d_m", continued_hold)
        self.assertIn("normal_transition_length_m", continued_hold)
        self.assertIn("normal_blend", continued_hold)
        self.assertIn('hold.reason = "mppi_brain:continued_lateral_hold"', continued_hold)

        hold_clearance = braced_block(
            node_source, "bool brainHoldOpponentSafe("
        )
        self.assertIn("footprintsHaveClearance", hold_clearance)
        self.assertIn("brain_hold_minimum_opponent_clearance_m_", hold_clearance)
        self.assertIn("referencePoseAtS", hold_clearance)
        self.assertIn("opponent.longitudinal_speed_mps * elapsed_sec", hold_clearance)
        self.assertIn("validation_horizon_sec", hold_clearance)
        self.assertIn("brain_hold_revalidation_horizon_sec_", hold_clearance)
        self.assertIn("predicted_ego_speed_mps", hold_clearance)
        self.assertIn("maximum_acceleration_mps2", hold_clearance)

        footprint_clearance = braced_block(
            node_source, "bool footprintsHaveClearance("
        )
        self.assertIn("first_radius_m", footprint_clearance)
        self.assertIn("second_radius_m", footprint_clearance)
        self.assertIn("required_clearance_m", footprint_clearance)

        path_constraint = braced_block(
            node_source, "mppi::RejectReason brainPathConstraintRejectReason("
        )
        self.assertIn("commandFootprintPathFree", path_constraint)
        self.assertIn("brainHoldOpponentSafe", path_constraint)
        self.assertIn("mppi::RejectReason::WALL", path_constraint)
        self.assertIn("mppi::RejectReason::COLLISION", path_constraint)

        core_source = read(MPPI_CORE)
        plan = braced_block(
            core_source, "PlanResult ReferenceSpaceMppiPlanner::plan("
        )
        self.assertIn("if (request.nominal_only)", plan)
        self.assertIn("result.valid_sample_count = 1U", plan)
        evaluate = braced_block(
            core_source, "ReferenceSpaceMppiPlanner::evaluate("
        )
        self.assertIn("finalSweptValidate(*reference, request, &result)", evaluate)
        self.assertIn("request.path_constraint_validator(*reference)", evaluate)

        proximity_guard = braced_block(
            node_source, "Command applyBrainProximityGuard("
        )
        self.assertIn("if (brain_enable_safe_stop_)", proximity_guard)
        self.assertIn("brain_minimum_rolling_speed_mps_", proximity_guard)
        self.assertIn("brain_minimum_proximity_speed_mps_", proximity_guard)
        self.assertIn("work.target_speed_mps + 0.10", proximity_guard)
        self.assertIn("brain_overtake_speed_mps_", proximity_guard)

    def test_mppi_rviz_paths_are_sample_validated(self) -> None:
        node_source = read(MPPI_NODE)
        self.assertNotIn("brainCandidateCertified", node_source)
        self.assertNotIn("referenceFootprintPathFree", node_source)

        command_wall_gate = braced_block(
            node_source, "bool commandFootprintPathFree("
        )
        self.assertIn("occupancyGridFootprintPathFree", command_wall_gate)

        candidate_marker = braced_block(
            node_source, "void publishCandidate("
        )
        self.assertIn("if (result.selected_reference.count < 2U)", candidate_marker)
        self.assertIn('"mppi_candidate_valid"', candidate_marker)
        self.assertIn('"mppi_candidate_rejected"', candidate_marker)
        self.assertIn("accepted ? 0.15F : 1.0F", candidate_marker)

        brain_receive = braced_block(node_source, "void receiveBrainCommand(")
        self.assertIn("clearCandidateMarkers(command.header)", brain_receive)

    def test_mppi_candidate_markers_are_one_complete_snapshot(self) -> None:
        node = read(MPPI_NODE)
        worker = braced_block(node, "void workerLoop()")
        self.assertIn("candidates_pub_->publish(candidate_markers)", worker)
        self.assertIn("certified[index], candidate_markers", worker)
        candidate = braced_block(node, "void publishCandidate(")
        self.assertNotIn("candidates_pub_->publish", candidate)
        self.assertIn('" rejected:"', candidate)
        self.assertIn('" feasible"', candidate)
        self.assertIn('"mppi_rollout_collision"', candidate)
        self.assertIn('"debug/candidates", rclcpp::QoS(1).reliable().transient_local()', node)
        for preset in ("autoware.rviz", "autoware_vehicle.rviz"):
            text = read(SUBMIT_ROOT.parent / "aichallenge_system" /
                        "aichallenge_system_launch" / "config" / preset)
            for name in ("MPPI Candidate Trajectories", "MPPI Applied Trajectory"):
                section = text.split("Name: " + name, 1)[1].split("- Class:", 1)[0]
                self.assertIn("Durability Policy: Transient Local", section)

    def test_mppi_local_horizon_is_separate_from_free_run_context(self) -> None:
        node_source = read(MPPI_NODE)
        work = braced_block(node_source, "std::optional<WorkItem> makeBrainWork(")
        self.assertIn("localDistance(request.ego.speed_mps)", work)
        self.assertNotIn("brain_reference_horizon_m_", work)
        self.assertIn("std::min(ds, remaining)", work)
        self.assertIn("request.horizon_steps_override = localSteps", work)
        self.assertNotIn("active_maneuver_length_m", work)
        base = braced_block(node_source, "makeBrainBaseCommand(const Command &source,")
        self.assertIn("brain_reference_horizon_m_", base)
        hold = braced_block(node_source, "Command refreshedBrainHold(")
        self.assertIn("trimLocalHold", hold)
        continued = braced_block(node_source, "Command continuedBrainHold(")
        self.assertIn("trimLocalHold", continued)
        self.assertNotIn("brain_reference_horizon_m_", continued)

    def test_mppi_publishes_heartbeat_before_queueing_brain_batch(self) -> None:
        node_source = read(MPPI_NODE)
        receive = braced_block(node_source, "void receiveBrainCommand(")
        heartbeat = receive.index('publish_committed_hold("brain:hold_planning")')
        queue = receive.index("pending_batch_ = std::move(batch)")
        self.assertLess(heartbeat, queue)
        self.assertIn("publishOutput(fallback)", receive[heartbeat:queue])
        publish = braced_block(node_source, "void publishOutput(")
        self.assertIn("sequenced_output_.publish", publish)
        worker = braced_block(node_source, "void workerLoop()")
        self.assertNotIn("output_pub_->publish", worker)
        self.assertIn("current && unexpired", worker)

    def test_mppi_brain_reference_is_anchored_at_measured_ego_pose(self) -> None:
        node_source = read(MPPI_NODE)
        brain_base = braced_block(
            node_source, "makeBrainBaseCommand(const Command &source,"
        )
        self.assertIn("first.pose = odometry.pose.pose", brain_base)
        self.assertIn("const std::size_t start = ego_projection.lower_index + 1U", brain_base)
        self.assertIn("connector_length_m", brain_base)
        self.assertNotIn("const std::size_t behind", brain_base)
        self.assertIn(
            "static_cast<float>(brain_cruise_speed_mps_)", brain_base
        )
        self.assertNotIn(
            "std::abs(static_cast<double>(point.longitudinal_velocity_mps))",
            brain_base,
        )

        target_selection = braced_block(
            node_source, "selectBrainTarget(const Trajectory &global_reference,"
        )
        self.assertIn("free_run_reference", target_selection)
        self.assertIn("pathFootprintsConflict", target_selection)
        self.assertIn("free_run_projection.distance_m", target_selection)
        self.assertIn("path_conflict", target_selection)
        self.assertIn("committed ||", target_selection)

        brain_work = braced_block(
            node_source, "std::optional<WorkItem> makeBrainWork("
        )
        self.assertIn("const auto &global_reference = *inputs.base_reference", brain_work)
        self.assertIn("projectOnTrajectory(\n        global_reference", brain_work)
        self.assertIn("work.command.trajectory.points.clear()", brain_work)
        self.assertIn("ego_projection.tangent_y * ego_projection.d_m", brain_work)
        self.assertIn("base.s_m = 0.0", brain_work)
        self.assertIn("request.ego.s_m = 0.0", brain_work)
        self.assertIn("request.ego.d_m = ego_projection.d_m", brain_work)
        self.assertIn("request.anchor_d_m = ego_projection.d_m", brain_work)
        self.assertIn("kNearTargetSpeedSearchDistanceM = 18.0", brain_work)
        self.assertIn("kNearTargetNominalSpeedScale = 1.00", brain_work)
        self.assertIn("kNearTargetMinimumSpeedScale = 0.20", brain_work)
        self.assertIn("kFarTargetMinimumSpeedScale = 0.35", brain_work)
        self.assertIn("base.speed_mps = brain_overtake_speed_mps_", brain_work)
        self.assertIn("passing_speed_floor_mps", brain_work)
        self.assertIn("brain_minimum_passing_advantage_mps_", brain_work)
        self.assertIn("request.minimum_speed_mps", brain_work)
        self.assertIn("heading_relative_to_reference_rad", brain_work)
        self.assertIn("ego_projection.lower_index + 1U", brain_work)
        self.assertIn("base.active_d_m = ego_projection.d_m", brain_work)
        self.assertNotIn("base.active_d_m = 0.0", brain_work)
        self.assertIn(
            "obstacle.s_m = obstacle_s_m",
            brain_work,
        )
        self.assertIn("signedReferenceDelta(", brain_work)

        brain_receive = braced_block(node_source, "void receiveBrainCommand(")
        self.assertIn("global_ego_projection.d_m > 0.0 ? 1 : -1", brain_receive)
        self.assertIn("base_command->trajectory", brain_receive)

    def test_initial_grid_creep_replaces_unconditional_direct_startup_line(
        self,
    ) -> None:
        planner_launch = ET.fromstring(
            read(
                STATE_LATTICE_ROOT
                / "launch/state_lattice_overtake_planner.launch.xml"
            )
        )
        defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in planner_launch.findall("arg")
        }
        self.assertEqual(defaults["startup_follow_creep_enabled"], "false")
        self.assertEqual(defaults["startup_follow_creep_max_speed_mps"], "0.5")
        self.assertEqual(defaults["startup_follow_creep_max_duration_sec"], "30.0")
        self.assertEqual(defaults["startup_follow_creep_max_distance_m"], "5.0")
        self.assertEqual(defaults["startup_follow_creep_release_speed_mps"], "2.0")
        self.assertEqual(defaults["direct_runtime_motion_start_speed_mps"], "0.10")
        self.assertEqual(
            defaults["direct_runtime_motion_start_required_samples"], "2"
        )
        planner_node = planner_launch.find("node")
        self.assertIsNotNone(planner_node)
        forwarded = {
            param.attrib.get("name"): param.attrib.get("value")
            for param in planner_node.findall("param")
        }
        self.assertEqual(
            forwarded["startup_follow_creep_enabled"],
            "$(var startup_follow_creep_enabled)",
        )

        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        planner_include = next(
            include
            for include in direct.findall("include")
            if "state_lattice_overtake_planner.launch.xml"
            in include.attrib.get("file", "")
        )
        direct_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in planner_include.findall("arg")
        }
        self.assertEqual(direct_values["direct_startup_straight_enabled"], "false")
        self.assertEqual(direct_values["startup_follow_creep_enabled"], "true")
        self.assertEqual(direct_values["startup_follow_creep_min_free_gap_m"], "0.5")
        self.assertEqual(direct_values["startup_follow_creep_release_speed_mps"], "2.0")
        self.assertEqual(
            direct_values["direct_runtime_motion_start_speed_mps"], "0.10"
        )
        self.assertEqual(
            direct_values["direct_runtime_motion_start_required_samples"], "2"
        )

        node_source = read(STATE_LATTICE_NODE)
        self.assertNotIn(
            "if (direct_trajectory_enabled_ &&\n"
            "        state_lattice_authority_race_arm_sub_ == nullptr)",
            node_source,
        )
        self.assertIn("advanceDirectRuntimeEpoch(now_sec, inputs_fresh)", node_source)

        # A YAML scalar can override an XML launch argument during ROS launch
        # parameter merging. Keep launch ownership of this direct-only switch.
        common_parameters = yaml.safe_load(read(STATE_LATTICE_PARAMS))
        node_parameters = common_parameters[
            "state_lattice_overtake_planner_node"
        ]["ros__parameters"]
        self.assertNotIn("startup_follow_creep_enabled", node_parameters)

    def test_directional_stuck_recovery_is_direct_route_only(self) -> None:
        planner_launch = ET.fromstring(
            read(
                STATE_LATTICE_ROOT
                / "launch/state_lattice_overtake_planner.launch.xml"
            )
        )
        defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in planner_launch.findall("arg")
        }
        self.assertEqual(defaults["direct_stuck_recovery_enabled"], "false")
        self.assertEqual(
            defaults["direct_stuck_reverse_brake_override_enabled"], "false"
        )
        self.assertEqual(defaults["direct_stuck_reverse_brake_command"], "1.0")
        self.assertEqual(defaults["direct_stuck_reverse_stop_hold_sec"], "0.20")
        self.assertEqual(defaults["allow_reverse"], "false")
        self.assertEqual(defaults["direct_braking_corridor_hold_sec"], "0.5")
        self.assertEqual(
            defaults["direct_braking_corridor_deceleration_mps2"], "3.0"
        )

        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        planner_include = next(
            include
            for include in direct.findall("include")
            if "state_lattice_overtake_planner.launch.xml"
            in include.attrib.get("file", "")
        )
        values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in planner_include.findall("arg")
        }
        self.assertEqual(values["direct_stuck_recovery_enabled"], "true")
        self.assertEqual(values["allow_reverse"], "true")
        self.assertEqual(values["direct_braking_corridor_hold_sec"], "0.5")
        self.assertEqual(
            values["direct_braking_corridor_deceleration_mps2"], "3.0"
        )
        self.assertEqual(values["direct_stuck_forward_speed_mps"], "1.0")
        self.assertEqual(values["direct_stuck_forward_distance_m"], "1.5")
        self.assertEqual(
            values["direct_stuck_forward_clearance_margin_m"], "0.25"
        )
        self.assertEqual(values["direct_stuck_forward_timeout_sec"], "3.0")
        self.assertEqual(values["direct_stuck_reverse_speed_mps"], "1.0")
        self.assertEqual(values["direct_stuck_reverse_distance_m"], "1.5")
        self.assertEqual(values["direct_stuck_reverse_stop_hold_sec"], "0.20")
        self.assertEqual(
            values["direct_stuck_reverse_clearance_margin_m"], "0.25"
        )
        self.assertEqual(
            values["direct_stuck_gear_command_topic"],
            "/control/command/gear_cmd",
        )
        self.assertEqual(
            values["direct_stuck_gear_status_topic"],
            "/vehicle/status/gear_status",
        )
        self.assertEqual(
            values["direct_stuck_reverse_brake_override_enabled"], "true"
        )
        self.assertEqual(values["direct_stuck_reverse_brake_command"], "1.0")

        node = read(STATE_LATTICE_NODE)
        self.assertIn("makeDirectBrakingCorridor(", node)
        self.assertIn("boundedExactCartesianExecution(", node)
        self.assertIn("validateTrajectoryForExecution(", node)
        self.assertIn("resolveOwnedFallback(", node)
        self.assertIn('"direct_owned_controlled_brake"', node)
        self.assertIn("directExecutionCommandsStationaryHold(*output)", node)
        self.assertIn("state_lattice_authority_race_arm_reset_pending_", node)
        self.assertIn("consumeRaceArmReset();", node)
        self.assertIn('output->reason = "direct_stuck_forward_recovery"', node)
        self.assertIn("directForwardWallBlocked()", node)
        self.assertIn("buildDirectForwardEscapePath()", node)
        self.assertIn("directForwardCorridorSafe(true)", node)
        self.assertIn("directForwardCorridorSafe(false)", node)
        self.assertIn('"front_wall_blocked"', node)
        self.assertIn('output->reason = "direct_stuck_reverse_recovery"', node)
        self.assertIn("-direct_stuck_reverse_speed_mps_", node)
        self.assertIn("directReverseCorridorSafe(true)", node)
        self.assertIn("directReverseCorridorSafe(false)", node)
        self.assertIn("direct_stuck_reverse_target_distance_m_", node)
        self.assertIn('"wall_clear_not_reached"', node)
        self.assertIn('"wall_reentry"', node)
        self.assertIn('"opponent_reentry:"', node)
        self.assertIn("direct_stuck_reverse_block_reason", node)
        stopped_confirmation = node.index(
            "now_sec - direct_stuck_candidate_since_sec_"
        )
        history_freeze = node.index("direct_stuck_history_frozen_ = true")
        reverse_entry_after_freeze = node.index(
            "beginDirectReverseRecovery(output, now_sec", history_freeze
        )
        reverse_builder = braced_block(node, "bool beginDirectReverseRecovery")
        self.assertIn("buildDirectReverseRetracePath()", reverse_builder)
        self.assertIn("planner_->resetManeuverState()", reverse_builder)
        self.assertLess(stopped_confirmation, history_freeze)
        self.assertLess(history_freeze, reverse_entry_after_freeze)
        reverse_first_entry = node.index(
            "beginDirectReverseRecovery(output, now_sec", history_freeze
        )
        forward_fallback = node.index("forward_candidate_safe", reverse_first_entry)
        self.assertLess(reverse_first_entry, forward_fallback)
        wait_forward = node.index("WAIT_FORWARD_GEAR")
        forward_drive_command = node.index("GearCommand::DRIVE", wait_forward)
        forward_drive_confirmation = node.index(
            "directDriveGearConfirmed(now_sec)", forward_drive_command
        )
        positive_output = node.index(
            'output->reason = "direct_stuck_forward_recovery"'
        )
        self.assertLess(forward_drive_command, forward_drive_confirmation)
        self.assertLess(forward_drive_confirmation, positive_output)
        recovery_entry = node.index("void applyDirectStuckRecovery")
        wait_reverse_brake = node.index(
            "DirectStuckRecoveryPhase::WAIT_REVERSE_BRAKE", recovery_entry
        )
        brake_confirmation = node.index(
            "directReverseBrakeConfirmed(now_sec)", wait_reverse_brake
        )
        brake_publish = node.index(
            "publishDirectRecoveryBrakeOverride(", wait_reverse_brake
        )
        reverse_command = node.index("GearCommand::REVERSE", brake_confirmation)
        reverse_confirmation = node.index("directReverseGearConfirmed(now_sec)")
        negative_output = node.index(
            'output->reason = "direct_stuck_reverse_recovery"'
        )
        self.assertLess(brake_publish, brake_confirmation)
        self.assertLess(brake_confirmation, reverse_command)
        self.assertLess(reverse_command, reverse_confirmation)
        self.assertLess(reverse_confirmation, negative_output)
        self.assertGreaterEqual(
            node.count("directReverseGearConfirmed(now_sec)"), 2
        )
        recovery_flow = braced_block(node, "void applyDirectStuckRecovery")
        invalid_state = recovery_flow[
            recovery_flow.index("if (!std::isfinite(now_sec)") :
            recovery_flow.index("const bool wall_contact")
        ]
        self.assertLess(
            invalid_state.index("publishDirectRecoveryBrakeOverride("),
            invalid_state.index("GearCommand::DRIVE"),
        )
        reverse_motion = recovery_flow.index(
            "DirectStuckRecoveryPhase::REVERSING"
        )
        reverse_stop = recovery_flow.index(
            "DirectStuckRecoveryPhase::STOPPING_REVERSE", reverse_motion
        )
        stationary_check = recovery_flow.index(
            "std::abs(ego_.speed_mps) > direct_stuck_speed_threshold_mps_",
            reverse_stop,
        )
        drive_wait = recovery_flow.index(
            "DirectStuckRecoveryPhase::WAIT_DRIVE_GEAR", reverse_stop
        )
        self.assertLess(reverse_motion, reverse_stop)
        self.assertLess(reverse_stop, stationary_check)
        self.assertLess(stationary_check, drive_wait)
        self.assertIn("direct_stuck_reverse_stop_hold_sec_", recovery_flow)
        self.assertIn(
            "DirectReverseGearWaitAction::RETURN_TO_DRIVE", node
        )
        self.assertIn(
            '"direct_stuck_recovery_reverse_gear_timeout_waiting_drive_gear"',
            node,
        )
        handback = braced_block(node, "void finishDirectStuckRecoveryCycle")
        self.assertIn("BehaviorMode::FREE_RUN", handback)
        self.assertIn("LateralOwner::BASE_REFERENCE", handback)
        self.assertIn(
            'clearDirectBrakingCorridor("stuck_recovery_handback")', handback
        )
        self.assertNotIn("setDirectStuckStopOutput", handback)
        reverse_gear_lost = recovery_flow[
            recovery_flow.index(
                "if (!directReverseGearConfirmed(now_sec))", reverse_motion
            ) :
            recovery_flow.index("direct_stuck_recovery_traveled_m_", reverse_motion)
        ]
        self.assertIn(
            "DirectStuckRecoveryPhase::STOPPING_REVERSE", reverse_gear_lost
        )
        self.assertNotIn("WAIT_REVERSE_BRAKE", reverse_gear_lost)
        self.assertIn(
            '"direct_stuck_recovery_reverse_blocked_stopping"', recovery_flow
        )
        reverse_report_contract = node[
            node.index("bool directReverseGearConfirmed") :
            node.index("bool directDriveGearConfirmed")
        ]
        self.assertIn("GearReport::REVERSE", reverse_report_contract)
        self.assertIn("GearReport::REVERSE_2", reverse_report_contract)
        self.assertNotIn("GearReport::PARK", reverse_report_contract)
        self.assertIn("/control/command/actuation_cmd", node)
        self.assertIn(
            "direct_stuck_reverse_brake_deceleration_mps2", node
        )
        self.assertIn("point.acceleration_mps2", node)
        self.assertIn(
            "DirectStuckRecoveryPhase::WAIT_REVERSE_BRAKE", node
        )
        self.assertIn("message->actuation.brake_cmd", node)
        self.assertIn("message.actuation.accel_cmd = 0.0", node)
        self.assertIn("message.actuation.brake_cmd =", node)
        self.assertIn("message.actuation.steer_cmd = 0.0", node)
        self.assertIn("releaseDirectRecoveryBrakeOverride()", node)
        self.assertIn("direct_stuck_positive_brake_receive_sec_", node)
        self.assertIn("direct_stuck_gear_phase_started_sec_", node)
        drive_command = recovery_flow.index("GearCommand::DRIVE", drive_wait)
        drive_confirmation = recovery_flow.index(
            "directDriveGearConfirmed(now_sec)", drive_command
        )
        forward_alignment = recovery_flow.index(
            "direct_stuck_recovery_waiting_forward_alignment",
            drive_confirmation,
        )
        self.assertLess(drive_command, drive_confirmation)
        self.assertLess(drive_confirmation, forward_alignment)

        bag_config = read(
            AICHALLENGE_ROOT
            / "workspace/src/aichallenge_tools/"
            "bag_manager_py/config/bag_manager.param.yaml"
        )
        self.assertIn("- /control/command/gear_cmd", bag_config)
        self.assertIn("- /vehicle/status/gear_status", bag_config)

        autostart_bag_config = read(
            SUBMIT_ROOT.parent
            / "aichallenge_system/autostart_orchestrator_py/config/"
            "autostart_orchestrator.param.yaml"
        )
        self.assertIn("- /control/command/gear_cmd", autostart_bag_config)
        self.assertIn("- /vehicle/status/gear_status", autostart_bag_config)

        headless_autostart = (
            AICHALLENGE_ROOT.parent
            / "tools/scripts/headless_overrides/aichallenge/workspace/src/"
            "aichallenge_system/autostart_orchestrator_py/config/"
            "autostart_orchestrator.param.yaml"
        )
        if headless_autostart.is_file():
            headless_bag_config = read(headless_autostart)
            self.assertIn("- /control/command/gear_cmd", headless_bag_config)
            self.assertIn("- /vehicle/status/gear_status", headless_bag_config)

    def test_direct_route_uses_steering_passthrough_end_to_end(self) -> None:
        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        planner_include = next(
            include
            for include in direct.findall("include")
            if "state_lattice_overtake_planner.launch.xml"
            in include.attrib.get("file", "")
        )
        planner_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in planner_include.findall("arg")
        }
        self.assertEqual(
            planner_values["pure_pursuit_steering_passthrough_enabled"],
            "true",
        )
        cma_include = next(
            include
            for include in direct.findall("include")
            if "cma_pure_pursuit.launch.xml" in include.attrib.get("file", "")
        )
        cma_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in cma_include.findall("arg")
        }
        self.assertEqual(
            cma_values["steering_command_passthrough_enabled"], "true"
        )

    def test_reverse_trajectory_uses_awsim_gear_relative_vehicle_command(self) -> None:
        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        cma_include = next(
            include
            for include in direct.findall("include")
            if "cma_pure_pursuit.launch.xml" in include.attrib.get("file", "")
        )
        cma_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in cma_include.findall("arg")
        }
        self.assertEqual(
            cma_values["gear_relative_reverse_command_enabled"], "true"
        )
        self.assertEqual(
            cma_values["input_gear_status"], "/vehicle/status/gear_status"
        )

        cma_launch = ET.fromstring(
            read(LAUNCH_ROOT / "control/cma_pure_pursuit.launch.xml")
        )
        defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in cma_launch.findall("arg")
        }
        self.assertEqual(
            defaults["gear_relative_reverse_command_enabled"], "false"
        )

        cma_source = read(
            SUBMIT_ROOT / "cma_pure_pursuit/src/simple_pure_pursuit.cpp"
        )
        steering_direction = cma_source.index("steeringForTravelDirection(")
        vehicle_conversion = cma_source.index(
            "applyGearRelativeReverseCommand("
        )
        command_publish = cma_source.index(
            "pub_cmd_->publish(command)", vehicle_conversion
        )
        self.assertLess(steering_direction, vehicle_conversion)
        self.assertLess(vehicle_conversion, command_publish)

    def test_direct_trajectory_flows_from_planner_through_cma_to_vehicle(self) -> None:
        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        cma_include = next(
            include
            for include in direct.findall("include")
            if "cma_pure_pursuit.launch.xml" in include.attrib.get("file", "")
        )
        cma_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in cma_include.findall("arg")
        }
        self.assertEqual(
            cma_values["input_trajectory"], "$(var direct_trajectory_topic)"
        )
        self.assertEqual(
            cma_values["input_direct_trajectory_command"],
            "$(var mppi_trajectory_command_topic)",
        )
        mppi_include = next(
            include
            for include in direct.findall("include")
            if "reference_space_mppi.launch.xml"
            in include.attrib.get("file", "")
        )
        mppi_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in mppi_include.findall("arg")
        }
        self.assertEqual(
            mppi_values["input_trajectory_command"],
            "$(var direct_trajectory_command_topic)",
        )
        self.assertEqual(
            mppi_values["output_trajectory_command"],
            "$(var mppi_trajectory_command_topic)",
        )
        self.assertEqual(cma_values["use_atomic_direct_trajectory_command"], "true")
        self.assertEqual(
            cma_values["output_control_cmd"], "$(var unboosted_control_topic)"
        )
        bridge = next(
            node
            for node in direct.findall("node")
            if node.attrib.get("exec") == "boost_command_bridge_node"
        )
        remaps = {
            remap.attrib.get("from"): remap.attrib.get("to")
            for remap in bridge.findall("remap")
        }
        self.assertEqual(
            remaps["input/control_cmd"], "$(var unboosted_control_topic)"
        )
        self.assertEqual(
            remaps["output/boost_command"], "/boost_commander/command"
        )
        bridge_params = {
            param.attrib.get("name"): param.attrib.get("value")
            for param in bridge.findall("param")
        }
        self.assertEqual(
            bridge_params["boost_acceleration_mps2"],
            "$(var boost_acceleration_mps2)",
        )
        planner_include = next(
            include
            for include in direct.findall("include")
            if "state_lattice_overtake_planner.launch.xml"
            in include.attrib.get("file", "")
        )
        planner_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in planner_include.findall("arg")
        }
        self.assertEqual(
            planner_args["reference_zero_boost_overtake_enabled"], "true"
        )
        self.assertEqual(
            planner_args["reference_zero_boost_max_forward_gap_m"], "12.0"
        )
        self.assertEqual(
            planner_args["reference_zero_boost_dynamic_max_forward_gap_m"],
            "20.0",
        )
        self.assertEqual(
            planner_args["overtake_boost_max_predicted_pass_time_sec"], "5.0"
        )
        self.assertEqual(planner_args["boost_intent_charge_count"], "1")
        self.assertEqual(
            planner_args["overtake_boost_acceleration_mps2"],
            "$(var boost_acceleration_mps2)",
        )
        self.assertTrue(
            any(
                node.attrib.get("exec") == "boost_commander"
                for node in direct.findall("node")
            )
        )

    def test_direct_trajectory_has_only_trajectory_rviz_consumers(self) -> None:
        for name in ("autoware.rviz", "autoware_vehicle.rviz"):
            source = read(RVIZ_CONFIG_ROOT / name)
            topic_index = source.index(
                "Value: /state_lattice/direct/trajectory"
            )
            display_start = source.rfind("- Class:", 0, topic_index)
            display = source[display_start:topic_index]
            self.assertIn("Class: rviz_plugins/Trajectory", display)
            self.assertNotIn("Class: rviz_default_plugins/Path", display)

    def test_direct_route_limits_cma_curvature_feedforward_to_maneuvers(self) -> None:
        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        cma_include = next(
            include
            for include in direct.findall("include")
            if "cma_pure_pursuit.launch.xml" in include.attrib.get("file", "")
        )
        values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in cma_include.findall("arg")
        }
        self.assertEqual(values["curvature_feedforward_enabled"], "true")
        self.assertEqual(values["curvature_feedforward_gain"], "0.60")
        self.assertEqual(
            values["curvature_feedforward_preview_distance_m"], "3.0"
        )
        self.assertEqual(
            values["curvature_feedforward_time_constant_sec"], "0.20"
        )
        self.assertEqual(values["curvature_feedforward_maneuver_only"], "true")
        self.assertEqual(
            values["maneuver_lookahead_uses_measured_speed"], "true"
        )
        self.assertEqual(
            values["maneuver_lookahead_time_constant_sec"], "0.20"
        )
        self.assertEqual(values["delay_compensation_enabled"], "true")
        self.assertEqual(
            values["hard_steering_angle_limit_rad"], "0.3665191429188092"
        )
        self.assertEqual(values["hard_steering_rate_limit_radps"], "4.0")
        self.assertEqual(values["steering_time_constant_sec"], "0.30")
        self.assertEqual(values["lateral_error_speed_gate_enabled"], "true")
        self.assertEqual(values["input_overtake_mode"], "/debug/overtake/mode")

        cma_mode_contract = read(
            SUBMIT_ROOT
            / "cma_pure_pursuit/include/simple_pure_pursuit/curvature_feedforward_mode.hpp"
        )
        maneuver_modes = braced_block(
            cma_mode_contract, "isCurvatureFeedforwardManeuverMode"
        )
        for mode in (
            "PREPARE_OVERTAKE_LEFT",
            "PREPARE_OVERTAKE_RIGHT",
            "OVERTAKE_LEFT",
            "OVERTAKE_RIGHT",
        ):
            self.assertIn(f'\"{mode}\"', maneuver_modes)
        self.assertNotIn('"MERGE_BACK"', maneuver_modes)
        self.assertNotIn('"FREE_RUN"', maneuver_modes)
        self.assertNotIn('"FOLLOW_BLOCKED"', maneuver_modes)
        self.assertNotIn('"YIELD_BEHIND"', maneuver_modes)
        cma_source = read(
            SUBMIT_ROOT / "cma_pure_pursuit/src/simple_pure_pursuit.cpp"
        )
        self.assertIn(
            "overtake_mode_fresh && "
            "isCurvatureFeedforwardManeuverMode(overtake_mode_)",
            cma_source,
        )
        self.assertIn("estimateDistanceWindowSignedCurvature(", cma_source)
        self.assertIn("timeDomainLowPass(", cma_source)
        self.assertIn("measured_control_dt_sec", cma_source)
        self.assertIn("selectLookaheadSpeedBasis(", cma_source)
        self.assertIn("smoothManeuverLookaheadDistance(", cma_source)
        self.assertIn("raw_lookahead_distance_m", cma_source)
        self.assertIn("lookahead_speed_basis_mps", cma_source)

    def test_direct_nonfollow_modes_without_candidate_publish_stationary_hold(self) -> None:
        node_source = read(
            STATE_LATTICE_ROOT
            / "src/state_lattice_overtake_planner_node.cpp"
        )
        publisher = braced_block(node_source, "void publishDirectTrajectory")
        hold_condition = publisher[
            publisher.index("const bool hold_corridor_without_geometry") :
            publisher.index("if (hold_corridor_without_geometry)")
        ]
        self.assertIn("direct_geometry == nullptr", hold_condition)
        self.assertNotIn("output.mode == BehaviorMode::FOLLOW_BLOCKED", hold_condition)
        self.assertIn("output.mode == BehaviorMode::MERGE_BACK", hold_condition)
        hold_body = braced_block(publisher, "if (hold_corridor_without_geometry)")
        self.assertEqual(hold_body.count("append(ego_.x, ego_.y, ego_.yaw, 0.0)"), 2)
        self.assertNotIn("hold_anchor.d", hold_body)
        self.assertNotIn("planner_speed_limit_mps", hold_body)
        self.assertIn(
            "FREE_RUN and FOLLOW_BLOCKED without selected geometry",
            publisher,
        )
        self.assertIn("validatedOutputReferenceExecution", node_source)
        self.assertIn("direct_high_speed_fallback_threshold_mps_", node_source)

    def test_direct_stuck_gear_command_uses_awsim_transient_local_qos(self) -> None:
        node_source = read(
            STATE_LATTICE_ROOT
            / "src/state_lattice_overtake_planner_node.cpp"
        )
        self.assertIn(
            "const auto gear_command_qos =\n"
            "        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();",
            node_source,
        )
        gear_publisher = node_source[
            node_source.index("direct_stuck_gear_command_pub_ =") :
            node_source.index("direct_stuck_gear_status_sub_ =")
        ]
        self.assertIn("gear_command_qos", gear_publisher)
        self.assertNotIn("control_qos", gear_publisher)

    def test_direct_candidate_is_not_extended_by_unchecked_frenet_offset(self) -> None:
        node_source = read(
            STATE_LATTICE_ROOT
            / "src/state_lattice_overtake_planner_node.cpp"
        )
        publisher = braced_block(node_source, "void publishDirectTrajectory")
        selected_branch = braced_block(
            publisher,
            "if (direct_geometry != nullptr &&",
        )
        self.assertIn("publishDirectTrajectoryMessage(message, output)", selected_branch)
        self.assertIn("return", selected_branch)
        self.assertNotIn("output_frame_.interpolate", selected_branch)

    def test_direct_route_releases_merge_at_route_specific_lateral_error(self) -> None:
        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        planner_include = next(
            include
            for include in direct.findall("include")
            if "state_lattice_overtake_planner.launch.xml"
            in include.attrib.get("file", "")
        )
        values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in planner_include.findall("arg")
        }
        self.assertEqual(values["free_run_return_required_cycles"], "1")
        self.assertEqual(values["return_lateral_error_m"], "1.00")
        self.assertEqual(values["direct_outer_speed_limit_mps"], "9.0")
        self.assertEqual(values["direct_merge_back_speed_limit_mps"], "6.0")
        self.assertEqual(values["direct_merge_back_distance_m"], "20.0")
        self.assertEqual(values["direct_merge_back_duration_sec"], "3.0")
        self.assertEqual(values["direct_merge_back_step_m"], "0.50")
        self.assertEqual(values["max_steer_rate_radps"], "4.0")

    def test_preventive_side_role_is_enabled_only_on_direct_route(self) -> None:
        parameter = "direct_overtake_stabilization_enabled"
        planner_launch_path = (
            STATE_LATTICE_ROOT
            / "launch/state_lattice_overtake_planner.launch.xml"
        )
        planner_launch = ET.fromstring(read(planner_launch_path))
        planner_arg = next(
            arg
            for arg in planner_launch.findall("arg")
            if arg.attrib.get("name") == parameter
        )
        self.assertEqual(planner_arg.attrib.get("default"), "false")
        planner_node = planner_launch.find("node")
        self.assertIsNotNone(planner_node)
        planner_param = next(
            param
            for param in planner_node.findall("param")
            if param.attrib.get("name") == parameter
        )
        self.assertEqual(planner_param.attrib.get("value"), f"$(var {parameter})")

        direct_path = (
            LAUNCH_ROOT / "control/state_lattice_pure_pursuit_direct.launch.xml"
        )
        direct = ET.fromstring(read(direct_path))
        direct_planner_include = next(
            include
            for include in direct.findall("include")
            if "state_lattice_overtake_planner.launch.xml"
            in include.attrib.get("file", "")
        )
        direct_value = next(
            arg.attrib.get("value")
            for arg in direct_planner_include.findall("arg")
            if arg.attrib.get("name") == parameter
        )
        self.assertEqual(direct_value, "true")

        for path in LAUNCH_ROOT.rglob("*.xml"):
            if path == direct_path:
                continue
            root = ET.fromstring(read(path))
            for arg in root.iter("arg"):
                if arg.attrib.get("name") == parameter:
                    self.assertNotEqual(arg.attrib.get("value"), "true", msg=str(path))

        planner_core = read(
            STATE_LATTICE_ROOT / "src/lattice_planner.cpp"
        )
        self.assertIn(
            "if (!config_.direct_overtake_stabilization_enabled)",
            planner_core,
        )

    def test_rear_traffic_immunity_is_direct_only_and_not_yaml_overridden(
        self,
    ) -> None:
        parameter = "ignore_rear_hard_cost_in_free_follow"
        planner_launch = ET.fromstring(
            read(
                STATE_LATTICE_ROOT
                / "launch/state_lattice_overtake_planner.launch.xml"
            )
        )
        defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in planner_launch.findall("arg")
        }
        self.assertEqual(defaults[parameter], "false")

        direct = ET.fromstring(
            read(
                LAUNCH_ROOT
                / "control/state_lattice_pure_pursuit_direct.launch.xml"
            )
        )
        planner_include = next(
            include
            for include in direct.findall("include")
            if "state_lattice_overtake_planner.launch.xml"
            in include.attrib.get("file", "")
        )
        direct_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in planner_include.findall("arg")
        }
        self.assertEqual(direct_values[parameter], "true")

        # ROS launch parameter files override duplicate scalar parameters even
        # when a named <param> appears later in the node action. Keeping this
        # key in the common YAML silently changed the direct route back to
        # false at runtime, so the absence itself is a required contract.
        common_parameters = yaml.safe_load(read(STATE_LATTICE_PARAMS))
        node_parameters = common_parameters[
            "state_lattice_overtake_planner_node"
        ]["ros__parameters"]
        self.assertNotIn(parameter, node_parameters)

    def test_v2_final_fence_is_private_diagnostic_and_default_off(self) -> None:
        parameter = "state_lattice_v2_final_fence_enabled"
        node_source = read(STATE_LATTICE_NODE)
        self.assertRegex(
            node_source,
            rf'declare_parameter<bool>\(\s*"{parameter}",\s*false\)',
        )
        planner_launch = STATE_LATTICE_ROOT / "launch/state_lattice_overtake_planner.launch.xml"
        root = ET.fromstring(read(planner_launch))
        defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in root.findall("arg")
        }
        self.assertEqual(defaults[parameter], "false")
        self.assertIn("/test/m4/state_lattice/v2_quiesce", node_source)
        self.assertIn("/test/m4/state_lattice/v2_final_fence", node_source)
        self.assertRegex(
            node_source,
            r"final_fence_admission\s*=\s*"
            r"state_lattice_v2_final_fence_enabled_\s*\?\s*"
            r"state_lattice_v2_final_fence_state_->tryAdmit",
        )
        for path in LAUNCH_ROOT.rglob("*.xml"):
            self.assertNotIn(parameter, read(path), msg=str(path))
        forbidden_roots = V2_FORBIDDEN_AUTHORITY_CONSUMER_ROOTS + (
            SUBMIT_ROOT / "simple_pure_pursuit",
        )
        for package_root in forbidden_roots:
            for path in package_root.rglob("*"):
                if path.is_file() and path.suffix in {".cpp", ".hpp", ".py", ".xml"}:
                    text = read(path)
                    self.assertNotIn("/test/m4/state_lattice/v2_quiesce", text)
                    self.assertNotIn("/test/m4/state_lattice/v2_final_fence", text)

    def test_exact_spatial_generation_is_not_live_publication_authority(self) -> None:
        params = yaml.safe_load(read(STATE_LATTICE_PARAMS))
        node_params = params["state_lattice_overtake_planner_node"]["ros__parameters"]
        self.assertNotIn(
            "experimental_exact_spatial_follow_shadow_enabled",
            node_params,
        )
        node_source = read(
            STATE_LATTICE_ROOT / "src/state_lattice_overtake_planner_node.cpp"
        )
        self.assertRegex(
            node_source,
            r'declare_parameter<bool>\(\s*'
            r'"experimental_exact_spatial_follow_shadow_enabled",\s*true\)',
        )
        self.assertRegex(
            node_source,
            r'declare_parameter<bool>\(\s*'
            r'"experimental_spatial_reference_override_live_publish_enabled",\s*'
            r'false\)',
        )

        launch = ET.fromstring(
            read(STATE_LATTICE_ROOT / "launch/state_lattice_overtake_planner.launch.xml")
        )
        args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in launch.findall("arg")
        }
        self.assertEqual(
            args["experimental_exact_spatial_follow_shadow_enabled"], "true"
        )
        self.assertEqual(
            args["experimental_spatial_reference_override_live_publish_enabled"],
            "false",
        )
        node = launch.find("node")
        self.assertIsNotNone(node)
        overrides = [
            param
            for param in node.findall("param")
            if param.attrib.get("name")
            == "experimental_exact_spatial_follow_shadow_enabled"
        ]
        self.assertEqual(len(overrides), 1)
        self.assertEqual(
            overrides[0].attrib.get("value"),
            "$(var experimental_exact_spatial_follow_shadow_enabled)",
        )
        live_overrides = [
            param
            for param in node.findall("param")
            if param.attrib.get("name")
            == "experimental_spatial_reference_override_live_publish_enabled"
        ]
        self.assertEqual(len(live_overrides), 1)
        self.assertEqual(
            live_overrides[0].attrib.get("value"),
            "$(var experimental_spatial_reference_override_live_publish_enabled)",
        )

    def test_v2_poc_identity_sideband_is_explicit_default_off_and_route_scoped(
        self,
    ) -> None:
        names = (
            "state_lattice_v2_live_proposal_publish_enabled",
            "state_lattice_v2_live_proposal_accept_enabled",
            "state_lattice_v2_producer_instance_id",
            "state_lattice_v2_pp_producer_instance_id",
            "state_lattice_v2_session_id",
        )
        expected_defaults = {
            names[0]: "false",
            names[1]: "false",
            names[2]: "0",
            names[3]: "",
            names[4]: "",
        }

        submit = ET.fromstring(read(LAUNCH_ROOT / "aichallenge_submit.launch.xml"))
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        for root in (submit, reference):
            defaults = {
                arg.attrib.get("name"): arg.attrib.get("default")
                for arg in root.findall("arg")
            }
            self.assertEqual(
                {name: defaults.get(name) for name in names},
                expected_defaults,
            )

        submit_reference = submit.find("include")
        self.assertIsNotNone(submit_reference)
        submit_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in submit_reference.findall("arg")
        }
        for name in names:
            self.assertEqual(submit_values[name], f"$(var {name})")

        groups = {
            group.attrib.get("if", ""): group
            for group in reference.findall("group")
        }
        state_lattice_group = next(
            group
            for condition, group in groups.items()
            if "state_lattice_pure_pursuit" in condition
        )
        state_lattice_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in state_lattice_group.find("include").findall("arg")
        }
        # The dedicated production route must complete the proposal/base
        # attestation exchange so it can emit paired Plan/SafetyConstraint.
        self.assertEqual(state_lattice_values[names[0]], "true")
        self.assertEqual(state_lattice_values[names[1]], "true")
        for name in names[2:]:
            self.assertEqual(state_lattice_values[name], f"$(var {name})")

        horizon_group = next(
            group
            for condition, group in groups.items()
            if "pure_pursuit_mpc_horizon" in condition
        )
        horizon_names = {
            arg.attrib.get("name")
            for arg in horizon_group.find("include").findall("arg")
        }
        self.assertTrue(set(names).isdisjoint(horizon_names))

        profile = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        profile_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in profile.findall("arg")
        }
        profile_expected_defaults = dict(expected_defaults)
        profile_expected_defaults[names[0]] = "true"
        profile_expected_defaults[names[1]] = "true"
        self.assertEqual(
            {name: profile_defaults.get(name) for name in names},
            profile_expected_defaults,
        )
        route_group = next(
            group
            for group in profile.findall("group")
            if group.attrib.get("if") == "$(var route_valid)"
        )
        route_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in route_group.find("include").findall("arg")
        }
        self.assertEqual(
            {name: route_values[name] for name in names},
            {
                names[0]: "$(var state_lattice_v2_live_proposal_publish_enabled)",
                names[1]: "$(var state_lattice_v2_live_proposal_accept_enabled)",
                names[2]: "4101",
                names[3]: "4201",
                names[4]: "1",
            },
        )

        system = ET.fromstring(read(SYSTEM_LAUNCH))
        system_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in system.findall("arg")
        }
        # The system launch owns only system-wide concerns. V2 defaults and
        # route selection are intentionally delegated to the submission
        # launch, so the parent must neither redeclare nor override them.
        self.assertTrue(set(names).isdisjoint(system_defaults))
        submit_includes = [
            include
            for include in system.iter("include")
            if "aichallenge_submit.launch.xml" in include.attrib.get("file", "")
        ]
        self.assertGreaterEqual(len(submit_includes), 1)
        for submit_include in submit_includes:
            system_values = {
                arg.attrib.get("name"): arg.attrib.get("value")
                for arg in submit_include.findall("arg")
            }
            self.assertTrue(set(names).isdisjoint(system_values))

        horizon = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        state_group = next(
            group
            for group in horizon.findall("group")
            if group.attrib.get("if") == "$(var state_lattice_pure_pursuit_enabled)"
        )
        planner_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in state_group.find("include").findall("arg")
        }
        self.assertEqual(
            planner_values["experimental_spatial_reference_override_live_publish_enabled"],
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )
        pp_include = next(
            include
            for include in horizon.findall("include")
            if "pure_pursuit.launch.xml" in include.attrib.get("file", "")
        )
        pp_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in pp_include.findall("arg")
        }
        self.assertEqual(
            pp_values["state_lattice_v4_poc_identity_gate_enabled"],
            "$(var state_lattice_v4_poc_identity_gate_enabled)",
        )
        self.assertEqual(
            pp_values["state_lattice_v4_poc_command_activation_enabled"],
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )

        # Route identity is owned by the submission launch. Repository-level
        # runners and the system/evaluation wrappers must not reintroduce a
        # second set of V2 defaults through environment passthrough.
        for runner_path in (
            path for path in (RUN_AUTOWARE, RUN_EVALUATION) if path is not None
        ):
            runner = read(runner_path)
            for name in names:
                self.assertNotIn(f"{name}:=", runner)
        if DOCKER_COMPOSE is not None and DOCKER_COMPOSE.is_file():
            compose = read(DOCKER_COMPOSE)
            for name in names:
                self.assertNotIn(name.upper(), compose)

        evaluation = ET.fromstring(read(EVALUATION_LAUNCH))
        evaluation_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in evaluation.findall("arg")
        }
        self.assertTrue(set(names).isdisjoint(evaluation_defaults))
        evaluation_system_includes = [
            include
            for group in evaluation.findall("group")
            for include in group.findall("include")
            if "aichallenge_system.launch.xml" in include.attrib.get("file", "")
        ]
        self.assertGreaterEqual(len(evaluation_system_includes), 1)
        for evaluation_system_include in evaluation_system_includes:
            evaluation_values = {
                arg.attrib.get("name"): arg.attrib.get("value")
                for arg in evaluation_system_include.findall("arg")
            }
            self.assertTrue(set(names).isdisjoint(evaluation_values))

    def test_v2_string_parameters_are_explicit_at_leaf_node_boundaries(
        self,
    ) -> None:
        pure_pursuit = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit.launch.xml")
        )
        pure_pursuit_params = {
            param.attrib.get("name"): param
            for param in pure_pursuit.find("node").findall("param")
        }
        for name in (
            "state_lattice_v2_expected_producer_instance_id",
            "state_lattice_v2_base_attestation_producer_instance_id",
            "state_lattice_v2_base_attestation_session_id",
        ):
            with self.subTest(node="pure_pursuit", name=name):
                self.assertEqual(pure_pursuit_params[name].attrib.get("type"), "str")

        state_lattice = ET.fromstring(
            read(
                STATE_LATTICE_ROOT
                / "launch/state_lattice_overtake_planner.launch.xml"
            )
        )
        state_lattice_params = {
            param.attrib.get("name"): param
            for param in state_lattice.find("node").findall("param")
        }
        for name in (
            "state_lattice_v2_expected_pp_producer_instance_id",
            "state_lattice_v2_expected_pp_session_id",
        ):
            with self.subTest(node="state_lattice", name=name):
                self.assertEqual(state_lattice_params[name].attrib.get("type"), "str")
                self.assertEqual(
                    state_lattice_params[name].attrib.get("value"), f"$(var {name})"
                )

        # The planner producer ID is intentionally numeric, unlike the
        # numeric-looking producer/session identities above.
        self.assertIsNone(
            state_lattice_params[
                "state_lattice_v2_producer_instance_id"
            ].attrib.get("type")
        )

    def test_v4_command_activation_is_dedicated_and_v2_direct_stays_off(
        self,
    ) -> None:
        activation_name = "state_lattice_v4_poc_command_activation_enabled"
        submit = ET.fromstring(read(LAUNCH_ROOT / "aichallenge_submit.launch.xml"))
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        for root in (submit, reference):
            defaults = {
                arg.attrib.get("name"): arg.attrib.get("default")
                for arg in root.findall("arg")
            }
            self.assertEqual(defaults.get(activation_name), "false")

        submit_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in submit.find("include").findall("arg")
        }
        self.assertEqual(submit_values.get(activation_name), f"$(var {activation_name})")
        state_profile_include = next(
            group.find("include")
            for group in reference.findall("group")
            if "state_lattice_pure_pursuit" in group.attrib.get("if", "")
        )
        reference_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in state_profile_include.findall("arg")
        }
        self.assertEqual(
            reference_values.get(activation_name),
            "$(var state_lattice_v4_awsim_poc_enabled)",
        )

        leaf = ET.fromstring(read(LAUNCH_ROOT / "control/pure_pursuit.launch.xml"))
        leaf_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in leaf.findall("arg")
        }
        self.assertEqual(
            leaf_defaults.get("state_lattice_v2_command_activation_enabled"),
            "false",
        )
        self.assertEqual(
            leaf_defaults.get("state_lattice_v4_poc_command_activation_enabled"),
            "false",
        )
        leaf_params = {
            param.attrib.get("name"): param.attrib.get("value")
            for param in leaf.find("node").findall("param")
        }
        self.assertEqual(
            leaf_params.get("state_lattice_v2_command_activation_enabled"),
            "$(var state_lattice_v2_command_activation_enabled)",
        )
        self.assertEqual(
            leaf_params.get("state_lattice_v4_poc_command_activation_enabled"),
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )

        shared = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        shared_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in shared.findall("arg")
        }
        self.assertEqual(
            shared_defaults.get("state_lattice_v2_command_activation_enabled"),
            "false",
        )
        self.assertEqual(
            shared_defaults.get("state_lattice_v4_poc_command_activation_enabled"),
            "false",
        )
        self.assertEqual(
            shared_defaults.get("state_lattice_v4_poc_identity_gate_enabled"),
            "false",
        )

        dedicated = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        dedicated_defaults = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in dedicated.findall("arg")
        }
        self.assertEqual(dedicated_defaults.get(activation_name), "false")
        shared_include = dedicated.find("group/include")
        self.assertIsNotNone(shared_include)
        dedicated_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in shared_include.findall("arg")
        }
        self.assertEqual(
            dedicated_values.get("state_lattice_v2_command_activation_enabled"),
            "false",
        )
        dedicated_identity_gate = dedicated_values.get(
            "state_lattice_v4_poc_identity_gate_enabled"
        )
        self.assertIsNotNone(dedicated_identity_gate)
        self.assertIn(
            "$(var state_lattice_v4_poc_identity_gate_enabled)",
            dedicated_identity_gate,
        )
        self.assertIn(
            "$(var state_lattice_v2_live_proposal_publish_enabled)",
            dedicated_identity_gate,
        )
        self.assertIn(
            "$(var state_lattice_v2_live_proposal_accept_enabled)",
            dedicated_identity_gate,
        )
        self.assertEqual(
            dedicated_values.get("state_lattice_v4_poc_command_activation_enabled"),
            "$(var state_lattice_v4_poc_command_activation_enabled)",
        )
        dedicated_require_safety = dedicated_values.get(
            "require_safety_constraint"
        )
        self.assertEqual(dedicated_require_safety, "true")
        self.assertEqual(
            dedicated_values.get("state_lattice_v2_live_proposal_publish_enabled"),
            "$(var state_lattice_v2_live_proposal_publish_enabled)",
        )
        self.assertEqual(
            dedicated_values.get("state_lattice_v2_live_proposal_accept_enabled"),
            "$(var state_lattice_v2_live_proposal_accept_enabled)",
        )

        require_safety_default = shared_defaults.get("require_safety_constraint")
        self.assertIsNotNone(require_safety_default)
        self.assertIn("$(var use_overtake_planner)", require_safety_default)
        self.assertIn("$(var overtake_control_route)", require_safety_default)
        self.assertIn("current", require_safety_default)
        mux_include = next(
            include
            for include in shared.findall("include")
            if "hybrid_control_mux.launch.xml" in include.attrib.get("file", "")
        )
        mux_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in mux_include.findall("arg")
        }
        self.assertEqual(
            mux_values.get("require_safety_constraint"),
            "$(var require_safety_constraint)",
        )

        pure_pursuit_includes = [
            include
            for include in shared.findall("include")
            if "pure_pursuit.launch.xml" in include.attrib.get("file", "")
        ]
        recovery_include = next(
            include
            for include in pure_pursuit_includes
            if any(
                arg.attrib.get("name") == "node_name"
                and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
                for arg in include.findall("arg")
            )
        )
        recovery_values = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in recovery_include.findall("arg")
        }
        for name in (
            "state_lattice_v2_live_proposal_accept_enabled",
            "state_lattice_v2_command_activation_enabled",
            "state_lattice_v4_poc_identity_gate_enabled",
            "state_lattice_v4_poc_command_activation_enabled",
        ):
            with self.subTest(recovery_arg=name):
                self.assertEqual(recovery_values.get(name), "false")

    def test_state_lattice_debug_layers_are_non_authoritative_publishers(
        self,
    ) -> None:
        node = read(STATE_LATTICE_NODE)
        for topic in (
            "/debug/overtake/planning_geometry",
            "/debug/overtake/trajectory_candidates",
            "/debug/overtake/selected_trajectory",
            "/debug/overtake/opponent_costmap",
            "/debug/overtake/wall_costmap",
        ):
            self.assertIn(f'"{topic}"', node)

        publish_costmap = braced_block(node, "void publishCostmap()")
        self.assertIn("map_.wallLevels()", publish_costmap)
        self.assertIn("objectCostLevel(", publish_costmap)
        self.assertIn("wall_costmap_pub_->publish", publish_costmap)
        self.assertIn("opponent_costmap_pub_->publish", publish_costmap)
        self.assertIn("inputsFresh(stamp.seconds())", publish_costmap)
        self.assertNotIn("override_pub_->publish", publish_costmap)
        self.assertNotIn("instant_control_pub_->publish", publish_costmap)

    def test_rviz_presets_expose_all_state_lattice_debug_layers(self) -> None:
        expected_topics = {
            "/debug/overtake/planning_geometry",
            "/debug/overtake/trajectory_candidates",
            "/debug/overtake/selected_trajectory",
            "/debug/overtake/opponent_costmap",
            "/debug/overtake/wall_costmap",
            "/state_lattice/direct/trajectory",
            "/debug/mppi/baseline",
            "/debug/mppi/candidates",
            "/debug/mppi/selected",
        }

        def all_displays(displays: list[dict]) -> list[dict]:
            flattened: list[dict] = []
            for display in displays:
                flattened.append(display)
                nested = display.get("Displays")
                if isinstance(nested, list):
                    flattened.extend(all_displays(nested))
            return flattened

        for filename in ("autoware.rviz", "autoware_vehicle.rviz"):
            config = yaml.safe_load(read(RVIZ_CONFIG_ROOT / filename))
            displays = all_displays(config["Visualization Manager"]["Displays"])
            state_lattice_group = next(
                display
                for display in displays
                if display.get("Name") == "State Lattice Debug"
            )
            self.assertTrue(state_lattice_group["Enabled"])
            topics = {
                display.get("Topic", {}).get("Value")
                for display in state_lattice_group["Displays"]
            }
            self.assertEqual(topics, expected_topics)
            cost_displays = [
                display
                for display in state_lattice_group["Displays"]
                if display.get("Class") == "rviz_default_plugins/Map"
            ]
            self.assertEqual(len(cost_displays), 2)
            self.assertTrue(
                all(display["Color Scheme"] == "costmap" for display in cost_displays)
            )
            wall_display = next(
                display
                for display in cost_displays
                if display["Topic"]["Value"] == "/debug/overtake/wall_costmap"
            )
            self.assertEqual(
                wall_display["Topic"]["Durability Policy"], "Transient Local"
            )

    def test_planner_config_is_the_route_owner(self) -> None:
        params = yaml.safe_load(read(PLANNER_PARAMS))["overtake_planner_node"][
            "ros__parameters"
        ]
        self.assertIn(
            params["control_route"],
            {
                "current",
                "state_lattice_pure_pursuit",
                "state_lattice_instant_mux",
            },
        )
        self.assertEqual(
            params["control_route"],
            "state_lattice_instant_mux",
        )
        self.assertEqual(params["trajectory_backend"], "current")
        resolver = read(ROUTE_RESOLVER)
        self.assertIn('"state_lattice_instant_mux"', resolver)
        self.assertIn('DEFAULT_ROUTE = "current"', resolver)

        node = read(PLANNER_NODE)
        self.assertRegex(
            node,
            r'declare_parameter<std::string>\(\s*"trajectory_backend",\s*"current"\s*\)',
        )

    def test_awsim_adapter_does_not_own_overtake_control_route(self) -> None:
        awsim_launch = read(AWSIM_MODE_LAUNCH)
        self.assertNotIn("overtake_control_route", awsim_launch)
        self.assertNotIn("state_lattice_instant_mux", awsim_launch)
        self.assertNotIn("/hybrid_control/state_lattice/control_cmd", awsim_launch)

    def test_runtime_route_is_resolved_from_planner_config(self) -> None:
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        self.assertFalse(
            any(
                arg.attrib.get("name") == "overtake_control_route"
                for arg in reference.findall("arg")
            )
        )
        route_let = next(
            item
            for item in reference.findall("let")
            if item.attrib.get("name") == "overtake_control_route"
        )
        self.assertIn("resolve_overtake_control_route", route_let.attrib["value"])
        route_config_lets = [
            item
            for item in reference.findall("let")
            if item.attrib.get("name") == "overtake_control_route_config"
        ]
        self.assertEqual(len(route_config_lets), 2)
        default_route_config = next(
            item
            for item in route_config_lets
            if "overtake_planner.param.yaml" in item.attrib["value"]
        )
        poc_route_config = next(
            item
            for item in route_config_lets
            if "v4_live_poc_route.param.yaml" in item.attrib["value"]
        )
        self.assertIn("overtake_planner.param.yaml", default_route_config.attrib["value"])
        self.assertIn("v4_live_poc_route.param.yaml", poc_route_config.attrib["value"])
        self.assertEqual(
            default_route_config.attrib["if"],
            "$(var overtake_route_required)",
        )
        self.assertIn("overtake_route_required", poc_route_config.attrib["if"])
        self.assertEqual(
            route_let.attrib["if"],
            "$(var overtake_route_required)",
        )
        fallback_route_let = next(
            item
            for item in reference.findall("let")
            if item.attrib.get("name") == "overtake_control_route"
            and item.attrib.get("value") == "current"
        )
        self.assertIn("overtake_route_required", fallback_route_let.attrib["if"])
        effective_let = next(
            item
            for item in reference.findall("let")
            if item.attrib.get("name") == "state_lattice_v4_awsim_poc_enabled"
        )
        self.assertIn("state_lattice_v4_poc_command_activation_enabled", effective_let.attrib["value"])
        self.assertIn("simulation", effective_let.attrib["value"])

        for relative in (
            "control/hybrid_delay_aware_mpc.launch.xml",
            "control/pure_pursuit_mpc_horizon.launch.xml",
        ):
            launch_file = LAUNCH_ROOT / relative
            root = ET.fromstring(read(launch_file))
            args = {
                arg.attrib.get("name"): arg.attrib.get("default")
                for arg in root.findall("arg")
            }
            with self.subTest(launch_file=launch_file):
                self.assertEqual(
                    args.get("overtake_control_route"), "current"
                )
                route_arg = next(
                    arg
                    for arg in root.findall("arg")
                    if arg.attrib.get("name") == "overtake_control_route"
                )
                self.assertEqual(
                    [choice.attrib.get("value") for choice in route_arg.findall("choice")],
                    [
                        "current",
                        "state_lattice_pure_pursuit",
                        "state_lattice_instant_mux",
                    ],
                )
                pp_groups = [
                    group
                    for group in root.findall("group")
                    if group.attrib.get("if")
                    == "$(var state_lattice_pure_pursuit_enabled)"
                ]
                instant_groups = [
                    group
                    for group in root.findall("group")
                    if group.attrib.get("if")
                    == "$(var state_lattice_instant_enabled)"
                ]
                self.assertEqual(len(pp_groups), 1)
                self.assertEqual(len(instant_groups), 1)
                pp_include = pp_groups[0].find("include")
                instant_include = instant_groups[0].find("include")
                self.assertIsNotNone(pp_include)
                self.assertIsNotNone(instant_include)
                self.assertIn(
                    "state_lattice_overtake_planner",
                    pp_include.attrib.get("file", ""),
                )
                self.assertIn(
                    "simple_state_lattice_planner",
                    instant_include.attrib.get("file", ""),
                )
                pp_args = {
                    arg.attrib.get("name"): arg.attrib.get("value")
                    for arg in pp_include.findall("arg")
                }
                instant_args = {
                    arg.attrib.get("name"): arg.attrib.get("value")
                    for arg in instant_include.findall("arg")
                }
                self.assertEqual(pp_args["instant_control_enabled"], "false")
                self.assertEqual(pp_args["live_control_output_enabled"], "true")
                self.assertEqual(
                    pp_args["output_reference_override"],
                    "/overtake/reference_override",
                )
                self.assertEqual(
                    pp_args["safety_evaluation_enabled"], "true"
                )
                self.assertEqual(
                    instant_args["live_control_output_enabled"], "true"
                )
                self.assertEqual(
                    instant_args["output_control_cmd"],
                    "/hybrid_control/state_lattice/control_cmd",
                )
                self.assertEqual(
                    instant_args["safety_evaluation_enabled"], "true"
                )

    def test_state_lattice_pure_pursuit_profile_has_no_live_mpc(self) -> None:
        reference = ET.fromstring(read(LAUNCH_ROOT / "reference.launch.xml"))
        profile_groups = [
            group
            for group in reference.findall("group")
            if "state_lattice_pure_pursuit" in group.attrib.get("if", "")
        ]
        self.assertEqual(len(profile_groups), 1)
        profile_include = profile_groups[0].find("include")
        self.assertIsNotNone(profile_include)
        self.assertTrue(
            profile_include.attrib["file"].endswith(
                "/launch/control/state_lattice_pure_pursuit.launch.xml"
            )
        )

        profile = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        route_group = next(
            group
            for group in profile.findall("group")
            if group.attrib.get("if") == "$(var route_valid)"
        )
        base_include = route_group.find("include")
        self.assertIsNotNone(base_include)
        profile_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in base_include.findall("arg")
        }
        self.assertEqual(profile_args["mpc_enabled"], "false")
        self.assertEqual(
            profile_args["state_lattice_mpc_health_speed_guard_enabled"],
            "false",
        )
        state_lattice_params = yaml.safe_load(read(STATE_LATTICE_PARAMS))[
            "state_lattice_overtake_planner_node"
        ]["ros__parameters"]
        self.assertNotIn(
            "mpc_health_speed_guard_enabled", state_lattice_params
        )
        self.assertEqual(
            profile_args["require_state_lattice_override_fresh"], "true"
        )

        base = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        delay_groups = [
            group
            for group in base.findall("group")
            if group.attrib.get("if") == "$(var mpc_enabled)"
            and any(
                "delay_aware_mpc.launch.xml"
                in include.attrib.get("file", "")
                for include in group.findall("include")
            )
        ]
        self.assertEqual(len(delay_groups), 1)
        pure_pursuit_include = next(
            include
            for include in base.findall("include")
            if include.attrib.get("file", "").endswith(
                "/launch/control/pure_pursuit.launch.xml"
            )
        )
        pure_pursuit_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in pure_pursuit_include.findall("arg")
        }
        self.assertEqual(
            pure_pursuit_args["use_mpc_predicted_horizon"],
            "$(var mpc_enabled)",
        )
        self.assertEqual(
            pure_pursuit_args["require_overtake_reference_override_fresh"],
            "$(var require_state_lattice_override_fresh)",
        )

    def test_state_lattice_profile_wires_primary_shadow_workers_only(self) -> None:
        profile = ET.fromstring(
            read(LAUNCH_ROOT / "control/state_lattice_pure_pursuit.launch.xml")
        )
        profile_args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in profile.findall("arg")
        }
        self.assertEqual(profile_args["c002ay0_shadow_capture_enabled"], "true")
        self.assertEqual(
            profile_args["state_lattice_source_binding_shadow_enabled"], "true"
        )
        self.assertEqual(
            profile_args["c002ay0_state_lattice_shadow_enabled"], "true"
        )
        for name in (
            "c002ay0_shadow_session_generation",
            "c002ay0_shadow_session_nonce",
            "c002ay0_state_lattice_shadow_session_generation",
            "c002ay0_state_lattice_shadow_session_nonce",
        ):
            self.assertGreater(int(profile_args[name]), 0)
        self.assertIn(
            "c002ay0_shadow_worker",
            profile_args["c002ay0_shadow_worker_path"],
        )
        self.assertIn(
            "c002ay0_state_lattice_shadow_worker",
            profile_args["c002ay0_state_lattice_shadow_worker_path"],
        )

        route_group = next(
            group
            for group in profile.findall("group")
            if group.attrib.get("if") == "$(var route_valid)"
        )
        horizon_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in route_group.find("include").findall("arg")
        }
        for name in (
            "c002ay0_shadow_capture_enabled",
            "state_lattice_source_binding_shadow_enabled",
            "c002ay0_shadow_worker_path",
            "c002ay0_shadow_session_generation",
            "c002ay0_shadow_session_nonce",
            "c002ay0_state_lattice_shadow_enabled",
            "c002ay0_state_lattice_shadow_worker_path",
            "c002ay0_state_lattice_shadow_session_generation",
            "c002ay0_state_lattice_shadow_session_nonce",
        ):
            self.assertEqual(horizon_args[name], "$(var " + name + ")")

        horizon = ET.fromstring(
            read(LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml")
        )
        primary = next(
            include
            for include in horizon.findall("include")
            if include.attrib.get("file", "").endswith(
                "/launch/control/pure_pursuit.launch.xml"
            )
            and not any(
                arg.attrib.get("name") == "node_name" for arg in include.findall("arg")
            )
        )
        primary_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in primary.findall("arg")
        }
        self.assertEqual(
            primary_args["state_lattice_source_binding_shadow_enabled"],
            "$(var state_lattice_source_binding_shadow_enabled)",
        )

        recovery = next(
            include
            for include in horizon.findall("include")
            if any(
                arg.attrib.get("name") == "node_name"
                and arg.attrib.get("value") == "wall_recovery_pure_pursuit_node"
                for arg in include.findall("arg")
            )
        )
        recovery_args = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in recovery.findall("arg")
        }
        self.assertEqual(recovery_args["c002ay0_shadow_worker_path"], "")
        self.assertEqual(recovery_args["c002ay0_shadow_session_generation"], "0")
        self.assertEqual(recovery_args["c002ay0_shadow_session_nonce"], "0")

    def test_container_and_public_launches_cannot_override_route(self) -> None:
        workspace_src = SUBMIT_ROOT.parent
        paths = [
            workspace_src
            / "aichallenge_system"
            / "aichallenge_system_launch"
            / "launch"
            / "aichallenge_system.launch.xml",
            LAUNCH_ROOT / "aichallenge_submit.launch.xml",
        ]
        submodule_root = next(
            (
                parent
                for parent in SUBMIT_ROOT.parents
                if (parent / "docker-compose.yml").is_file()
            ),
            None,
        )
        if submodule_root is not None:
            paths.extend(
                (
                    submodule_root / "Makefile",
                    submodule_root / "docker-compose.yml",
                    submodule_root / "aichallenge" / "run_autoware.bash",
                )
            )
        for path in paths:
            source = read(path)
            with self.subTest(path=path):
                self.assertNotIn("OVERTAKE_CONTROL_ROUTE", source)
                self.assertNotIn("overtake_control_route", source)

    def test_route_resolver_allowlists_and_fails_closed(self) -> None:
        def resolve(route_yaml: str) -> str:
            with tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / "route.yaml"
                config.write_text(route_yaml, encoding="utf-8")
                result = subprocess.run(
                    [sys.executable, str(ROUTE_RESOLVER), str(config)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                return result.stdout.strip()

        valid = """
overtake_planner_node:
  ros__parameters:
    control_route: state_lattice_instant_mux
"""
        invalid = """
overtake_planner_node:
  ros__parameters:
    control_route: unsafe_unknown_route
"""
        missing = """
overtake_planner_node:
  ros__parameters: {}
"""
        self.assertEqual(resolve(valid), "state_lattice_instant_mux")
        self.assertEqual(resolve(invalid), "current")
        self.assertEqual(resolve(missing), "current")

    def test_live_state_lattice_output_fails_closed_if_safety_is_disabled(
        self,
    ) -> None:
        params = read(STATE_LATTICE_PARAMS)
        self.assertNotRegex(params, r"(?m)^\s*live_control_output_enabled:")
        self.assertNotRegex(params, r"(?m)^\s*instant_control_enabled:")
        self.assertNotRegex(params, r"(?m)^\s*safety_evaluation_enabled:")
        node = read(STATE_LATTICE_NODE)
        self.assertIn(
            "live control output requires safety_evaluation_enabled=true",
            node,
        )
        self.assertRegex(
            node,
            r"if\s*\(\s*!live_control_output_enabled_\s*\|\|\s*"
            r"!instant_control_enabled_\s*\)\s*\{\s*return;\s*\}",
        )

    def test_authority_publisher_topics_and_types_remain_single_writer(self) -> None:
        node = read(PLANNER_NODE)
        for topic, message_type in EXPECTED_AUTHORITY_PUBLISHERS.items():
            pattern = re.compile(
                r"create_publisher\s*<\s*"
                + re.escape(message_type)
                + r"\s*>\s*\(\s*\""
                + re.escape(topic)
                + r"\"",
                re.MULTILINE,
            )
            with self.subTest(topic=topic, message_type=message_type):
                self.assertEqual(len(pattern.findall(node)), 1)

    def test_exact_offline_route_replaces_current_authority_owner(self) -> None:
        launch_path = (
            LAUNCH_ROOT
            / "test_only/state_lattice_pp_mux_offline_replay.launch.xml"
        )
        source = read(launch_path)
        self.assertIn(
            'name="state_lattice_authority_publish_enabled" '
            'value="$(var exact_cartesian_enabled)"',
            source,
        )
        self.assertIn(
            'name="state_lattice_authority_tracking_topic" '
            'value="$(var root)/pp/tracking_status"',
            source,
        )
        self.assertIn(
            'name="state_lattice_authority_race_arm_topic" '
            'value="$(var root)/input/race_armed"',
            source,
        )
        self.assertIn('<node if="$(var use_current_authority)"', source)
        self.assertIn("exact_cartesian_enabled)' != 'true", source)
        node_source = read(STATE_LATTICE_NODE)
        self.assertRegex(
            node_source,
            r'declare_parameter<bool>\(\s*'
            r'"state_lattice_authority_publish_enabled",\s*false\)',
        )

    def test_production_exact_route_explicitly_enables_state_authority(self) -> None:
        planner_launch = read(
            STATE_LATTICE_ROOT / "launch/state_lattice_overtake_planner.launch.xml"
        )
        self.assertIn(
            'name="state_lattice_authority_publish_enabled" default="false"',
            planner_launch,
        )
        self.assertIn(
            'name="state_lattice_authority_publish_enabled" '
            'value="$(var state_lattice_authority_publish_enabled)"',
            planner_launch,
        )
        control_launch = read(
            LAUNCH_ROOT / "control/pure_pursuit_mpc_horizon.launch.xml"
        )
        self.assertIn(
            'name="state_lattice_authority_publish_enabled" '
            'value="true"',
            control_launch,
        )

    def test_v2_evidence_topics_have_no_mux_or_mpc_authority_consumer(self) -> None:
        for root in V2_FORBIDDEN_AUTHORITY_CONSUMER_ROOTS:
            source = "\n".join(
                path.read_text(encoding="utf-8", errors="replace")
                for path in root.rglob("*")
                if path.is_file()
                and path.suffix in {".cpp", ".hpp", ".py", ".xml", ".yaml"}
            )
            for topic in V2_EVIDENCE_TOPICS:
                with self.subTest(root=root.name, topic=topic):
                    self.assertNotIn(topic, source)

    def test_backend_validation_accepts_shadow_and_reserved_candidate(self) -> None:
        node = read(PLANNER_NODE)
        marker = 'if (config.trajectory_backend != "current"'
        validation_start = node.index(marker)
        validation = node[validation_start : validation_start + 600]
        self.assertIn('config.trajectory_backend != "state_lattice_shadow"', validation)
        self.assertIn('config.trajectory_backend != "state_lattice_candidate"', validation)

    def test_reserved_candidate_branch_only_warns(self) -> None:
        node = read(PLANNER_NODE)
        candidate_branch = braced_block(
            node, 'if (config.trajectory_backend == "state_lattice_candidate")'
        )
        self.assertIn("RCLCPP_WARN", candidate_branch)
        self.assertRegex(
            candidate_branch,
            r"reserved; live\s+\"\s*\"candidates remain current",
        )
        self.assertNotRegex(
            candidate_branch,
            r"\b(?:StateLatticeShadowAdapter|ShadowGeometryGenerator)\b",
        )
        self.assertNotRegex(candidate_branch, r"\bcore_?\s*(?:->|\.)")

    def test_adapter_and_generator_are_confined_to_core_diagnostic(self) -> None:
        core = read(PLANNER_CORE)
        diagnostic = braced_block(
            core,
            "OvertakePlannerCore::evaluateStateLatticeShadowComparison",
        )
        self.assertIn("StateLatticeShadowAdapter", diagnostic)
        self.assertIn("ShadowGeometryGenerator", diagnostic)
        self.assertEqual(
            core.count("evaluateStateLatticeShadowComparison("),
            2,
            "one definition plus one diagnostic-only update callsite",
        )
        core_without_diagnostic = core.replace(diagnostic, "", 1)
        self.assertNotIn("StateLatticeShadowAdapter", core_without_diagnostic)
        self.assertNotIn("ShadowGeometryGenerator", core_without_diagnostic)

        node = read(PLANNER_NODE)
        self.assertNotIn("StateLatticeShadowAdapter", node)
        self.assertNotIn("ShadowGeometryGenerator", node)
        debug = braced_block(node, "void publishDebug(")
        self.assertIn("state_lattice_shadow_comparison", debug)
        self.assertNotIn(
            "state_lattice_shadow_comparison",
            node.replace(debug, "", 1),
            "shadow record may only be serialized by the debug path",
        )


if __name__ == "__main__":
    unittest.main()
