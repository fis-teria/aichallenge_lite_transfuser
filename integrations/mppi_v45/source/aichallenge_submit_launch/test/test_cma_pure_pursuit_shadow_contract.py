#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / "launch" / "control"


class CmaPurePursuitShadowLaunchContractTest(unittest.TestCase):
    def test_all_controller_outputs_are_shadow_namespaced(self):
        root = ET.parse(
            LAUNCH_DIR / "cma_pure_pursuit_shadow.launch.xml"
        ).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("value")
            for element in root.findall(".//arg")
            if "value" in element.attrib
        }
        wrapper_defaults = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("./arg")
        }

        self.assertEqual(arguments["node_name"], "cma_pure_pursuit_shadow")
        self.assertEqual(
            arguments["input_trajectory"],
            "$(var input_trajectory)",
        )
        self.assertEqual(
            wrapper_defaults["input_trajectory"],
            "/control/shadow/cma_pp/trajectory",
        )
        self.assertEqual(
            arguments["input_baseline_trajectory"],
            "$(var input_baseline_trajectory)",
        )
        self.assertEqual(
            arguments["input_path_source"],
            "$(var input_path_source)",
        )
        self.assertEqual(arguments["split_trajectory_inputs_enabled"], "false")
        self.assertEqual(
            arguments["curvature_feedforward_maneuver_only"], "false"
        )
        self.assertEqual(arguments["wheel_base"], "1.087")
        self.assertEqual(arguments["lookahead_gain"], "0.20")
        self.assertEqual(arguments["lookahead_min_distance"], "2.0")
        self.assertEqual(
            arguments["curvature_lookahead_min_distance"], "2.0"
        )
        self.assertEqual(
            arguments["steering_command_passthrough_enabled"], "false"
        )
        self.assertEqual(arguments["hard_steering_angle_limit_rad"], "0.64")
        self.assertEqual(
            wrapper_defaults["output_control_cmd"],
            "/control/shadow/cma_pp/control_cmd",
        )
        for name in (
            "output_control_cmd",
            "output_raw_control_cmd",
            "output_debug",
            "output_lookahead_point",
            "output_near_lookahead_point",
        ):
            self.assertTrue(arguments[name].startswith("$(var "))
        self.assertEqual(arguments["diagnostic_trace_enabled"], "true")
        self.assertEqual(
            arguments["diagnostic_trace_maneuver_only"], "true"
        )
        self.assertEqual(arguments["diagnostic_trace_period_sec"], "0.02")
        self.assertEqual(
            arguments["continuous_preview_interpolation_enabled"], "true"
        )
        self.assertEqual(arguments["actual_lookahead_distance_blend"], "1.0")
        self.assertEqual(arguments["runtime_timing_metrics_enabled"], "true")
        self.assertEqual(arguments["runtime_timing_report_period_sec"], "1.0")
        self.assertEqual(arguments["runtime_timing_deadline_sec"], "0.033333333")

    def test_interpolation_blend_and_timing_are_wrapper_only(self):
        base_root = ET.parse(
            LAUNCH_DIR / "cma_pure_pursuit.launch.xml"
        ).getroot()
        expected_base_defaults = {
            "continuous_preview_interpolation_enabled": "false",
            "actual_lookahead_distance_blend": "0.0",
            "runtime_timing_metrics_enabled": "false",
        }
        for name, expected_default in expected_base_defaults.items():
            base_arg = base_root.find(f"./arg[@name='{name}']")
            self.assertIsNotNone(base_arg)
            self.assertEqual(base_arg.attrib["default"], expected_default)

        direct_root = ET.parse(
            LAUNCH_DIR / "state_lattice_pure_pursuit_direct.launch.xml"
        ).getroot()
        for name in expected_base_defaults:
            direct_overrides = direct_root.findall(f".//arg[@name='{name}']")
            self.assertEqual(direct_overrides, [], f"unexpected override: {name}")

    def test_mpc_launch_uses_one_switch_for_producer_and_shadow_node(self):
        root = ET.parse(LAUNCH_DIR / "mpc.launch.xml").getroot()
        shadow_arg = root.find("./arg[@name='cpp_cma_shadow_enabled']")
        self.assertIsNotNone(shadow_arg)
        self.assertEqual(shadow_arg.attrib["default"], "true")

        shadow_param = root.find(
            ".//param[@name='cpp_cma_shadow_enabled']"
        )
        self.assertIsNotNone(shadow_param)
        self.assertEqual(
            shadow_param.attrib["value"], "$(var cma_runtime_enabled)"
        )
        curve_speed_arg = root.find(
            "./arg[@name='cpp_cma_curve_target_speed_mps']"
        )
        self.assertIsNotNone(curve_speed_arg)
        self.assertEqual(curve_speed_arg.attrib["default"], "10.0")
        curve_speed_param = root.find(
            ".//param[@name='cpp_cma_curve_target_speed_mps']"
        )
        self.assertIsNotNone(curve_speed_param)
        self.assertEqual(
            curve_speed_param.attrib["value"],
            "$(var cpp_cma_curve_target_speed_mps)",
        )
        shadow_group = root.find(
            ".//group[@if='$(var cma_runtime_enabled)']"
        )
        self.assertIsNotNone(shadow_group)
        shadow_include = shadow_group.find("./include")
        self.assertIsNotNone(shadow_include)
        shadow_arguments = {
            element.attrib["name"]: element.attrib.get("value")
            for element in shadow_include.findall("./arg")
        }
        self.assertNotIn("input_baseline_trajectory", shadow_arguments)

        mux_arg = root.find(
            "./arg[@name='lateral_longitudinal_shadow_mux_enabled']"
        )
        self.assertIsNotNone(mux_arg)
        self.assertEqual(mux_arg.attrib["default"], "true")
        mux_group = shadow_group.find(
            ".//group[@if='$(var mux_runtime_enabled)']"
        )
        self.assertIsNotNone(mux_group)
        mux_include = mux_group.find("./include")
        self.assertIsNotNone(mux_include)
        self.assertIn(
            "hybrid_control_mux)/launch/"
            "lateral_longitudinal_shadow_mux.launch.xml",
            mux_include.attrib["file"],
        )
        mux_arguments = {
            element.attrib["name"]: element.attrib.get("value")
            for element in mux_include.findall("./arg")
        }
        self.assertEqual(
            mux_arguments["input_mpc_cmd"], "$(var mpc_control_command_topic)"
        )
        self.assertEqual(
            mux_arguments["input_cma_cmd"],
            "/control/internal/cma_pp/control_cmd",
        )
        self.assertEqual(
            mux_arguments["output_control_cmd"],
            "$(var mux_output_control_command_topic)",
        )
        self.assertEqual(
            mux_arguments["authority_enabled"],
            "$(var cma_mux_authority_enabled)",
        )

    def test_authority_mode_separates_mpc_raw_and_mux_production_topics(self):
        root = ET.parse(LAUNCH_DIR / "mpc.launch.xml").getroot()
        authority_arg = root.find("./arg[@name='cma_mux_authority_enabled']")
        self.assertIsNotNone(authority_arg)
        self.assertEqual(authority_arg.attrib["default"], "false")

        lets = [
            element
            for element in root.findall("./let")
            if element.attrib.get("if") == "$(var cma_mux_authority_enabled)"
        ]
        authority_values = {
            element.attrib["name"]: element.attrib["value"] for element in lets
        }
        self.assertEqual(
            authority_values["mpc_control_command_topic"],
            "/control/internal/mpc/control_cmd",
        )
        self.assertEqual(
            authority_values["mux_output_control_command_topic"],
            "/control/command/control_cmd",
        )

    def test_mpc_launch_has_no_legacy_overtake_connector_parameters(self):
        root = ET.parse(LAUNCH_DIR / "mpc.launch.xml").getroot()
        argument_names = {
            element.attrib["name"] for element in root.findall("./arg")
        }
        parameter_names = {
            element.attrib["name"] for element in root.findall(".//param")
        }
        removed_names = {
            "dynamic_avoidance_shadow_enabled",
            "dynamic_avoidance_connector_authority_enabled",
            "dynamic_avoidance_connector_deadline_sec",
            "wall_line_23kmh_shadow_enabled",
            "wall_line_23kmh_overtake_enabled",
            "wall_line_shadow_speed_kmh",
        }
        self.assertTrue(argument_names.isdisjoint(removed_names))
        self.assertTrue(parameter_names.isdisjoint(removed_names))


if __name__ == "__main__":
    unittest.main()
