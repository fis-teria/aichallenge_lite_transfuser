"""Resolve the deployed XML with ROS 2 Humble without starting ROS processes."""
from __future__ import annotations

import hashlib
from pathlib import Path
import unittest

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


REFERENCE = Path(__file__).resolve().parents[1] / "launch/reference.launch.xml"
SPEED_KEYS = (
    "brain.cruise_speed_mps",
    "brain.overtake_speed_mps",
    "brain.minimum_proximity_speed_mps",
)


def resolve_profile(profile: str, method: str = "mppi", ceiling: float = 10.0,
                    simulation: bool = True) -> dict:
    """Resolve actual include arguments and node parameters; never execute a node."""
    context = LaunchContext()
    context.launch_configurations.update(
        simulation=str(simulation).lower(), use_sim_time=str(simulation).lower(), control_method=method,
        mppi_reference_profile=profile,
        reference_execution_speed_cap_mps=str(ceiling),
    )
    resolved: dict = {}
    safe_actions = {
        "DeclareLaunchArgument", "SetLaunchConfiguration", "GroupAction",
        "PushLaunchConfigurations", "PopLaunchConfigurations",
        "PushEnvironment", "PopEnvironment", "PushRosNamespace",
    }

    def visit(entity: object) -> None:
        condition = getattr(entity, "condition", None)
        if condition is not None and not condition.evaluate(context):
            return
        if isinstance(entity, LaunchDescription):
            for child in entity.entities:
                visit(child)
        elif isinstance(entity, Node):
            package = perform_substitutions(context, entity._Node__package)
            if package not in ("simple_trajectory_generator", "reference_space_mppi_planner",
                               "mppi_recovery_controller"):
                return
            # Humble's expansion method writes temporary parameter YAML only.
            # Node.execute() is deliberately never called by this test.
            entity._perform_substitutions(context)
            params: dict = {}
            paths: list[str] = []
            for filename, is_file in entity._Node__expanded_parameter_arguments:
                if not is_file:
                    raise AssertionError("Expected ROS parameter files")
                paths.append(filename)
                document = yaml.safe_load(Path(filename).read_text())
                for node_params in document.values():
                    if isinstance(node_params, dict):
                        params.update(node_params.get("ros__parameters", {}))
            resolved[package] = {"params": params, "files": paths}
        elif isinstance(entity, IncludeLaunchDescription):
            location = perform_substitutions(
                context, normalize_to_list_of_substitutions(
                    entity.launch_description_source._LaunchDescriptionSource__location))
            if location.endswith(("/control/mppi.launch.xml", "/reference_space_mppi.launch.xml")):
                for child in entity.execute(context) or []:
                    visit(child)
        elif type(entity).__name__ in safe_actions:
            for child in entity.execute(context) or []:
                visit(child)

    visit(AnyLaunchDescriptionSource(str(REFERENCE)).get_launch_description(context))
    return resolved


class TestMppiCmaReferenceProfiles(unittest.TestCase):
    def test_simulation_flag_reaches_both_awsim_consumers(self) -> None:
        for simulation in (False, True):
            for profile in ("race", "normal", "leader", "legacy"):
                with self.subTest(simulation=simulation, profile=profile):
                    result = resolve_profile(profile, simulation=simulation)
                    for package in ("simple_trajectory_generator", "mppi_recovery_controller"):
                        self.assertEqual(result[package]["params"]["simulation"], simulation)

    def test_race_uses_first_lap_normal_and_leader_routes(self) -> None:
        result = resolve_profile("race")
        trajectory = result["simple_trajectory_generator"]["params"]
        self.assertTrue(trajectory["race_reference.enabled"])
        self.assertFalse(trajectory["dual_reference.enabled"])
        for key, filename in (
            ("csv_path", "in_corce_line.csv"),
            ("dual_reference.lap1_csv_path", "in_corce_line.csv"),
            ("dual_reference.lap2plus_csv_path", "mppi_cma_normal35_20260913.csv"),
            ("race_reference.leader_csv_path", "mppi_cma_leader75_20260913.csv"),
        ):
            self.assertEqual(Path(trajectory[key]).name, filename)
            self.assertTrue(Path(trajectory[key]).is_file())
        for key in SPEED_KEYS:
            self.assertAlmostEqual(result["reference_space_mppi_planner"]["params"][key], 35 / 3.6)

    def assert_profile(self, profile: str, target_mps: float, reference_sha256: str) -> None:
        result = resolve_profile(profile)
        trajectory = result["simple_trajectory_generator"]["params"]
        self.assertAlmostEqual(trajectory["execution_profile.max_speed_mps"], target_mps)
        self.assertFalse(trajectory["dual_reference.enabled"])
        self.assertFalse(trajectory["race_reference.enabled"])
        self.assertEqual(hashlib.sha256(Path(trajectory["csv_path"]).read_bytes()).hexdigest(),
                         reference_sha256)
        planner = result["reference_space_mppi_planner"]["params"]
        for key in SPEED_KEYS:
            self.assertAlmostEqual(planner[key], target_mps)
        self.assertAlmostEqual(planner["steering_acceleration_hold_maximum_acceleration_mps2"], 0.6)

    def test_normal_uses_repeatedly_validated_35_kmh_assets(self) -> None:
        self.assert_profile("normal", 35.0 / 3.6,
                            "9890ede856055dad3602386bd74f66c4abcb3ddd5f1e9566eb9cb69267c01e69")

    def test_leader_uses_repeatedly_validated_7_5_mps_assets(self) -> None:
        self.assert_profile("leader", 7.5,
                            "c08b99cc68b81ae2174f8e8deb9fbc9876e22e0cb3e3e00a0478863ea5adc6bb")

    def test_lower_operator_speed_ceiling_reaches_reference_and_planner(self) -> None:
        for profile in ("race", "normal", "leader"):
            with self.subTest(profile=profile):
                result = resolve_profile(profile, ceiling=5.0)
                self.assertEqual(result["simple_trajectory_generator"]["params"]
                                 ["execution_profile.max_speed_mps"], 5.0)
                for key in SPEED_KEYS:
                    self.assertEqual(result["reference_space_mppi_planner"]["params"][key], 5.0)

    def test_legacy_profile_retains_old_path_and_speed(self) -> None:
        result = resolve_profile("legacy")
        trajectory = result["simple_trajectory_generator"]["params"]
        self.assertEqual(Path(trajectory["csv_path"]).name, "in_corce_line_straight_smooth.csv")
        self.assertEqual(trajectory["execution_profile.max_speed_mps"], 10.0)
        for key in SPEED_KEYS:
            self.assertEqual(result["reference_space_mppi_planner"]["params"][key], 10.0)

    def test_other_controller_retains_dual_reference_and_speed(self) -> None:
        result = resolve_profile("leader", method="mpc")
        trajectory = result["simple_trajectory_generator"]["params"]
        self.assertEqual(Path(trajectory["csv_path"]).name, "in_corce_line_straight_smooth.csv")
        self.assertEqual(trajectory["execution_profile.max_speed_mps"], 10.0)
        self.assertTrue(trajectory["dual_reference.enabled"])
        self.assertFalse(trajectory["race_reference.enabled"])
        self.assertNotIn("reference_space_mppi_planner", result)

    def test_invalid_profile_fails_before_starting_nodes(self) -> None:
        with self.assertRaises(Exception):
            resolve_profile("invalid-profile")


if __name__ == "__main__":
    unittest.main(verbosity=2)
