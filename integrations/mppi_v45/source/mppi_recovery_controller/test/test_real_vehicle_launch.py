from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


SUBMIT = Path(__file__).resolve().parents[2]
LAUNCH = SUBMIT / 'aichallenge_submit_launch'


def resolve(value, **settings):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros.substitutions')
    from launch import LaunchContext
    from launch.frontend.parse_substitution import parse_substitution
    from launch.utilities import perform_substitutions
    context = LaunchContext()
    context.launch_configurations.update(settings)
    return perform_substitutions(context, parse_substitution(value))


def test_real_profile_matches_planner_tracker_and_recovery():
    profile = yaml.safe_load((LAUNCH / 'config/mppi_vehicle_real.param.yaml').read_text())
    shared = profile['/**']['ros__parameters']
    planner = profile['reference_space_mppi_planner']['ros__parameters']
    tracker = profile['cma_pure_pursuit_node']['ros__parameters']
    assert shared['physical_tire_steering_rate_radps'] == pytest.approx(22.8 * 3.141592653589793 / 180)
    assert planner['steering_control_delay_sec'] == planner['prediction.cma_control_delay_sec'] == tracker['pp_control_delay_sec']
    assert planner['prediction.cma_steering_time_constant_sec'] == shared['steering_time_constant_sec']
    assert planner['maximum_acceleration_mps2'] == tracker['longitudinal_acceleration_limit']
    assert planner['prediction.awsim_vehicle_response_enabled'] is False
    recovery = profile['mppi_recovery_controller']['ros__parameters']
    assert 0 < recovery['maximum_recovery_propulsion_mps2'] <= tracker['longitudinal_acceleration_limit']
    assert profile['mppi_real_actuation_filter']['ros__parameters']['no_progress_timeout_sec'] < 1.0


def test_simulation_overlay_does_not_override_existing_configuration():
    profile = yaml.safe_load((LAUNCH / 'config/mppi_vehicle_simulation.param.yaml').read_text())
    assert profile == {'/**': {'ros__parameters': {}}}


@pytest.mark.parametrize('method,simulation,expected', [
    ('mppi', 'false', '/mppi/internal/actuation_cmd'),
    ('mppi', 'true', '/control/command/actuation_cmd'),
    ('mpc', 'false', '/control/command/actuation_cmd'),
])
def test_only_real_mppi_converter_output_goes_through_filter(method, simulation, expected):
    root = ET.parse(LAUNCH / 'launch/reference.launch.xml').getroot()
    converter = next(node for node in root.iter('include') if 'raw_vehicle_converter.launch.xml' in node.attrib['file'])
    output = next(arg.attrib['value'] for arg in converter if arg.attrib.get('name') == 'output_actuation_cmd')
    assert resolve(output, control_method=method, simulation=simulation) == expected


@pytest.mark.parametrize('simulation,expected', [('false', 'real'), ('true', 'simulation')])
def test_profile_selection_and_filter_condition(simulation, expected):
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch.conditions import UnlessCondition
    from launch.frontend.parse_substitution import parse_substitution
    root = ET.parse(LAUNCH / 'launch/control/mppi.launch.xml').getroot()
    profile = next(arg.attrib['default'] for arg in root.findall('arg') if arg.attrib['name'] == 'vehicle_param_file')
    assert resolve(profile, simulation=simulation).endswith(f'mppi_vehicle_{expected}.param.yaml')
    node = next(n for n in root.findall('node') if n.attrib['exec'] == 'real_actuation_filter')
    context = LaunchContext()
    context.launch_configurations['simulation'] = simulation
    assert UnlessCondition(parse_substitution(node.attrib['unless'])).evaluate(context) is (simulation == 'false')
    remaps = {r.attrib['from']: r.attrib['to'] for r in node.findall('remap')}
    assert remaps['input/actuation_cmd'] == '/mppi/internal/actuation_cmd'
    assert remaps['output/actuation_cmd'] == '/control/command/actuation_cmd'
