"""Static launch isolation and existing-speed-profile wiring checks."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]/'integrations/pp_reference'


def test_default_is_pp_not_mpc_horizon():
    assert 'CONTROL_METHOD ?= pure_pursuit\n' in (ROOT/'Makefile').read_text()


def test_pp_retains_monitor_and_excludes_mpc():
    tree = ET.parse(ROOT/'reference.launch.xml')
    group = next(g for g in tree.getroot().findall('group')
                 if g.get('if') == "$(eval \"'$(var control_method)' == 'pure_pursuit'\")")
    includes = group.findall('include')
    assert len(includes) == 2
    assert 'overtake_planner.launch.xml' in includes[0].get('file')
    assert 'control/pure_pursuit.launch.xml' in includes[1].get('file')
    args = {a.get('name'):a.get('value') for a in includes[1].findall('arg')}
    assert args['use_mpc_predicted_horizon'] == 'false'
    assert args['use_external_target_vel'] == 'false'
    assert args['stop_on_stale_input'] == 'true'
    assert args['require_overtake_reference_override_fresh'] == 'true'


def test_speed_cap_is_applied_to_reference_generation():
    tree = ET.parse(ROOT/'reference.launch.xml')
    cap = next(p for p in tree.iter('param') if p.get('name')=='execution_profile.max_speed_mps')
    assert cap.get('value') == '$(var effective_reference_speed_cap_mps)'
    overrides = [x for x in tree.iter('let') if x.get('name')=='effective_reference_speed_cap_mps']
    assert len(overrides) == 2
    assert 'min(' in overrides[1].get('value')
    assert '5.555555555555555' in overrides[1].get('value')
