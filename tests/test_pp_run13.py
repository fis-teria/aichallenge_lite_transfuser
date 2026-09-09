"""Finite gain-change trial contract; synthetic forward-speed domain only."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def test_gain_keeps_start_acceleration_within_consumer_contract():
    tree = ET.parse(ROOT/'integrations/pp_reference/reference.launch.xml')
    group = next(g for g in tree.getroot().findall('group')
                 if g.get('if') == "$(eval \"'$(var control_method)' == 'pure_pursuit'\")")
    args = {a.get('name'): a.get('value') for a in group.findall('include')[1].findall('arg')}
    gain = float(args['speed_proportional_gain'])
    assert gain == 0.5
    for target, current, expected in [(20/3.6, 0, 25/9), (5, 5, 0), (0, 5, -2.5)]:
        acceleration = gain*(target-current)
        assert abs(acceleration-expected) < 1e-12
        assert acceleration <= 3
    # This is not a general clamp; reverse/invalid states must not be excused.
    assert gain*((20/3.6)-(-2)) > 3


def test_trial_is_finite_and_preserves_history():
    run = (ROOT/'integrations/pp_run13/run.py').read_text()
    guard = (ROOT/'integrations/pp_run13/guard.py').read_text()
    ast.parse(run)
    ast.parse(guard)
    assert 'pp_reference_lap_12/budget.json' in run
    assert 'wall_s=120.,forward=0,mpc=0' in run
    assert "V4_SHADOW_ENABLED='false'" in run
    assert "acceleration>3.0" in guard
    assert 'ACCELERATION_INPUT_LIMIT' in guard
