"""Static isolated-run checks; no ROS, model, sensor or control execution."""
import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1] / 'integrations' / 'mpc_speed20'


def test_only_speed_ceiling_changes():
    original = yaml.safe_load((ROOT / 'config.original.yaml').read_text())
    limited = yaml.safe_load((ROOT / 'config.yaml').read_text())
    assert original['mpc']['v_max'] == 35.0
    assert limited['mpc']['v_max'] == 20.0
    limited['mpc']['v_max'] = original['mpc']['v_max']
    assert limited == original


def test_scripts_parse_and_preserve_bounds():
    for name in ('run.py', 'guard.py'):
        ast.parse((ROOT / name).read_text())
    run = (ROOT / 'run.py').read_text()
    assert "V4_SHADOW_ENABLED='false'" in run
    assert 'wall_s=120.,forward=0,mpc=60000' in run
    assert 'powered_s=90.' in run
    assert 'REPEATED_MPC_FAILURE' in run
    assert 'COLLISION_REPORTED' in run
    assert "signal.signal(signal.SIGTERM,interrupted)" in run


def test_corner_candidate_changes_only_lateral_acceleration_budget():
    baseline = yaml.safe_load((ROOT / 'config.yaml').read_text())
    candidate = yaml.safe_load((ROOT / 'config.corner_candidate.yaml').read_text())
    assert candidate['mpc']['ay_max'] == 3.0
    assert candidate['mpc']['v_max'] == 20.0
    candidate['mpc']['ay_max'] = baseline['mpc']['ay_max']
    assert candidate == baseline
