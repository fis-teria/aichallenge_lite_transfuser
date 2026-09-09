import ast
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'integrations/pp_run12'

def test_pp_scope():
    for name in ('run.py','guard.py'):
        ast.parse((ROOT/name).read_text())
    run=(ROOT/'run.py').read_text()
    assert "CONTROL_METHOD='pure_pursuit'" in run
    assert "'CONTROL_METHOD=pure_pursuit'" in run
    assert 'wall_s=120.,forward=0,mpc=0' in run
    assert "V4_SHADOW_ENABLED='false'" in run
    assert "mpc_guard_lap_11/budget.json" in run
    guard=(ROOT/'guard.py').read_text()
    assert "!='/simple_pure_pursuit_node'" in guard
    assert "'raw_limit_rad':0.64,'gain':1.0" in guard
    assert 'RAW_COMMAND_NONFINITE' in guard
