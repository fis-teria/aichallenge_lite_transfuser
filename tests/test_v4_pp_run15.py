import ast
from pathlib import Path

def test_finite_trial_and_startup_independent_shutdown():
    path=Path(__file__).resolve().parents[1]/'integrations/v4_pp_run15/run.py'
    source=path.read_text()
    ast.parse(source)
    watcher=source.split('def watch():',1)[1].split('threading.Thread',1)[0]
    assert "record.get('event')=='SESSION_END'" in watcher
    assert "v4_pp_shadow_15/d1/autoware.log" in watcher
    assert "v4_pp_shadow_14/budget.json" in source
    assert 'wall_s=120.,forward=40,mpc=0' in source
    assert "V4_SHADOW_ENABLED='true'" in source
