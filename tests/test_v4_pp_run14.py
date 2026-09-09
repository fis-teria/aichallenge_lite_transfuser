import ast
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'integrations/v4_pp_run14'

def test_shadow_trial_is_bounded_and_pp_only():
    run=(ROOT/'run.py').read_text()
    ast.parse(run)
    assert "V4_SHADOW_ENABLED='true'" in run
    assert 'forward=40,mpc=0' in run
    assert 'pp_speed20_lap_13/budget.json' in run
    assert "'CONTROL_METHOD=pure_pursuit'" in run
    config=json.loads((ROOT/'live.template.json').read_text())
    assert config['command_binding']['source']=='final_fallback'
    assert config['command_binding']['producer_id']=='/simple_pure_pursuit_node'
    assert config['topics']['command']=='/control/command/control_cmd'
    assert config['envelope']['forward_limit']==40
    assert config['inference_start_policy']=='INPUT_READY_SHADOW'
