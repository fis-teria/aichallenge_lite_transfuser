"""Frozen independent runs, transport boundaries and failed-run dispatch gates."""
import ast
import importlib.util
import json
from pathlib import Path, PureWindowsPath
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
try:
    SPEC=importlib.util.spec_from_file_location('random_collection_test',ROOT/'tools/collect_time_random_recovery.py')
    M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)
finally:
    sys.path.pop(0)


def plan():
    return json.loads((ROOT/'configs/time_path_p1/random_recovery_20260915.json').read_bytes())


def test_frozen_two_run_holdout_uses_new_ids_and_only_three_runtime_overlays():
    p=plan();M.validate_plan(p)
    assert [r['split'] for r in p['runs']]==['train','validation']
    assert len({r['seed'] for r in p['runs']})==2
    assert all(int(r['run_id'].split('r')[-1])>61 for r in p['runs'])
    assert len(M.RUNTIME_FILES)==3 and all((ROOT/f).is_file() for f in M.RUNTIME_FILES)


@pytest.mark.parametrize('key,value',[('maximum_attempts',3),('target_speed_mps',2.),
    ('minimum_valid_anchors_per_event',0),('minimum_events_per_run',1),
    ('remote_root','/tmp/outside'),('native_root','/mnt/e'),('model_training',True)])
def test_scope_changes_fail_before_dispatch(key,value):
    p=plan();p[key]=value
    with pytest.raises(ValueError): M.validate_plan(p)


@pytest.mark.parametrize('change',['seed','split','order'])
def test_run_assignment_and_seeds_cannot_drift(change):
    p=plan()
    if change=='order': p['runs'].reverse()
    else: p['runs'][0][change]=p['runs'][1][change]
    with pytest.raises(ValueError): M.validate_plan(p)


@pytest.mark.parametrize('path_class',[Path,PureWindowsPath])
def test_collect_transport_remains_posix_with_bounded_known_results(monkeypatch,path_class):
    commands=[];scripts=[];ship=[]
    monkeypatch.setattr(M,'Path',path_class)
    monkeypatch.setattr(M.subprocess,'run',lambda command,**kwargs: commands.append(command))
    monkeypatch.setattr(M,'ship_pair',lambda *args,**kwargs:ship.append((args,kwargs)))
    def remote(host,code):
        ast.parse(code);scripts.append(code)
        return json.dumps(dict(complete=True,result=dict(status='COMPLETE_LAP',cleanup_errors=[],
            nodes=dict(closed_bag=True),last_control=dict(fault=None,stop_confirmed=True,
                random_pulse=dict(state=dict(completed_events=3))))))
    monkeypatch.setattr(M,'remote_python',remote)
    M.collect(plan())
    assert len(scripts)==2 and len(ship)==1 and all('\\' not in v for cmd in commands for v in cmd)
    assert ship[0][1]['prefix']==M.PREFIX
    assert all("p=Path('/home/graneple/" in s for s in scripts)


@pytest.mark.parametrize('kind',['runtime','unconfirmed'])
def test_failed_first_run_cannot_launch_the_second_or_ship(monkeypatch,kind):
    commands=[]
    monkeypatch.setattr(M.subprocess,'run',lambda command,**kwargs:commands.append(command))
    monkeypatch.setattr(M,'ship_pair',lambda *a,**k:pytest.fail('must preserve failed run'))
    result=dict(status='FAILED' if kind=='runtime' else 'COMPLETE_LAP',cleanup_errors=[],nodes=dict(closed_bag=True),
        last_control=dict(fault=None,stop_confirmed=True,random_pulse=dict(state=dict(completed_events=0))))
    monkeypatch.setattr(M,'remote_python',lambda *a,**k:json.dumps(dict(complete=True,result=result)))
    with pytest.raises(RuntimeError):M.collect(plan())
    assert len(commands)==1 and 'codex-time-recovery-random-r62' in commands[0]


def test_sealed_attempts_cannot_start(tmp_path,monkeypatch):
    p=plan();p['remote_root']=str(tmp_path)
    M.write(tmp_path/'campaign_20260914.json',dict(sealed=True))
    M.write(tmp_path/'deployment.json',{})
    monkeypatch.setattr(M.subprocess,'Popen',lambda *a,**k:pytest.fail('must not start'))
    with pytest.raises(AssertionError):M.start(p,tmp_path/'collection_plan.json',p['runs'][0]['run_id'])
