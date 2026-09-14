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


def test_corrected_revision_preserves_seeds_and_split_but_never_reuses_attempt_ids():
    p=json.loads((ROOT/'configs/time_path_p1/random_recovery_confirmed_20260915.json').read_bytes())
    M.validate_plan(p)
    assert {r['run_id'] for r in p['runs']}.isdisjoint({r['run_id'] for r in plan()['runs']})
    assert [(r['seed'],r['split']) for r in p['runs']]==[(r['seed'],r['split']) for r in plan()['runs']]
    p['revision']='initial_v1'
    with pytest.raises(ValueError):M.validate_plan(p)


def test_partial_closed_batch_round_trips_before_any_cleanup(tmp_path,monkeypatch):
    import hashlib
    import sqlite3
    spec=importlib.util.spec_from_file_location('partial_move_test',ROOT/'docs/evidence/time_recovery_batches_20260914/move_pair.py')
    move=importlib.util.module_from_spec(spec);spec.loader.exec_module(move)
    root=tmp_path/'remote';root.mkdir();analysis=tmp_path/'analysis';analysis.mkdir();raw=tmp_path/'raw';raw.mkdir()
    name='codex-time-recovery-random-r62';run=root/name;(run/'bag').mkdir(parents=True)
    with sqlite3.connect(run/'bag/data.db3') as db:db.execute('CREATE TABLE sample (n INTEGER)')
    M.write(run/'result.json',dict(nodes=dict(closed_bag=True),cleanup_errors=[]))
    M.write(run/'transfer_manifest.json',{'bag/data.db3':dict(bytes=(run/'bag/data.db3').stat().st_size,
        sha256=hashlib.sha256((run/'bag/data.db3').read_bytes()).hexdigest())})
    M.write(root/'campaign_20260914.json',dict(batch_size=2,attempts=[dict(run_id=name,state='STARTED')]))
    move.REMOTE=root;move.ANALYSIS=analysis;move.RAW=raw
    monkeypatch.setattr(move.subprocess,'check_output',lambda *a,**k:'')
    move.pack(root,'partial',[name])
    import shutil
    for suffix in ['.tar.gz','_snapshot.json','_shipping.json']:shutil.copy2(root/('partial'+suffix),analysis/('partial'+suffix))
    move.verify(analysis,'partial',[name],move.sha(root/'partial_snapshot.json'))
    assert (run/'bag/data.db3').exists() and (raw/name/'bag/data.db3').exists()
    assert M.read(analysis/'partial_verified.json')['all_sqlite_quick_checks_passed']


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
