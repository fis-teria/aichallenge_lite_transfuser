"""Two fixed run IDs: prepare, start, move, and audit bounded random recovery."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time
from typing import Any

from collect_time_recovery_phases import read, write, sha, paths, remote_python, ship_pair

PREFIX = 'random_pair01_20260915'
RUNTIME_FILES = (
    'src/aic_transfuser_lite/data/time_random_steering_pulse_v1.py',
    'src/aic_transfuser_lite/data/time_recovery_collection_v1.py',
    'tools/time_recovery_collector_node.py',
)


def validate_plan(plan: dict[str, Any]) -> None:
    revision = plan.get('revision', 'initial_v1')
    if revision not in ('initial_v1', 'confirmation_history_v2'):
        raise ValueError('FROZEN_RANDOM_REVISION')
    first = 62 if revision == 'initial_v1' else 64
    name = 'time_recovery_random_20260915' if revision == 'initial_v1' else 'time_recovery_random_confirmed_20260915'
    expected = [dict(run_id=f'codex-time-recovery-random-r{first}', side='left', seed=915062, split='train', pair=1),
                dict(run_id=f'codex-time-recovery-random-r{first+1}', side='right', seed=915063, split='validation', pair=1)]
    if (plan['schema'] != 'measured_random_recovery_collection_v1' or plan['maximum_attempts'] != 2
            or plan['batch_size'] != 2 or plan['runs'] != expected or plan['target_speed_mps'] != 5/3.6
            or plan['minimum_events_per_run'] != 2 or plan['minimum_valid_anchors_per_event'] != 60
            or plan['minimum_dual_nominal_targets_per_run'] != 1
            or plan['native_root'] != '/home/thistle/e2e_autonomous'
            or plan['analysis_name'] != name
            or plan['remote_root'] != '/home/graneple/e2e_autonomous/'+name
            or plan['base_source_campaign'] != '/home/graneple/e2e_autonomous/time_recovery_outward_20260914'
            or plan['base_source_commit'] != 'a1c9e5ad0a5c274826601338355b75d34b268186'
            or plan['previous_reserved_test_usage'] != 'sealed' or plan['model_training'] or plan['model_evaluation']):
        raise ValueError('FROZEN_RANDOM_COLLECTION_PLAN')


def prepare(plan: dict[str, Any], plan_path: Path, test_gate: Path) -> None:
    """Native WSL only: matched real nominal guides and exact tested overlays."""
    from dataclasses import asdict
    import numpy as np
    from aic_transfuser_lite.data.time_random_steering_pulse_v1 import SCHEMA, RandomPulseConfig, random_schedule, validate_random_guide
    from aic_transfuser_lite.data.time_steering_pulse_v1 import SteeringPulseConfig

    repo = Path.cwd().resolve()
    if repo != Path('/home/thistle/e2e_autonomous/e2e_lite_transfuser'):
        raise ValueError('NATIVE_WSL_PREPARATION_REQUIRED')
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    gate = read(test_gate)
    assert gate['commit'] == commit and gate['full_exit'] == 0 and sha(Path(gate['log'])) == gate['log_sha256']
    out, raw = paths(plan); out.mkdir(exist_ok=False); raw.mkdir(exist_ok=False)
    (out/'references').mkdir()
    manifest = dict(commit=commit, plan_sha256=sha(plan_path), files={}, references={}, source_references={}, schedules={})
    for row, old_num in zip(plan['runs'], (38, 39)):
        old = Path(plan['native_root'])/f"raw/time_recovery_expansion_20260914/codex-time-recovery-pulse{row['side']}-r{old_num}/reference.json"
        ref = read(old); pulse = ref['steering_pulse']; guide = np.asarray(pulse['nominal_guide'])
        cfg = RandomPulseConfig(row['seed']); template = SteeringPulseConfig(**{**pulse['config'], 'amplitude_rad':.1})
        validate_random_guide(cfg, template, guide)
        assert ref['intervals'] == [] and ref['signed_offset_m'] == 0 and ref['reference_xy_m'] == ref['baseline_xy_m']
        pulse.update(schema=SCHEMA, config=asdict(template), random_config=asdict(cfg))
        path = out/'references'/(row['side']+'.json'); write(path, ref)
        manifest['references'][path.name] = sha(path)
        manifest['source_references'][old.as_posix()] = sha(old)
        manifest['schedules'][row['run_id']] = random_schedule(cfg)
    for rel in RUNTIME_FILES:
        manifest['files']['source/'+rel] = sha(repo/rel)
    with tarfile.open(out/'runtime_overlay.tar.gz', 'x:gz') as archive:
        for rel in RUNTIME_FILES:
            archive.add(repo/rel, arcname='source/'+rel, recursive=False)
        for row in plan['runs']:
            archive.add(out/'references'/(row['side']+'.json'), arcname='references/'+row['side']+'.json', recursive=False)
    manifest['overlay_sha256'] = sha(out/'runtime_overlay.tar.gz')
    write(out/'preparation.json', manifest)
    shutil.copy2(plan_path, out/'collection_plan.json'); shutil.copy2(test_gate, out/'test_gate.json')
    print(json.dumps(manifest), flush=True)


def setup(plan: dict[str, Any], plan_path: Path) -> None:
    """Overlay only three runtime files on the hash-verified existing collector."""
    root = Path(plan['remote_root']); source = Path(plan['base_source_campaign'])
    assert root.resolve() == root and plan_path.resolve() == root/'collection_plan.json'
    assert not (root/'source').exists() and not subprocess.check_output(['docker','ps','-q'],text=True).strip()
    assert shutil.disk_usage(root).free >= 12*2**30
    old = read(source/'deployment.json'); old_ledger = read(source/'campaign_20260914.json')
    assert old['source_commit'] == plan['base_source_commit'] and old_ledger['sealed']
    assert all(r['state'] == 'WSL_MOVED' for r in old_ledger['attempts'])
    for rel, digest in old['files'].items():
        assert (source/rel).resolve().is_relative_to(source) and sha(source/rel) == digest
        target = root/rel; target.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(source/rel,target)
    prep = read(root/'preparation.json')
    assert prep['plan_sha256'] == sha(plan_path) and sha(root/'runtime_overlay.tar.gz') == prep['overlay_sha256']
    expected = {**prep['files'], **{'references/'+k:v for k,v in prep['references'].items()}}
    assert set(prep['files']) == {'source/'+p for p in RUNTIME_FILES}
    with tarfile.open(root/'runtime_overlay.tar.gz') as archive:
        members = archive.getmembers(); assert len(members) == len(expected) and {m.name for m in members} == set(expected)
        for m in members:
            assert m.isfile(); payload = archive.extractfile(m).read()
            import hashlib
            assert hashlib.sha256(payload).hexdigest() == expected[m.name]
            target = root/m.name; assert target.resolve().is_relative_to(root)
            target.parent.mkdir(parents=True,exist_ok=True); target.write_bytes(payload)
    for side in ('left','right'):
        shutil.copy2(source/'references'/(side+'.csv'),root/'references'/(side+'.csv'))
    files = {rel:sha(root/rel) for rel in set(old['files']) | set(prep['files'])}
    write(root/'deployment.json', dict(source_commit=prep['commit'],base_source_commit=old['source_commit'],
          files=files,overlay_files=prep['files'],scope='EXACT_THREE_FILE_OVERLAY_ON_SEALED_COLLECTOR'))
    (root/'deployed_commit.txt').write_text(prep['commit']+'\n')
    shutil.copy2(source/'speed_gate.json',root/'speed_gate.json')
    write(root/'campaign_20260914.json', dict(scope='RANDOM_RECOVERY_PILOT',maximum_attempts=2,batch_size=2,
          source_commit=prep['commit'],plan_sha256=sha(plan_path),planned={r['run_id']:r['side'] for r in plan['runs']},
          attempts=[],sealed=False))
    repo = '/home/graneple/git/autononous_ai/aichallenge-racingkart'
    import hashlib
    write(root/'host_before.json', dict(head=subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip(),
        git_status_sha256=hashlib.sha256(subprocess.check_output(['git','-C',repo,'status','--porcelain'])).hexdigest(),
        containers=[json.loads(r) for r in subprocess.check_output(['docker','ps','-a','--format','{{json .}}'],text=True).splitlines()],
        compose=json.loads(subprocess.check_output(['docker','compose','ls','--all','--format','json'],text=True)),
        free_bytes=shutil.disk_usage(root).free))
    print('RANDOM_SETUP_COMPLETE',len(files),flush=True)


def start(plan: dict[str, Any], plan_path: Path, run_id: str) -> None:
    root = Path(plan['remote_root']); assigned = next(r for r in plan['runs'] if r['run_id']==run_id)
    ledger = read(root/'campaign_20260914.json'); deployment = read(root/'deployment.json')
    assert not ledger['sealed'] and ledger['maximum_attempts'] == 2 and len(ledger['attempts']) < 2
    assert ledger['plan_sha256'] == sha(plan_path) and plan['runs'][len(ledger['attempts'])] == assigned
    assert not (root/run_id).exists() and not subprocess.check_output(['docker','ps','-q'],text=True).strip()
    assert shutil.disk_usage(root).free >= 12*2**30
    for rel,digest in deployment['files'].items():
        assert (root/rel).resolve().is_relative_to(root) and sha(root/rel)==digest,rel
    gate = read(root/'test_gate.json')
    assert gate['commit'] == deployment['source_commit'] == (root/'deployed_commit.txt').read_text().strip()
    assert gate['full_exit']==0 and read(root/'speed_gate.json')['all_within_005_mps']
    prep = read(root/'preparation.json'); ref=root/'references'/(assigned['side']+'.json')
    assert sha(ref)==prep['references'][ref.name] and sha(ref.with_suffix('.csv'))==read(ref)['reference_sha256']
    for previous in ledger['attempts']:
        result=read(root/previous['run_id']/'result.json')
        assert result['status']=='COMPLETE_LAP' and result['last_control']['fault'] is None and result['nodes']['closed_bag']
    command=['timeout','--signal=TERM','--kill-after=20s','1980s','python3',str(root/'source/tools/run_time_recovery_awsim.py'),
             '--campaign-root',str(root),'--run-id',run_id,'--side',assigned['side'],'--speed-policy','aligned_gain4_v1','--separate-cpus']
    row=dict(**assigned,state='STARTING',started_unix_s=time.time(),command=command)
    ledger['attempts'].append(row);write(root/'campaign_20260914.json',ledger,replace=True)
    env=dict(os.environ,PYTHONPATH=str(root/'source/src'),DISPLAY=':1',XAUTHORITY='/run/user/1000/gdm/Xauthority')
    with (root/(run_id+'_supervisor.log')).open('x') as stream:
        child=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    row.update(state='STARTED',pid=child.pid);write(root/'campaign_20260914.json',ledger,replace=True)
    print(json.dumps(row),flush=True)


def collect(plan: dict[str, Any]) -> None:
    root=Path(plan['remote_root']);host='graneple@192.168.3.10'
    for row in plan['runs']:
        subprocess.run(['ssh',host,'python3',(root/'collect_time_random_recovery.py').as_posix(),'start',
            '--plan',(root/'collection_plan.json').as_posix(),'--run-id',row['run_id']],check=True,timeout=60)
        deadline=time.monotonic()+2030
        while time.monotonic()<deadline:
            code=f"""
from pathlib import Path
import json
p=Path({(root/row['run_id']).as_posix()!r})
if (p/'result.json').exists() and (p/'transfer_manifest.json').exists():
 print(json.dumps(dict(complete=True,result=json.loads((p/'result.json').read_bytes()))))
else:
 d=json.loads((p/'control_heartbeat.json').read_bytes()) if (p/'control_heartbeat.json').exists() else {{}}
 print(json.dumps(dict(complete=False,phase=d.get('phase'),fault=d.get('fault'),sim_ns=d.get('sim_ns'),
  s_m=(d.get('projection') or {{}}).get('s_m'),random_state=d.get('random_pulse',{{}}).get('state'))))
"""
            value=json.loads(remote_python(host,code))
            if value['complete']:
                r=value['result'];print('RUN_FINISHED',row['run_id'],r['status'],flush=True)
                if not (r['status']=='COMPLETE_LAP' and r['last_control']['fault'] is None
                        and r['last_control']['stop_confirmed'] and r['nodes']['closed_bag'] and not r['cleanup_errors']):
                    raise RuntimeError('RANDOM_RUN_FAILED_PRESERVE_NO_RETRY:'+row['run_id'])
                if r['last_control']['random_pulse']['state']['completed_events'] < plan['minimum_events_per_run']:
                    raise RuntimeError('INSUFFICIENT_CONFIRMED_EVENTS_PRESERVE_NO_RETRY:'+row['run_id'])
                break
            print('RUN_PROGRESS',row['run_id'],json.dumps(value),flush=True);time.sleep(30)
        else:
            raise TimeoutError('RANDOM_OUTER_DEADLINE')
    ship_pair(plan,1,root=root,prefix=PREFIX)
    filename = ('random_recovery_confirmed_20260915.json' if plan.get('revision') == 'confirmation_history_v2'
                else 'random_recovery_20260915.json')
    command=('cd /home/thistle/e2e_autonomous/e2e_lite_transfuser && tools/with_wsl_training_lock.sh env PYTHONPATH=src '
             '.venv/bin/python -u tools/collect_time_random_recovery.py audit --plan configs/time_path_p1/'+filename)
    subprocess.run(['ssh','codex-wsl',command],check=True,timeout=2400)
    print('RANDOM_COLLECTION_AND_AUDIT_COMPLETE',flush=True)


def audit(plan: dict[str, Any], plan_path: Path) -> None:
    """All candidates, event-specific measured states, observed teachers and inputs."""
    import numpy as np
    from aic_transfuser_lite.data.time_dataset_v1 import TimeDatasetConfig
    from aic_transfuser_lite.data.time_recovery_training_v1 import materialize_recovery_run
    from aic_transfuser_lite.data.time_training_cache_v1 import _prepare_run
    sys.path.insert(0,str(Path('docs/evidence/time_recovery_expansion_20260914').resolve()))
    from summarize_expansion import alternate_state, goal, bins
    out,raw=paths(plan);receipt=read(out/(PREFIX+'_verified.json'))
    assert receipt['all_files_and_directory_structure_identical'] and receipt['all_sqlite_quick_checks_passed']
    assert [r['run_id'] for r in receipt['runs']]==[r['run_id'] for r in plan['runs']]
    types=Path(plan['native_root'])/'runs/time_recovery_collection_20260913/types'
    alt_path=Path(plan['native_root'])/'runs/time_steering_pulse_20260914/alternate_nominal_guide_r31.json'
    guide_b=np.asarray(read(alt_path)['guide']);reports=[]
    for row in plan['runs']:
        name=row['run_id'];suffix=name.split('-')[-1]
        commands=[('bag_audit',['tools/audit_time_recovery_collection.py','--run',str(raw/name),'--types',str(types)]),
            ('causal_probe',['docs/evidence/time_recovery_collection_20260913/causal_replay_probe.py','--run',str(raw/name),
                '--types',str(types),'--all-candidates']),
            ('state_audit',['docs/evidence/time_steering_pulse_20260914/analyze_pulse.py','--run',str(raw/name),
                '--probe',str(out/(suffix+'_causal_probe.json'))])]
        for kind,command in commands:
            target=out/(suffix+'_'+kind+'.json');assert not target.exists()
            with target.with_suffix('.log').open('x') as stream:
                status=subprocess.run([sys.executable,*command,'--output',str(target)],stdout=stream,stderr=subprocess.STDOUT,timeout=600)
            print('AUDIT_STAGE',name,kind,status.returncode,flush=True)
            if status.returncode: raise RuntimeError(target.with_suffix('.log').read_text()[-3000:])
        states=read(out/(suffix+'_state_audit.json'));guide_a=np.asarray(read(raw/name/'reference.json')['steering_pulse']['nominal_guide'])
        events=[]
        for event in states['random_events']:
            accepted=[r for r in states['records'] if r['accepted'] and r['recovery_event_id']==event['event_id']]
            alt=[alternate_state(r,guide_a,guide_b) for r in accepted]
            targets=[a['anchor_id'] for a,b in zip(accepted,alt) if goal(a) and goal(b)]
            events.append(dict(**event,accepted=bins(accepted),alternate=bins(alt),target_anchor_ids=targets))
        report=dict(run_id=name,split=row['split'],seed=row['seed'],events=events,
            lap_seconds=read(raw/name/'result.json')['judge_laps'][0]['lap_seconds'],
            accepted=sum(e['accepted']['count'] for e in events),targets=sum(len(e['target_anchor_ids']) for e in events))
        write(out/(suffix+'_event_summary.json'),report)
        assert len(events)>=plan['minimum_events_per_run'] and all(e['recovery_confirmed'] for e in events), 'RANDOM_RECOVERY_NOT_CONFIRMED'
        assert all(e['accepted']['count']>=plan['minimum_valid_anchors_per_event'] for e in events), 'RANDOM_EVENT_TEACHER_COVERAGE'
        assert report['targets']>=plan['minimum_dual_nominal_targets_per_run'], 'RANDOM_NO_OUTWARD_TEACHERS'
        destination=out/'materialized'/name
        material=materialize_recovery_run(raw/name,destination,split=row['split'],types=types,previous_probe=out/(suffix+'_causal_probe.json'))
        cache=out/'prepared'/row['split']/name;prepared=_prepare_run(destination,cache,name,TimeDatasetConfig())
        assert prepared['input_valid']==prepared['anchors']==material['accepted']==report['accepted']
        assert not prepared['input_reason_refinements']
        anchors=[json.loads(s) for s in (destination/'anchors.jsonl').read_text().splitlines()]
        assert {r['recovery_event_id'] for r in anchors}=={e['event_id'] for e in events}
        report.update(prepared=prepared,files={p.relative_to(out).as_posix():sha(p)
            for directory in (destination,cache) for p in sorted(directory.iterdir()) if p.is_file()})
        reports.append(report);print('RANDOM_INPUTS_AND_TEACHERS_PREPARED',name,report['accepted'],flush=True)
    final=dict(status='PASS',scope='RECOVERY_TEACHER_DATA_QUALITY_NOT_MODEL_EVALUATION',
        plan_sha256=sha(plan_path),runs=reports,independent_runs=len(reports),
        total_events=sum(len(r['events']) for r in reports),total_anchors=sum(r['accepted'] for r in reports),
        total_targets=sum(r['targets'] for r in reports),model_trained=False,model_evaluated=False,
        previous_reserved_test_read=False,alternate_guide_sha256=sha(alt_path))
    write(out/'collection_index.json',final);print(json.dumps(final),flush=True)


def main() -> None:
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prepare','setup','start','collect','ship','audit'])
    ap.add_argument('--plan',type=Path,required=True);ap.add_argument('--test-gate',type=Path);ap.add_argument('--run-id')
    args=ap.parse_args();plan=read(args.plan);validate_plan(plan)
    if args.mode=='prepare':
        if args.test_gate is None: ap.error('--test-gate required')
        prepare(plan,args.plan,args.test_gate)
    elif args.mode=='setup': setup(plan,args.plan)
    elif args.mode=='start': start(plan,args.plan,args.run_id)
    elif args.mode=='collect': collect(plan)
    elif args.mode=='ship': ship_pair(plan,1,root=Path(plan['remote_root']),prefix=PREFIX)
    elif args.mode=='audit': audit(plan,args.plan)


if __name__=='__main__':
    main()
