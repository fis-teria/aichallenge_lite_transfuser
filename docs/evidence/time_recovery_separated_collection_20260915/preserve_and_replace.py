"""Preserve one pre-pulse timeout and add exactly one replacement, never retry indefinitely."""
import manage as m
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--install-only',action='store_true');args=ap.parse_args()

preserve_code=rf'''
from pathlib import Path
import json,hashlib,subprocess
root=Path({m.ROOT!r});p=root/'codex-time-recovery-separated-g05-left'
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
data=(p/'control.jsonl').read_bytes();manifest=json.loads((p/'transfer_manifest.json').read_bytes())
assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
rows=[json.loads(s) for s in data.splitlines()];result=json.loads((p/'result.json').read_bytes())
assert result['status']=='STOPPED_FAILED' and result['last_control']['fault']=='COLLECTION_COMPUTATION_TIMEOUT'
assert result['last_control']['stop_confirmed'] and result['nodes']['closed_bag'] and not result['cleanup_errors']
assert result['last_control']['random_pulse']['state']['event_id']==0
assert all(not r.get('pulse',{{}}).get('requested_rad',0) for r in rows)
i=next(i for i,r in enumerate(rows) if r.get('reason')=='COLLECTION_COMPUTATION_TIMEOUT')
r=rows[i]
diagnosis=dict(run_id=p.name,result_status=result['status'],fault=result['last_control']['fault'],
 observed_events=0,training_anchors=0,excluded_from_training=True,first_fault=r,
 previous_two_rows=rows[max(0,i-2):i],control_sha256=hashlib.sha256(data).hexdigest(),
 result_canonical_sha256=hashlib.sha256(json.dumps(result,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
 confirmed_stop=True,closed_bag=True,cleanup_errors=[],
 conclusion='PRE_EVENT_SNAPSHOT_RETRY_STALE_STEERING_TOTAL_DECISION_104MS_EXCEEDS_UNCHANGED_80MS',
 boundary='Host CPU contention not conclusively attributable; no model or disturbance action at failure',
 response='Preserve complete failed raw; one replacement with unchanged runtime and safety thresholds')
with (root/'failed_attempt_diagnosis.json').open('x') as f:json.dump(diagnosis,f,indent=2)
print(json.dumps(dict(fault_progress_m=r['projection']['s_m'],decision_wall_ms=r['decision_wall_ms'],
 processing_ms=r['processing_ms'],snapshot_retry_reasons=r['snapshot_retry_reasons'],events=0)))
'''
if not args.install_only:
    m.remote(preserve_code)
    m.copy_to_wsl(['failed_attempt_diagnosis.json'])
    m.ship(5,prefix='separated_failed_g05_left_20260915')
m.remote(rf'''
from pathlib import Path
import copy,json,hashlib,subprocess,time
root=Path({m.ROOT!r});ledger_path=root/'campaign_20260914.json';original=ledger_path.read_bytes();ledger=json.loads(original)
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
assert len(ledger['attempts'])==9 and ledger['maximum_attempts']==10 and len(ledger['planned_runs'])==10
assert all(a['state']=='WSL_MOVED' for a in ledger['attempts']) and not ledger['sealed']
failed='codex-time-recovery-separated-g05-left';replacement=failed+'-r1'
assert ledger['attempts'][-1]['run_id']==failed and ledger['planned_runs'][8]['run_id']==failed
plan_bytes=(root/'selected_site_plan.json').read_bytes();plan=json.loads(plan_bytes)
assert hashlib.sha256(plan_bytes).hexdigest()==ledger['selection_sha256']
diagnosis=json.loads((root/'failed_attempt_diagnosis.json').read_bytes())
receipt=json.loads((root/'separated_failed_g05_left_20260915_verified.json').read_bytes())
assert receipt['all_files_and_directory_structure_identical'] and receipt['all_sqlite_quick_checks_passed']
assert [r['run_id'] for r in receipt['runs']]==[failed]
amendment=dict(scope='ONE_REPLACEMENT_FOR_PRESERVED_PRE_EVENT_TIMEOUT',created_unix_s=time.time(),
 original_maximum_attempts=10,maximum_attempts=11,maximum_successful_laps=10,maximum_pulse_events=22,
 additional_attempts=1,remaining_attempts=2,failed_run_id=failed,replacement_run_id=replacement,
 original_selection_sha256=ledger['selection_sha256'],original_campaign_sha256=hashlib.sha256(original).hexdigest(),
 failed_result_canonical_sha256=diagnosis['result_canonical_sha256'],failed_snapshot_sha256=receipt['snapshot_sha256'],
 runtime_source_commit=(root/'deployed_commit.txt').read_text().strip(),runtime_change=False,
 safety_threshold_change=False,additional_retry_on_failure=False,
 collection_task='Complete the same S00 plus 10 sites, left/right; failed event count is zero')
with (root/'before_replacement_campaign_20260914.json').open('xb') as f:f.write(original)
amendment_bytes=(json.dumps(amendment,indent=2)+'\n').encode()
with (root/'replacement_amendment.json').open('xb') as f:f.write(amendment_bytes)
new=copy.deepcopy(ledger['planned_runs'][8]);new.update(run_id=replacement,replacement_for=failed)
ledger['planned_runs'].insert(9,new)
ledger.update(maximum_attempts=11,excluded_failed_runs=[failed],replacement_amendment_sha256=hashlib.sha256(amendment_bytes).hexdigest())
plan['runs'][8]=copy.deepcopy(new)
plan['replacement_amendment_sha256']=ledger['replacement_amendment_sha256']
assert len(plan['runs'])==10 and len({{r['run_id'] for r in plan['runs']}})==10
assert sum(len(r['sites']) for r in plan['runs'])==22
with (root/'effective_data_plan.json').open('x') as f:json.dump(plan,f,indent=2)
ledger_path.write_text(json.dumps(ledger,indent=2)+'\n')
print(json.dumps(amendment))
''')
m.copy_to_wsl(['replacement_amendment.json','before_replacement_campaign_20260914.json','effective_data_plan.json'])
print('FAILED_RAW_VERIFIED_AND_FINITE_REPLACEMENT_INSTALLED',flush=True)
