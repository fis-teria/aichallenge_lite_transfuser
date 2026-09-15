"""Seal final host checks and copy small reviewable artifacts, never raw data to Git."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import manage as m

m.remote(rf'''
from pathlib import Path
import hashlib,json,shutil,subprocess
root=Path({m.ROOT!r});ledger=json.loads((root/'campaign_20260914.json').read_text())
amendment=json.loads((root/'replacement_amendment.json').read_bytes())
assert ledger['sealed'] and len(ledger['attempts'])==ledger['maximum_attempts']==amendment['maximum_attempts']==11
assert all(r['state']=='WSL_MOVED' for r in ledger['attempts'])
results=[json.loads((root/r['run_id']/'MOVED_TO_WSL.json').read_bytes())['result'] for r in ledger['attempts']]
good=[r for r in results if r['status']=='COMPLETE_LAP' and not r['last_control']['fault']]
failed=[r for r in results if r not in good]
assert len(good)==10 and len(failed)==1 and failed[0]['run_id']==amendment['failed_run_id']
assert failed[0]['last_control']['random_pulse']['state']['event_id']==0
assert all(r['last_control']['stop_confirmed'] and r['nodes']['closed_bag'] and not r['cleanup_errors'] for r in results)
assert sum(r['last_control']['random_pulse']['state']['completed_events'] for r in good)==22
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
before=json.loads((root/'host_before.json').read_text());repo='/home/graneple/git/autononous_ai/aichallenge-racingkart'
head=subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip()
status=hashlib.sha256(subprocess.check_output(['git','-C',repo,'status','--porcelain'])).hexdigest()
assert head==before['head'] and status==before['git_status_sha256']
deployment=json.loads((root/'deployment.json').read_text());gate=json.loads((root/'test_gate.json').read_text())
assert deployment['source_commit']==gate['commit']==(root/'deployed_commit.txt').read_text().strip() and gate['full_exit']==0
for rel,digest in deployment['files'].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==digest,rel
v=dict(sealed=True,attempted_runs=11,successful_laps=10,failed_pre_event_attempts=1,
 all_raw_moved_to_verified_wsl=True,containers_running=[],
 original_checkout_head_preserved=True,original_checkout_status_preserved=True,
 runtime_commit=deployment['source_commit'],runtime_files_verified=len(deployment['files']),free_bytes=shutil.disk_usage(root).free)
with (root/'final_environment.json').open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(v))
''')
for name in ('final_environment.json','campaign_20260914.json','deployment.json','test_gate.json','marker_official_image_smoke.json'):
    dest=m.UNC/('final_'+name if (m.UNC/name).exists() else name)
    assert not dest.exists()
    subprocess.run(['scp',m.HOST+':'+m.ROOT+'/'+name,str(dest)],check=True)
dest=m.REPO/'docs/evidence/time_recovery_separated_collection_20260915';dest.mkdir(exist_ok=False)
sources=[]
for pattern in ('*gate*.json','*environment.json','*deployment.json','campaign_20260914.json','selected_site_plan.json',
    'plan_map_annotation_erratum.json','pair*_audit.json','pair*_holds.json','*_collection_summary.json',
    'collection_index.json','*_verified.json','*_shipping.json','*_cleanup.json','*official_image_smoke.json','s00_observed_comparison.png',
    'before_duplicate_clock_fix_*.json','hold_audit_correction.json','hold_evidence_detail.json',
    'replacement_amendment.json','effective_data_plan.json','before_replacement_campaign_20260914.json','failed_attempt_diagnosis.json',
    'completion_verification.json'):
    sources.extend(m.UNC.glob(pattern))
head='52891011a2a311cf52303b33baaca5e6d666e292'
sources.extend([m.UNC.parent/f'time_site_recovery_gate_{head[:7]}.json',m.UNC.parent/f'time_site_recovery_full_{head[:7]}.log'])
sources.extend([m.UNC.parent/'time_site_recovery_gate_1d20998.json',m.UNC.parent/'time_site_recovery_full_1d20998.log'])
sources.extend(m.UNC.glob('visual_proofs/*g01*.png'));sources.extend(m.UNC.glob('visual_proofs/*g01*.json'))
sources.extend(m.HERE/p for p in ('manage.py','plan_native.py','install_plan.py','audit_native.py','assess_native.py',
    'assess_pair.py','continue_after_pilot.py','summarize_native.py','collect_evidence.py','live_event.py','brief_status.py',
    'record_plan_annotation.py','watch_marker.py','fetch_marker_proofs.py','deploy_source.py','marker_ros_smoke.py',
    'correct_hold_audit.py','investigate_hold.py','live_hold_gaps.py','explain_hold_evidence.py',
    'preserve_and_replace.py','continue_final_pair.py','diagnose_failed_attempt.py','verify_completion_native.py',
    'finalize_native.py','append_final_report.py'))
records=[]
for source in sorted(set(sources)):
    assert source.stat().st_size<3*1024**2,source
    target=dest/source.name;assert not target.exists(),target
    shutil.copy2(source,target)
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    assert hashlib.sha256(target.read_bytes()).hexdigest()==digest
    records.append(dict(path=target.name,source_path=str(source),bytes=target.stat().st_size,sha256=digest))
(dest/'artifact_manifest.json').write_text(json.dumps(dict(runtime_commit=head,files=records,raw_bags_or_weights_in_git=False),indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(evidence=str(dest),files=len(records),bytes=sum(r['bytes'] for r in records))))
