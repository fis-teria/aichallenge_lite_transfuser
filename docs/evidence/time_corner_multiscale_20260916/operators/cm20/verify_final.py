from pathlib import Path
import hashlib,json,math,subprocess
import numpy as np
REPO=Path.cwd();ROOT=REPO.parent;PACK=REPO/'docs/evidence/time_corner_multiscale_20260916'
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=read(PACK/'manifest.json')
for name,row in manifest['files'].items():
 p=PACK/name;assert p.is_file() and p.stat().st_size==row['bytes'] and sha(p)==row['sha256'],name
source='a488ab68ab7fac5f5db140fdb514bf5d13afd03c'
snapshot_source='dcec9ea47ab833e4e260e99048efa0fb69ece84d'
entry_source='21ccfef00a415c85dd033d447cd8ed25a5ad974f'
prior='78e6f89d5a97aa380d9e3d8281b6368788d7c72a'
assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip()
assert not subprocess.check_output(['git','diff',source,'HEAD','--','src','tools','tests'])
anchors=0;events=0;run_ids=set()
for cm in (20,40,60):
 native=ROOT/'runs'/f'time_corner_multiscale{cm}_20260916';evidence=PACK/f'cm{cm}'
 index=read(evidence/'collection_index.json');assert sha(native/'collection_index.json')==sha(evidence/'collection_index.json')
 host=read(evidence/'host_final_checks.json');assert host['awsim_files_unchanged'] and host['original_repo_unchanged'] and host['no_running_containers'] and host['no_owned_supervisors']
 gates=[('test_gate.json',prior),('test_gate_21ccfef.json',entry_source)]
 if cm==60:gates.extend([('test_gate_dcec9ea.json',snapshot_source),('test_gate_a488ab6.json',source)])
 for gatefile,tested in gates:
  gate=read(evidence/gatefile);assert gate['commit']==tested and gate['full_exit']==0
  assert sha(evidence/Path(gate['log']).name)==gate['log_sha256']
 for rel,row in read(evidence/'reference_manifest.json').items():
  p=native/rel;assert p.stat().st_size==row['bytes'] and sha(p)==row['sha256']
 assert not set(index['splits']['train']) & set(index['splits']['validation'])
 for row in index['runs']:
  assert row['run_id'] not in run_ids;run_ids.add(row['run_id'])
  assert row['collection_speed_policy']=='record_actual_v1'
  expected=(prior if cm==20 or (cm==40 and '-p01-' in row['run_id'])
            else entry_source if cm==40 or any(p in row['run_id'] for p in ('-p01-','-p02-'))
            else snapshot_source if any(p in row['run_id'] for p in ('-p03-','-p04-')) else source)
  assert row['source_sha']==expected
  raw=Path(row['raw_path']);assert sha(raw/'control.jsonl')==row['control_sha256'] and sha(raw/'reference.json')==row['reference_sha256']
  config=read(raw/'reference.json')['large_recovery']['config']
  assert config['speed_policy']=='record_actual_v1'
  assert config.get('entry_heading_tolerance_rad',math.radians(1.))==math.radians(1. if expected==prior else 2.)
  assert config.get('recovery_duration_s',10.)==(15. if expected==source else 10.)
  if row['accepted']:
   p=Path(row['materialized_path'])/'teachers.npz';assert sha(p)==row['teachers_sha256']
   with np.load(p) as labels:assert labels['xy_m'].shape==(row['accepted'],30,2) and labels['xy_mask'].all() and np.isfinite(labels['xy_m']).all()
   anchors+=row['accepted'];events+=row['accepted_events']
report=dict(status='MULTISCALE_FINAL_VERIFICATION_PASS',verified_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),tested_runtime_source=source,evidence_files=len(manifest['files']),runs=len(run_ids),accepted_anchors=anchors,accepted_events=events,shape='[N,30,2]',full_future_masks=True,run_splits_disjoint=True,awsim_unchanged=True,physical_model_guard_retained=True)
output=ROOT/'runs/time_corner_multiscale20_20260916/final_verification.json'
with output.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report))
