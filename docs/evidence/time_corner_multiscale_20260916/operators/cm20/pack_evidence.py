from pathlib import Path
import shutil,hashlib,json
REPO=Path(__file__).resolve().parents[2]
BASE=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous')
DEST=REPO/'docs/evidence/time_corner_multiscale_20260916'
assert not DEST.exists()
DEST.mkdir(parents=True)
def copy(src,dst):
 assert src.is_file() and src.stat().st_size<4_000_000,(src,src.stat().st_size)
 dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for cm in (20,40,60):
 src=BASE/'runs'/f'time_corner_multiscale{cm}_20260916';dst=DEST/f'cm{cm}'
 index=json.loads((src/'collection_index.json').read_bytes())
 for name in ('collection_index.json','coverage_final.json','catalog.json','map_screen.json','test_gate.json','late_addition_screen.json','speed_abort_replay.json','map_rejection_diagnosis.json','near_corner_screen.json','critical_state_final.json','combined_20cm_coverage.json','multiscale_summary.json','collection_schedule.json','c06_wider_heading_replay.json','c06_preparation_snapshot.json','c06_start_window_diagnosis.json'):
  if (src/name).exists():copy(src/name,dst/name)
 for pattern in ('full_*.log','focused_*.log','test_gate_*.json','collection_schedule_*.json','*_diagnosis.json','pair*_resource_monitor.jsonl',f'corner{cm}_pair*_verified.json',f'corner{cm}_pair*_cleanup.json'):
  for path in sorted(src.glob(pattern)):copy(path,dst/path.name)
 for path in sorted((src/'plans').glob('*.json')):copy(path,dst/'plans'/path.name)
 reference_manifest={}
 for path in sorted((src/'references').rglob('*')):
  if path.is_file():reference_manifest[path.relative_to(src).as_posix()]=dict(bytes=path.stat().st_size,sha256=sha(path))
 (dst/'reference_manifest.json').write_text(json.dumps(reference_manifest,indent=2))
 for run in index['runs']:
  name=run['run_id'];raw=BASE/'raw'/f'time_corner_multiscale{cm}_20260916'/name
  for suffix in ('_collection_summary.json','_clock.json','_anchor_states.json'):
   if (src/(name+suffix)).exists():copy(src/(name+suffix),dst/'run_reports'/name/(suffix[1:]))
  for namefile in ('result.json','parallel_admission.json','disturbance_markers.json','compose.json'):
   if (raw/namefile).exists():copy(raw/namefile,dst/'run_reports'/name/namefile)
 copy(src/'clock_audit.py',DEST/'operators'/f'cm{cm}'/'clock_audit.py')
 operators=REPO/'tmp'/f'time_corner_multiscale{cm}_20260916'
 for path in sorted(operators.glob('*.py')):copy(path,DEST/'operators'/f'cm{cm}'/path.name)
 for path in sorted(operators.glob('revision_*.json')):copy(path,dst/path.name)
 copy(operators/'host_final_checks.json',dst/'host_final_checks.json')
for rel in ('tmp/time_parallel_multiscale_20260916/ops.py','tmp/time_recovery_60cm_20260916/ops60.py','tmp/time_recovery_large_live_20260915/manage_live.py','tmp/time_recovery_separated_20260915/manage.py','tmp/time_ten_site_recovery_plan_20260915/sync_native_transport.py'):
 copy(REPO/rel,DEST/'operator_dependencies'/rel)
(DEST/'.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
(DEST/'README.md').write_text('''# Corner recovery collection: 20 / 40 / 60 cm

Each collection_index.json indexes verified observed teachers, causal input caches and raw runs.
Teachers contain the actual future 3 seconds, 30 xy points. Preparation paths are not labels.
New runs explicitly record record_actual_v1; acquisition upper-speed exceedances do not discard events.
The independent physical-model domain, stopping sweep, sensor and nonfinite monitors remain.

cm20/multiscale_summary.json aggregates the phases and measured concurrent moving intervals.
cm20/combined_20cm_coverage.json combines old/new 20cm data under the original entry-state criteria.
Nearby C03A/C06A are not counted as the original C03/C06 entry targets.

Large raw bags, teacher arrays, caches and reference arrays remain in native WSL, outside Git.
reference_manifest.json permits hash verification of generated paths. run_reports contains actual
speed, targets, domains and controller results. These are teacher collection results, not evidence
of improved closed-loop learned E2E performance.

operators are historical evidence, not launchers from this archived location. To reproduce,
restore operator_dependencies to the original tmp layout and put each cm20/40/60 operator under
tmp/time_corner_multiscale{cm}_20260916. Original campaigns are sealed: use new roots and finite
budgets for a new campaign. Do not modify AWSIM itself.
''',encoding='utf-8')
files={p.relative_to(DEST).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(DEST.rglob('*')) if p.is_file()}
assert sum(r['bytes'] for r in files.values())<20_000_000
(DEST/'manifest.json').write_text(json.dumps(dict(schema='corner_multiscale_evidence_v1',files=files,bytes=sum(r['bytes'] for r in files.values())),indent=2))
print(json.dumps(dict(files=len(files),bytes=sum(r['bytes'] for r in files.values()),path=str(DEST))))
