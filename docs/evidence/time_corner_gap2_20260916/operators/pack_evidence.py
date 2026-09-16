"""Archive compact provenance; retain bags and tensors only in native WSL."""
from pathlib import Path
import hashlib, json, shutil
import ops_corner as m

src=m.UNC;dest=m.REPO/'docs/evidence/time_corner_gap2_20260916'
dest.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def copy(a,b):
    assert a.is_file() and a.stat().st_size<4_000_000,a
    b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a,b)

for pattern in ('collection_index.json','coverage_final.json','candidate_screen.json','test_gate*.json',
                'full_*.log','focused_*.log','adaptive_pair*.json','coverage_after_pair*.json',
                '*overrides.json','*override.json','*screen.json','pair*_entry_diagnosis.json',
                'fault_diagnosis_pair*.json','pair*_resource_monitor.jsonl','cornergap2_pair*_verified.json',
                'cornergap2_pair*_cleanup.json','parallel_observation.json','remaining_coverage_diagnosis.json'):
    for p in sorted(src.glob(pattern)):copy(p,dest/p.name)
for p in sorted((src/'plans').glob('*.json')):copy(p,dest/'plans'/p.name)
references={p.relative_to(src).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted((src/'references').rglob('*')) if p.is_file()}
(dest/'reference_manifest.json').write_text(json.dumps(references,indent=2))
index=json.loads((src/'collection_index.json').read_bytes())
raw=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered')/m.RAW.lstrip('/')
for run in index['runs']:
    name=run['run_id']
    for suffix in ('_collection_summary.json','_clock.json','_anchor_states.json'):
        if (src/(name+suffix)).exists():copy(src/(name+suffix),dest/'run_reports'/name/suffix[1:])
    for filename in ('result.json','parallel_admission.json','disturbance_markers.json','compose.json'):
        if (raw/name/filename).exists():copy(raw/name/filename,dest/'run_reports'/name/filename)
copy(src/'clock_audit.py',dest/'operators/clock_audit.py')
for p in sorted(m.HERE.glob('*.py')):copy(p,dest/'operators'/p.name)
copy(m.HERE/'host_final_checks.json',dest/'host_final_checks.json')
for pattern in ('bootstrap.json','revision_*.json'):
    for p in sorted(m.HERE.glob(pattern)):copy(p,dest/p.name)
(dest/'.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
(dest/'README.md').write_text('''# Remaining corner recovery collection

collection_index.json indexes verified native WSL bags and observed 3-second teachers
with causal camera/LiDAR/ego input. Whole runs are split into train/validation.
coverage_final.json includes the previous sealed gap collection and earlier datasets.
Original corner targets and coverage requirements stay unchanged. Preparation paths,
including blended command origins, are not labels. Static screens are not dynamic proof.
Source test gates and live results record which runtime revision each run used.

Raw bags and tensor arrays are outside Git. reference_manifest.json preserves hashes.
Operators are historical evidence; restore under tmp/time_corner_gap2_20260916,
using the transport dependencies in ../time_corner_multiscale_20260916/operator_dependencies.
Sealed roots must not be reused for new collection.
''',encoding='utf-8')
files={p.relative_to(dest).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(dest.rglob('*')) if p.is_file()}
assert sum(r['bytes'] for r in files.values())<20_000_000
(dest/'manifest.json').write_text(json.dumps(dict(schema='corner_gap2_evidence_v1',files=files,bytes=sum(r['bytes'] for r in files.values())),indent=2))
print(json.dumps(dict(files=len(files),bytes=sum(r['bytes'] for r in files.values()),path=str(dest))))
