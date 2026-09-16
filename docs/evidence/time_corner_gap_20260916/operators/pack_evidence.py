"""Preserve compact proof; bags, reference arrays and training tensors stay native."""
from pathlib import Path
import hashlib
import json
import shutil

REPO = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
BASE = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous')
SRC = BASE / 'runs/time_corner_gap_20260916'
DEST = REPO / 'docs/evidence/time_corner_gap_20260916'
assert not DEST.exists()
DEST.mkdir(parents=True)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

def copy(source: Path, destination: Path) -> None:
    assert source.is_file() and source.stat().st_size < 4_000_000, source
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)

index = json.loads((SRC/'collection_index.json').read_bytes())
names = ('collection_index.json', 'coverage_final.json', 'candidate_screen.json', 'initial_diagnosis.json',
    'initial_schedule.json', 'effective_schedule.json', 'parallel_observation.json', 'c06_goal_geometry.json',
    'c06_20_bias_candidates.json', 'c06_20_override.json', 'completion_counter_compatibility.json', 'test_gate.json')
for name in names:
    copy(SRC/name, DEST/name)
for pattern in ('full_*.log', 'focused_*.log', 'test_gate_*.json', 'adaptive_pair*.json', '*_entry_diagnosis.json',
                'coverage_after_pair*.json', 'entry_availability*.json', 'entry_calibration*.json', 'entry_window*.json', 'fault_diagnosis_pair*.json', 'pair*_resource_monitor.jsonl', 'cornergap_pair*_verified.json', 'cornergap_pair*_cleanup.json'):
    for path in sorted(SRC.glob(pattern)):
        copy(path, DEST/path.name)
for path in sorted((SRC/'plans').glob('*.json')):
    copy(path, DEST/'plans'/path.name)
references = {p.relative_to(SRC).as_posix():dict(bytes=p.stat().st_size, sha256=sha(p))
              for p in sorted((SRC/'references').rglob('*')) if p.is_file()}
(DEST/'reference_manifest.json').write_text(json.dumps(references, indent=2))
for run in index['runs']:
    name = run['run_id']
    raw = BASE/'raw/time_corner_gap_20260916'/name
    for suffix in ('_collection_summary.json', '_clock.json', '_anchor_states.json'):
        if (SRC/(name+suffix)).exists():
            copy(SRC/(name+suffix), DEST/'run_reports'/name/suffix[1:])
    for filename in ('result.json', 'parallel_admission.json', 'disturbance_markers.json', 'compose.json'):
        if (raw/filename).exists():
            copy(raw/filename, DEST/'run_reports'/name/filename)
copy(SRC/'clock_audit.py', DEST/'operators/clock_audit.py')
for path in sorted(HERE.glob('*.py')):
    copy(path, DEST/'operators'/path.name)
for path in sorted(HERE.glob('revision_*.json')):
    copy(path, DEST/path.name)
copy(HERE/'host_final_checks.json', DEST/'host_final_checks.json')
(DEST/'.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
(DEST/'README.md').write_text('''# Targeted corner gap collection

collection_index.json indexes native WSL bags, observed 3-second teachers (30 xy points)
and causal camera/LiDAR/ego caches. Whole runs are separated into train/validation.
coverage_final.json compares original unchanged entry-state requirements before/after,
including earlier sealed collections. Preparation paths and failed events are not labels.

The heading/offset calibration changes preparation paths only. Physical stopping,
sensor and timing guards remain. record_actual_v1 retains acquisition overspeed samples.
The C06 map diagnostics and actual failed runs are retained; neither static feasibility
nor completion of a collection lap demonstrates learned E2E driving improvement.

Large arrays and bags remain outside Git. reference_manifest.json records path hashes.
operators are historical evidence. Restore under tmp/time_corner_gap_20260916 before use,
with transport dependencies recorded in ../time_corner_multiscale_20260916/operator_dependencies.
Campaign roots are sealed; fresh runs require fresh roots and a finite budget.
''', encoding='utf-8')
files = {p.relative_to(DEST).as_posix():dict(bytes=p.stat().st_size, sha256=sha(p))
         for p in sorted(DEST.rglob('*')) if p.is_file()}
assert sum(r['bytes'] for r in files.values()) < 20_000_000
(DEST/'manifest.json').write_text(json.dumps(dict(schema='corner_gap_evidence_v1', files=files,
    bytes=sum(r['bytes'] for r in files.values())), indent=2))
print(json.dumps(dict(files=len(files), bytes=sum(r['bytes'] for r in files.values()), path=str(DEST))))
