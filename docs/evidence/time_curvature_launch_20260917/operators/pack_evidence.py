"""Preserve small trial evidence; raw bags and weights stay on Linux."""
from pathlib import Path
import hashlib
import json
import shutil

here = Path(__file__).resolve().parent
repo = here.parents[1]
native = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs\time_curvature_launch_20260917')
raw = native / 'raw/codex-time-curve15-lap02'
out = repo / 'docs/evidence/time_curvature_launch_20260917'
assert json.loads((native / 'transfer_verification.json').read_bytes())['status'] == 'PASS'
assert json.loads((native / 'evaluation/summary.json').read_bytes())['control_replay']['status'] == 'PASS'
out.mkdir(parents=True, exist_ok=False)
copied = {}


def copy(src: Path, dest: Path) -> None:
    assert src.is_file() and src.stat().st_size < 5_000_000, src
    assert src.suffix in ('.json', '.py', '.log', '.png'), src
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    assert src.read_bytes() == dest.read_bytes(), src
    copied[dest.relative_to(out).as_posix()] = str(src)


for path in sorted(native.iterdir()):
    if path.suffix in ('.json', '.log'):
        copy(path, out / 'checks' / path.name)
for folder in ['evaluation', 'stop_location', 'baseline_replay', 'offline_preview']:
    directory = native / folder
    if directory.exists():
        for path in sorted(directory.iterdir()):
            if path.suffix in ('.json', '.png'):
                copy(path, out / folder / path.name)
for name in ['host_result.json', 'lap_progress.json', 'trial_config.json', 'drive_authorized.json',
             'control_heartbeat.json', 'inference_heartbeat.json', 'transfer_manifest.json', 'video_recording.json']:
    copy(raw / name, out / 'recorded' / name)
for path in sorted(here.glob('*.py')):
    copy(path, out / 'operators' / path.name)
(out / '.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
(out / 'source_paths.json').write_text(json.dumps(copied, indent=2) + '\n', encoding='utf-8')
manifest = {p.relative_to(out).as_posix(): dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            for p in sorted(out.rglob('*')) if p.is_file()}
(out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
print(json.dumps(dict(status='PACKED', files=len(manifest), bytes=sum(v['bytes'] for v in manifest.values()))))
