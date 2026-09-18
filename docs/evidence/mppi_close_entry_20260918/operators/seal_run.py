"""Seal a closed collection run; stream a hash-checked tar without staging a copy."""
from __future__ import annotations
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tarfile

REPO=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
ROOT=Path('/home/graneple/e2e_autonomous/mppi_close_adjust_20260918')

def sha(p: Path) -> str:
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''): h.update(block)
    return h.hexdigest()

def main() -> None:
    p=argparse.ArgumentParser();p.add_argument('run_id');p.add_argument('--export',action='store_true');a=p.parse_args()
    assert re.fullmatch(r'lidar-v45-[a-z0-9-]+',a.run_id)
    run=REPO/'output/scenario_tool'/a.run_id
    manifest=ROOT/(a.run_id+'-manifest.json')
    assert not subprocess.check_output(['docker','ps','-aq','--filter','label=com.docker.compose.project=codex-'+a.run_id],text=True).strip()
    if not a.export:
        assert (run/'d1/e2e-rosbag-status.txt').read_text().strip()=='completed'
        files={f'raw/{a.run_id}/{f.relative_to(run)}':f for f in sorted(run.rglob('*')) if f.is_file() and not f.is_symlink()}
        for f in sorted(ROOT.iterdir()):
            if f.is_file() and (f.name in ['collect_v45.py','teacher_lidar_v45.bash','seal_run.py','check_teacher_graph.py','live_graph.py']
                    or f.name.startswith(a.run_id) and f!=manifest):
                files['provenance/'+f.name]=f
        for suffix in ['yaml','json']:
            f=ROOT/'scenarios'/(a.run_id+'.'+suffix);files['provenance/scenarios/'+f.name]=f
        for name in ['runtime-identity.json','build-result.json','test-results.txt']:
            f=ROOT/'runtime'/name;files['provenance/runtime/'+name]=f
        for f in sorted((ROOT/'source').rglob('*')):
            if f.is_file() and '__pycache__' not in f.parts:
                files['provenance/source/'+f.relative_to(ROOT/'source').as_posix()]=f
        for f in ROOT.glob('runtime-smoke*.log'):
            files['provenance/'+f.name]=f
        rows={n:dict(source=str(f),bytes=f.stat().st_size,sha256=sha(f)) for n,f in files.items()}
        doc=dict(schema_version=1,run_id=a.run_id,files=rows)
        with manifest.open('x') as f:json.dump(doc,f,indent=2);f.write('\n')
        print(json.dumps(dict(run_id=a.run_id,files=len(rows),bytes=sum(r['bytes'] for r in rows.values()))));return
    doc=json.loads(manifest.read_text())
    with gzip.GzipFile(fileobj=sys.stdout.buffer,mode='wb',compresslevel=1) as compressed:
        with tarfile.open(fileobj=compressed,mode='w|') as archive:
            archive.add(manifest,arcname='export_manifest.json',recursive=False)
            for name,row in doc['files'].items():
                f=Path(row['source']);assert f.stat().st_size==row['bytes'] and sha(f)==row['sha256'],name
                archive.add(f,arcname=name,recursive=False)

if __name__=='__main__':main()
