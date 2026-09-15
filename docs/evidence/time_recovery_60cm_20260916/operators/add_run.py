"""Deploy a new immutable reference and append bounded planned runs."""
import argparse,hashlib,json,tarfile
import ops60 as m

ap=argparse.ArgumentParser();ap.add_argument('--reference',required=True);ap.add_argument('--side',choices=['left','right'],required=True);ap.add_argument('--numbers',type=int,nargs='+',required=True);a=ap.parse_args()
assert a.reference.startswith('d60_g03_'+a.side)
archive=m.HERE/('references_'+a.reference+'.tar.gz')
with tarfile.open(archive,'x:gz') as tf:
    for p in sorted((m.UNC/a.reference).iterdir()):
        assert p.is_file()
        tf.add(p,arcname='planned_references/'+a.reference+'/'+p.name,recursive=False)
digest=hashlib.sha256(archive.read_bytes()).hexdigest();m.copy_to_host([archive])
m.remote('ROOT='+repr(m.ROOT)+'\nARCHIVE='+repr(archive.name)+'\nSHA='+repr(digest)+'\nTAG='+repr(a.reference)+'\nSIDE='+repr(a.side)+'\nNUMBERS='+repr(a.numbers)+'\n'+r'''
from pathlib import Path
import hashlib,json,tarfile,subprocess
root=Path(ROOT)
assert not subprocess.check_output(['docker','ps','-q'],text=True).strip()
ledger=json.loads((root/'campaign_20260914.json').read_text());assert not ledger['sealed']
assert len(ledger['attempts'])<ledger['maximum_attempts'] and ledger['maximum_event_cap']==3
assert hashlib.sha256((root/ARCHIVE).read_bytes()).hexdigest()==SHA
assert not (root/'planned_references'/TAG).exists()
with tarfile.open(root/ARCHIVE) as tf:
    for member in tf.getmembers():
        dest=root/member.name
        assert dest.resolve().is_relative_to(root/'planned_references'/TAG) and member.isfile()
        dest.parent.mkdir(parents=True,exist_ok=True)
        with dest.open('xb') as f:f.write(tf.extractfile(member).read())
ref=json.loads((root/'planned_references'/TAG/(SIDE+'.json')).read_text())
assert ref['large_recovery']['map_screen_pass'] and ref['large_recovery']['config']['event_cap']==3
for number in NUMBERS:
    name='codex-time-recovery-60cm-d60-g03-'+SIDE+'-r%02d'%number
    existing=next((r for r in ledger['planned_runs'] if r['run_id']==name),None)
    assert not any(r['run_id']==name for r in ledger['attempts'])
    row=dict(run_id=name,side=SIDE,pair=number if SIDE=='left' else 10+number,event_cap=3,reference=TAG)
    if existing:existing.update(row)
    else:ledger['planned_runs'].append(row)
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print(json.dumps(dict(status='PLANNED_REFERENCE_ADDED',reference=TAG,side=SIDE,numbers=NUMBERS)))
''')
