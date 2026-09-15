from manage import remote, ROOT, UNC, HOST, HERE
import json
from pathlib import Path
import shutil
import subprocess

names=json.loads(remote(r'''
from pathlib import Path
import json
p=Path('/home/graneple/e2e_autonomous/time_recovery_sites_20260915/visual_proofs')
print(json.dumps([q.stem for q in p.glob('*.json')]))
'''))
destination=UNC/'visual_proofs';destination.mkdir(exist_ok=True)
for name in names:
    for ext in ('.json','.xwd'):
        dst=destination/(name+ext)
        if not dst.exists():
            subprocess.run(['scp',HOST+':'+ROOT+'/visual_proofs/'+name+ext,str(dst)],check=True)
remote(r'''
from pathlib import Path
import hashlib,json,struct
from PIL import Image
root=Path('/home/thistle/e2e_autonomous/runs/time_recovery_sites_20260915/visual_proofs')
for p in root.glob('*.xwd'):
 data=p.read_bytes();meta=json.loads(p.with_suffix('.json').read_text())
 assert hashlib.sha256(data).hexdigest()==meta['xwd_sha256']
 out=p.with_suffix('.png')
 if out.exists():continue
 h=struct.unpack('>25I',data[:100])
 assert h[1:4]==(7,2,24) and h[6:8]==(0,0) and h[11] in (24,32) and h[14:17]==(0xFF0000,0xFF00,0xFF)
 pixels=data[h[0]+h[19]*12:];assert len(pixels)==h[12]*h[5]
 Image.frombytes('RGB',(h[4],h[5]),pixels,'raw','BGR' if h[11]==24 else 'BGRX',h[12],1).save(out)
 print(json.dumps(dict(file=str(out),source_sha256=meta['xwd_sha256'],generated=False)))
''',native=True,lock=True)
for name in names:
    for ext in ('.json','.png'):
        src=destination/(name+ext);dst=HERE/(name+ext)
        if not dst.exists():shutil.copy2(src,dst)
