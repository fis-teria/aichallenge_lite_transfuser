"""Preserve one ordinary-RViz XWD capture as a lossless, hash-linked PNG."""
import argparse,hashlib,json,struct,subprocess
from PIL import Image
import ops60 as m

ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);a=ap.parse_args()
assert a.run.startswith('codex-time-recovery-60cm-d60-g03-')
summary=json.loads((m.UNC/(a.run+'_collection_summary.json')).read_bytes())
assert summary['three_event_qualified']
prefix=a.run+'_event3'
for suffix in ('.json','.xwd'):
    dest=m.HERE/(prefix+suffix);assert not dest.exists()
    subprocess.run(['scp',m.HOST+':'+m.ROOT+'/visual_proofs/'+prefix+suffix,str(dest)],check=True,timeout=60)
proof=json.loads((m.HERE/(prefix+'.json')).read_bytes());data=(m.HERE/(prefix+'.xwd')).read_bytes()
sha=lambda b:hashlib.sha256(b).hexdigest()
assert sha(data)==proof['xwd_sha256']
assert len(proof['marker_locations']['events'])==3 and 'rviz2' in proof['heartbeat']['marker_subscribers']
h=struct.unpack('>25I',data[:100])
assert h[1:4]==(7,2,24) and h[6:8]==(0,0)
assert h[11] in (24,32) and h[14:17]==(0xff0000,0xff00,0xff)
pixels=data[h[0]+h[19]*12:];assert len(pixels)==h[12]*h[5]
short=a.run.removeprefix('codex-time-recovery-60cm-d60-g03-')
png=m.UNC/('rviz_'+short+'_event3.png');assert not png.exists()
Image.frombytes('RGB',(h[4],h[5]),pixels,'raw','BGR' if h[11]==24 else 'BGRX',h[12],1).save(png)
provenance=dict(run_id=a.run,source_xwd_sha256=sha(data),capture=proof,png_sha256=sha(png.read_bytes()),
 conversion='Lossless XWD BGR/BGRX to RGB PNG; no pixel edits',
 scope='Ordinary RViz path and preparation-start marker display; target displacement proven separately by causal camera anchor measurements')
with png.with_name(png.stem+'_provenance.json').open('x') as f:json.dump(provenance,f,indent=2)
print(json.dumps(dict(png=str(png),width=h[4],height=h[5])))
