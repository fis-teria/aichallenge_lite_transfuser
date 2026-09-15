from pathlib import Path
import argparse
import ops60 as m

ap=argparse.ArgumentParser()
ap.add_argument('--run',required=True)
ap.add_argument('--split',choices=['train','validation'],required=True)
a=ap.parse_args()
assert a.run.startswith('codex-time-recovery-60cm-')
m.remote('OUT='+repr(m.OUT)+'\nRAW='+repr(m.RAW)+'\nNAME='+repr(a.run)+'\nSPLIT='+repr(a.split)+'\n'+r'''
from pathlib import Path
import json,subprocess,sys
out=Path(OUT)
for script,args in [('audit60.py',['--run',NAME,'--split',SPLIT]),('clock_audit.py',['--raw',str(Path(RAW)/NAME),'--output',str(out/(NAME+'_clock.json'))])]:
    log=out/(NAME+'_'+Path(script).stem+'_execution.log')
    with log.open('x') as stream:
        r=subprocess.run([sys.executable,str(out/script),*args],stdout=stream,stderr=subprocess.STDOUT,timeout=1800)
    print(json.dumps(dict(script=script,exit=r.returncode,log=str(log))),flush=True)
    assert r.returncode==0,log.read_text()[-6000:]
report=json.loads((out/(NAME+'_collection_summary.json')).read_text())
print(json.dumps({k:v for k,v in report.items() if k not in ('prepared','marker_locations','input_reasons','teacher_reasons')}))
''',native=True,lock=True,timeout=1900)
