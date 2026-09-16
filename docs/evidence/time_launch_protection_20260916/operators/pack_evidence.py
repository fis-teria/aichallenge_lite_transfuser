"""Copy bounded reports with byte hashes; all model/data files remain native."""
from pathlib import Path
import hashlib
import json
import shutil

repo=Path(__file__).resolve().parents[2]
base=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs')
source=base/'time_launch_protection_v2_20260916'
out=repo/'docs/evidence/time_launch_protection_20260916'
assert json.loads((source/'driver_status.json').read_bytes())['status']=='COMPLETE'
assert json.loads((source/'summary/comparison.json').read_bytes())['status']=='PASS'
out.mkdir(parents=True,exist_ok=False)
copied={}

def copy(src,dest):
    assert src.is_file() and src.stat().st_size<5_000_000,src
    assert src.suffix not in ('.pt','.pth','.npy','.db3','.mcap','.bag','.tar'),src
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dest)
    assert src.read_bytes()==dest.read_bytes()
    copied[dest.relative_to(out).as_posix()]=str(src)

for name in ['driver_status.json','driver_history.jsonl','pytest.log','teacher_oracle_audit.json',
             'teacher_oracle.log','prepare.log','existing_gate.log','protocol_control.log',
             'launch_balanced.log','final_evaluation.log']:
    copy(source/name,out/'checks'/name)
for folder in ['summary','evaluation_existing','evaluation_final']:
    for path in sorted((source/folder).iterdir()):
        if path.suffix in ('.json','.png'):copy(path,out/folder/path.name)
for name in ['launch_cases.json','baseline_launch_metadata.json']:
    copy(source/'preparation'/name,out/'preparation'/name)
for arm in ['protocol_control','launch_balanced']:
    for name in ['status.json','plan.json','history.json','result.json','verification.json',
                 'initial_validation.json','best_validation.json',
                 'validation_epoch_01.json','validation_epoch_02.json','validation_epoch_03.json']:
        copy(source/arm/name,out/arm/name)
for name in ['driver_status.json','driver_history.jsonl','teacher_oracle_audit.json','protocol_control.log']:
    copy(base/'time_launch_protection_20260916'/name,out/'superseded_gate_attempt'/name)
for name in ['run_native_v2.py','oracle_native.py','summarize_native.py','diagnose_normal_native.py']:
    copy(source/name,out/'operators'/name)
copy(Path(__file__),out/'operators/pack_evidence.py')
(out/'.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
(out/'source_paths.json').write_text(json.dumps(copied,indent=2)+'\n',encoding='utf-8')
manifest={p.relative_to(out).as_posix():dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
          for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status='PACKED',files=len(manifest),bytes=sum(r['bytes'] for r in manifest.values()))))
