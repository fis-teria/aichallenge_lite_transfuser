"""Copy bounded diagnosis evidence from native WSL; never copy bags or weights."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

REPO=Path(r'E:\workspace\e2e_lite_transfuser')
BASE=Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous')
ROOT=BASE/'runs/time_launch_regression_20260916'
OUT=REPO/'docs/evidence/time_launch_regression_20260916'
OUT.mkdir(parents=True,exist_ok=False)

def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

sources={}
def copy(source: Path, target: str) -> None:
    assert source.is_file() and source.stat().st_size<2_000_000,source
    assert source.suffix not in ('.db3','.pt','.pth','.npy','.npz'),source
    destination=OUT/target
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(source,destination)
    assert sha(source)==sha(destination)
    sources[target]=dict(source=str(source),sha256=sha(source))

for group,names in {
    'nominal':['summary.json','provenance.json','inventory.json'],
    'baseline':['summary.json','provenance.json','anchors.json','old_identity.json','epoch1_identity.json','epoch2_identity.json','epoch3_identity.json'],
    'supplement':['paired_geometry_summary.json','near_arm_cases.json','representative_case.json',
        'paired_geometry.png','paired_acceptance_matrix.png','plot_provenance.json',
        'failed_trial_delay.json','training_audit.json','waiting_input_pairs.json','provenance.json'],
}.items():
    for name in names:copy(ROOT/group/name,f'{group}/{name}')
copy(ROOT/'pytest.log','checks/pytest.log')
copy(ROOT/'implementation_verification.json','checks/implementation_verification.json')
copy(BASE/'runs/time_recovery_multiscale_20260916/training/result.json','training/old_result.json')
copy(BASE/'runs/time_corner_retraining_20260916/training_serial/result.json','training/new_result.json')
copy(BASE/'runs/time_corner_retraining_20260916/training_serial/history.json','training/history.json')
for name in ['compare_native.py','supplement_native.py','render_acceptance_native.py']:
    source=REPO/'tmp/time_launch_regression_20260916'/name
    assert sha(source)==sha(ROOT/'operators'/name)
    copy(source,'operators/'+name)
copy(Path(__file__),'operators/pack_evidence.py')
(OUT/'.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
proof=dict(source_git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
    sources=sources,excluded='raw sensor data, weights, full arrays, full per-control diagnostics',
    original_supplement_line_plot_replaced_in_bundle_by_exact_count_matrix=True,
    read_only_analysis=True,optimizer_steps=0,new_awsim_runs=0,sealed_test_read=False,
    pytest=dict(exit_code=0,passed=2767,skipped=4,warnings=84,elapsed_s=141.94))
(OUT/'checks/bundle_provenance.json').write_text(json.dumps(proof,indent=2)+'\n',encoding='utf-8')
manifest={p.relative_to(OUT).as_posix():dict(bytes=p.stat().st_size,sha256=sha(p))
    for p in sorted(OUT.rglob('*')) if p.is_file()}
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(files=len(manifest),bytes=sum(v['bytes'] for v in manifest.values()),output=str(OUT))))
