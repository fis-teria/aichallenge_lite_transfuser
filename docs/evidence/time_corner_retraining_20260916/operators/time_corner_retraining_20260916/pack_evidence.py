"""Copy selected small reports; datasets, raw bags and weights stay in WSL."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import shutil

repo = Path(__file__).resolve().parents[2]
base = Path(r'\\wsl.localhost\Ubuntu-22.04-Recovered\home\thistle\e2e_autonomous\runs')
out = repo/'docs/evidence/time_corner_retraining_20260916'
out.mkdir(parents=True, exist_ok=False)
sources = {
    'checks': (base/'time_corner_retraining_serial_checks_20260916',
               ['pytest.log','test_gate.json','pipeline_status.json']),
    'interrupted_attempt': (base/'time_corner_retraining_checks_20260916',
               ['pytest.log','test_gate.json','pipeline_status.json','audit.log','prepare.log','train.log']),
    'data': (base/'time_corner_retraining_20260916', ['resolved_plan.json','geometry_identity.json']),
    'training': (base/'time_corner_retraining_20260916/training_serial',
                 ['plan.json','history.json','result.json','verification.json','initial_validation.json',
                  'best_validation.json','validation_epoch_01.json','validation_epoch_02.json','validation_epoch_03.json']),
    'comparison': (base/'time_corner_retraining_20260916/comparison', ['summary.json']),
    'awsim': (base/'time_corner_model_lap_20260916',
             ['deployment_gate.json','loader_prefix_parity.json','ros_smoke_summary.json','shipping.json','transfer_verification.json',
              'post_environment.json','awsim_preserved.json','runner_start.json','runner_exit.json',
              'deployment_transfer_verification.json','evaluation_exit.json','source_manifest.json',
              'source_verification.json','install_verification.json','build_exit.json','smoke_exit.json',
              'awsim_reference_snapshot.json']),
    'awsim/evaluation': (base/'time_corner_model_lap_20260916/evaluation',
                         ['summary.json','raw_time_paths.png','control_timeline.png']),
    'awsim/launch_diagnosis': (base/'time_corner_model_lap_20260916/launch_diagnosis',
                         ['summary.json','commands.json','launch_geometry.png','normal_rviz.png',
                          'rviz_drive_002.png','rviz_after_freeze.png','image_conversion.json']),
    'awsim/run': (base/'time_corner_model_lap_20260916/raw/codex-time-corner-lap01',
                 ['host_result.json','lap_progress.json']),
}
for folder, (source, names) in sources.items():
    dest = out/folder
    dest.mkdir(parents=True)
    for name in names:
        p = source/name
        assert p.is_file() and p.stat().st_size < 5_000_000, p
        shutil.copy2(p, dest/name)
source = base/'time_corner_retraining_20260916/source_verification.json'
proof = json.loads(source.read_bytes())
compact = dict(status=proof['status'],source_verification_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    totals=proof['totals'],raw_rehashed=proof['raw_rehashed'],all_prepared_arrays_replayed=proof['all_prepared_arrays_replayed'],
    runs=[{k:r[k] for k in ['run_id','split','accepted_anchor_count','accepted_event_count','summary_sha256']} for r in proof['runs']])
(out/'data/source_summary.json').write_text(json.dumps(compact,indent=2),encoding='utf-8')
source = base/'time_corner_retraining_20260916/data_and_budget_verification.json'
proof = json.loads(source.read_bytes())
proof['preparation_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
proof['target_anchor_count'] = len(proof.pop('target_anchor_ids'))
proof['sampler']['per_anchor_repeat_histogram'] = dict(Counter(proof['sampler'].pop('per_anchor_presentations').values()))
(out/'data/sampler_summary.json').write_text(json.dumps(proof,indent=2),encoding='utf-8')
for folder in ['time_corner_retraining_20260916','time_corner_model_lap_20260916']:
    dest = out/'operators'/folder
    dest.mkdir(parents=True)
    for p in (repo/'tmp'/folder).glob('*.py'):
        shutil.copy2(p,dest/p.name)
(out/'.gitattributes').write_bytes(b'* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n')
manifest = {p.relative_to(out).as_posix():dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(dict(status='PACKED',files=len(manifest),bytes=sum(r['bytes'] for r in manifest.values()))))
