"""Check the frozen gate using exact teachers, before new training results exist."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from aic_transfuser_lite.evaluation.time_stage_selection_v1 import StageSelectionPolicy, select_stage_candidate

old=Path('../runs/time_launch_protection_20260916')
out=Path('../runs/time_launch_protection_v2_20260916')
cases={r['case_id']:r for r in json.loads((old/'preparation/launch_cases.json').read_bytes())}
reports=[]
for name in ['initial','previous_epoch2']:
    report=json.loads((old/'evaluation_existing'/(name+'.json')).read_bytes())
    for row in report['launch']:
        row['teacher_steer_rad']=cases[row['case_id']]['teacher'].get('steer_rad')
    reports.append(report)
oracle=deepcopy(reports[0]);oracle['candidate_id']='teacher_oracle'
for row in oracle['xy'].values():row.update(ade_m=0.,endpoint_3s_m=0.)
for row in oracle['launch']:
    teacher=cases[row['case_id']]['teacher']
    row.update(accepted=teacher['accepted'],steer_rad=teacher.get('steer_rad'),reason=teacher['reason'])
selection=select_stage_candidate([*reports,oracle],baseline_id='initial',policy=StageSelectionPolicy())
assert selection['decisions'][-1]['eligible']
assert selection['selected_candidate_id']=='teacher_oracle'
assert not selection['decisions'][1]['eligible']
assert 'launch:PP_REJECTED' in selection['decisions'][1]['reasons']
evidence=dict(status='PASS',purpose='teacher consistency only, no checkpoint promotion',selection=selection,
    prior_audit_sha256=hashlib.sha256((old/'teacher_oracle_audit.json').read_bytes()).hexdigest(),
    interrupted_training_status=json.loads((old/'protocol_control/status.json').read_bytes()),
    interrupted_training_history=json.loads((old/'protocol_control/history.json').read_bytes())
        if (old/'protocol_control/history.json').exists() else [],
    new_trained_validation_seen=False)
assert not evidence['interrupted_training_history']
(out/'teacher_oracle_audit.json').write_text(json.dumps(evidence,indent=2)+'\n')
print(json.dumps(dict(status='PASS',teacher_cases=len(cases),teacher_eligible=True,known_bad_rejected=True)),flush=True)
