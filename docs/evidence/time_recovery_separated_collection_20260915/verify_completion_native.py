"""Validate the final finite amendment, coverage and whole-run split against raw provenance."""
from collections import Counter
import hashlib
import json
from pathlib import Path

OUT=Path('/home/thistle/e2e_autonomous/runs/time_recovery_separated_20260915')
RAW=Path('/home/thistle/e2e_autonomous/raw/time_recovery_separated_20260915')
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
original=read(OUT/'selected_site_plan.json');effective=read(OUT/'effective_data_plan.json')
amendment=read(OUT/'replacement_amendment.json');index=read(OUT/'collection_index.json')
assert index['pending_runs']==[] and index['completed_audited_runs']==10
assert index['successful_events']==index['observed_events']==22
assert index['plan_sha256']==amendment['original_selection_sha256']==sha(OUT/'selected_site_plan.json')
assert effective['replacement_amendment_sha256']==sha(OUT/'replacement_amendment.json')
assert index['effective_data_plan_sha256']==sha(OUT/'effective_data_plan.json')
modified=dict(effective);modified.pop('replacement_amendment_sha256')
modified['runs']=[dict(r) for r in effective['runs']]
replacement=modified['runs'][8]
assert replacement.pop('replacement_for')==amendment['failed_run_id']
assert replacement['run_id']==amendment['replacement_run_id']
replacement['run_id']=amendment['failed_run_id']
assert modified==original, 'OTHER_SELECTION_OR_SPLIT_CHANGE'
names={r['run_id'] for r in index['runs']};assert len(names)==10
assert names=={r['run_id'] for r in effective['runs']}
assert amendment['failed_run_id'] not in names
train={r['run_id'] for r in effective['runs'] if r['split']=='train'}
validation={r['run_id'] for r in effective['runs'] if r['split']=='validation'}
assert len(train)==8 and len(validation)==2 and not train&validation
coverage=Counter((h['site_id'],h['sign']) for h in index['holds'])
assert len(coverage)==22 and set(coverage.values())=={1}
assert {s for s,_ in coverage}=={'S00',*(f'R{i:02d}' for i in range(1,11))}
assert all(coverage[(site,sign)]==1 for site,_ in coverage for sign in (-1,1))
for row in effective['runs']:
    root=RAW/row['run_id'];result=read(root/'result.json')
    assert sha(root/'reference.json')==row['reference_sha256']
    assert result['source_sha']==amendment['runtime_source_commit']
    assert result['status']=='COMPLETE_LAP' and not result['last_control']['fault']
    assert result['last_control']['stop_confirmed'] and result['nodes']['closed_bag'] and not result['cleanup_errors']
    summary=read(OUT/(row['run_id']+'_collection_summary.json'))
    assert summary['split']==row['split'] and summary['all_sites_have_at_least_60_anchors']
    assert summary['accepted']==summary['prepared']['input_valid']==summary['prepared']['anchors']
    assert summary['prepared']['real_input_replay_equal'] and not summary['prepared']['input_reason_refinements']
assert not index['training_started'] and not index['sealed_test_read']
value=dict(status='COMPLETE_VERIFIED',successful_laps=10,successful_events=22,
    attempted_runs=11,failed_pre_event_runs_preserved=1,all_11_sites_both_directions=True,
    each_event_has_at_least_60_anchors=True,accepted_anchors_by_split=index['accepted_anchors_by_split'],
    train_runs=sorted(train),validation_runs=sorted(validation),whole_run_split_disjoint=True,
    original_site_reference_seed_split_unchanged=True,only_data_run_id_replaced=True,
    all_normal_stop_closed_bag=True,all_real_input_replay_equal=True,
    runtime_commit=amendment['runtime_source_commit'],runtime_or_safety_change_on_retry=False,
    collection_index_sha256=sha(OUT/'collection_index.json'),training_started=False,sealed_test_read=False)
with (OUT/'completion_verification.json').open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
print(json.dumps(value))
