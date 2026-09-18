from pathlib import Path
import json,hashlib,collections
root=Path('/home/thistle/e2e_autonomous/runs/mppi_v45_pc10_20260918')
selected=root/'collect10_curated_v1';validated=root/'collect10_validation_v1'
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
manifest=json.loads((selected/'selection_manifest.json').read_text())
replay=json.loads((validated/'summary.json').read_text())
assert len(manifest['runs'])==len(replay['runs'])==6
assert sha(selected/'selection_manifest.json')==replay['selection_manifest_sha256']
for name,digest in manifest['output_sha256'].items():assert sha(selected/name)==digest,name
rows=[];reason_counts=collections.Counter();use_counts=collections.Counter();all_bytes=0;all_files=0
for m,r in zip(manifest['runs'],replay['runs']):
    name=m['run_id'];assert name==r['run_id'] and m['selected']==r['replay_verified']
    if m['selected']:assert sha(validated/(name+'.pt'))==r['shard_sha256']
    collected=root/'collected'/name;receipt=json.loads((collected/'transfer_verified.json').read_text())
    assert receipt['all_sha256_match']
    export=json.loads((collected/'export_manifest.json').read_text())
    all_bytes+=sum(x['bytes'] for x in export['files'].values());all_files+=len(export['files'])
    identity=json.loads((collected/'raw'/name/'d1/teacher-runtime-identity.json').read_text())
    assert not identity['early_entry_search'] and abs(identity['speed_cap_mps']-10/3.6)<1e-12
    outcome=json.loads((collected/'raw'/name/'result.json').read_text())
    geometry=json.loads((root/'collect10_prefix_clearance_v1'/name/'summary.json').read_text())
    coverage=r['coverage'];quality=coverage['teacher_pose_prefix']['quality']
    reason_counts.update(m.get('reasons',{}));use_counts.update(m.get('uses',{}))
    rows.append(dict(run_id=name,verdict=outcome['scenario_verdict'],official_penalties=coverage['official_penalties'],
      pose_quality=quality,source_camera_anchors=m['source_camera_anchors'],prefix_candidates=m['prefix_candidates'],
      selected=m['selected'],hold=m['hold'],exclude=m['exclude'],thinned=m['eligible_thinned'],uses=m['uses'],
      split_group=m['split_group'],split=m['split'],replay_verified=r['replay_verified'],
      front_0to6m_abs_y_le1m=r['front_0to6m_abs_y_le1m'],front_0to6m=r['front_0to6m'],front_0to12m=r['front_0to12m'],
      projected_clearance=geometry.get('objects'),wall_overlap_samples=geometry.get('recorded_pose_wall_overlap_samples'),
      shard_sha256=r.get('shard_sha256')))
report=dict(runs=rows,totals=manifest['totals'],uses=dict(use_counts),hold_exclude_reason_occurrences=dict(reason_counts),
      selected_replay_verified=sum(r['replay_verified'] for r in rows),
      frontal_close_selected=sum(r['front_0to6m_abs_y_le1m'] for r in rows),
      config=manifest['config'],selection_manifest_sha256=sha(selected/'selection_manifest.json'),
      replay_summary_sha256=sha(validated/'summary.json'),source_commit=replay['source_commit'],
      verified_transfer_bytes=all_bytes,verified_transfer_files=all_files,training_performed=False,
      allowed_targets=['xy_m','velocity_mps'],stop_mode_supervision=False,
      split_note='Existing two placement groups remain train-only; no heldout creation or frame split.',
      limitations='Six independently recorded drives, not independent frames/events. Selection screens observed motion and map agreement; it does not certify full 3D clearance. No E2E model promotion.')
with (root/'collect10_selection_report.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(dict(totals=report['totals'],uses=report['uses'],replay=report['selected_replay_verified'],frontal_close=report['frontal_close_selected'],reasons=report['hold_exclude_reason_occurrences'],runs=[dict(run=r['run_id'],selected=r['selected'],hold=r['hold'],exclude=r['exclude'],pose=r['pose_quality']['reason']) for r in rows]),indent=2))
