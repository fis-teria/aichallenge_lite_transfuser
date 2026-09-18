"""Bounded read-only raw replay plus separate, explicitly tolerant selection."""
from pathlib import Path
import hashlib,json,subprocess,sys
repo=Path.cwd();sys.path[:0]=[str(repo),str(repo/'tools')]
import torch
from tools.curate_native_teacher_data import curate
from aic_transfuser_lite.data.native_teacher_curation import NativeCurationConfig
from tools.train_time_native_replay import prepare_native_selection
root=Path('/home/thistle/e2e_autonomous')
collection=root/'runs/mppi_v45_pc10_20260918'
for run in ['lidar-v45-pc10-expand-aleft-a01','lidar-v45-pc10-expand-aright-a01']:
    subprocess.run([sys.executable,str(Path(__file__).with_name('audit_collected.py')),run],check=True)
selection=collection/'expand_alignment_tolerant_v2'
manifest=curate(collection,selection,NativeCurationConfig(require_scan_map_alignment=False),
    run_pattern='lidar-v45-pc10-expand-*',prefix_directory='expand_pose_prefix_v1',
    clearance_directory='expand_prefix_clearance_v1')
assert all(r['split']=='train' for r in manifest['runs'])
assert {r['run_id'] for r in manifest['runs']}=={
    'lidar-v45-pc10-expand-'+suffix for suffix in ('left-a01','left-a02','right-a01','aleft-a01','aright-a01')}
output=root/'runs/avoidance_expand_replay_20260919';output.mkdir(exist_ok=False)
plan=dict(native_collection=str(collection.relative_to(root)),native_selection=selection.name,
    selection_manifest_sha256=hashlib.sha256((selection/'selection_manifest.json').read_bytes()).hexdigest(),
    native_anchors=manifest['totals']['selected'],native_split_group='expand_train_groups',
    native_split_groups=['straight_b_center','failed_e2e_cone_location'],native_source_split='train')
(output/'plan.json').write_text(json.dumps(plan,indent=2))
torch.set_num_threads(4)
additions=prepare_native_selection(root,plan,output)
proof=dict(plan=plan,source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    additions=additions,all_raw_histories_and_30_xy_velocity_targets_reproduced=True,
    trained=False,active_comparison_population_unchanged=True,
    heldout_runs=['lidar-v45-pc10-expand-holdoutcone-a01','lidar-v45-pc10-expand-holdoutbox-a01'])
(output/'proof.json').write_text(json.dumps(proof,indent=2))
print('NEW_COLLECTION_REPLAY_PASS',json.dumps(dict(anchors=plan['native_anchors'],runs=len(additions))),flush=True)
