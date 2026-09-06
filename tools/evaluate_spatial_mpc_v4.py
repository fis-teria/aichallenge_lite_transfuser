"""Explicit saved packet + synthetic offline control experiment. Never imports model/ROS."""
from __future__ import annotations
import argparse
import io
import json
import os
import sys
from pathlib import Path, PurePosixPath
import time
import zipfile
import importlib.metadata
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import canonical, sha, plain
from aic_transfuser_lite.evaluation.spatial_tracking_sim_v4 import synthetic_scenes, execute_scene

EXPECTED={
'evidence/validation_predictions.npz':'d033997378bdcb9fa0cfc7efb038632ee840a0057561b43142ba75b53ee3d90f',
'artifacts/selection.json':'ef08f7a66f20f43c684b2d373fa3d1802b4ba6629455d1118cd56378354af91a',
'artifacts/input_contract.json':'efd0348c9b6ff79d02ce68a1aad72440c12dce878c36abd997f86abcf26aac6e',
'artifacts/teacher_contract.json':'8c600845610e298c5ec54cc116e4080d9f10281ec5b0960af32babc269887356',
'artifacts/annotation_addendum.json':'b7e2bc0fa433b9f8c095f7076d648b44654707e0076d76cd147458623fbda2d5'}
IDENTITIES={'artifacts/input_contract.json':'77fff3f9c5875b6f116cebe35fde88d42d459aa04c195676c44e5c6caca5edc7',
'artifacts/teacher_contract.json':'cdb668834d4baf60c31fa5f934d01d8782f02544bc6fa29053d5a850432cc344',
'artifacts/selection.json':'59393d98ad4e55a59da515ff324a147995889e98b48b3a7ac676660400e3851f'}
PREFIX='spatial_v4_validation_review_20260906_153a22a/'
ALLOW=['PACKAGE_MANIFEST.json','README_REVIEW.md','provenance/changed_files.json','artifacts/execution_manifest.json',*EXPECTED]


def write(path: Path, value) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as stream: stream.write(canonical(value))


def saved_inputs(packet: Path, output: Path) -> tuple[list,dict]:
    """Read ONLY allowlisted ZIP entries, not paths embedded in their contents."""
    with zipfile.ZipFile(packet) as z:
        names=[i.filename for i in z.infolist()]
        if len(names)!=len(set(names)): raise ValueError('DUPLICATE_ZIP_ENTRY')
        for i in z.infolist():
            p=PurePosixPath(i.filename)
            if p.is_absolute() or '..' in p.parts or '\\' in i.filename or (i.external_attr>>16)&0o170000==0o120000:
                raise ValueError('UNSAFE_ZIP_ENTRY')
        blobs={}
        for name in ALLOW:
            if PREFIX+name not in names:
                if name=='artifacts/annotation_addendum.json': continue
                raise ValueError('MISSING_ENTRY:'+name)
            i=z.getinfo(PREFIX+name)
            if i.file_size>3_000_000: raise ValueError('ENTRY_SIZE_LIMIT')
            blobs[name]=z.read(i)
        manifest=json.loads(blobs['PACKAGE_MANIFEST.json'])
        entries={i['path']:i for i in manifest['files']}
        for name,blob in blobs.items():
            if name=='PACKAGE_MANIFEST.json': continue
            if name not in entries or entries[name]['size_bytes']!=len(blob) or entries[name]['sha256']!=sha(blob):
                raise ValueError('MANIFEST_HASH:'+name)
            if name in EXPECTED and sha(blob)!=EXPECTED[name]: raise ValueError('EXPECTED_HASH:'+name)
        for name,identity in IDENTITIES.items():
            doc=json.loads(blobs[name])
            if doc['identity']!=identity or sha(canonical({k:v for k,v in doc.items() if k!='identity'}))!=identity:
                raise ValueError('IDENTITY:'+name)
        execution=json.loads(blobs['artifacts/execution_manifest.json'])
        if execution['checkpoint_sha256']!='0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f':
            raise ValueError('CHECKPOINT_EXPECTATION')
        selection=json.loads(blobs['artifacts/selection.json'])['selected']
        with np.load(io.BytesIO(blobs['evidence/validation_predictions.npz']),allow_pickle=False) as a:
            if set(a.files)!={'xy','sample_ids','processed'}: raise ValueError('NPZ_KEYS')
            arrays={k:a[k].copy() for k in a.files}
        if any(v.dtype.hasobject for v in arrays.values()): raise ValueError('OBJECT_ARRAY')
        xy=arrays['xy']; ids=arrays['sample_ids'].tolist(); processed=arrays['processed']
        if xy.shape!=(180,20,2) or xy.dtype!=np.float32 or len(ids)!=len(set(ids)): raise ValueError('NPZ_CONTRACT')
        if ids!=[r['sample_id'] for r in selection]: raise ValueError('SELECTION_ORDER')
        selected=[]; main=[r for r in selection if r['role']=='validation_main']
        runs=sorted({r['run_id'] for r in main})
        if len(runs)!=5: raise ValueError('MAIN_RUN_COUNT')
        for run in runs: selected.extend(sorted([r for r in main if r['run_id']==run],key=lambda r:r['sample_id'])[:2])
        worst='20260902-131505__epoch0000__76292918933'
        match=[r for r in selection if r['sample_id']==worst]
        if match and worst not in [r['sample_id'] for r in selected]: selected+=match
        remaining=sorted([r for r in selection if r['shape']=='straight' and r['sample_id'] not in [v['sample_id'] for v in selected]],key=lambda r:r['sample_id'])
        if remaining: selected.append(remaining[0])
        if len(selected)>12: raise ValueError('SELECTION_LIMIT')
        inventory=dict(source_npz_sha256=sha(blobs['evidence/validation_predictions.npz']),selection_rule='5 main runs first 2 lexicographic sample IDs; fixed worst; first remaining straight',
            selected_metadata=selected,source_count=len(selection),worst_missing=not bool(match),unprocessed_ids=[ids[i] for i in range(len(ids)) if not processed[i]])
        write(output/'scenario_selection.json',inventory)  # Before any MPC.
        for name,blob in blobs.items():
            path=output/'source'/name; path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(blob)
        scenes=[]
        for r in selected:
            i=ids.index(r['sample_id'])
            scenes.append(dict(id='saved_'+str(i),source_sample_id=r['sample_id'],source_run_id=r['run_id'],
                saved=True,permission='RUN',region=None,obstacles=[],initial_state=[0,0,0,0,0],
                t_obs=r['stamp_ns'],raw_xy=xy[i].copy(),nominal_s=np.arange(1,21,dtype=float)/10,
                not_processed=not bool(processed[i])))
        return scenes,inventory


class Budget:
    """One task file, shared by smoke/main/retry. Existing counters never reset."""
    def __init__(self,path: Path,limits: dict,mode: str):
        self.path=path; self.limits=limits; self.start=time.monotonic()
        if path.exists(): self.value=json.loads(path.read_text())
        else: self.value=dict(cycles=0,active_seconds=0.,main_runs=0,invocations=[])
        self.previous_seconds=self.value['active_seconds']
        if mode=='main':
            if self.value['main_runs']>=limits['main_runs']: raise ValueError('MAIN_RUN_LIMIT')
            self.value['main_runs']+=1
        self.value['invocations'].append(dict(mode=mode,code_commit=os.environ.get('V4_MPC_COMMIT','UNKNOWN')))
        self.save()
    def save(self):
        self.value['active_seconds']=self.previous_seconds+time.monotonic()-self.start
        self.path.write_bytes(canonical(self.value))
    def consume(self):
        self.save()
        if self.value['cycles']>=self.limits['total_cycles'] or self.value['active_seconds']>=self.limits['active_seconds']: return False
        self.value['cycles']+=1; self.save(); return True


def figures(folder: Path, scene_id: str) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=[json.loads(v) for v in (folder/'cycles.jsonl').read_text().splitlines()]
    if not rows: return
    p=json.loads((folder/'path.json').read_text()); states=np.array([r['next_state'] for r in rows])
    xy=np.array(p['prepared_xy']); t=np.array([r['virtual_time_s'] for r in rows]); u=np.array([r['plant']['applied'] for r in rows])
    fig,axes=plt.subplots(1,3,figsize=(12,3.4))
    axes[0].plot(xy[:,0],xy[:,1],label='fixed polyline'); axes[0].plot(states[:,0],states[:,1],label='plant RK4')
    axes[0].axis('equal'); axes[0].legend(); axes[0].set_xlabel('x [m]'); axes[0].set_ylabel('y [m]')
    axes[1].plot(t,states[:,3]); axes[1].set_ylabel('v [m/s]'); axes[1].set_xlabel('virtual t [s]')
    axes[2].plot(t,u[:,0],label='a [m/s²]'); axes[2].plot(t,u[:,1],label='delta rate [rad/s]'); axes[2].legend()
    fig.suptitle(scene_id+' / FROZEN PATH VIRTUAL TEST ONLY'); fig.tight_layout(); fig.savefig(folder/'trajectory.png',dpi=130); plt.close(fig)


def main() -> int:
    p=argparse.ArgumentParser()
    p.add_argument('--packet',type=Path,required=True); p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True); p.add_argument('--budget',type=Path,required=True)
    p.add_argument('--mode',choices=['smoke','main'],required=True)
    args=p.parse_args(); cfg=yaml.safe_load(args.config.read_text()); args.output.mkdir(parents=True,exist_ok=False)
    budget=Budget(args.budget,cfg['budgets'],args.mode)
    results=[]; input_state='AVAILABLE'; inventory=None
    try:
        write(args.output/'resolved_config.json',cfg)
        write(args.output/'environment.json',{k:importlib.metadata.version(k) for k in ('numpy','scipy','matplotlib','pytest')})
        try: saved,inventory=saved_inputs(args.packet,args.output/'inputs')
        except Exception as exc:
            saved=[]; input_state='BLOCKED_INPUT:'+repr(exc); write(args.output/'input_error.json',dict(reason=input_state))
        scenes=synthetic_scenes()
        if args.mode=='smoke': scenes=scenes[:1]
        else: scenes+=saved
        write(args.output/'scenario_selection.json',dict(config_sha256=sha(canonical(cfg)),scenes=scenes,
            saved_input_status=input_state,selection_before_solver=True))
        for scene in scenes:
            folder=args.output/'scenes'/scene['id']; folder.mkdir(parents=True)
            with (folder/'cycles.jsonl').open('xb') as log:
                def emit(kind,value):
                    if kind=='cycle': log.write(canonical(value)+b'\n'); log.flush()
                    else: write(folder/(kind+'.json'),value)
                if scene.get('not_processed'):
                    summary=dict(scene_id=scene['id'],execution='REJECTED_UNPROCESSED',cycles=0,solver_calls=0,tracking_success=False)
                else: summary=execute_scene(scene,cfg,budget.consume,emit)
                results.append(summary); write(folder/'summary.json',summary)
            np.savez_compressed(folder/'raw_path.npz',raw_xy=scene['raw_xy'],nominal_s=scene['nominal_s'])
            figures(folder,scene['id'])
            print(json.dumps(plain(summary),sort_keys=True),flush=True)
        write(args.output/'summary.json',dict(scenes=results,saved_input_status=input_state,
            control_closed_loop_scope='FROZEN_PATH_AND_SYNTHETIC_REFERENCE',code_commit=os.environ.get('V4_MPC_COMMIT','UNKNOWN'),
            total_cycles=sum(r['cycles'] for r in results),solver_calls=sum(r['solver_calls'] for r in results),
            new_model_inferences=0,training_optimizer_steps=0,real_ros_calls=0,real_control_publish=0,
            budget=budget.value))
        return 0
    finally: budget.save()


if __name__=='__main__': raise SystemExit(main())
