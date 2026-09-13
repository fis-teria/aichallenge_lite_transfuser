"""Post hoc curve comparison in native WSL; keeps the original pilot gates.

All offsets and progress are in metres. This compares unique measured pose
captures at common baseline progress and never extrapolates a nominal trace.
"""
import argparse
from pathlib import Path
import json,numpy as np,hashlib
base=Path('/home/thistle/e2e_autonomous');a=base/'runs/time_recovery_batches_20260914'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--pair',type=int,choices=(2,3),default=2)
args=parser.parse_args()
run_names=(('codex-time-recovery-left020-r23','codex-time-recovery-right020-r24') if args.pair==2 else
           ('codex-time-recovery-left040-r25','codex-time-recovery-right040-r26'))
output_name='curve020_nominal_comparison.json' if args.pair==2 else 'curve040_nominal_comparison.json'
def trace(run: Path) -> tuple[dict, list[dict], np.ndarray, np.ndarray]:
 manifest=json.loads((run/'transfer_manifest.json').read_text())
 for name in ('reference.json','control.jsonl'):
  content=(run/name).read_bytes()
  assert len(content)==manifest[name]['bytes'] and hashlib.sha256(content).hexdigest()==manifest[name]['sha256']
 ref=json.loads((run/'reference.json').read_text())
 rows=[json.loads(l) for l in (run/'control.jsonl').read_text().splitlines()]
 rows=[r for r in rows if r['reason']=='RECOVERY_TEACHER_TRACKING' and r['projection'] and 273.5330395126295<=r['projection']['s_m']<296.5330395126295]
 s=np.asarray([r['projection']['s_m'] for r in rows]);v=np.asarray([r['projection']['offset_m'] for r in rows]);assert len(s)>100 and (np.diff(s)>=0).all()
 stamps=np.asarray([r['current_pose']['stamp_ns'] for r in rows]);assert (np.diff(stamps)>=0).all()
 for i in np.where(np.diff(stamps)==0)[0]:
  assert v[i]==v[i+1] and rows[i]['current_pose']==rows[i+1]['current_pose']
 _,keep=np.unique(stamps,return_index=True);rows=[rows[i] for i in keep];s=s[keep];v=v[keep]
 return ref,rows,s,v
nominal=[]
for n in ('codex-time-recovery-right020-r19','codex-time-recovery-left020-r20'):
 run=base/'raw/time_recovery_collection_20260913'/n
 ref,rows,s,v=trace(run);assert all(r['phase']=='baseline' for r in rows) and (np.diff(s)>0).all()
 nominal.append((n,ref,s,v))
records=[]
for n in run_names:
 run=base/'raw/time_recovery_batches_20260914'/n
 ref,rows,s,v=trace(run)
 for _,old,_,_ in nominal:np.testing.assert_array_equal(ref['baseline_xy_m'],old['baseline_xy_m'])
 by_phase={}
 for phase in ('hold','recovery','after'):
  mask=np.asarray([r['phase']==phase if phase!='after' else r['projection']['s_m']>=291.5330395126295 for r in rows])
  mask &= (s>=max(x[2][0] for x in nominal)) & (s<=min(x[2][-1] for x in nominal))
  ss=s[mask];vv=v[mask]
  assert len(ss)>0 and all(old_s[0]<=ss.min() and ss.max()<=old_s[-1] for _,_,old_s,_ in nominal)
  matched=np.mean([np.interp(ss,old_s,old_v) for _,_,old_s,old_v in nominal],axis=0)
  delta=vv-matched
  by_phase[phase]={'samples':len(ss),'matched_s_range_m':[float(ss.min()),float(ss.max())],'raw_offset_median_m':float(np.median(vv)),'nominal_matched_offset_median_m':float(np.median(matched)),'delta_from_nominal_median_m':float(np.median(delta)),'delta_from_nominal_max_absolute_m':float(np.abs(delta).max())}
 records.append({'run_id':n,'requested_offset_m':ref['signed_offset_m'],'phase_metrics':by_phase})
out={'scope':'POST_HOC_DIAGNOSTIC_ONLY_ORIGINAL_ACCEPTANCE_GATES_UNCHANGED','nominal_runs':[n for n,_,_,_ in nominal],'method':'Deduplicate identical capture poses, then linear interpolation of nominal measured lateral offset at matching baseline progress; average of two unperturbed passes; common progress range only, no extrapolation','note':'Separates shared tracking/projection offset from added perturbation; does not identify their individual causes or validate a model. Not a substitute acceptance metric.','records':records}
(a/output_name).write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
