"""Summarize finite pilots against both measured nominal runs in native WSL."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from analyze_pulse import bins


def goal(row: dict) -> bool:
    return (.05 <= abs(row['left_m']) <= .25
        and math.radians(2.) <= abs(row['heading_rad']) <= math.radians(4.)
        and row['left_m']*row['heading_rad'] > 0)


def recovery_time(rows: list[dict]) -> dict:
    """First observed 1s hold after the post-release peak; no gap above 150ms."""
    rows=[r for r in rows if 0. <= r['seconds_after_release'] <= 10.]
    if not rows: return dict(confirmed=False)
    peak=max(range(len(rows)),key=lambda i:abs(rows[i]['left_m']))
    maximum=abs(rows[peak]['left_m']); threshold=min(.1,maximum/2.)
    start=None; previous=None; result=None
    for r in rows[peak:]:
        t=r['seconds_after_release']
        good=abs(r['left_m'])<=threshold and abs(r['heading_rad'])<=math.radians(2.)
        if not good:
            start=None
        elif start is None or previous is None or t-previous>.15:
            start=t
        if start is not None and t-start>=1.:
            result=t; break
        previous=t
    return dict(confirmed=result is not None,hold_confirmed_at_s=result,
        peak_lateral_m=maximum,peak_at_s=rows[peak]['seconds_after_release'],lateral_threshold_m=threshold)


def smoke() -> None:
    rows=[dict(seconds_after_release=float(t),left_m=.12 if t==0 else .04,heading_rad=.01) for t in np.arange(0.,2.01,.05)]
    assert recovery_time(rows)['confirmed']
    assert not recovery_time(rows[::4])['confirmed']
    assert goal(dict(left_m=.08,heading_rad=.05))
    assert not goal(dict(left_m=.08,heading_rad=-.05))
    print('PILOT_SUMMARY_SMOKE_PASS',flush=True)


def main() -> None:
    ap=argparse.ArgumentParser()
    ap.add_argument('--root',type=Path)
    ap.add_argument('--runs',nargs='+',choices=('r32','r33','r34','r35'))
    ap.add_argument('--output',type=Path)
    ap.add_argument('--smoke-only',action='store_true')
    args=ap.parse_args(); smoke()
    if args.smoke_only: return
    if args.root is None or not args.runs or args.output is None:
        ap.error('--root, --runs and --output required')
    assert not args.output.exists() and len(set(args.runs))==len(args.runs)
    root=args.root.resolve(); out=root/'runs/time_steering_pulse_20260914'
    alt_path=out/'alternate_nominal_guide_r31.json'; alt=json.loads(alt_path.read_text())
    guide_b=np.asarray(alt['guide']); summaries=[]; source_hashes={str(alt_path):hashlib.sha256(alt_path.read_bytes()).hexdigest()}
    fig,axes=plt.subplots(1,2,figsize=(12,4.7),layout='constrained')
    for axis,sign,side in zip(axes,(1,-1),('left','right')):
        axis.add_patch(Rectangle((5. if sign>0 else -25.,2. if sign>0 else -4.),20.,2.,color='green',alpha=.10,label='Target region'))
        axis.set(xlim=(0.,14.) if sign>0 else (-14.,0.),ylim=(-1.5,4.5) if sign>0 else (-4.5,1.5),xlabel='Lateral error from nominal (cm)',ylabel='Body heading error (deg)',title=side.capitalize()+' perturbation')
        axis.grid(alpha=.25)
    for suffix in args.runs:
        p=out/(suffix+'_state_audit.json'); d=json.loads(p.read_text()); source_hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
        raw=root/'raw/time_steering_pulse_20260914'/d['run_id']
        ref=json.loads((raw/'reference.json').read_text()); guide_a=np.asarray(ref['steering_pulse']['nominal_guide'])
        accepted=[r for r in d['records'] if r['accepted']]; alternate=[]
        for r in accepted:
            s=r['base_s_m']; assert guide_b[0,0]<=s<=guide_b[-1,0]
            lateral=r['left_m']+np.interp(s,guide_a[:,0],guide_a[:,1])-np.interp(s,guide_b[:,0],guide_b[:,1])
            heading=r['heading_rad']+np.interp(s,guide_a[:,0],np.unwrap(guide_a[:,2]))-np.interp(s,guide_b[:,0],np.unwrap(guide_b[:,2]))
            alternate.append({**r,'left_m':float(lateral),'heading_rad':math.atan2(math.sin(heading),math.cos(heading))})
        pub=d['first_zero_publication_ns']; controls=[json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]
        active=[r for r in controls if r.get('publication') and pub<=r['publication']['sim_ns']<pub+10_000_000_000 and r['reason']=='RECOVERY_TEACHER_TRACKING']
        result=json.loads((raw/'result.json').read_text())
        summaries.append(dict(run_id=d['run_id'],config=d['config'],status=d['raw_status'],lap_seconds=result['judge_laps'][0]['lap_seconds'],
            accepted_primary=d['accepted'],accepted_alternate=bins(alternate),
            target_count_agreeing_with_both_nominals=sum(goal(a) and goal(b) for a,b in zip(accepted,alternate)),
            first_05s_accepted=d['first_05s_accepted'],first_015s_otherwise_usable=d['first_015s_otherwise_usable'],
            recovery=recovery_time(d['control_states']),minimum_guard_ray_margin_m=min(r['guard']['minimum_ray_margin_m'] for r in active),
            max_abs_issued_steering_rad=max(abs(r['issued_angle_rad']) for r in active),
            target_anchor_ids=[r['anchor_id'] for r in accepted if goal(r)]))
        axis=axes[0 if d['config']['amplitude_rad']>0 else 1]; color='tab:orange' if d['config'].get('plateau_s',0)>0 else 'tab:blue'
        shown=[r for r in d['records'] if 0<=r['seconds_after_release']<=1.5 and r.get('left_m') is not None]
        axis.plot([r['left_m']*100 for r in shown],[math.degrees(r['heading_rad']) for r in shown],color=color,label=suffix+(' plateau' if color=='tab:orange' else ' sine'))
        for marker,chosen in [('o',[r for r in shown if r['accepted']]),('x',[r for r in shown if not r['phase_margin_pass']])]:
            axis.scatter([r['left_m']*100 for r in chosen],[math.degrees(r['heading_rad']) for r in chosen],marker=marker,color=color,s=28)
    for axis in axes: axis.legend(loc='lower left',fontsize=8)
    fig.suptitle('First 1.5 s after zero publication: dots = accepted cameras, crosses = excluded start margin',fontsize=11)
    fig.savefig(args.output.with_suffix('.png'),dpi=170); plt.close(fig)
    report=dict(scope='FOUR_CALIBRATION_RUNS; NOT_TRAINING_OR_E2E_EVALUATION',runs=summaries,input_hashes=source_hashes,
        total_accepted=sum(r['accepted_primary']['count'] for r in summaries),total_primary_target_anchors=sum(len(r['target_anchor_ids']) for r in summaries),
        total_targets_agreeing_with_both_nominals=sum(r['target_count_agreeing_with_both_nominals'] for r in summaries),
        independent_runs=len(summaries),final_test_eligible=False)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
