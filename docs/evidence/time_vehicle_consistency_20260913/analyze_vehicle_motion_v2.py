"""Offline, capture-bracketed vehicle response diagnosis; never commands motion."""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
args=ap.parse_args();args.output.mkdir(exist_ok=False)
control=[json.loads(s) for s in (args.run/'control.jsonl').read_text().splitlines()]
records=[json.loads(s) for s in (args.run/'vehicle_observations.jsonl').read_text().splitlines()]
armed=next(r['sim_ns'] for r in control if r['event']=='ARMED')
ending=min([r['sim_ns'] for r in control if r['event']=='SCAN_GUARD_REJECTED'] or [max(r['sim_ns'] for r in control if r['sim_ns'] is not None)])
sources=[r for r in records if r['event']=='MOTION_SOURCES' and r['sim_ns'] is not None and armed<=r['sim_ns']<=ending]
expected={'imu':['/awsim_d1'],'velocity':['/awsim_d1'],'steering':['/awsim_d1'],'pose':['/localization/ekf_localizer']}
assert sources and all(r['sources']==expected for r in sources), 'SOURCE_IDENTITY'
quality={}; streams={}; frames={}
for role in expected:
    rows=[r for r in records if r['event']=='VEHICLE_OBSERVATION' and r['role']==role and armed<=r['stamp_ns']<=ending]
    assert rows and {r['epoch'] for r in rows}=={'0'}, 'EPOCH_OR_MISSING:'+role
    unique={}; ambiguous=set()
    for r in rows:
        if role=='pose':
            assert r['frame']=='map' and r['child_frame']=='base_link'
            value=[*r['position_xyz_m'],*r['quaternion_xyzw']]
        elif role=='velocity':
            assert r['frame']=='base_link';value=r['longitudinal_lateral_mps_heading_radps']
        elif role=='imu':
            assert r['frame'] in ('imu_link','tamagawa/imu_link');value=r['angular_xyz_radps']
        else:value=[r['tire_rad']]
        vector=np.array(value,float)
        if r['stamp_ns'] in unique and not np.array_equal(unique[r['stamp_ns']],vector,equal_nan=True):
            ambiguous.add(r['stamp_ns'])
        unique[r['stamp_ns']]=np.full_like(vector,np.nan) if r['stamp_ns'] in ambiguous else vector
    ordered=sorted(unique);v=np.array([unique[t] for t in ordered]);t=(np.array(ordered,dtype=np.int64)-armed)/1e9
    quality[role]={'messages':len(rows),'unique_captures':len(t),'ambiguous_captures_excluded':len(ambiguous),'invalid_captures':int(np.sum(~np.isfinite(v).all(axis=1))),
                   'maximum_gap_s':float(np.max(np.diff(t)))}
    if role=='pose':
        finite=np.isfinite(v).all(axis=1);q=v[finite,3:];assert np.all(abs(np.linalg.norm(q,axis=1)-1)<.01),'QUATERNION'
        yaw=np.full(len(v),np.nan);yaw[finite]=np.unwrap(np.arctan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2)))
        v=np.column_stack([v[:,:3],yaw])
    streams[role]=(t,v);frames[role]=sorted({r.get('frame','physical_tire_angle') for r in rows})
grid=np.arange(max(s[0][0] for s in streams.values()),min(s[0][-1] for s in streams.values()),.01)
valid=np.ones(len(grid),bool);aligned={}
for role,(t,v) in streams.items():
    right=np.searchsorted(t,grid,side='left');right=np.clip(right,0,len(t)-1);left=np.maximum(0,right-1)
    exact=t[right]==grid;left=np.where(exact,right,left)
    accepted=(grid-t[left]<=.05)&(t[right]-grid<=.05)&np.isfinite(v[left]).all(axis=1)&np.isfinite(v[right]).all(axis=1)
    a=np.column_stack([np.interp(grid,t,v[:,i]) for i in range(v.shape[1])]);aligned[role]=a;valid &= accepted
    quality[role]['unbracketed_grid_points']=int(np.sum(~accepted))
    if role in ('imu','velocity'):
        bad=np.abs(a[:,2])>2.;valid &= ~bad;quality[role]['angular_outlier_grid_points']=int(bad.sum())
kernel=np.ones(51)/50;kernel[[0,-1]]*=.5
def mean(a):return np.convolve(a,kernel,mode='valid')
v=mean(aligned['velocity'][:,0]);steer=mean(aligned['steering'][:,0]);t=grid[25:-25]
nominal=mean(aligned['velocity'][:,0]*np.tan(aligned['steering'][:,0])/1.087)
imu=mean(aligned['imu'][:,2]);report=mean(aligned['velocity'][:,2]);pose=(aligned['pose'][50:,3]-aligned['pose'][:-50,3])/.5
good=(np.convolve(valid.astype(int),np.ones(51,dtype=int),mode='valid')==51)&(v>.8)&(abs(nominal)>.04)&(t>10.)
steady=good&(np.ptp(np.lib.stride_tricks.sliding_window_view(aligned['steering'][:,0],51),axis=1)<.005)
fits=[]
for label,signal in [('IMU',imu),('VelocityReport',report),('EKF_pose',pose)]:
    for name,selection in [('turning',good),('low_angle_change',steady),('left',good&(steer>0)),('right',good&(steer<0))]:
        x=nominal[selection];y=signal[selection]
        assert len(x)>5, name
        fits.append({'signal':label,'selection':name,'samples':len(x),'least_squares_gain':float(x@y/(x@x)),
                     'ratio_quantiles_10_50_90':np.quantile(y/x,[.1,.5,.9]).tolist(),'kinematic_mae_radps':float(np.mean(abs(y-x)))})
kfits=[]
for lo,hi in [(10.,45.),(45.,float(t[-1])),(10.,float(t[-1]))]:
    mask=steady&(t>=lo)&(t<=hi)
    options=[]
    for k in np.arange(0.,.2001,.0005):
        pred=nominal[mask]*1.087/(1.087+k*v[mask]**2)
        options.append((float(np.mean(abs(pred-imu[mask]))),float(k)))
    error,k=min(options);kfits.append(dict(window_s=[lo,hi],samples=int(mask.sum()),best_gradient_s2_per_m=k,mae_radps=error))
summary={'scope':'OFFLINE_CAPTURE_ALIGNED_RESPONSE_DIAGNOSIS_NOT_RUNTIME_CALIBRATION','armed_ns':armed,'end_ns':ending,'source_graph_samples':len(sources),'frames':frames,'quality':quality,
         'sample_period_s':.01,'integration_window_s':.5,'maximum_capture_bracket_s':.05,'fits':fits,'understeer_gradient_exploration':kfits,
         'imu_vs_report_mae_radps':float(np.mean(abs(imu[good]-report[good]))),'imu_vs_pose_mae_radps':float(np.mean(abs(imu[good]-pose[good]))),
         'boundary':'IMU and velocity reports are simulator-derived signals. Grade, motion transients and localization remain possible confounders; no runtime constants are changed by this analysis.',
         'sha256':{p:hashlib.sha256((args.run/p).read_bytes()).hexdigest() for p in ['control.jsonl','vehicle_observations.jsonl']}}
(args.output/'summary.json').write_text(json.dumps(summary,indent=2));np.savez_compressed(args.output/'aligned_windows.npz',t=t,v=v,steer=steer,nominal=nominal,imu=imu,report=report,pose=pose,good=good,steady=steady)
fig,ax=plt.subplots(2,1,figsize=(11,7),sharex=True)
for label,signal in [('Bicycle from actual tire',nominal),('IMU',imu),('Velocity report',report),('EKF pose difference',pose)]:ax[0].plot(t,signal,label=label,linewidth=.85)
ax[0].set_ylabel('Yaw rate [rad/s]');ax[0].legend();ax[0].grid(alpha=.3)
ax[1].plot(t[good],imu[good]/nominal[good],'.',markersize=1,label='IMU / ideal bicycle');ax[1].set_ylim(.5,1.5);ax[1].set_ylabel('Response ratio');ax[1].set_xlabel('Simulation seconds from arm');ax[1].grid(alpha=.3)
fig.tight_layout();fig.savefig(args.output/'yaw_comparison.png',dpi=130)
print(json.dumps(summary,indent=2))
