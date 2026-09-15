"""Summarize the completed WSL diagnostic, including actual 60 cm states."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import math
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

sys.path.insert(0, str(Path.cwd()/'tools'))
from analyze_time_corner_learning import fit_indices, groups, population, neighborhood
from analyze_time_recovery_fit import report_group
from compare_time_training_methods import pp_rows
from aic_transfuser_lite.data.time_training_cache_v1 import TimeTrainingCacheDataset, _sha

ROOT=Path('/home/thistle/e2e_autonomous')
OUT=ROOT/'runs/time_corner4_learning_diagnosis_20260916_fp32'
read=lambda p:json.loads(p.read_text())
summary=read(OUT/'summary.json');assert summary['status']=='COMPLETE'
manifest=read(OUT/'artifact_manifest.json')
for item in manifest['files']:
    p=OUT/item['path'];assert p.stat().st_size==item['bytes'] and _sha(p)==item['sha256']
plan=read(Path('configs/time_path_p1/recovery_multiscale_20260916.json'))
cache=ROOT/plan['cache'];identity=read(cache/'identity.json')
assert identity['manifest_sha256']==summary['cache_sha256']
controller=read(ROOT/'runs/time_multiscale_model_lap_20260916/raw/codex-time-multiscale-lap01/trial_config.json')
fonts=[Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'),Path('/mnt/c/Windows/Fonts/meiryo.ttc')]
font=next(p for p in fonts if p.exists());font_manager.fontManager.addfont(str(font))
plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams['axes.unicode_minus']=False

def stats(values):
    if not len(values):return {'count':0}
    a=np.asarray(values,float)
    assert np.isfinite(a).all()
    return dict(count=len(a),min=float(a.min()),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))

def compact(group):
    if not group['anchor_count']:return dict(anchors=0,runs=0)
    return dict(anchors=group['anchor_count'],runs=group['run_count'],xy3s_m=group['xy']['run_macro_mean']['3s']['raw_error_m'],
        left_bias3s_m=group['components']['3s']['left_bias_m'],left_mae3s_m=group['components']['3s']['left_mae_m'],
        pp_error_rad=group['pp']['run_macro_penalized_rad'],pp_rejected=group['pp']['candidate_rejected'],
        endpoint_cm=group['endpoint_pooled_cm'])

result=dict(status='COMPLETE',scope='POST_SELECTION_DIAGNOSTIC_NO_NEW_FIT_OR_SELECTION',
    primary_summary_sha256=_sha(OUT/'summary.json'),models={m:{s:{k:compact(v) for k,v in d.items()}
        for s,d in r['splits'].items()} for m,r in summary['models'].items()},corner_state_distribution={},observed_60cm={},joint_coverage={},corner_event_states={})
all_states={s:read(OUT/(s+'_states.json')) for s in ('train','validation')}
state_manifest_path=Path('docs/evidence/time_recovery_60cm_20260916/manifest.json')
state_manifest=read(state_manifest_path)['files'];source_files={}
for split,rows in all_states.items():
    corner=[r for r in rows if r['status']=='CORNER']
    event_keys=sorted({(r['run_id'],r['event_id']) for r in corner if r['recovery']})
    result['corner_event_states'][split]=[dict(run_id=rid,event_id=event,anchors=len(rs),
        **{k:stats([r[k] for r in rs]) for k in ('base_s_m','left_m','heading_rad','speed_mps')})
        for rid,event in event_keys for rs in [[r for r in corner if (r['run_id'],r['event_id'])==(rid,event)]]]
    partial_tolerances={'progress_only':(3.,1e6,math.pi,1e6),
        'progress_lateral':(3.,.15,math.pi,1e6),
        'progress_lateral_heading':(3.,.15,math.radians(5.),1e6),
        'progress_lateral_speed':(3.,.15,math.pi,.25),
        'all_four':(3.,.15,math.radians(5.),.25)}
    result['joint_coverage'][split]=dict(
        outer_20cm_heading_positive=population([r for r in corner if r['left_m']>=.2 and r['heading_rad']>=math.radians(3.)]),
        near_line_heading_positive=population([r for r in corner if abs(r['left_m'])<.2 and r['heading_rad']>=math.radians(3.)]),
        per_runtime_query=[{k:population([rows[i] for i in neighborhood(rows,q,v)]) for k,v in partial_tolerances.items()} for q in summary['runtime_queries']])
    result['corner_state_distribution'][split]={}
    for label,rs in [('nominal_5kmh',[r for r in corner if r['run_id'].startswith('5kmh_')]),
                     ('nominal_8kmh',[r for r in corner if r['run_id'].startswith('8kmh_')]),
                     ('recovery',[r for r in corner if r['recovery']])]:
        result['corner_state_distribution'][split][label]=dict(population=population(rs),
            **{key:stats([r[key] for r in rs]) for key in ('speed_mps','left_m','heading_rad','base_s_m')})
    ds=TimeTrainingCacheDataset(cache,split,verify_hashes=False)
    ix=fit_indices(rows);pos={ds.anchor_ids[index]:j for j,index in enumerate(ix)}
    selected=[]
    for spec in plan['additions']:
        if spec['split']!=split or spec['analysis']!='runs/time_recovery_60cm_20260916':continue
        filename=spec['run_id']+'_anchor_states.json';path=ROOT/spec['analysis']/filename
        assert _sha(path)==state_manifest[filename]['sha256'];source_files[str(path)]=_sha(path)
        states=read(path);assert len(states)==spec['anchors']
        for state in states:
            if .55<=abs(state['lateral_m'])<=.65:
                j=pos[state['anchor_id']];assert ds.run_ids[ix[j]]==spec['run_id']
                selected.append((j,state))
    bands={'observed_55_to_65cm':selected,
           'observed_left_55_to_65cm':[(i,s) for i,s in selected if s['lateral_m']>0],
           'observed_right_55_to_65cm':[(i,s) for i,s in selected if s['lateral_m']<0]}
    result['observed_60cm'][split]={}
    for band,items in bands.items():
        ids=[i for i,_ in items];indices=[ix[i] for i in ids]
        states=[dict(rows[ix[i]], observed_lateral_m=s['lateral_m'],observed_heading_rad=s['heading_rad']) for i,s in items]
        report=dict(population=population(states),observed_heading_rad=stats([s['observed_heading_rad'] for s in states]),models={})
        if ids:
            truth=pp_rows(ds,indices,ds.targets[indices],controller,teacher=True)
            for model in summary['models']:
                prediction=np.load(OUT/(model+'_'+split+'_predictions.npy'),allow_pickle=False)[ids]
                proposed=pp_rows(ds,indices,prediction,controller)
                report['models'][model]=compact(report_group(prediction,ds.targets[indices],[ds.run_ids[i] for i in indices],truth,proposed))
        result['observed_60cm'][split][band]=report
result['observed_state_source_manifest_sha256']=_sha(state_manifest_path)
result['observed_state_sources']=source_files

fig,axes=plt.subplots(1,2,figsize=(12,5.5))
train=[r for r in all_states['train'] if r['status']=='CORNER']
for name,rs,color in [('通常走行',[r for r in train if not r['recovery']],'#357db0'),
                      ('復帰',[r for r in train if r['recovery']],'#26a276')]:
    axes[0].scatter([r['base_s_m'] for r in rs],[r['left_m'] for r in rs],s=9,alpha=.35,color=color,label=f'学習・{name} ({len(rs)}アンカー)')
    axes[1].scatter([r['left_m'] for r in rs],np.rad2deg([r['heading_rad'] for r in rs]),s=9,alpha=.35,color=color,label=f'学習・{name}')
for q in summary['runtime_queries']:
    for ax,point in zip(axes,((q['base_s_m'],q['left_m']),(q['left_m'],math.degrees(q['heading_rad'])))):
        ax.scatter(*point,c='#c72d43',marker='X',s=85,zorder=5)
        ax.annotate(f"{q['time_s']:.1f}s",point,xytext=(5,5),textcoords='offset points',fontsize=9,color='#982232')
axes[0].set(xlabel='基準経路上の進行 [m]',ylabel='完走教師r30の線から左方向 [m]',title='今回のカーブに含まれる学習状態')
axes[1].set(xlabel='完走教師r30の線から左方向 [m]',ylabel='教師の向きに対する左向き誤差 [°]',title='横位置と向きの組合せ（赤×が失敗走行）')
for ax in axes:ax.axhline(0,c='gray',lw=.7);ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('学習データの状態分布：連続アンカーは独立したイベント数ではない',fontsize=12)
fig.tight_layout();fig.savefig(OUT/'coverage.png',dpi=150);plt.close(fig)

fig,axes=plt.subplots(1,3,figsize=(13,4.5))
model_names=list(summary['models']);x=np.arange(len(model_names))
for ax,(key,title) in zip(axes,[('corner_nominal_5kmh','当該カーブ・通常5km/h群'),('corner_recovery','当該カーブ・復帰群'),('all_recovery','全復帰群')]):
    for split,label,color in [('train','学習データ','#307eb0'),('validation','別runの検証データ','#cb6941')]:
        values=[result['models'][m][split][key] for m in model_names]
        if values[0]['anchors']:
            ax.plot(x,[v['xy3s_m']*100 for v in values],'o-',color=color,label=f"{label} ({values[0]['anchors']} / {values[0]['runs']} run)")
    ax.set_xticks(x,['学習前','epoch1','epoch2','採用epoch3']);ax.set(title=title,ylabel='3秒先XY誤差 [cm]（run等重み）')
    ax.set_ylim(bottom=0);ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('既存教師の再現度：同じ入力を保存済み4段階の重みで評価',fontsize=12)
fig.tight_layout();fig.savefig(OUT/'fit.png',dpi=150);plt.close(fig)

with (OUT/'compact_report.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
with (OUT/'report_manifest.json').open('x') as f:json.dump(dict(files={p.name:dict(bytes=p.stat().st_size,sha256=_sha(p)) for p in [OUT/'compact_report.json',OUT/'coverage.png',OUT/'fit.png']}),f,indent=2)
print(json.dumps(result))
