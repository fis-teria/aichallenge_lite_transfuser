"""Report stored replay validity without connecting invalid trajectory gaps."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

out=Path(__file__).parent;root=out.parents[2]
paths=[root/'tmp/lidar_map_runtime_20260918/replay05',
       root/'tmp/lidar_map_aggressive_20260918/unlimited01',root/'tmp/lidar_map_aggressive_20260918/aggressive01']
labels=['従来の補正上限あり','補正上限だけ解除','強補正＋探索拡大＋自動復帰']
colors=['#8793a4','#3688bd','#d27929'];stats=[]
plt.rcParams.update({'font.family':FontProperties(fname='C:/Windows/Fonts/meiryo.ttc').get_name(),'font.size':11})
fig,(ax,bx)=plt.subplots(1,2,figsize=(13,5.7),gridspec_kw={'width_ratios':[1,1.4]})
fig.subplots_adjust(left=.07,right=.97,top=.76,bottom=.23,wspace=.24)
for i,(path,label,color) in enumerate(zip(paths,labels,colors)):
    rows=[json.loads(s) for s in (path/'records.jsonl').read_text().splitlines()]
    valid=[r for r in rows if r['valid']];summary=json.loads((path/'summary.json').read_text())
    spans=[];start=None;previous=None
    for row in rows:
        t=row['stamp_ns']/1e9
        if row['valid'] and start is None:start=t
        if not row['valid'] and start is not None:spans.append([start,previous]);start=None
        previous=t
    if start is not None:spans.append([start,previous])
    percent=100*len(valid)/len(rows)
    ax.bar(i,percent,color=color,width=.6)
    ax.text(i,percent+2,f'{percent:.1f}%\n{len(valid)}/{len(rows)}',ha='center',fontsize=11)
    bx.broken_barh([(a,b-a+.2) for a,b in spans],(2-i-.24,.48),facecolors=color)
    stats.append(dict(mode=summary.get('correction_mode','bounded'),valid=len(valid),total=len(rows),
        valid_percent=percent,valid_spans_s=spans,
        max_accepted_correction_m=max(r['match']['correction_m'] for r in valid),
        max_accepted_correction_deg=max(float(np.degrees(r['match']['correction_rad'])) for r in valid)))
ax.set_xticks(range(3),['従来','上限解除','強補正']);ax.set_ylim(0,50)
ax.set_ylabel('照合時刻のうち有効だった割合 [%]');ax.set_title('同じ2029回の照合を比較')
ax.grid(axis='y',alpha=.2)
bx.set_yticks([2,1,0],['従来','上限解除','強補正']);bx.set_ylim(-.65,2.65)
bx.axvline(45.789,color='#ad354b',ls=':',label='従来の最初の拒否')
bx.set_xlim(0,420);bx.set_xlabel('記録内のシミュレーション時刻 [s]')
bx.set_title('色付き部分が有効区間／空白は不成立');bx.legend(loc='upper right',fontsize=9);bx.grid(axis='x',alpha=.2)
fig.suptitle('補正上限の解除で照合は先へ進む。全周の安定追従は未達。',fontsize=19,x=.07,ha='left',y=.95)
fig.text(.07,.84,'同じ保存データ・地図・初期位置で比較 ／ GNSS・IMU・記録済み自己位置を入力に使わない再生試験',fontsize=11,color='#4a5664')
fig.text(.07,.12,'強補正：位置・角度の補正上限なし、探索半径3 m、反復40回、失敗後も自動で再照合。',fontsize=11)
fig.text(.07,.065,'有効率は実装の照合条件を満たした割合で、正解位置への一致率やAWSIM実走の完走率ではありません。',fontsize=10,color='#4a5664')
fig.savefig(out/'comparison.png',dpi=160)
(out/'comparison.json').write_bytes((json.dumps(stats,indent=2)+'\n').encode())
