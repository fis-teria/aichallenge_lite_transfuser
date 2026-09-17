"""Plot evaluation-only reference errors; leave invalid spans disconnected."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

out=Path(__file__).parent;root=out.parents[2]
data=json.loads((root/'tmp/lidar_map_reference_20260918/details.json').read_text())
plt.rcParams.update({'font.family':FontProperties(fname='C:/Windows/Fonts/meiryo.ttc').get_name(),'font.size':11})
fig,(ax,bx)=plt.subplots(2,1,figsize=(11,6.7),sharex=True)
fig.subplots_adjust(left=.09,right=.97,top=.78,bottom=.17,hspace=.18)
for name,label,color in [('bounded','従来','#7b8999'),('unlimited','上限解除','#3688bd'),('simulation_aggressive','強補正','#ce7328')]:
    r=data[name]; t=np.array([x['stamp_ns']/1e9 for x in r])
    ax.plot(t,[x.get('xy_error_m',np.nan) for x in r],color=color,label=label,lw=1.5)
    bx.plot(t,[x.get('yaw_error_deg',np.nan) for x in r],color=color,lw=1.5)
ax.axhline(1,c='#40937b',ls='--',lw=1,label='位置差 1 m')
bx.axhline(10,c='#40937b',ls='--',lw=1)
ax.set_ylabel('参考基準との位置差 [m]');bx.set_ylabel('参考基準との向き差 [°]')
bx.set_xlabel('記録内のシミュレーション時刻 [s]')
ax.set_ylim(0,60);bx.set_ylim(0,150);bx.set_xlim(0,420)
for a in [ax,bx]:a.grid(alpha=.2)
ax.legend(loc='upper left',ncol=4,fontsize=10)
fig.suptitle('強補正で「有効」とされた出力にも、大きな位置ずれがある',fontsize=19,x=.09,ha='left',y=.96)
fig.text(.09,.85,'基準：記録済み /localization/pose（GNSS/IMU系の推定位置）。AWSIMの真値位置ではありません。',fontsize=11,color='#4b5667')
fig.text(.09,.08,'位置合わせ後の推定を固定して評価。座標合わせの再学習・平行移動・回転フィットなし。空白は有効出力なし。',fontsize=10,color='#4b5667')
fig.savefig(out/'reference_errors.png',dpi=160)
