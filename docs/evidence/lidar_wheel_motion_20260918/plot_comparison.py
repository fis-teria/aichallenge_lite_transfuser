import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

out=Path(__file__).parent
r=json.loads((out/'pairs.json').read_text())
font=FontProperties(fname='C:/Windows/Fonts/meiryo.ttc')
plt.rcParams.update({'font.family':font.get_name(),'font.size':11,'axes.unicode_minus':False})
t=np.array([x['t1_ns']/1e9 for x in r]);wheel=np.array([x['wheel'] for x in r]);lidar=np.array([x['lidar'] for x in r])
diff=np.array([x['difference'] for x in r]);bad=np.array([not x['diagnostic_consistent'] for x in r])
event=next(x for x in r if x['map_status'].startswith('REJECTED'));te=event['t1_ns']/1e9
fig,axes=plt.subplots(2,2,figsize=(14,8.8))
fig.subplots_adjust(left=.075,right=.97,top=.83,bottom=.14,hspace=.43,wspace=.24)
fig.suptitle('LiDARと車輪の移動量を、同じ約0.2秒間で比較',x=.075,ha='left',y=.97,fontsize=22)
fig.text(.075,.89,'保存走行データ 35〜50 s ／ LiDARは地図・車輪の情報なしで推定 ／ 73区間中72区間が診断の整合条件を満たす',fontsize=11,color='#4a5664')

for ax in axes.flat:
    ax.grid(alpha=.17)

for ax,column,title,ylabel,scale in [
    (axes[0,0],0,'前方への移動量','1区間の前方移動 [m]',1.),
    (axes[0,1],2,'向きの変化（左回転が正）','1区間の向き変化 [°]',180/np.pi)]:
    ax.plot(t,wheel[:,column]*scale,color='#2674b8',lw=2,label='車輪オドメトリ')
    ax.plot(t,lidar[:,column]*scale,color='#d37427',lw=1.3,alpha=.9,label='LiDARスキャン間推定')
    ax.scatter(t[bad],lidar[bad,column]*scale,marker='x',s=60,c='#444',zorder=5,label='弱い形状条件の1区間')
    ax.axvline(te,color='#a63042',ls=':',lw=1.8)
    ax.set_title(title,pad=9);ax.set_ylabel(ylabel);ax.set_xlabel('シミュレーション時刻 [s]')
    ax.set_xlim(35,50)
axes[0,0].legend(fontsize=8,loc='upper left')

ax=axes[1,0]
ax.plot(t,np.linalg.norm(diff[:,:2],axis=1),color='#218761',lw=1.6,label='LiDARと車輪の移動量の差')
mc=np.array([np.nan if x['map_correction_m'] is None else x['map_correction_m'] for x in r])
ax.plot(t,mc,c='#c13d4d',lw=1.6,label='地図照合が要求した位置補正')
ax.axvline(te,c='#a63042',ls=':',lw=1.8)
ax.scatter([te],[np.linalg.norm(event['difference'][:2])],s=70,color='#218761',zorder=4)
ax.annotate('拒否開始時：差 0.059 m',xy=(te,np.linalg.norm(event['difference'][:2])),xytext=(41.4,.4),
            fontsize=10,color='#17654a',arrowprops={'arrowstyle':'->','color':'#218761'})
ax.set_xlim(41,48);ax.set_ylim(0,1.55)
ax.set_title('急増したのは地図への補正要求',pad=9)
ax.set_xlabel('シミュレーション時刻 [s]');ax.set_ylabel('距離 [m]');ax.legend(fontsize=8,loc='upper left')

ax=axes[1,1];ax.axis('off')
ax.text(0,1,'LiDAR予測へ差し替えても、地図補正は拒否',fontsize=14,fontweight='bold',transform=ax.transAxes)
ax.text(0,.79,'要求された地図補正：0.868 → 0.777 m',fontsize=14,color='#a63042',transform=ax.transAxes)
ax.text(0,.64,'通常の採用上限：0.45 m',fontsize=11,transform=ax.transAxes)
ax.text(0,.43,'45.589 → 46.590 s（約1秒）の直接照合',fontsize=12,transform=ax.transAxes)
ax.text(0,.27,'車輪との移動量差：0.007 m ／ 向きの差：0.19°',fontsize=12,transform=ax.transAxes)
ax.text(0,.04,'※LiDARを真値とする精度ではありません。\n予測の差し替えは問題の1ステップのみ。全周検証ではありません。',fontsize=9,linespacing=1.7,color='#4a5664',transform=ax.transAxes)
fig.text(.075,.065,'移動量は前時刻の車両座標で比較。LiDAR取付位置1.65 mによる回転時の移動差を補正。点線は最初の地図補正拒否。',fontsize=10,color='#4a5664')
fig.text(.075,.035,'LiDARは真値ではありません。順逆照合・初期値変更・形状の情報量を確認したオフライン診断で、制御設定は変更していません。',fontsize=10,color='#4a5664')
fig.savefig(out/'motion_comparison.png',dpi=160)
fig.savefig(out/'motion_comparison.pdf')

metrics={'pairs':len(r),'diagnostic_consistent':int((~bad).sum()),
         'translation_difference_m':dict(zip(['median','p95','max'],np.quantile(np.linalg.norm(diff[:,:2],axis=1),[.5,.95,1]))),
         'yaw_difference_deg':dict(zip(['median','p95','max'],np.quantile(abs(np.degrees(diff[:,2])),[.5,.95,1]))),
         'first_rejection':{'t0_s':event['t0_ns']/1e9,'t1_s':te,'wheel':event['wheel'],'lidar':event['lidar'],
                            'translation_difference_m':float(np.linalg.norm(event['difference'][:2])),
                            'yaw_difference_deg':float(np.degrees(event['difference'][2])),
                            'map_correction_m':event['map_correction_m']}}
(out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
