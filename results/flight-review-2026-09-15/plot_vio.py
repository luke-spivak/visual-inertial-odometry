import os
os.environ['MPLCONFIGDIR']='/private/tmp/vio-mpl'
import json,glob,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
p=Path('results/flight-review-2026-09-15');x=json.load(open(next(p.glob('log_16*.json'))))
fig,axes=plt.subplots(3,3,figsize=(15,9),sharex='col',layout='constrained')
for col,(arm,start,loit,land,end) in enumerate([(315.727,319.564,325.366,356.896,360.511),(411.828,415.807,419.535,452.484,456.314),(490.129,494.081,499.568,531.964,544.916)]):
 def data(k,f):
  z=[v for v in x[k] if start-6<=v['TimeUS']/1e6<=end+3];return np.array([v['TimeUS']/1e6-start for v in z]),np.array([v[f] for v in z])
 def pre(k,f):return np.median([v[f] for v in x[k] if arm-2<=v['TimeUS']/1e6<=arm])
 home=next(v['Alt'] for v in x['ORGN'] if v['Type']==1 and abs(v['TimeUS']/1e6-arm)<.02)-350.74
 for f,label,color in [('DAlt','Controller target','#111827'),('Alt','EKF altitude','#2563eb'),('BAlt','Barometer','#d97706')]:
  t,y=data('CTUN',f);axes[0,col].plot(t,y-home,label=label,color=color,lw=1.3,alpha=.85)
 t,y=data('VISP','PZ');axes[0,col].plot(t,-y+pre('VISP','PZ'),label='VIO rise from pre-arm pad',color='#16a34a',lw=1.8)
 for k,f,label,color,sgn in [('VISV','VZ','VIO upward velocity','#16a34a',-1),('XKF1','VD','EKF upward velocity','#2563eb',-1)]:
  t,y=data(k,f);axes[1,col].plot(t,sgn*y,label=label,color=color,lw=1.4)
 t,y=data('XKF3','IVD');axes[2,col].plot(t,y,color='#7c3aed',label='Logged vertical velocity innovation')
 for row in range(3):
  ax=axes[row,col];ax.grid(alpha=.2);ax.axvline(0,color='k',ls=':',lw=.8);ax.axvline(land-start,color='k',ls='--',lw=.8);ax.axhline(0,color='gray',lw=.6)
  ax.set_xlim(-6,end-start+3)
 axes[0,col].set_ylim(-1.8,3.0);axes[1,col].set_ylim(-.8,.8);axes[2,col].set_ylim(-.4,.4)
 axes[0,col].set_title(f'Completed mission {col+1} · FC {start:.1f}–{end:.1f} s')
 axes[2,col].set_xlabel('Seconds from AUTO start (dashed line = LAND)')
axes[0,0].set_ylabel('Height (m)');axes[1,0].set_ylabel('Upward velocity (m/s)');axes[2,0].set_ylabel('Innovation (m/s)')
for row in range(3):axes[row,0].legend(fontsize=8,loc='upper right')
fig.suptitle('The controller reaches 1.5 m in its estimate while VIO sees a much lower aircraft',fontsize=15)
fig.savefig(p/'altitude-comparison.png',dpi=170)
