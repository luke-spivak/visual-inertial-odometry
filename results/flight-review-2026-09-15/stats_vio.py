import json,glob,numpy as np
x=json.load(open(glob.glob('results/flight-review-2026-09-15/log_16*.json')[0]))
def sel(k,a,b):return [v for v in x[k] if a<=v['TimeUS']/1e6<=b]
def med(k,f,a,b):return float(np.median([v[f] for v in sel(k,a,b)]))
for a,b,h1,h2 in [(201.53,250.83,214,227),(315.73,360.51,331,354),(411.83,456.31,426,450),(490.13,544.92,507,530)]:
 print('\nFLIGHT',a,b)
 for k,fs in [('CTUN',['DAlt','Alt','BAlt','ThO','CRt']),('VISP',['PZ','PX','PY']),('VISV',['VZ']),('XKF4',['SV','SP','SH','SM']),('XKF3',['IVD','IPD']),('BARO',['Press','Temp']),('VIBE',['VibeX','VibeY','VibeZ'])]:
  for f in fs:
   val=[v[f] for v in sel(k,h1,h2)]
   print(k,f,'pre',round(med(k,f,a-2,a),3),'hover median/min/max',np.round([np.median(val),min(val),max(val)],3),'post',round(med(k,f,b+1,b+3),3))
 print('sample t DAlt Alt BAlt VIO-up vz ThO')
 for t in np.arange(a,b,2):
  def near(k,f):return round(min(x[k],key=lambda v:abs(v['TimeUS']/1e6-t))[f],3)
  print(round(t,1),*[near(k,f) for k,f in [('CTUN','DAlt'),('CTUN','Alt'),('CTUN','BAlt'),('VISP','PZ'),('VISV','VZ'),('CTUN','ThO')]])
for k in ['VISP','VISV']:
 z=x[k];dt=np.diff([v['TimeUS']/1e6 for v in z]);print(k,'count',len(z),'ignored',sum(v['Ign']!=0 for v in z),'reset',set(v['Rst'] for v in z),'dt max/p99',max(dt),np.quantile(dt,.99))
