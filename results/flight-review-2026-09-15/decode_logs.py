from pymavlink import mavutil
from pathlib import Path
import json, collections
out=Path('/Users/luke/Projects/visual-inertial-odometry/results/flight-review-2026-09-15')
for f in sorted(out.glob('*.bin')):
 d=collections.defaultdict(list); mlog=mavutil.mavlink_connection(str(f))
 while (m:=mlog.recv_match()) is not None:
  o=m.to_dict(); d[m.get_type()].append(o)
 (out/(f.stem+'.json')).write_text(json.dumps(d,default=str))
 print('\nFILE', f.name, 'counts', {k:len(v) for k,v in d.items() if k not in ['FMT','FMTU','UNIT','MULT','PARM']})
 for k in ['MSG','MODE','ARM','EV','ERR','CMD','ORGN']:
  print(k, d[k])
 print('PARAMS',[(x['Name'],x['Value']) for x in d['PARM'] if x['Name'].startswith(('EK3_SRC','VISO','MIS_','RNGFND','WP_','SURFTRAK','BARO'))])
 for k in ['CTUN','PSCD','XKF1','XKF3','XKF4','VISP','VISV','RFND','POS','GPS','BARO']:
  print(k, d[k][:1])
