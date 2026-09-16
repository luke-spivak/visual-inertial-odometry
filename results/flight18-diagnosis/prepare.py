import json
from pathlib import Path
p=Path('datasets/flights/run-20260915-153908')
out=Path('results/flight18-diagnosis/config')
out.mkdir(exist_ok=True)
for name,data in json.loads(Path(str(p)+'.recording.json').read_text())['config_files'].items():
    (out/name).write_text(data)
stamps=json.loads(Path(str(p)+'.meta.json').read_text())
Path(str(p)+'.timestamps.txt').write_text(''.join(str(x['SensorTimestamp'])+'\n' for x in stamps))
