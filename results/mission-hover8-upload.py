"""One-shot requested mission change; disarmed only, with readback verification."""
import json
import time
from pathlib import Path
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/cu.usbmodem2101', baud=115200, source_system=254)
h = m.wait_heartbeat(timeout=10)
assert h is not None and h.get_srcSystem() == 1 and h.get_srcComponent() == 1
sysid, compid = 1, 1

def disarmed():
    h = m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    assert h is not None and h.get_srcComponent() == 1 and not h.base_mode & 128, 'Not confirmed disarmed'

def download():
    m.mav.mission_request_list_send(sysid, compid)
    count = m.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    assert count is not None and count.count == 4, 'Unexpected mission count'
    rows = []
    for seq in range(count.count):
        m.mav.mission_request_int_send(sysid, compid, seq)
        x = m.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        assert x is not None and x.seq == seq, 'Mission read failed'
        rows.append(x.to_dict())
    m.mav.mission_ack_send(sysid, compid, 0)
    return rows

try:
    disarmed()
    before = download()
    assert [r['command'] for r in before] == [16, 22, 19, 21]
    assert all(before[i]['frame'] == 3 and before[i]['x'] == 0 and before[i]['y'] == 0 for i in [1, 2, 3])
    folder = Path('results/mission-hover8-' + time.strftime('%Y%m%d-%H%M%S'))
    folder.mkdir()
    (folder / 'before.json').write_text(json.dumps(before, indent=2))
    wanted = [dict(r) for r in before]
    wanted[1]['z'] = wanted[2]['z'] = 8.0
    wanted[2]['param1'] = 30.0
    disarmed()
    m.mav.mission_count_send(sysid, compid, len(wanted))
    sent = set()
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        x = m.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK', 'HEARTBEAT'], blocking=True, timeout=5)
        assert x is not None, 'Upload timed out'
        if x.get_type() == 'HEARTBEAT':
            assert not x.base_mode & 128, 'Aircraft armed during upload'
            continue
        if x.get_type() == 'MISSION_ACK':
            assert x.type == 0 and sent == set(range(4)), 'Upload rejected or incomplete'
            break
        r = wanted[x.seq]
        send = m.mav.mission_item_int_send if x.get_type() == 'MISSION_REQUEST_INT' else m.mav.mission_item_send
        send(sysid, compid, r['seq'], r['frame'], r['command'], r['current'], r['autocontinue'],
             r['param1'], r['param2'], r['param3'], r['param4'], r['x'], r['y'], r['z'])
        sent.add(x.seq)
    else:
        raise RuntimeError('No upload acknowledgment')
    actual = download()
    fields = ['seq', 'frame', 'command', 'autocontinue', 'param1', 'param2', 'param3', 'param4', 'x', 'y', 'z']
    for a, b in zip(actual, wanted):
        assert all(a[k] == b[k] for k in fields), (a, b)
    (folder / 'verified.json').write_text(json.dumps(actual, indent=2))
    print('VERIFIED: takeoff 8 m relative home; loiter 8 m for 30 s; land. Backup/readback:', folder)
finally:
    m.close()
