"""Compare recorded clocks and issued-publication continuity without weakening gates."""
from pathlib import Path
import argparse
import hashlib
import json
import sqlite3
import struct
import numpy as np
from aic_transfuser_lite.data.time_large_recovery_v1 import large_recovery_events


def stats(values):
    a=np.asarray(values,dtype=np.float64)
    return dict(count=len(a),median_ms=float(np.median(a)) if len(a) else None,
                p95_ms=float(np.percentile(a,95)) if len(a) else None,
                max_ms=float(a.max()) if len(a) else None,
                over_150ms=int((a>150.0001).sum()))


def audit(raw):
    rows=[json.loads(s) for s in (raw/'control.jsonl').read_text().splitlines()]
    events=large_recovery_events(rows)
    clocks=[]
    for db in sorted(raw.glob('bag/*.db3')):
        with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as con:
            tid=con.execute("select id from topics where name='/clock'").fetchone()[0]
            for receipt,blob in con.execute('select timestamp,data from messages where topic_id=? order by id',(tid,)):
                assert len(blob)==12 and blob[:4]==b'\x00\x01\x00\x00'
                sec,nsec=struct.unpack_from('<iI',blob,4)
                clocks.append((sec*10**9+nsec,receipt))
    pubs=[r['publication'] for r in rows if r.get('publication') is not None]
    records=[]
    for event in events:
        start=event['release_ns']
        if start is None:continue
        end=event['end_publication_ns'] or start+10*10**9
        # The extra 3 s includes the full teacher future horizon.
        window_end=end+3*10**9
        selected=[p for p in pubs if start<=p['sim_ns']<=window_end]
        c=[p for p in clocks if start<=p[0]<=window_end]
        gaps=[]
        for a,b in zip(selected,selected[1:]):
            d=(b['sim_ns']-a['sim_ns'])/1e6
            if d>150.0001:
                gaps.append(dict(start_sim_ns=a['sim_ns'],end_sim_ns=b['sim_ns'],sim_ms=d,
                                 wall_ms=(b['monotonic_ns']-a['monotonic_ns'])/1e6))
        records.append(dict(site_id=event['site_id'],release_ns=start,end_ns=window_end,
            recovered=event['recovery_confirmed'] and event['completed'],
            control_sim_gap=stats([(b['sim_ns']-a['sim_ns'])/1e6 for a,b in zip(selected,selected[1:])]),
            control_wall_gap=stats([(b['monotonic_ns']-a['monotonic_ns'])/1e6 for a,b in zip(selected,selected[1:])]),
            clock_sim_gap=stats([(b[0]-a[0])/1e6 for a,b in zip(c,c[1:])]),
            large_control_gaps=gaps))
    moving=[r for r in rows if r.get('phase') not in ('invalid','braking')]
    recv={}
    for r in rows:
        for role,v in r.get('trajectory_receive_stats',{}).items():
            recv[role]=v
    return dict(run_id=raw.name,control_sha256=hashlib.sha256((raw/'control.jsonl').read_bytes()).hexdigest(),
                events=records,trajectory_receive_stats=recv,
                decision_wall=stats([r['decision_wall_ms'] for r in moving if 'decision_wall_ms' in r]),
                processing=stats([r['processing_ms'] for r in moving if 'processing_ms' in r]),
                scope='Measured publication timestamps and recorded clock; not actuator command delivery latency')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--raw',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    r=audit(a.raw)
    with a.output.open('x') as f:json.dump(r,f,indent=2,allow_nan=False)
    print(json.dumps(r))
