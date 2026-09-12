"""Score closed episodes from native results and timestamp-aligned position telemetry."""
from __future__ import annotations
import json
import math
from pathlib import Path
import re
import sys
import numpy as np
from geometry import convex_overlap, vehicle_polygon
from shared_course_state import native_vehicle_status

ROOT=Path('/home/si26-pc008/cma_mppi_20260912')


def analyze(output: Path, vehicle: int = 1) -> dict:
    runtime=json.loads((output/'runtime_result.json').read_text())
    if not runtime['ok'] or not runtime.get('started'):
        raise RuntimeError('Infrastructure/initialization failure: '+runtime['reason'])
    config=json.loads((output/'config.json').read_text())
    shared = config.get('vehicle_count', 1) == 4
    if vehicle not in range(1, config.get('vehicle_count', 1) + 1):
        raise ValueError('Vehicle number is outside this episode')
    native=json.loads((output/f'd{vehicle}-result-details.json').read_text())
    cal=json.loads((ROOT/'calibration.json').read_text())
    polygon=np.array(cal['ot_lane_polygon_map_m'])
    odometry={};gps={};debug={};commands={};ranks=[];race_rows=[]
    with (output/'samples.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if shared:
                matches=[car for car in row['vehicles'] if car['vehicle_number']==vehicle]
                if len(matches)!=1:raise RuntimeError('Missing or duplicate vehicle telemetry')
                row={**row,**matches[0]}
                row['status']=native_vehicle_status(row.get('summary'),vehicle)
                for car in (row.get('summary') or {}).get('vehicles',[]):
                    if car['vehicle_number']==vehicle and car['finished']:
                        row['finish']=True
            ego=row.get('ego')
            if ego:odometry[ego['stamp_s']]=ego
            status=row.get('status')
            if not row.get('started') or row.get('finish') or row.get('admin','').lower()!='start' or not status or not 0<=status[1]<=2:
                continue
            race_rows.append(row)
            ranks.append(int(status[4]))
            if row.get('gnss'):gps[row['gnss']['stamp_s']]=row['gnss']
            if row.get('command'):commands[row['command']['stamp_s']]=row['command']
            if row.get('debug'):
                normalized=re.sub(r'(?<![A-Za-z])(-?inf|nan)(?=[,}\]])','null',row['debug'])
                d=json.loads(normalized)
                debug[d['control_command_stamp_sec']]=d
    valid_ranks = set(ranks) <= {1, 2, 3, 4} if shared else set(ranks)=={1}
    if len(gps)<100 or len(odometry)<100 or len(debug)<100 or not ranks or not valid_ranks:
        raise RuntimeError('Required position, rank or controller telemetry missing')
    if any(abs(float(d['steering_acceleration_hold_maximum_acceleration_mps2'])-.6)>1e-6 for d in debug.values()):
        raise RuntimeError('Current corner-acceleration contract not active')
    poses=sorted(odometry.values(),key=lambda r:r['stamp_s'])
    ts=np.array([r['stamp_s'] for r in poses]);yaw=np.unwrap([r['yaw_rad'] for r in poses])
    samples=sorted(gps.values(),key=lambda r:r['stamp_s'])
    gts=np.array([r['stamp_s'] for r in samples])
    if np.max(np.diff(gts))>.3:
        raise RuntimeError('GNSS telemetry gap exceeds 0.3 s')
    aligned_yaw=np.interp(gts,ts,yaw)
    overlap_s=0.0; overlaps=0; trace=[]
    for i,(sample,angle) in enumerate(zip(samples,aligned_yaw)):
        if not ts[0]<=sample['stamp_s']<=ts[-1]:
            raise RuntimeError('GNSS pose lacks orientation time support')
        body=vehicle_polygon(sample['x_m'],sample['y_m'],float(angle))
        # A cheap bounding-box test avoids unnecessary SAT work away from the OT zone.
        possible=bool(np.all(body.max(axis=0)+.1>=polygon.min(axis=0)) and
                      np.all(polygon.max(axis=0)+.1>=body.min(axis=0)))
        overlap=possible and convex_overlap(body,polygon,.1)
        dt=gts[i+1]-gts[i] if i+1<len(gts) else 0.0
        if overlap:overlap_s+=float(dt);overlaps+=1
        trace.append([sample['stamp_s'],sample['x_m'],sample['y_m'],float(angle),int(overlap)])
    finish_time=float(native['total_lap_time'])
    events=[e for e in native['penalty_events'] if e['lap']<=2 and
            (not native['finished'] or e['race_time']<=finish_time+1e-3)]
    penalties={kind:sum(e['kind']==kind for e in events) for kind in ['crash','wall','over','block']}
    completed=bool(native['finished'] and native['lap_count']==2 and len(native['laps'])==2)
    lap=float(native['laps'][1]) if completed else 260.0
    hard_count=sum(penalties[k] for k in ['crash','wall','over'])
    # Preference is lexicographic: complete/contact-free, no OT intrusion, then lap time.
    objective=(0 if completed and hard_count==0 else 1_000_000+10_000*hard_count)
    objective+=(0 if overlaps==0 else 10_000+1_000*overlap_s)+lap
    ds=sorted(debug.values(),key=lambda d:d['control_command_stamp_sec'])
    gentle_s=0.0
    for previous,current in zip(ds,ds[1:]):
        dt=current['control_command_stamp_sec']-previous['control_command_stamp_sec']
        if 0<dt<=.3 and abs(previous['signed_curvature_1pm'])>=.06 and previous['steering_acceleration_hold_active'] and previous['commanded_acceleration_mps2']>.05:
            gentle_s+=dt
    speeds=[r['ego']['speed_mps'] for r in race_rows if r.get('ego')]
    result={'episode':output.name+(f'-d{vehicle}' if shared else ''),'completed':completed,'flying_lap_s':lap,
            'laps_s':native['laps'],'penalties':penalties,'ot_overlap_s':overlap_s,
            'ot_overlap_samples':overlaps,'ot_test_margin_m':.1,'position_source':'timestamp-aligned AWSIM GNSS with EKF yaw',
            'rank_values':sorted(set(ranks)),'handicap_enabled':config['handicap'],
            'target_mps':config['target_mps'],'measured_speed_median_mps':float(np.median(speeds)),
            'measured_speed_max_mps':float(max(speeds)),
            'command_speed_max_mps':max(v['speed_mps'] for v in commands.values()),
            'gentle_acceleration_in_corners_s':gentle_s,'objective':objective,
            'preferred_feasible':completed and hard_count==0 and overlaps==0}
    if shared:
        result.update(vehicle_number=vehicle, shared_course=True,
                      ignore_other_vehicles=config['ignore_other_vehicles'],
                      rank1_sample_fraction=sum(rank==1 for rank in ranks)/len(ranks))
    suffix=f'-d{vehicle}' if shared else ''
    (output/f'metrics{suffix}.json').write_text(json.dumps(result,indent=2))
    with (output/f'track{suffix}.csv').open('w') as stream:
        stream.write('stamp_s,x_m,y_m,yaw_rad,ot_overlap\n')
        for row in trace:stream.write(','.join(map(str,row))+'\n')
    return result


if __name__=='__main__':
    print(json.dumps(analyze(Path(sys.argv[1]))),flush=True)
