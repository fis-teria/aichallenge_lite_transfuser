"""Inspect logged proximity observations without labelling them physical contact."""
import hashlib
import json
from pathlib import Path
import subprocess

root=Path('/home/thistle/e2e_autonomous/runs/time_preview20_20260917')
raw=root/'raw/codex-time-timepreview20-lap01';out=root/'evaluation'
manifest={r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
data=(raw/'control.jsonl').read_bytes()
assert hashlib.sha256(data).hexdigest()==manifest['control.jsonl']['sha256']
rows=[json.loads(line) for line in data.splitlines()]
armed=next(r['sim_ns'] for r in rows if r.get('event')=='ARMED')
observations=[r for r in rows if r.get('event')=='COMMAND_SENT' and r.get('reason')=='TIME_PATH_TRACKING'
              and r.get('details',{}).get('obstacle_guard',{}).get('would_stop_reason')]
result=dict(scope='PADDED_MONITOR_INTRUSION_NOT_PHYSICAL_CONTACT_MEASUREMENT',
    command_count=len(observations),commands=[dict(sim_after_arm_s=(r['sim_ns']-armed)/1e9,
        speed_kmh=r['speed_mps']*3.6,target_speed_kmh=r['target_speed_mps']*3.6,
        acceleration_mps2=r['acceleration_mps2'],pose=r['details']['current_pose'],
        minimum_ray_margin_m=r['details']['obstacle_guard']['minimum_ray_margin_m'],
        occupied_ray_count=r['details']['obstacle_guard']['occupied_ray_count']) for r in observations],
    physical_contact_confirmed=False,frames=[])
metadata=json.loads((raw/'video_recording.json').read_bytes())
if observations:
    role='awsim';item=metadata['streams'][role];video=raw/item['file']
    assert hashlib.sha256(video.read_bytes()).hexdigest()==manifest[video.name]['sha256']
    first=(observations[0]['monotonic_ns']-metadata['start_monotonic_ns'])/1e9
    duration=float(item['probe']['format']['duration'])
    for label,offset in [('before',-.5),('during',0.),('after',1.)]:
        t=min(duration-1,max(0.,first+offset));target=out/f'proximity_{label}.png'
        assert not target.exists()
        subprocess.run(['ffmpeg','-nostdin','-v','error','-ss',str(t),'-i',str(video),'-frames:v','1',str(target)],check=True,timeout=30)
        result['frames'].append(dict(file=target.name,video_s=t,source_video_sha256=manifest[video.name]['sha256']))
(out/'proximity_observations.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
