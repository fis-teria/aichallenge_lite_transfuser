"""Decode real recordings in WSL and extract unaltered video frames for QA."""
from pathlib import Path
import hashlib
import json
import subprocess

root = Path('/home/thistle/e2e_autonomous/runs/time_curvature_response_20260917')
raw = root/'raw/codex-time-curve15-response-lap04'
out = root/'evaluation'
manifest = {r['path']:r for r in json.loads((raw/'transfer_manifest.json').read_bytes())}
metadata = json.loads((raw/'video_recording.json').read_bytes())
control = [json.loads(line) for line in (raw/'control.jsonl').read_bytes().splitlines()]
armed = next(r for r in control if r['event']=='ARMED')
rows = []
for role, item in metadata['streams'].items():
    path = raw/item['file']
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert digest == manifest[path.name]['sha256']
    probe = json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries',
        'stream=codec_name,width,height,avg_frame_rate,nb_frames:format=duration,size','-of','json',str(path)]))
    assert probe == item['probe'], (probe,item['probe'])
    check = subprocess.run(['ffmpeg','-nostdin','-v','error','-i',str(path),'-f','null','-'],
        capture_output=True,text=True,timeout=180)
    assert check.returncode == 0 and not check.stderr, check.stderr
    duration = float(probe['format']['duration'])
    after_arm = (armed['monotonic_ns']-metadata['start_monotonic_ns'])/1e9
    seek = min(max(0., after_arm+5.), max(0.,duration-1.))
    thumbnail = out/(role+'_video_frame.png')
    assert not thumbnail.exists()
    subprocess.run(['ffmpeg','-nostdin','-v','error','-ss',str(seek),'-i',str(path),'-frames:v','1',str(thumbnail)],check=True,timeout=30)
    rows.append(dict(role=role,file=item['file'],sha256=digest,bytes=path.stat().st_size,
        duration_s=duration,frames=int(probe['streams'][0]['nb_frames']),full_decode='PASS',
        recording_ready_before_arm=metadata['ready_monotonic_ns']<armed['monotonic_ns'],
        interruption=metadata.get('interruptions',{}).get(role),thumbnail=thumbnail.name,thumbnail_video_s=seek))
(out/'video_verification.json').write_text(json.dumps(dict(status='PASS',recording_status=metadata['status'],videos=rows),indent=2))
print(json.dumps(rows))
