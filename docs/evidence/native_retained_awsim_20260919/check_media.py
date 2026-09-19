"""Decode both recordings and make a timestamped encounter contact sheet."""
from pathlib import Path
import json
import subprocess
import sys

from PIL import Image, ImageDraw

root = Path('/home/thistle/e2e_autonomous/runs/time_native_retained_awsim_20260919')
kind = sys.argv[1]
assert kind in ('cone', 'box')
run = root / kind / ('codex-time-retained-' + kind + '-01')
output = root / (kind + '_media')
output.mkdir()
results = {}
for role in ('awsim', 'rviz'):
    media = run / (role + '.mp4')
    decode = subprocess.run(['ffmpeg', '-hide_banner', '-v', 'error', '-xerror', '-i', str(media),
                             '-f', 'null', '-'], capture_output=True, text=True, timeout=120)
    assert decode.returncode == 0, (role, decode.stderr)
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-count_frames', '-show_entries',
        'stream=codec_name,width,height,nb_read_frames:format=duration,size', '-of', 'json', str(media)], timeout=120))
    duration = float(probe['format']['duration'])
    times = [min(t, duration - .8) for t in (18., 21., 24., 27., 30., 33.)]
    if kind == 'cone5':
        times = [max(0., duration - t) for t in (16., 13., 10., 7., 4., 1.)]
    sheet = Image.new('RGB', (1280, 720), '#181b21')
    draw = ImageDraw.Draw(sheet)
    for index, stamp in enumerate(times):
        path = output / f'{role}_{index:02d}.png'
        subprocess.run(['ffmpeg', '-hide_banner', '-v', 'error', '-n', '-ss', str(stamp), '-i', str(media),
                        '-frames:v', '1', str(path)], check=True, timeout=30)
        frame = Image.open(path)
        frame.thumbnail((422, 330))
        x = index % 3 * 426
        y = index // 3 * 360
        draw.text((x + 5, y + 5), f'{kind} / {role} / video {stamp:.1f} s', fill='white')
        sheet.paste(frame, (x, y + 25))
    sheet.save(output / (role + '_encounter.jpg'), quality=92)
    results[role] = dict(status='PASS', decoded_all_frames=True, probe=probe, frame_video_times_s=times,
                         decode_stderr=decode.stderr)
with (output / 'summary.json').open('x') as f:
    json.dump(results, f, indent=2)
print(json.dumps(results))
