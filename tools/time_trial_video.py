"""Record only the owned AWSIM and normal RViz windows, outside simulator assets."""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import subprocess
import time
from typing import Any


def select_video_windows(tree: str, rviz_window: str) -> dict[str, str]:
    """Require one visible simulator window and the already verified RViz ID."""
    candidates: list[str] = []
    rviz_found = False
    for line in tree.splitlines():
        match = re.match(r'\s*(0x[0-9a-fA-F]+)\s+.*?\s(\d+)x(\d+)[+-]', line)
        if not match or min(int(match[2]), int(match[3])) < 100:
            continue
        window = match[1]
        if window == rviz_window and 'autoware.rviz' in line:
            rviz_found = True
        elif re.search(r'awsim|racing.?kart|\("unity"', line, re.IGNORECASE):
            candidates.append(window)
    if not rviz_found or len(candidates) != 1:
        raise ValueError('VIDEO_OWNED_WINDOWS_NOT_UNIQUE')
    return {'awsim': candidates[0], 'rviz': rviz_window}


class TrialVideo:
    """Finite 10 fps capture; caller freezes the simulator before stopping us."""

    def __init__(self, output: Path, run_id: str, display: str, windows: dict[str, str]) -> None:
        if (not re.fullmatch(r'codex-time-[a-z0-9-]+', run_id)
                or not re.fullmatch(r':[0-9]+', display)
                or set(windows) != {'awsim', 'rviz'}
                or len(set(windows.values())) != 2
                or any(not re.fullmatch(r'0x[0-9a-fA-F]+', v) for v in windows.values())):
            raise ValueError('VIDEO_SCOPE')
        self.output = output
        self.run_id = run_id
        self.display = display
        self.windows = windows
        self.processes: list[tuple[str, subprocess.Popen, Any]] = []
        self.record: dict[str, Any] = {'status': 'STARTING', 'fps': 10, 'audio': False,
            'scope': 'OWNED_AWSIM_AND_RVIZ_WINDOWS_ONLY', 'windows': windows, 'commands': {},
            'start_monotonic_ns': time.monotonic_ns(), 'streams': {}}
        self.image = subprocess.check_output(['docker', 'image', 'inspect', 'codex-time-video:20260917',
                                              '--format', '{{.Id}}'], text=True, timeout=10).strip()
        if not re.fullmatch(r'sha256:[0-9a-f]{64}', self.image):
            raise ValueError('VIDEO_IMAGE_ID')
        self.record['image_id'] = self.image

    def start(self) -> None:
        for role, window in self.windows.items():
            name = self.run_id+'-video-'+role
            command = ['docker', 'run', '--rm', '--network', 'none', '--user', f'{os.getuid()}:{os.getgid()}',
                '--name', name, '-v', '/tmp/.X11-unix:/tmp/.X11-unix:ro', '-v', str(self.output)+':/video',
                '--entrypoint', 'ffmpeg', self.image, '-hide_banner', '-loglevel', 'warning', '-nostdin', '-n',
                '-f', 'x11grab', '-framerate', '10', '-draw_mouse', '0', '-window_id', window, '-i', self.display,
                '-an', '-vf', 'scale=1280:-2', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '23',
                '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-t', '720',
                '-progress', '/video/'+role+'_video.progress', '/video/'+role+'.mp4']
            self.record['commands'][role] = command
            stream = (self.output/(role+'_video.log')).open('x')
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            self.processes.append((role, process, stream))
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            self.assert_alive()
            if all((self.output/(role+'_video.progress')).exists()
                   and any(int(n) > 0 for n in re.findall(r'^frame=(\d+)$',
                       (self.output/(role+'_video.progress')).read_text(), re.MULTILINE)) for role in self.windows):
                self.record['status'] = 'RECORDING'
                self.record['ready_monotonic_ns'] = time.monotonic_ns()
                self._save()
                return
            time.sleep(.1)
        raise RuntimeError('VIDEO_FIRST_FRAME_TIMEOUT')

    def assert_alive(self) -> None:
        for role, process, _ in self.processes:
            if process.poll() is not None:
                raise RuntimeError('VIDEO_RECORDER_EXIT:'+role)

    def _save(self) -> None:
        (self.output/'video_recording.json').write_text(json.dumps(self.record, indent=2))

    def stop(self) -> list[str]:
        errors: list[str] = []
        for role, process, stream in self.processes:
            name = self.run_id+'-video-'+role
            try:
                if process.poll() is None:
                    subprocess.run(['docker', 'kill', '--signal=INT', name], capture_output=True, timeout=5)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        subprocess.run(['docker', 'kill', name], capture_output=True, timeout=5, check=True)
                        process.wait(timeout=5)
                        raise RuntimeError('VIDEO_FINALIZE_TIMEOUT:'+role)
                data = subprocess.check_output(['docker', 'run', '--rm', '--network', 'none',
                    '-v', str(self.output)+':/video:ro', '--entrypoint', 'ffprobe', self.image,
                    '-v', 'error', '-show_entries', 'stream=codec_name,width,height,avg_frame_rate,nb_frames:format=duration,size',
                    '-of', 'json', '/video/'+role+'.mp4'], timeout=15)
                probe = json.loads(data)
                if (float(probe['format']['duration']) <= 0 or len(probe['streams']) != 1
                        or probe['streams'][0]['codec_name'] != 'h264'):
                    raise ValueError('VIDEO_STREAM_INVALID:'+role)
                self.record['streams'][role] = {'file': role+'.mp4', 'ffmpeg_exit': process.returncode, 'probe': probe}
            except Exception as exc:
                errors.append(str(exc))
            finally:
                stream.close()
        self.processes.clear()
        self.record.update(status='PASS' if not errors else 'FAILED', errors=errors, end_monotonic_ns=time.monotonic_ns())
        self._save()
        return errors
