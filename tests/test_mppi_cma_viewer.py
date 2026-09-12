"""A disappearing X11 window must not kill HTTP updates or later layout attempts."""
from __future__ import annotations

import importlib
import json
from pathlib import Path
import subprocess
import threading
from urllib.request import urlopen

import pytest

from tools.mppi_cma.dashboard import server
from tools.mppi_cma.viewer_layout import try_arrange


def layout_fixture(gui: Path, action: str) -> None:
    (gui / 'layout_sim_windows.py').write_text('''
from types import SimpleNamespace as Obj
import os
import time
def list_client_windows():
    return [Obj(title='/cma-mppi-rviz-viewer.rviz - RViz', window_id=1),
            Obj(title='MPPI dashboard', window_id=2),
            Obj(title='Unrelated - RViz', window_id=3)]
def active_workarea():
    return Obj(x=0, y=0, width=1000, height=800)
def Rect(*args):
    return args
class X11Mover:
    def place(self, window_id, rect):
        assert window_id in (1, 2), 'unrelated RViz must remain untouched'
        ''' + action + '''
    def close(self):
        pass
''')


def test_native_exit_does_not_stop_http_and_next_attempt_recovers(tmp_path, capsys):
    (tmp_path / 'search').mkdir()
    state = tmp_path / 'search/state.json'
    state.write_text(json.dumps({'completed': True, 'phase': 'complete', 'conditions': {},
                                'new_episodes_started': 0, 'maximum_new_episodes': 64}))
    (tmp_path / 'snapshot').mkdir()
    (tmp_path / 'snapshot/base_reference.csv').write_text('x_m,y_m\n1,2\n')
    (tmp_path / 'calibration.json').write_text('{"ot_lane_polygon_map_m": []}')
    original = state.read_bytes()
    httpd = server(tmp_path, port=0)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{httpd.server_port}/api/state'
    try:
        layout_fixture(tmp_path, "os.write(2, b'X Error: BadWindow\\n'); os._exit(1)")
        assert not try_arrange(tmp_path)
        assert 'BadWindow' in capsys.readouterr().out
        with urlopen(url, timeout=3) as response:
            assert json.load(response)['completed']
        layout_fixture(tmp_path, 'pass')
        assert try_arrange(tmp_path)
        with urlopen(url, timeout=3) as response:
            assert json.load(response)['completed']
        assert state.read_bytes() == original
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=3)


def test_layout_timeout_is_bounded_and_retryable(tmp_path, capsys):
    layout_fixture(tmp_path, 'time.sleep(10)')
    assert not try_arrange(tmp_path, timeout_s=0.1)
    assert 'Window layout retry' in capsys.readouterr().out
    layout_fixture(tmp_path, 'pass')
    assert try_arrange(tmp_path)


@pytest.mark.parametrize('timeout_s', [0, -1, float('nan'), float('inf')])
def test_layout_rejects_invalid_seconds(tmp_path, timeout_s):
    with pytest.raises(ValueError, match='timeout_s'):
        try_arrange(tmp_path, timeout_s=timeout_s)


def test_orphan_cleanup_checks_ownership_before_removing(tmp_path, monkeypatch):
    pytest.importorskip('fcntl')
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools/mppi_cma'))
    viewer = importlib.import_module('show_live')
    record = {'Id': 'owned-id', 'Name': '/cma-mppi-rviz-viewer',
              'Config': {'Image': 'frozen-image', 'Cmd': ['rviz2', '-d', '/viewer.rviz']},
              'Mounts': [{'Source': str(tmp_path / 'gui/live.rviz'), 'Destination': '/viewer.rviz'}]}
    calls = []

    def docker(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 0, json.dumps([record]), '')

    monkeypatch.setattr(viewer.subprocess, 'run', docker)
    viewer.clear_orphan_viewer(tmp_path, 'frozen-image')
    assert calls[-1] == ['docker', 'rm', '-f', 'owned-id']
    for source, image, cmd in [('elsewhere/live.rviz', 'frozen-image', ['rviz2']),
                               (str(tmp_path / 'gui/live.rviz'), 'other-image', ['rviz2']),
                               (str(tmp_path / 'gui/live.rviz'), 'frozen-image', ['awsim'])]:
        record['Mounts'][0]['Source'] = source
        record['Config'].update(Image=image, Cmd=cmd)
        calls.clear()
        with pytest.raises(RuntimeError, match='does not belong'):
            viewer.clear_orphan_viewer(tmp_path, 'frozen-image')
        assert len(calls) == 1 and calls[0][1] == 'inspect'


def test_missing_orphan_is_ok_but_docker_error_is_reported(tmp_path, monkeypatch):
    pytest.importorskip('fcntl')
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / 'tools/mppi_cma'))
    viewer = importlib.import_module('show_live')
    monkeypatch.setattr(viewer.subprocess, 'run', lambda command, **kwargs:
                        subprocess.CompletedProcess(command, 1, '', 'Error: No such object: viewer'))
    viewer.clear_orphan_viewer(tmp_path, 'image')
    monkeypatch.setattr(viewer.subprocess, 'run', lambda command, **kwargs:
                        subprocess.CompletedProcess(command, 1, '', 'Cannot connect to Docker daemon'))
    with pytest.raises(RuntimeError, match='Cannot inspect'):
        viewer.clear_orphan_viewer(tmp_path, 'image')
