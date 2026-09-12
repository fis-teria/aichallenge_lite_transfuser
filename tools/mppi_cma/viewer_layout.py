"""Keep native X11 window races outside the dashboard/viewer process."""
from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
import subprocess
import sys


VIEWER_CONTAINER = 'cma-mppi-rviz-viewer'
VIEWER_CONFIG = '/' + VIEWER_CONTAINER + '.rviz'


def arrange_once(gui: Path) -> bool:
    """Place this viewer and its dashboard; call only in an expendable child."""
    spec = importlib.util.spec_from_file_location('cma_window_layout', gui / 'layout_sim_windows.py')
    layout = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = layout
    spec.loader.exec_module(layout)
    windows = layout.list_client_windows()
    rviz = [w for w in windows if VIEWER_CONFIG in w.title]
    browser = [w for w in windows if 'MPPI' in w.title and 'rviz' not in w.title.lower()]
    if not rviz:
        return False
    area = layout.active_workarea()
    mover = layout.X11Mover()
    try:
        mover.place(rviz[-1].window_id, layout.Rect(area.x, area.y, area.width // 2, area.height))
        if browser:
            mover.place(browser[-1].window_id, layout.Rect(area.x + area.width // 2, area.y,
                                                         area.width // 2, area.height))
        return bool(browser)
    finally:
        mover.close()


def try_arrange(gui: Path, timeout_s: float = 8.0) -> bool:
    """Bound one layout attempt in seconds; report failures and permit retries.

    Xlib's default BadWindow handler exits natively, bypassing Python exception
    handling. The child boundary also protects HTTP updates from a stuck mover.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('layout timeout_s must be finite and positive')
    try:
        result = subprocess.run([sys.executable, str(Path(__file__).resolve()), str(gui)],
                                capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f'Window layout retry: {error}', flush=True)
        return False
    if result.returncode not in (0, 3):
        detail = (result.stderr.strip() or result.stdout.strip())[-1200:]
        print(f'Window layout retry (exit {result.returncode}): {detail}', flush=True)
    return result.returncode == 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('gui', type=Path)
    raise SystemExit(0 if arrange_once(parser.parse_args().gui) else 3)
