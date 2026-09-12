"""Prepare visualization-only assets from this study's frozen controller inputs."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess

import yaml


def prepare(root: Path) -> None:
    gui = root / 'gui'
    gui.mkdir(exist_ok=True)
    environment = json.loads((root / 'environment.json').read_text())
    source = Path(environment['source_root'])
    shutil.copy2(source / 'head_to_head/window_layout/layout_sim_windows.py', gui / 'layout_sim_windows.py')
    original_path = gui / 'original.rviz'
    if not original_path.exists():
        content = subprocess.check_output(
            ['docker', 'run', '--rm', '--network', 'none', '--entrypoint', 'cat',
             environment['controller_image_id'],
             '/aichallenge/workspace/install/aichallenge_system_launch/share/aichallenge_system_launch/config/autoware.rviz'],
            timeout=30)
        original_path.write_bytes(content)
    original = yaml.safe_load(original_path.read_text())
    flat = [item for group in original['Visualization Manager']['Displays'] for item in group.get('Displays', [])]
    displays = [item for item in flat if item['Name'] in {'VehicleModel', 'KinematicState', 'ScenarioTrajectory'}]
    for item in displays:
        if item['Name'] == 'KinematicState':
            item['Keep'] = 25
        if item['Name'] == 'ScenarioTrajectory':
            item['View Text Velocity']['Value'] = False
            item['View Path'].update({'Constant Color': True, 'Color': '80; 170; 255', 'Alpha': .9})
    for name, topic in [('MPPI Candidates', '/debug/mppi/candidates'),
                        ('MPPI Selected', '/debug/mppi/selected'), ('MPPI Baseline', '/debug/mppi/baseline')]:
        displays.append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': name, 'Enabled': True,
                         'Topic': {'Value': topic, 'Depth': 1, 'Reliability Policy': 'Reliable',
                                   'Durability Policy': 'Volatile'}})
    displays.insert(0, {'Class': 'rviz_default_plugins/Map', 'Name': 'Course walls', 'Enabled': True, 'Alpha': .8,
                        'Topic': {'Value': '/debug/mppi/wall_map', 'Depth': 1, 'Reliability Policy': 'Reliable',
                                  'Durability Policy': 'Transient Local'}})
    with (root / 'snapshot/base_reference.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    x, y = [float(row['x_m']) for row in rows], [float(row['y_m']) for row in rows]
    config = {
        'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'}, {'Class': 'rviz_common/Views', 'Name': 'Views'}],
        'Visualization Manager': {
            'Class': '', 'Displays': displays,
            'Global Options': {'Background Color': '20; 29; 44', 'Fixed Frame': 'map', 'Frame Rate': 15},
            'Tools': [{'Class': 'rviz_default_plugins/' + name} for name in ('MoveCamera', 'Select', 'FocusCamera')],
            'Views': {'Current': {'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Current View',
                                  'Target Frame': 'map', 'Angle': 0., 'Scale': 12.,
                                  'X': (min(x) + max(x)) / 2, 'Y': (min(y) + max(y)) / 2}, 'Saved': []}},
        'Window Geometry': {'Width': 1280, 'Height': 1500, 'X': 0, 'Y': 27,
                            'Hide Left Dock': True, 'Hide Right Dock': True}}
    (gui / 'live.rviz').write_text(yaml.safe_dump(config, sort_keys=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    prepare(parser.parse_args().root)
