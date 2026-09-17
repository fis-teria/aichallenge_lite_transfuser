"""Add a stock Path display to the existing Autoware RViz configuration."""
from __future__ import annotations
import argparse
from pathlib import Path
import re

DISPLAY = '''    - Class: rviz_default_plugins/Path
      Name: V4-20 raw prediction
      Enabled: true
      Topic:
        Value: /visualization/v4_20/raw_path
        Reliability Policy: Reliable
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 1
      Color: 255; 60; 180
      Line Style: Billboards
      Line Width: 0.08
      Buffer Length: 1
'''


def integrate(text: str, *, ten: bool = False, time_path: bool = False) -> str:
    if ten and time_path:
        raise ValueError('AMBIGUOUS_MODEL_DISPLAY')
    display = DISPLAY.replace('V4-20', 'V4-10').replace('v4_20', 'v4_10') if ten else DISPLAY
    topic = '/visualization/v4_10/raw_path' if ten else '/visualization/v4_20/raw_path'
    if time_path:
        display = DISPLAY.replace('V4-20 raw prediction', 'Time model raw prediction').replace('v4_20', 'time_path')
        topic = '/visualization/time_path/raw_path'
    if topic in text:
        raise ValueError('ALREADY_INTEGRATED')
    anchor='Visualization Manager:\n  Class: ""\n  Displays:\n'
    if text.count(anchor)!=1:
        raise ValueError('UNKNOWN_RVIZ_LAYOUT')
    return text.replace(anchor,anchor+display,1)


def ensure_time_path(text: str) -> str:
    """Enable the existing stock E2E Path without changing other RViz settings."""
    topic = '/visualization/time_path/raw_path'
    if topic not in text:
        return integrate(text, time_path=True)
    blocks = list(re.finditer(r'^    - Class:.*?(?=^    - Class:|^  [^ ]|\Z)', text, re.M | re.S))
    matches = [m for m in blocks if topic in m[0]]
    if len(matches) != 1 or not matches[0][0].startswith('    - Class: rviz_default_plugins/Path\n'):
        raise ValueError('AMBIGUOUS_TIME_PATH_DISPLAY')
    match = matches[0]
    block, count = re.subn(r'^      Enabled: (?:true|false)$', '      Enabled: true', match[0], flags=re.M)
    if count != 1:
        raise ValueError('UNKNOWN_PATH_ENABLED_SETTING')
    return text[:match.start()] + block + text[match.end():]


def follow_ego_view(text: str) -> str:
    """Use the stock top-down view near the car so the short E2E path is legible."""
    match = re.search(r'^  Views:\n    Current:\n.*?(?=^    Saved:)', text, re.M | re.S)
    if match is None or '      Class: rviz_default_plugins/TopDownOrtho\n' not in match[0]:
        raise ValueError('UNKNOWN_CURRENT_RVIZ_VIEW')
    block = match[0]
    for key, value in (('Target Frame', 'base_link'), ('Scale', '60'), ('X', '0'), ('Y', '0')):
        block, count = re.subn(r'^      '+key+r': [^\n]+$', '      '+key+': '+value, block, flags=re.M)
        if count != 1:
            raise ValueError('UNKNOWN_CURRENT_RVIZ_VIEW_FIELD:'+key)
    return text[:match.start()] + block + text[match.end():]


def enable_lidar_map_comparison(text: str) -> str:
    """Add cyan corrected and orange original scans; retain the normal map.

    The wide, vehicle-centred view exposes incorrect map matches as well as
    small offsets. Original scan ranges/TF and vehicle control are unchanged.
    """
    if '/time_path/localization/scan' in text:
        raise ValueError('LOCALIZATION_DISPLAY_ALREADY_PRESENT')
    anchor = 'Visualization Manager:\n  Class: ""\n  Displays:\n'
    if text.count(anchor) != 1:
        raise ValueError('UNKNOWN_RVIZ_LAYOUT')
    displays = ''
    for name, topic, color in (
        ('Corrected LiDAR - cyan', '/time_path/localization/scan', '0; 255; 255'),
        ('Original LiDAR - orange', '/sensing/lidar/scan', '255; 140; 30'),
    ):
        displays += f'''    - Class: rviz_default_plugins/LaserScan
      Name: {name}
      Enabled: true
      Topic:
        Value: {topic}
        Reliability Policy: Best Effort
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 5
      Style: Points
      Size (Pixels): 4
      Color Transformer: FlatColor
      Color: {color}
      Position Transformer: XYZ
      Decay Time: 0.3
      Use Fixed Frame: true
'''
    displays += '''    - Class: rviz_default_plugins/Pose
      Name: Corrected vehicle pose - cyan
      Enabled: true
      Topic:
        Value: /time_path/localization/pose
        Reliability Policy: Reliable
        Durability Policy: Volatile
        History Policy: Keep Last
        Depth: 5
      Color: 0; 255; 255
      Shape: Arrow
      Shaft Length: 2
      Shaft Radius: 0.2
      Head Length: 1
      Head Radius: 0.5
'''
    result = text.replace(anchor, anchor+displays, 1)
    result = follow_ego_view(result)
    return result.replace('      Scale: 60\n', '      Scale: 10\n', 1)


def main() -> None:
    ap=argparse.ArgumentParser();ap.add_argument('config',type=Path)
    ap.add_argument('--ten',action='store_true')
    ap.add_argument('--time-path', action='store_true')
    args=ap.parse_args();p=args.config
    original=p.read_bytes()
    text=original.decode('utf-8')
    newline='\r\n' if '\r\n' in text else '\n'
    value=integrate(text.replace('\r\n','\n'),ten=args.ten,time_path=args.time_path).replace('\n',newline).encode('utf-8')
    suffix = '.before-time-path' if args.time_path else ('.before-v4-10' if args.ten else '.before-v4-20')
    backup=p.with_name(p.name+suffix)
    with backup.open('xb') as f:f.write(original)
    p.write_bytes(value)

if __name__=='__main__':main()
