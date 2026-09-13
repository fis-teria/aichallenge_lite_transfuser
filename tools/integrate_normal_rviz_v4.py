"""Add a stock Path display to the existing Autoware RViz configuration."""
from __future__ import annotations
import argparse
from pathlib import Path

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
