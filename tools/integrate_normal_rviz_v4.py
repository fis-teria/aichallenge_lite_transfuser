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


def integrate(text: str) -> str:
    if '/visualization/v4_20/raw_path' in text:
        raise ValueError('ALREADY_INTEGRATED')
    anchor='Visualization Manager:\n  Class: ""\n  Displays:\n'
    if text.count(anchor)!=1:
        raise ValueError('UNKNOWN_RVIZ_LAYOUT')
    return text.replace(anchor,anchor+DISPLAY,1)


def main() -> None:
    ap=argparse.ArgumentParser();ap.add_argument('config',type=Path)
    args=ap.parse_args();p=args.config
    original=p.read_bytes()
    text=original.decode('utf-8')
    newline='\r\n' if '\r\n' in text else '\n'
    value=integrate(text.replace('\r\n','\n')).replace('\n',newline).encode('utf-8')
    backup=p.with_name(p.name+'.before-v4-20')
    with backup.open('xb') as f:f.write(original)
    p.write_bytes(value)

if __name__=='__main__':main()
