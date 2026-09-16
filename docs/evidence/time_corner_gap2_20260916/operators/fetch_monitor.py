import argparse
import ops_corner as m
ap=argparse.ArgumentParser();ap.add_argument('--pair',type=int,required=True);a=ap.parse_args()
m.transport.ROOT=m.ROOT;m.transport.OUT=m.OUT;m.transport.UNC=m.UNC
m.transport.copy_to_wsl([f'pair{a.pair:02}_resource_monitor.jsonl',f'pair{a.pair:02}_monitor_supervisor.log'])
