"""Synthetic evidence only: no fixed checkpoint reads or real sensors."""
import argparse
import importlib.util
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))

from aic_transfuser_lite.runtime.spatial_recording_v4 import encoded,sha
from aic_transfuser_lite.runtime.spatial_runtime_v4 import SpatialRuntimeV4


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('synthetic_fixtures_only',ROOT/'tests/test_spatial_runtime_v4.py')
    fixture=importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture)
    records=fixture.records(); events=[]
    for mode in ('normal','nan','scalar','dtype','exception'):
        core=SpatialRuntimeV4(fixture.Fake(mode),records)
        events.append(core.infer(fixture.batch()))
    core=SpatialRuntimeV4(fixture.Fake(),records)
    def fail_copy(_): raise RuntimeError('synthetic GPU copy failure')
    core.snapshot=fail_copy
    events.append(core.infer(fixture.batch()))
    dropped=records.accept(); dropped['payload'].update(event_type='DROP',status='DROPPED',reason='synthetic before input build')
    records.counts['dropped'].add(dropped['payload']['candidate_sequence'])
    events.extend([dropped,records.event('SESSION_END')])
    blob=b''
    for event in events:
        records.validate(event); blob+=encoded(event)+b'\n'
    (args.output/'synthetic_events.jsonl').write_bytes(blob)
    result=dict(synthetic_forward_calls=6,fixed_checkpoint_forward_calls=0,events=len(events),semantic_validation='PASS',sha256=sha(blob))
    (args.output/'manifest.json').write_bytes(encoded(result)+b'\n')
    print(encoded(result).decode())
