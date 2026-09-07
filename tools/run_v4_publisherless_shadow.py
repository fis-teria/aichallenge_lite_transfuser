"""Finite publisherless shadow harness. Fixture execution only in this revision.

Real ROS transport remains BLOCKED until passive command and time-bound pose
bindings are supplied. Does not call existing driving runner or launch AWSIM.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
import multiprocessing as mp
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from aic_transfuser_lite.runtime.publisherless_shadow_v4 import Envelope


def supervise(process, *, wall_s: float, monotonic=time.monotonic) -> dict:
    """Own child only. No broad kill, simulator operations, retry or process lookup.

    Entire budget includes start/load/capture; last 5s reserved for teardown.
    If OS cannot reap the child, return failure rather than claim successful end.
    """
    if not 0 < wall_s <= 120:
        raise ValueError('OUTER_WALL_LIMIT')
    start = monotonic()
    process.start()
    process.join(max(0., wall_s-5-(monotonic()-start)))
    timed_out = process.is_alive()
    if timed_out:
        process.terminate()
        process.join(min(2., max(0.,wall_s-(monotonic()-start))))
    if process.is_alive():
        process.kill()
        process.join(max(0.,wall_s-(monotonic()-start)))
    alive = process.is_alive()
    return dict(timed_out=timed_out, child_reaped=not alive, exitcode=process.exitcode,
                wall_s=monotonic()-start, retry=False, actual_control_publish=False,
                simulator_started=False, actual_new_model_inference=False)


def fixture_child(output: str, envelope: dict) -> None:
    """No checkpoint/sensors; source-labelled artificial 20-point output."""
    import numpy as np
    from types import SimpleNamespace
    from aic_transfuser_lite.control.path_control_bridge import ShadowBridge, Limits, PathPose, Vehicle
    from aic_transfuser_lite.runtime.publisherless_shadow_v4 import ShadowSession
    class Adapter:
        commands = []
        def reset(self, reason): self.commands=[]
        def append(self, obs, cutoff): return 'ACCEPTED'
        def add_command(self, cmd): self.commands.append(cmd)
        def build(self, cutoff): return None, {'command_policy':'ARTIFICIAL_FIXTURE_ONLY'}
    p=Limits('ARTIFICIAL_TEST_ONLY',True,1.,-.2,.6,1.,.8,.5,1.,.3,
             .5,.1,.5,1.,.6,.8,1.,2.,.2,.01,.05,.1)
    limit=Envelope(**envelope)
    with (Path(output)/'trace.jsonl').open('x', encoding='utf8') as stream:
        used=0
        def emit(record):
            nonlocal used
            record['fixture_only']=True
            blob=json.dumps(record, allow_nan=False)+'\n';used+=len(blob.encode())
            if used>limit.log_bytes: raise RuntimeError('LOG_BUDGET')
            stream.write(blob);stream.flush()
        def infer(batch): return np.c_[np.linspace(.1,2.,20),np.zeros(20)].astype(np.float32)
        session=ShadowSession(limit,Adapter(),infer,ShadowBridge(p,fixture_mode=True),emit)
        for i in range(limit.forward_limit):
            if not session.active(): break
            obs=SimpleNamespace(sample_id=f'fixture{i}',camera=SimpleNamespace(epoch='0',clock_id='FIXTURE',header_ns=1_000_000_000+i*100_000_000))
            now=1.+i*.1
            tf=PathPose(obs.sample_id,now,'FIXTURE','0',(0.,0.,0.),'ARTIFICIAL_POSE')
            session.observation(obs,(SimpleNamespace(source='nominal'),),tf,finalized_ns=1,now_s=now)
            session.control_tick(now,'FIXTURE','0',Vehicle(str(i),now,'FIXTURE','0',(.2,0.,0.),0.,0.))
        emit(dict(event='FINISHED',forward_attempts=session.forward_calls,
                  actual_new_model_inference=False,control_publish_count=0,gear_publish_count=0,mode_publish_count=0))


def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture',action='store_true')
    parser.add_argument('--session-id',required=True)
    parser.add_argument('--wall-seconds',type=float,default=120.)
    parser.add_argument('--forward-limit',type=int,default=40)
    args=parser.parse_args()
    if not args.fixture:
        raise ValueError('LIVE_BLOCKED_PASSIVE_COMMAND_POSE_AND_ISOLATION_BINDINGS_REQUIRED')
    if not 5 < args.wall_seconds <=120: raise ValueError('OUTER_WALL_LIMIT')
    envelope=Envelope(args.session_id,args.wall_seconds-5,args.forward_limit,200,
                      time.time()+args.wall_seconds+1,4*1024**2)
    envelope.validate(time.time())
    args.output.mkdir(parents=True,exist_ok=False)
    process=mp.get_context('spawn').Process(target=fixture_child,args=(str(args.output),asdict(envelope)))
    result=supervise(process,wall_s=args.wall_seconds)
    result.update(fixture_only=True,envelope=asdict(envelope),
                  cumulative_live_budget_unchanged=True,live_approval_gate='PENDING_EXPLICIT_BINDINGS')
    (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result))
    return 0 if result['exitcode']==0 and result['child_reaped'] else 1


if __name__=='__main__': raise SystemExit(main())
