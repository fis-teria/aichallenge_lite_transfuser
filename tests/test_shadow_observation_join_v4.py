from types import SimpleNamespace as O
import numpy as np
from aic_transfuser_lite.runtime.shadow_observation_join_v4 import ShadowObservationJoin
from aic_transfuser_lite.runtime.spatial_sim_adapter_v4 import Sample


def setup():
    calls=[];events=[]
    session=O(adapter=O(reset=lambda r:None),bridge=O(_invalidate=lambda r:None),
              observation=lambda *a,**kw:calls.append((a,kw)))
    join=ShadowObservationJoin(session,lambda:(),events.append,clock_id='test',monotonic_id='process',
                               pose_frame='odom',pose_evidence='SYNTHETIC_ONLY')
    return join,calls,events


def feed(join,pose=True):
    ns=1_000_000_000
    join.add('camera',Sample(ns,10,np.zeros((256,384,3),np.uint8),'camera_optical_link','0'))
    scan=dict(ranges=np.ones(750),angle_min=-1.5666074752807617,angle_increment=.004188789986073971,range_min=0.,range_max=25.)
    join.add('lidar',Sample(ns+10_000_000,20,scan,'lidar','0'))
    for t,x in ((ns-20_000_000,0.),(ns+20_000_000,2.)):
        join.add('velocity',Sample(t,20,[x,0.,0.],'base_link','0'))
        join.add('steering',Sample(t,20,[0.],'steering_tire_angle','0'))
        if pose: join.add('pose',Sample(t,20,[x,0.,0.],'odom','0'))


def test_observation_time_interpolation_and_single_submission():
    join,calls,events=setup();feed(join);join.tick(30,1.05);join.tick(31,1.05)
    assert len(calls)==1
    obs,commands,pose=calls[0][0]
    assert obs.ego_si[0]==1. and pose.base_in_local==(1.,0.,0.)
    assert pose.stamp_s==1. and obs.ego_stamp.header_ns==1_000_000_000
    assert obs.lidar.header_ns==1_010_000_000  # not falsely exact scan alignment
    assert obs.camera.clock_id=='test' and events[-1]['event']=='JOIN_READY'


def test_missing_pose_expires_without_forward():
    join,calls,events=setup();feed(join,False);join.tick(30,1.05)
    assert not calls and events[-1]['event']=='JOIN_WAIT'
    join.tick(300_000_010,1.3)
    assert not calls and not join.pending and events[-1]['reason']=='JOIN_DEADLINE'


def test_future_receipt_not_used_and_reset_clears():
    join,calls,events=setup();feed(join);join.tick(15,1.05)
    assert not calls
    join.reset('CLOCK_RESET','1');join.tick(30,1.05)
    assert not calls and all(not q for q in join.streams.values())


def test_slow_forward_does_not_reuse_cutoff_for_another_candidate():
    join,calls,events=setup();feed(join)
    # A second synthetic pending image would be geometrically supported by the
    # same bracket, but needs a new worker tick after the first forward.
    join.add('camera',Sample(1_001_000_000,11,np.zeros((256,384,3),np.uint8),'camera_optical_link','0'))
    join.tick(30,1.05)
    assert len(calls)==1 and len(join.pending)==1
    join.tick(300_000_011,1.35)
    assert len(calls)==1 and not join.pending
    assert events[-1]['reason']=='JOIN_DEADLINE'
