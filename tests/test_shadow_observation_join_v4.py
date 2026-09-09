from types import SimpleNamespace as O
from dataclasses import replace
import numpy as np
import pytest
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
    # Select the latest supported image; do not forward the obsolete one later.
    join.add('camera',Sample(1_001_000_000,11,np.zeros((256,384,3),np.uint8),'camera_optical_link','0'))
    join.tick(30,1.05)
    assert len(calls)==1 and not join.pending
    assert calls[0][0][0].camera.header_ns==1_001_000_000
    assert any(e.get('reason')=='SUPERSEDED_BY_READY' for e in events)
    join.tick(300_000_011,1.35)
    assert len(calls)==1 and not join.pending
    assert events[-1]['event']=='JOIN_READY'


def test_newest_incomplete_does_not_starve_ready_camera():
    join,calls,events=setup();feed(join)
    join.add('camera',Sample(1_100_000_000,11,np.zeros((256,384,3),np.uint8),'camera_optical_link','0'))
    join.tick(30,1.12)
    assert len(calls)==1 and calls[0][0][0].camera.header_ns==1_000_000_000
    assert len(join.pending)==1 and join.pending[0].ns==1_100_000_000
    join.tick(31,1.12)
    assert len(calls)==1  # sensor/tick updates never repeat the same image


def test_off_grid_is_terminal_without_deadline_wait():
    join,calls,events=setup();feed(join)
    join.add('camera',Sample(1_050_000_000,11,np.zeros((256,384,3),np.uint8),'camera_optical_link','0'))
    join.tick(30,1.1)
    assert len(calls)==1 and not join.pending
    assert any(e.get('reason')=='CAMERA_GRID_TOLERANCE' and e['event']=='JOIN_REJECTED' for e in events)
    assert calls[0][0][0].grid_ns==1_000_000_000


@pytest.mark.parametrize('role',['lidar','velocity','steering','pose'])
def test_identical_timestamp_retains_original_evidence(role):
    join,calls,events=setup();feed(join)
    old=join.streams[role][0];count=len(join.streams[role])
    join.add(role,replace(old,received_ns=25))
    assert len(join.streams[role])==count and join.streams[role][0].received_ns==20
    join.tick(30,1.05)
    assert len(calls)==1
    assert any(e['event']=='JOIN_DUPLICATE' and e['role']==role for e in events)


def test_conflicting_timestamp_latches_until_explicit_reset():
    join,calls,events=setup();feed(join)
    old=join.streams['velocity'][0]
    with pytest.raises(ValueError,match='CONFLICTING_TIMESTAMP:velocity'):
        join.add('velocity',replace(old,value=[99.,0.,0.]))
    join.tick(30,1.05)
    assert not calls and join.fault
    join.reset('CLOCK_RESET','0');feed(join);join.tick(30,1.05)
    assert len(calls)==1


def test_empty_lidar_waits_and_later_arrival_triggers_once():
    join,calls,events=setup();feed(join)
    scan=join.streams['lidar'].pop()
    join.tick(30,1.05)
    assert not calls and len(join.pending)==1 and events[-1]['reason']=='LIDAR_stream_empty'
    join.add('lidar',replace(scan,received_ns=31));join.tick(32,1.05)
    join.tick(33,1.05)
    assert len(calls)==1


def test_newest_future_receipt_is_not_selected():
    join,calls,events=setup();feed(join)
    join.add('camera',Sample(1_001_000_000,40,np.zeros((256,384,3),np.uint8),'camera_optical_link','0'))
    join.tick(30,1.05)
    assert len(calls)==1 and calls[0][0][0].camera.header_ns==1_000_000_000
    assert len(join.pending)==1


def test_105ms_camera_keeps_100ms_contract_and_rejects_only_off_grid():
    join,calls,events=setup();feed(join);join.tick(30,1.0)
    scan=join.streams['lidar'][0].value
    for i in range(1,21):
        ns=1_000_000_000+i*105_000_000
        receipt=100+i*100
        join.add('camera',Sample(ns,receipt,np.zeros((256,384,3),np.uint8),'camera_optical_link','0'))
        join.add('lidar',Sample(ns,receipt,scan,'lidar','0'))
        for role,value,frame in [('velocity',[1.,0.,0.],'base_link'),
                                 ('steering',[0.],'steering_tire_angle'),('pose',[0.,0.,0.],'odom')]:
            join.add(role,Sample(ns,receipt,value,frame,'0'))
        before=len(calls);join.tick(receipt+1,ns*1e-9)
        distance=min((i*5)%100,100-(i*5)%100)
        assert len(calls)==before+(distance<=40)
        assert not join.pending
    assert len(calls)==18  # indices 9, 10, 11 are outside the unchanged 40ms tolerance
    for args,_ in calls:
        obs=args[0]
        assert (obs.grid_ns-1_000_000_000)%100_000_000==0
        assert abs(obs.camera.header_ns-obs.grid_ns)<=40_000_000
