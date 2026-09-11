from dataclasses import replace
from pathlib import Path
import math
import numpy as np
import pytest
from aic_transfuser_lite.data.time_history_v1 import TimeEvent, select_time_history, materialize_history, read_time_events
from aic_transfuser_lite.data.time_teacher_v1 import build_time_teacher
from aic_transfuser_lite.data.mcap_converter_v2 import TimedPose, TimedVelocity, RunStreams
from aic_transfuser_lite.data.clock_segments import ClockEpoch
from aic_transfuser_lite.data.canonical_converter_v3 import _dense_future_state, _index_run_streams, load_dataset_v3_converter_config


def event(t, payload, role='pose', epoch='e', available=None, sequence=0):
    return TimeEvent(role,'r',epoch,'sim','receipt',t,t if available is None else available,sequence,payload)


def pose(t, x=None, frame='map', child='rear_axle'):
    return TimedPose(t,t*1e-9 if x is None else x,0.,0.,frame,child)


def teacher(events, anchor=None, **kwargs):
    return build_time_teacher(events, anchor or event(0,pose(0)),epoch_start_ns=0,
                              epoch_end_ns=kwargs.pop('end',3_000_000_000),**kwargs)


def test_b01_old_converter_rejects_endpoint_outside_epoch():
    cfg=load_dataset_v3_converter_config(Path('configs/data/dataset_v3.yaml'))
    epoch=ClockEpoch('e',0,3,0,310_000_000,0,310_000_000,None)
    results=[]
    for x in (0.32,99.):
        poses=(pose(0),pose(280_000_000),pose(320_000_000,x))
        stream=RunStreams((),(),poses,(TimedVelocity(300_000_000,1.,0.,0.),),(),(),(),(),{}, {})
        results.append(_dense_future_state(_index_run_streams(stream),observation=pose(0),epoch=epoch,config=cfg))
    assert not results[0].valid[2] and not results[1].valid[2]


@pytest.mark.parametrize('frame,child',[('other_map','rear_axle'),('map','other_body')])
def test_b02_old_and_new_reject_anchor_future_frame_change(frame,child):
    poses=tuple(pose(i*100_000_000,frame=frame,child=child) for i in range(1,31))
    labels=teacher([event(p.timestamp_ns,p) for p in poses])
    assert not labels.xy_mask.any()
    stream=RunStreams((),(),poses,tuple(TimedVelocity(p.timestamp_ns,1.,0.,0.) for p in poses),(),(),(),(),{}, {})
    cfg=load_dataset_v3_converter_config(Path('configs/data/dataset_v3.yaml'))
    epoch=ClockEpoch('e',0,30,0,3_000_000_000,0,3_000_000_000,None)
    assert not _dense_future_state(_index_run_streams(stream),observation=pose(0),epoch=epoch,config=cfg).valid.any()


def test_xy_survives_missing_velocity_and_commands_and_rotation_sign():
    obs=replace(pose(0),yaw_world_rad=math.pi/2)
    events=[event(i*100_000_000,pose(i*100_000_000)) for i in range(1,31)]
    labels=teacher(events,event(0,obs))
    assert labels.xy_mask.all() and not labels.velocity_mask.any()
    np.testing.assert_allclose(labels.xy_m[:,1],-np.arange(1,31)/10,atol=1e-6)
    with_velocity=teacher(events+[event(i*100_000_000,TimedVelocity(i*100_000_000,1.,0.,0.),'velocity') for i in range(1,31)],event(0,obs))
    np.testing.assert_array_equal(labels.xy_m,with_velocity.xy_m)
    assert with_velocity.velocity_mask.all()


def test_epoch_intervention_endpoints_stationary_and_interval_masks():
    events=[event(i*100_000_000,pose(i*100_000_000,0.)) for i in range(1,31)]
    assert teacher(events).xy_mask.all()
    assert not teacher(events,intervention_ns=3_000_000_000).xy_mask.any()
    # Target is pre-intervention but right interpolation endpoint is post-intervention.
    boundary=[event(2_960_000_000,pose(2_960_000_000)),event(3_040_000_000,pose(3_040_000_000))]
    labels=teacher(boundary,end=3_100_000_000,intervention_ns=3_020_000_000)
    assert not labels.xy_mask[-1]
    events[1]=replace(events[1],epoch='other')
    labels=teacher(events)
    assert labels.xy_mask[0] and not labels.xy_mask[1] and labels.xy_mask[2]
    assert not labels.interval_mask[1:3].any()


def test_d02_late_duplicate_cannot_change_past_history_tensor():
    early=event(1_000_000_000,np.array([1.]),'camera',available=1_010_000_000)
    late=replace(early,payload=np.array([99.]),available_ns=2_000_000_000,sequence=1)
    def selected(events):
        return select_time_history(events,role='camera',run='r',epoch='e',capture_clock='sim',
            available_clock='receipt',freeze_ns=1_500_000_000,observation_ns=1_000_000_000,length=4,tolerance_ns=0)
    offline=materialize_history(selected([early,late]),lambda p:p,shape=(1,))
    online=materialize_history(selected([early]),lambda p:p,shape=(1,))
    for a,b in zip(offline,online):np.testing.assert_array_equal(a,b)
    assert offline[1].tolist()==[False,False,False,True]
    assert selected([replace(early,available_clock='unrelated')])[-1] is None


def test_event_reader_preserves_duplicates_before_legacy_dedup(monkeypatch):
    import aic_transfuser_lite.data.time_history_v1 as module
    first=replace(pose(10),bag_timestamp_ns=100)
    second=replace(first,x_world_m=99.,bag_timestamp_ns=200)
    def read(bag, *, event_sink, optional_roles):
        assert {"velocity", "nominal_command", "final_command"} <= optional_roles
        for i,p in enumerate((first,second)):event_sink('pose',p,i)
    monkeypatch.setattr(module,'read_run_messages_v2',read)
    epoch=ClockEpoch('e',0,2,0,300,0,100,None)
    values=read_time_events(Path('unused'),run='r',epochs=[epoch],capture_clock='sim')
    assert len(values)==2 and values[0].sequence!=values[1].sequence
    assert values[0].availability_source=='bag_receipt_proxy'


def test_raw_reader_sink_sees_both_records_before_last_wins(tmp_path,monkeypatch):
    from types import SimpleNamespace as NS
    import rosbags.highlevel
    from aic_transfuser_lite.data.mcap_converter_v2 import read_run_messages_v2,DATASET_V2_TOPICS
    (tmp_path/'metadata.yaml').write_text('synthetic')
    connections=[NS(topic=c.name,msgtype=c.message_type) for c in DATASET_V2_TOPICS if c.role in {'camera','lidar','pose'}]
    pose_topic=next(c.name for c in DATASET_V2_TOPICS if c.role=='pose')
    connection=next(c for c in connections if c.topic==pose_topic)
    def message(x):
        return NS(header=NS(stamp=NS(sec=1,nanosec=0),frame_id='map'),child_frame_id='rear_axle',
            pose=NS(pose=NS(position=NS(x=x,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
    class Reader:
        def __init__(self,*args):self.connections=connections
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def messages(self,**kwargs):return iter([(connection,1_010_000_000,message(1.)),(connection,2_000_000_000,message(99.))])
        def deserialize(self,data,kind):return data
    monkeypatch.setattr(rosbags.highlevel,'AnyReader',Reader)
    seen=[]
    decoded=read_run_messages_v2(tmp_path,event_sink=lambda *args:seen.append(args),
        optional_roles=frozenset({'velocity','nominal_command','final_command','gear','actual_steering'}))
    assert len(seen)==2 and len(decoded.poses)==1
    assert seen[0][1].x_world_m==1. and seen[1][1].x_world_m==99.
    assert seen[0][2]<seen[1][2]
