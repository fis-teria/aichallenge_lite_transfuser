"""Synthetic only. No checkpoint, ROS initialization, Dataset, training or optimizer."""
from copy import deepcopy
from dataclasses import replace
import importlib.util
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import torch

from aic_transfuser_lite.data.spatial_diagnostic_inputs_v4 import build_inputs, INPUT_FIELDS
from aic_transfuser_lite.runtime.spatial_input_v4 import SpatialInputV4, GridObservation, PassiveCommand, Stamp, freeze_batch
from aic_transfuser_lite.runtime.spatial_recording_v4 import Records, PrivateWriter, geometry, encoded, sha
from aic_transfuser_lite.runtime.spatial_runtime_v4 import SpatialRuntimeV4

ROOT=Path(__file__).resolve().parents[1]


def frame(i: int, **kwargs) -> GridObservation:
    ns=1_000_000_000+i*100_000_000
    s=Stamp(ns,ns,ns)
    return GridObservation(ns,s,s,s,np.full((8,12,3),i,dtype=np.uint8),np.full(750,5,dtype=np.float32),(1.,0.,0.,0.),sample_id=str(i),**kwargs)


def batch():
    a=SpatialInputV4(command_binding_known=True)
    a.append(frame(0),2_000_000_000)
    return a.build(2_000_000_000)[0]


class Fake(torch.nn.Module):
    def __init__(self, mode='normal'):
        super().__init__()
        self.mode=mode
        self.calls=0
        self.register_buffer('fixed',torch.tensor(1.))

    def forward(self,b):
        self.calls+=1
        assert b.targets is None
        if self.mode=='exception':
            raise RuntimeError('fake forward exception')
        if self.mode=='scalar':
            return torch.tensor(1.)
        xy=torch.zeros(1,20,2)
        xy[0,:,0]=torch.arange(1,21)*.1
        if self.mode=='dtype':
            return xy.double()
        if self.mode=='nan':
            xy[0,1,0]=float('nan')
            xy[0,0,1]=-0.0
        return xy


def records():
    return Records(ROOT/'schemas',mode='SYNTHETIC')


def test_input_offline_parity():
    adapter=SpatialInputV4(command_binding_known=True)
    rows,assets=[],{}
    for i in range(12):
        f=frame(i)
        adapter.add_command(PassiveCommand(f.camera,.1,2.,.2))
        assert adapter.append(f,f.camera.available_ns)=='ACCEPTED'
        image=io.BytesIO(); Image.fromarray(f.rgb).save(image,format='PNG')
        assets[f'i{i}']=image.getvalue()
        for key,value in ((f'l{i}',f.ranges_m),(f'v{i}',np.ones(750,dtype=bool))):
            buf=io.BytesIO(); np.save(buf,value); assets[key]=buf.getvalue()
        rows.append(dict(sample_id=str(i),run_id='synthetic',segment_id='0',grid_stamp_ns=str(f.grid_ns),
            image_path=f'i{i}',lidar_path=f'l{i}',lidar_valid_path=f'v{i}',camera_delta_ms='0',lidar_delta_ms='0',
            velocity_longitudinal_mps='1',velocity_lateral_mps='0',yaw_rate_rps='0',actual_steering_rad='0',actual_steering_valid='true',
            nominal_command=json.dumps(dict(valid=True,steering_rad=.1,speed_mps=2.,acceleration_mps2=.2)),final_command='{}'))
        expected,_=build_inputs(rows,i,assets.__getitem__)
        actual,_=adapter.build(f.camera.available_ns)
        for name in INPUT_FIELDS:
            assert torch.equal(getattr(expected,name),getattr(actual,name)),name


def test_padding_binding_and_late_command():
    a=SpatialInputV4()
    a.append(frame(0),2_000_000_000)
    with pytest.raises(ValueError,match='BINDING'):
        a.build(2_000_000_000)
    a.command_binding_known=True
    first,_=a.build(2_000_000_000)
    assert first.image_mask.tolist()==[[False,False,False,True]]
    assert not first.command_mask.any()
    late=Stamp(1_000_000_000,3_000_000_000,3_000_000_000)
    a.add_command(PassiveCommand(late,0.,1.,0.))
    a.append(frame(1),2_000_000_000)
    with pytest.raises(ValueError,match='MISSING'):
        a.build(2_000_000_000)


@pytest.mark.parametrize('source,verified,valid,expected',[('nominal',False,True,True),('final_fallback',False,True,False),('final_fallback',True,True,True),('nominal',False,False,False)])
def test_command_priority(source,verified,valid,expected):
    a=SpatialInputV4(command_binding_known=True,final_fallback_verified=verified)
    a.append(frame(0),2_000_000_000)
    a.add_command(PassiveCommand(frame(0).camera,.1,2.,0.,source,valid))
    a.append(frame(1),2_000_000_000)
    if expected:
        result,_=a.build(2_000_000_000)
        assert result.command_mask[0,-1]
    else:
        with pytest.raises(ValueError): a.build(2_000_000_000)


def test_reset_duplicate_and_snapshot_isolation():
    a=SpatialInputV4(command_binding_known=True)
    assert a.append(frame(0),2_000_000_000)=='ACCEPTED'
    assert a.append(frame(0),2_000_000_000)=='DUPLICATE'
    b,_=a.build(2_000_000_000)
    copied=freeze_batch(b)
    b.image.zero_()
    assert not torch.equal(copied.image,b.image)
    a.add_command(PassiveCommand(frame(0).camera,0.,1.,0.))
    a.append(frame(5),2_000_000_000)
    assert a.last_reason=='GAP_RESET' and not a.commands
    a.append(frame(1),2_000_000_000)
    assert a.last_reason=='CLOCK_REGRESSION'


@pytest.mark.parametrize('mode,status',[('normal','SHAPE_FINITE_ONLY'),('nan','NONFINITE'),('scalar','OUTPUT_CONTRACT_ERROR'),('dtype','OUTPUT_CONTRACT_ERROR'),('exception','FORWARD_EXCEPTION')])
def test_states(mode,status):
    r=records(); model=Fake(mode); core=SpatialRuntimeV4(model,r)
    before=model.fixed.clone()
    event=core.infer(batch())
    r.validate(event)
    assert event['payload']['output']['status']==status
    assert model.calls==core.forward_calls==1
    assert torch.equal(before,model.fixed)
    if mode=='scalar': assert event['payload']['output']['actual_shape']==[]
    if mode=='nan': assert bytes.fromhex(event['payload']['output']['float32_le_hex'])[4:8]==bytes.fromhex('00000080')


def test_snapshot_failure_and_budget(monkeypatch):
    r=records(); core=SpatialRuntimeV4(Fake(),r,forward_limit=1)
    monkeypatch.setattr(core,'snapshot',lambda _: (_ for _ in ()).throw(RuntimeError('copy failed')))
    event=core.infer(batch()); r.validate(event)
    assert event['payload']['output']['status']=='SNAPSHOT_ERROR'
    second=core.infer(batch()); r.validate(second)
    assert second['payload']['output']['status']=='NOT_INFERRED'
    assert core.forward_calls==1


def test_geometry_and_semantic_rejections():
    xy=np.zeros((20,2),dtype=np.float32); xy[:,0]=np.arange(20)
    xy[1,0]=np.nan
    g=geometry(xy)
    assert g['chords'][0]['status']=='UNKNOWN_NONFINITE' and g['chords'][0]['length_m'] is None
    assert all(v is None for v in g['cumulative_polyline_m'][1:]) and g['spacing_m'][4]==1
    assert geometry(np.zeros((20,2),dtype=np.float32))['chords'][0]['status']=='UNKNOWN_DEGENERATE'
    r=records(); e=SpatialRuntimeV4(Fake(),r).infer(batch())
    broken=deepcopy(e); broken['payload']['output']['model_xy_m'][0][0]=999
    with pytest.raises(ValueError): r.validate(broken)
    broken=deepcopy(e); broken['payload']['history']['input_descriptors'].reverse()
    with pytest.raises(ValueError): r.validate(broken)
    broken=deepcopy(e); broken['payload']['timing']['inference_duration']['ns']='-1'
    with pytest.raises(ValueError): r.validate(broken)


def test_writer_receipt_and_fail_closed(tmp_path):
    r=records(); writer=PrivateWriter(tmp_path/'events.jsonl',r)
    core=SpatialRuntimeV4(Fake(),r,writer)
    core.infer(batch()); writer.close()
    lines=(tmp_path/'events.jsonl').read_bytes().splitlines()
    assert len(lines)==2
    receipt=json.loads(lines[1])
    assert receipt['payload']['commit_receipt']['original_bytes_hash']==sha(lines[0])
    r.validate(receipt)
    writer=PrivateWriter(tmp_path/'fail.jsonl',r,max_file_bytes=1)
    core=SpatialRuntimeV4(Fake(),r,writer)
    core.infer(batch())
    assert writer.failed
    core.infer(batch())
    assert core.forward_calls==1
    writer.close()


def test_queue_and_limits(tmp_path):
    r=records(); w=PrivateWriter(tmp_path/'q.jsonl',r,capacity=1)
    e=r.accept(); e['payload'].update(event_type='DROP',status='DROPPED')
    assert w.enqueue(e)
    assert not w.enqueue(r.accept()) and w.failed
    w.close()
    w=PrivateWriter(tmp_path/'small.jsonl',r,max_record_bytes=1)
    assert not w.enqueue(r.accept())
    w.close()
    r.validate(r.event('SESSION_END'))


def test_transport_record_and_no_teacher():
    a=SpatialInputV4(command_binding_known=True)
    a.append(frame(0),2_000_000_000)
    b,p=a.build(2_000_000_000)
    e=SpatialRuntimeV4(Fake(),records()).infer(b,p)
    assert e['payload']['output']['status']=='SHAPE_FINITE_ONLY'
    b=replace(b,targets=object())
    second=SpatialRuntimeV4(Fake(),records()).infer(b)
    assert e['payload']['output']['float32_le_hex']==second['payload']['output']['float32_le_hex']


def wrapper_module():
    path=ROOT/'ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime/spatial_path_shadow_node_v4.py'
    spec=importlib.util.spec_from_file_location('isolated_v4_wrapper',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def test_mock_wrapper_forbidden_endpoints():
    class Node:
        def __init__(self): self.subs=[]
        def create_subscription(self,*args): self.subs.append(args)
        def __getattr__(self,key): raise AssertionError('forbidden endpoint '+key)
    node=Node(); module=wrapper_module()
    roles=('image','lidar','velocity','steering','nominal')
    types={k:object for k in roles}; types['sensor_qos']='synthetic_best_effort'
    module.SpatialPathShadowWrapperV4(node,SpatialRuntimeV4(Fake(),records()),SpatialInputV4(command_binding_known=True),types,{k:'/fake/'+k for k in roles})
    assert len(node.subs)==5
    with pytest.raises(RuntimeError,match='LIVE_BOOTSTRAP_BLOCKED'): module.main()


def test_launch_and_static_nonactuation():
    import ast
    root=ROOT/'ros2_ws/src/aic_e2e_runtime'
    launch=(root/'launch/spatial_path_shadow_v4.launch.py').read_text()
    assert "default_value='false'" in launch and 'IfCondition' in launch and 'IncludeLaunchDescription' not in launch
    source=(root/'aic_e2e_runtime/spatial_path_shadow_node_v4.py').read_text()
    tree=ast.parse(source)
    banned={'create_publisher','create_client','create_service','set_parameters','publish','call_async'}
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in banned for n in ast.walk(tree))
    assert 'inference_node_v3' not in source


def test_full_draft2020_schema():
    module=pytest.importorskip('jsonschema',reason='existing environment lacks jsonschema; no dependency installation authorized')
    if not hasattr(module,'Draft202012Validator'):
        pytest.skip('existing validator lacks draft2020-12')
    r=records()
    for mode in ('normal','nan','scalar','dtype','exception'):
        r.validate_schema(SpatialRuntimeV4(Fake(mode),r).infer(batch()))
    for kind in ('LOGGER_HEALTH','SESSION_END','RESET'):
        r.validate_schema(r.event(kind))


def test_range_validity_is_sensor_geometry_not_normalization_limit():
    a=SpatialInputV4(command_binding_known=True)
    f=replace(frame(0),ranges_m=np.full(750,30,dtype=np.float32),range_max_m=40.)
    a.append(f,2_000_000_000)
    b,_=a.build(2_000_000_000)
    assert torch.all(b.lidar[0,-1]==1)  # clipped range 1, but valid channel still 1


def test_future_clock_mismatch_and_nominal_precedence():
    a=SpatialInputV4(command_binding_known=True,final_fallback_verified=True)
    a.append(frame(0),2_000_000_000)
    s=frame(0).camera
    a.add_command(PassiveCommand(s,.1,1.,0.))
    a.add_command(PassiveCommand(s,.2,2.,0.,source='final_fallback'))
    assert a.add_command(PassiveCommand(replace(s,epoch='other'),.3,3.,0.))=='COMMAND_EPOCH_REJECTED'
    a.append(frame(1),2_000_000_000)
    b,p=a.build(2_000_000_000)
    assert b.command_history[0,-1,1]==1 and p['commands'][-1].source=='nominal'


def test_wrapper_fake_stream_reaches_same_forward_once():
    from dataclasses import replace
    r=records(); core=SpatialRuntimeV4(Fake(),r)
    class Node:
        def create_subscription(self,*args): pass
        def __getattr__(self,name): raise AssertionError(name)
    roles=('image','lidar','velocity','steering','nominal')
    types={k:object for k in roles}; types['sensor_qos']=object()
    ticks=iter(range(2_000_000_000,2_000_000_200))
    w=wrapper_module().SpatialPathShadowWrapperV4(Node(),core,SpatialInputV4(command_binding_known=True),types,{k:'/fake/'+k for k in roles},clock=lambda:next(ticks))
    stamp=SimpleNamespace(sec=1,nanosec=0)
    header=SimpleNamespace(stamp=stamp)
    w.receive('velocity',SimpleNamespace(header=header,longitudinal_velocity=1.,lateral_velocity=0.,heading_rate=0.))
    w.receive('steering',SimpleNamespace(header=header,steering_tire_angle=0.))
    w.receive('lidar',SimpleNamespace(header=header,ranges=[5.]*750,range_min=0.,range_max=25.))
    w.receive('image',SimpleNamespace(header=header,encoding='rgb8',height=8,width=12,step=36,data=bytes(8*12*3)))
    assert core.forward_calls==1 and w.results[-1]['payload']['output']['status']=='SHAPE_FINITE_ONLY'


class ManualClock:
    def __init__(self, ns=1_000_000): self.ns=ns
    def __call__(self): return self.ns
    def advance(self, ns): self.ns+=ns


def candidate_harness(tmp_path, *, capacity=16, wait=100):
    clock=ManualClock()
    r=Records(ROOT/'schemas',mode='SYNTHETIC',clock=clock)
    writer=PrivateWriter(tmp_path/'candidate_events.jsonl',r)
    core=SpatialRuntimeV4(Fake(),r,writer)
    class Node:
        def create_subscription(self,*args): pass
        def __getattr__(self,name): raise AssertionError('forbidden endpoint '+name)
    roles=('image','lidar','velocity','steering','nominal')
    types={k:object for k in roles}; types['sensor_qos']=object()
    w=wrapper_module().SpatialPathShadowWrapperV4(Node(),core,SpatialInputV4(command_binding_known=True),types,
        {k:'/fake/'+k for k in roles},max_sync_wait_ns=wait,candidate_capacity=capacity)
    return clock,r,writer,core,w


def fake_message(role, ns=1_000_000_000):
    value=SimpleNamespace(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
    fields=dict(header=SimpleNamespace(stamp=value))
    if role=='image': fields.update(encoding='rgb8',height=8,width=12,step=36,data=bytes(8*12*3))
    if role=='lidar': fields.update(ranges=[5.]*750,range_min=0.,range_max=25.)
    if role=='velocity': fields.update(longitudinal_velocity=1.,lateral_velocity=0.,heading_rate=0.)
    if role=='steering': fields.update(steering_tire_angle=0.)
    return SimpleNamespace(**fields)


def sources(w, ns):
    for role in ('velocity','steering','lidar'): w.receive(role,fake_message(role,ns))


def trace_evidence(name, contexts, extra=None):
    """Write only fake events from this test run, to an explicitly requested new dir."""
    folder=os.environ.get('V4_TRACE_DIR')
    if folder:
        root=Path(folder); root.mkdir(parents=True,exist_ok=True)
        value=dict(mode='SYNTHETIC',fixed_checkpoint_reads=0,fixed_checkpoint_forward_calls=0,
                   traces=[c.trace() for c in contexts],events=[c.event for c in contexts],extra=extra)
        with (root/(name+'.json')).open('xb') as stream: stream.write(encoded(value)+b'\n')


def test_deadline_releases_m1_on_tick_and_late_sources_do_not_revive(tmp_path):
    clock,r,writer,core,w=candidate_harness(tmp_path)
    w.receive('image',fake_message('image',1_000_000_000))  # M0: no ego
    clock.advance(50)
    sources(w,1_100_000_000)
    w.receive('image',fake_message('image',1_100_000_000))  # M1: complete, behind M0
    assert core.forward_calls==0 and len(r.counts['accepted'])==2
    clock.advance(50); w.tick()  # No callback needed.
    assert [c.event['payload']['reason'] for c in w.completed][0]=='SYNC_DEADLINE'
    assert core.forward_calls==1 and len(w.pending)==0 and len(w.completed)==2
    sources(w,1_000_000_000); w.tick()
    assert core.forward_calls==1 and len(r.counts['accepted'])==2
    assert [c.event['execution']['forward_calls'] for c in w.completed]==[0,1]
    trace_evidence('deadline_m0_m1',list(w.completed))
    writer.close()


def test_missing_duplicate_overflow_reset_each_camera_terminal_once(tmp_path):
    clock,r,writer,core,w=candidate_harness(tmp_path,capacity=1)
    w.receive('image',SimpleNamespace())
    w.receive('image',fake_message('image',2_000_000_000))
    w.receive('image',fake_message('image',2_000_000_000))
    w.receive('image',fake_message('image',2_100_000_000))
    w.receive('image',fake_message('image',1_000_000_000))
    clock.advance(100); w.tick()
    assert r.sequence==5 and len(w.completed)==5 and core.forward_calls==0
    reasons=[c.event['payload']['reason'] for c in w.completed]
    assert any('MISSING_HEADER' in v for v in reasons)
    assert 'DUPLICATE_IMAGE' in reasons and 'CAMERA_QUEUE_OVERFLOW' in reasons and 'CAMERA_CLOCK_RESET' in reasons
    assert len({c.event['payload']['record_id'] for c in w.completed})==5
    assert w.completed[0].event['payload']['output']['t_obs']['status']=='MISSING'
    trace_evidence('rejections_reset',list(w.completed))
    writer.close()


def test_real_finalize_after_preprocess_and_copy(tmp_path,monkeypatch):
    clock,r,writer,core,w=candidate_harness(tmp_path)
    original=w.adapter.append
    def slow_append(*args):
        result=original(*args); clock.advance(20); return result
    monkeypatch.setattr(w.adapter,'append',slow_append)
    original_forward=core.model.forward
    def slow_forward(b):
        clock.advance(10); return original_forward(b)
    monkeypatch.setattr(core.model,'forward',slow_forward)
    original_snapshot=core.snapshot
    def slow_snapshot(t):
        clock.advance(10); return original_snapshot(t)
    monkeypatch.setattr(core,'snapshot',slow_snapshot)
    sources(w,1_000_000_000); w.receive('image',fake_message('image'))
    c=w.completed[0]; t=c.event['payload']['timing']
    assert int(c.selection_cutoff['ns'])<int(t['input_finalized']['ns'])<=int(t['inference_start']['ns'])<int(t['inference_api_return']['ns'])<int(t['snapshot_ready']['ns'])
    assert int(t['candidate_received']['ns'])<=int(c.selection_cutoff['ns'])
    assert int(t['event_observed']['ns'])>=int(t['snapshot_ready']['ns'])
    assert t['end_to_end_age']['status']=='KNOWN'
    trace_evidence('cutoff_finalize_copy',[c])
    writer.close()


@pytest.mark.parametrize('different', ['clock','epoch'])
def test_different_clock_or_epoch_duration_unknown(different):
    clock=ManualClock(); r=Records(ROOT/'schemas',mode='SYNTHETIC',clock=clock)
    core=SpatialRuntimeV4(Fake(),r)
    received=r.now()
    received['clock_id' if different=='clock' else 'epoch_id']='other'
    c=core.accept_candidate(received)
    core.infer(batch(),candidate=c)
    assert c.event['payload']['timing']['end_to_end_age']['status']=='UNKNOWN'
    assert c.event['payload']['timing']['candidate_received']==received


def test_shape_and_budget_drop_are_persisted_with_same_context(tmp_path):
    r=records(); writer=PrivateWriter(tmp_path/'drops.jsonl',r)
    core=SpatialRuntimeV4(Fake(),r,writer,forward_limit=0)
    c=core.accept_candidate(); core.infer(batch(),candidate=c); core.infer(batch(),candidate=c)
    assert r.sequence==1 and core.forward_calls==0 and c.terminal
    core.forward_limit=1
    bad=replace(batch(),image=torch.zeros(1))
    d=core.accept_candidate(); core.infer(bad,candidate=d)
    writer.close()
    lines=[json.loads(x) for x in (tmp_path/'drops.jsonl').read_text().splitlines()]
    assert [e['payload']['event_type'] for e in lines]==['DROP','WRITER_RECEIPT','DROP','WRITER_RECEIPT']
    trace_evidence('shape_budget_drop',[c,d])


def test_callback_errors_and_recording_error_do_not_reaccept(tmp_path,monkeypatch):
    clock,r,writer,core,w=candidate_harness(tmp_path)
    w.receive('image',SimpleNamespace(header=SimpleNamespace()))  # AttributeError
    w.receive('nominal',SimpleNamespace(header=fake_message('image').header))  # noncamera rejection
    assert r.sequence==1 and w.sensor_received['nominal']==1
    sources(w,1_000_000_000)
    bad=fake_message('image'); bad.data=None
    w.receive('image',bad)  # TypeError, same camera context
    assert r.sequence==2 and core.forward_calls==0
    sources(w,1_100_000_000)
    monkeypatch.setattr(r,'validate',lambda _: (_ for _ in ()).throw(TypeError('synthetic serializer failure')))
    w.receive('image',fake_message('image',1_100_000_000))
    c=w.completed[-1]
    assert r.sequence==3 and core.forward_calls==1 and c.terminal
    assert c.event['payload']['output']['status']=='SHAPE_FINITE_ONLY'
    assert c.event['execution']['forward_calls']==1 and c.persistence['error'] is not None
    core.infer(batch(),candidate=c)
    assert core.forward_calls==1 and r.sequence==3
    trace_evidence('callback_and_recording_errors',list(w.completed))
    writer.close()


def test_closed_writer_stops_new_forward_and_close_is_idempotent(tmp_path):
    r=records(); w=PrivateWriter(tmp_path/'closed.jsonl',r); core=SpatialRuntimeV4(Fake(),r,w)
    first=core.accept_candidate(); core.infer(batch(),candidate=first)
    w.close(); w.close()
    second=core.accept_candidate(); core.infer(batch(),candidate=second)
    assert w.state=='CLOSED' and core.forward_calls==1
    assert second.event['payload']['output']['status']=='NOT_INFERRED'
    assert second.persistence['error']=='LOGGER_CLOSED'
    trace_evidence('logger_closed',[first,second])


def test_receipt_failure_preserves_known_data_and_first_error(tmp_path,monkeypatch):
    r=records(); w=PrivateWriter(tmp_path/'receipt_fail.jsonl',r); core=SpatialRuntimeV4(Fake(),r,w)
    original=w._write; n=0
    def fail_receipt(blob):
        nonlocal n
        n+=1
        if n==2: raise OSError('RECEIPT_FAIL_FIRST')
        original(blob)
    monkeypatch.setattr(w,'_write',fail_receipt)
    c=core.accept_candidate(); core.infer(batch(),candidate=c)
    assert c.persistence['data_write']=='KNOWN_WRITE_COMPLETED_NOT_DURABLE' and c.persistence['receipt_write']=='UNKNOWN'
    assert core.forward_calls==1 and c.event['payload']['output']['status']=='SHAPE_FINITE_ONLY'
    w.fail('SECOND'); assert 'RECEIPT_FAIL_FIRST' in w.error
    d=core.accept_candidate(); core.infer(batch(),candidate=d)
    assert core.forward_calls==1 and w.state=='FAILED'
    trace_evidence('receipt_failure',[c,d])
    w.close(); w.close()


def test_missing_command_and_invalid_ego_are_not_padding():
    a=SpatialInputV4(command_binding_known=True)
    for i in range(3):
        f=frame(i)
        if i==0: a.add_command(PassiveCommand(f.camera,.1,1.,0.))
        if i==1: f=replace(f,ego_si=(float('nan'),)*4,steering_valid=False)
        a.append(f,2_000_000_000)
    b,p=a.build(2_000_000_000)
    r=records(); core=SpatialRuntimeV4(Fake(),r); c=core.accept_candidate(); core.infer(b,p,candidate=c)
    h=c.event['payload']['history']
    assert h['command'][0]['padding'] and h['command'][0]['reason']=='WARM_UP_PADDING'
    assert not h['command'][-1]['padding'] and not h['command'][-1]['mask_used']
    assert h['command'][-1]['sample_id']=='1' and h['command'][-1]['source']=='COMMAND_MISSING_OR_INVALID'
    assert not h['ego'][-2]['padding'] and h['ego'][-2]['feature_mask_used']==[False]*4
    assert not b.command_mask[0,-1] and not b.ego_feature_mask[0,-2].any()
    trace_evidence('missing_not_padding',[c])


def test_tensor_only_padding_stays_unknown():
    e=SpatialRuntimeV4(Fake(),records()).infer(batch())
    assert 'PADDING_UNKNOWN' in e['payload']['history']['command'][0]['reason']
