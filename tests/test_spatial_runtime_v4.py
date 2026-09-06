"""Synthetic only. No checkpoint, ROS initialization, Dataset, training or optimizer."""
from copy import deepcopy
from dataclasses import replace
import importlib.util
import io
import json
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
