"""Bootstrap integration uses ONLY fake ROS/loader and synthetic temporary records."""
from copy import deepcopy
import builtins
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from aic_transfuser_lite.runtime import spatial_bootstrap_v4 as boot
from aic_transfuser_lite.runtime import spatial_runtime_v4 as runtime
from aic_transfuser_lite.runtime.spatial_recording_v4 import PrivateWriter, Records
from test_spatial_runtime_v4 import Fake, ManualClock, fake_message, wrapper_module, ROOT


@pytest.fixture(autouse=True)
def prohibit_real_dependencies(monkeypatch):
    def forbidden(*args,**kwargs): raise AssertionError('REAL ROS / CHECKPOINT FORBIDDEN')
    monkeypatch.setattr(runtime,'load_fixed',forbidden)
    for name in ('load_model','context','node','executor','message_types'):
        monkeypatch.setattr(boot.RealDependencies,name,forbidden)
    original_import=builtins.__import__
    def guarded_import(name,*args,**kwargs):
        if name.split('.')[0] in ('rclpy','sensor_msgs','autoware_auto_vehicle_msgs','autoware_auto_control_msgs'):
            forbidden()
        return original_import(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded_import)
    for method in ('open','stat'):
        original=getattr(Path,method)
        def guarded_path(path,*args,_original=original,**kwargs):
            if any(part in str(path).replace('\\','/').split('/') for part in ('checkpoints','datasets','raw')):
                forbidden()
            return _original(path,*args,**kwargs)
        monkeypatch.setattr(Path,method,guarded_path)


def config(tmp_path):
    cfg=boot.load_config(ROOT/'ros2_ws/src/aic_e2e_runtime/config/spatial_path_shadow_v4.param.yaml')
    cfg.update(enabled=True,code_sha256='a'*64,output_dir=str(tmp_path/'private'))
    cfg['ros']=dict(environment='FAKE_NO_DDS',namespace='/fixture',domain_id=23)
    cfg['limits']=dict(grid_period_ns=100_000_000,max_sync_wait_ns=300_000_000,candidate_capacity=16,
        queue_capacity=4,max_record_bytes=131072,max_file_bytes=4_000_000,max_candidates=20,max_forward_calls=6,
        session_duration_s=.8,poll_period_s=.05,shutdown_grace_s=.1)
    proof=dict(file='tests/test_spatial_bootstrap_v4.py',commit='0'*40,basis='FAKE fixture only; not live evidence')
    cfg['synchronization'].update(grid_phase='ROS_EPOCH_ZERO_NEAREST',evidence=deepcopy(proof))
    for role,b in cfg['bindings'].items():
        b.update(topic='/fake/'+role,frame='camera' if role=='image' else 'laser' if role=='lidar' else 'base_link',
                 producer_role='PASSIVE_NOMINAL_BEFORE_ACTUATION' if role=='nominal' else 'FAKE_'+role,
                 evidence=deepcopy(proof))
    cfg['bindings']['image']['sensor']=dict(height=8,width=12,step=36,encoding='rgb8')
    cfg['bindings']['lidar']['sensor']=dict(beams=750,angle_min=-1.,angle_max=1.996,angle_increment=.004,range_min=0.,range_max=25.)
    return cfg


def approval(cfg):
    return dict(version='spatial_v4_run_authorization_v1',code_sha256=cfg['code_sha256'],config_sha256=boot.digest(cfg),
        binding_sha256=boot.digest(cfg['bindings']),checkpoint_sha256=boot.CHECKPOINT_ID,ros=deepcopy(cfg['ros']),
        output_dir=cfg['output_dir'],limits=deepcopy(cfg['limits']),roles=list(boot.ROLES),live_sensor_subscription=True,
        fixed_checkpoint_inference=True,control_connection_enabled=False,training_authorized=False,
        runtime_promotion_authorized=False,fixture=True,approved_by='FAKE_TEST_ONLY',approved_at='FAKE_TIME',scope_reason='FAKE_NO_DDS_NO_WEIGHTS')


def message(role, ns=1_000_000_000):
    if role=='nominal':
        return SimpleNamespace(stamp=SimpleNamespace(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000),
            lateral=SimpleNamespace(steering_tire_angle=.1),longitudinal=SimpleNamespace(speed=1.,acceleration=0.))
    msg=fake_message(role,ns)
    msg.header.frame_id='camera' if role=='image' else 'laser' if role=='lidar' else 'base_link'
    if role=='lidar': msg.angle_min=-1.; msg.angle_max=1.996; msg.angle_increment=.004
    return msg


class Dependencies:
    fixture=True
    def __init__(self, actions=(), fail=None, writer_mode=None):
        self.clock=ManualClock(0); self.actions=list(actions); self.fail_at=fail; self.log=[]; self.writer_mode=writer_mode
        self.node_object=None; self.model=None; self.writer_object=None; self.wrappers=[]
    def step(self,stage):
        self.log.append(stage)
        if self.fail_at==stage: raise RuntimeError('FAKE_FAILURE:'+stage)
    def load_model(self):
        self.step('loader'); self.model=Fake()
        return self.model,dict(before_sha256=boot.CHECKPOINT_ID,after_sha256=boot.CHECKPOINT_ID,fixture=True)
    def context(self):
        self.step('context_factory'); parent=self
        class Context:
            handle=None
            def init(self,**kwargs): parent.step('context_init'); self.handle=object()
            def ok(self): return self.handle is not None
            def try_shutdown(self): parent.step('context_shutdown')
            def destroy(self): parent.step('context_destroy'); self.handle=None
        return Context()
    def node(self,context,cfg):
        self.step('node_factory'); parent=self
        class Node:
            def __init__(self): self.callbacks={}
            def create_subscription(self,kind,topic,callback,qos):
                parent.step('subscription'); self.callbacks[topic.rsplit('/',1)[-1]]=callback
            def destroy_node(self): parent.step('node_destroy')
            def __getattr__(self,name): raise AssertionError('forbidden node method '+name)
        self.node_object=Node(); return self.node_object
    def executor(self,context):
        self.step('executor_factory'); parent=self
        class Executor:
            def add_node(self,node): parent.step('add_node')
            def spin_once(self,timeout_sec):
                parent.step('spin_once'); parent.clock.advance(round(timeout_sec*1e9))
                if parent.actions: parent.actions.pop(0)(parent)
            def shutdown(self,timeout_sec): parent.step('executor_shutdown'); return True
        return Executor()
    def message_types(self):
        self.step('message_types'); return {**{r:object for r in boot.ROLES},'sensor_qos':object()}
    def writer(self,*args,**kwargs):
        self.step('writer_factory'); w=PrivateWriter(*args,**kwargs); self.writer_object=w
        original=w._write; calls=0
        def write(blob):
            nonlocal calls
            calls+=1
            if self.writer_mode=='receipt' and calls==2: raise OSError('FAKE_RECEIPT_FAILURE')
            original(blob)
        w._write=write
        if self.writer_mode=='closed': w.close()
        return w
    def wrapper(self,*args,**kwargs):
        self.step('wrapper_factory'); w=wrapper_module().SpatialPathShadowWrapperV4(*args,**kwargs)
        self.wrappers.append(w); return w
    def send(self,role,ns=1_000_000_000): self.node_object.callbacks[role](message(role,ns))


def complete_input(deps, ns=1_000_000_000):
    deps.send('nominal',ns-50_000_000)
    for role in ('velocity','steering','lidar','image'): deps.send(role,ns)


def execute(cfg,deps,auth=None):
    return boot.run(cfg,approval(cfg) if auth is None else auth,actual_code_id='a'*64,schema_dir=ROOT/'schemas',
                    wrapper_factory=deps.wrapper,dependencies=deps)


def evidence(name,cfg,deps,result):
    folder=os.environ.get('V4_BOOTSTRAP_TRACE_DIR')
    if not folder: return
    root=Path(folder); root.mkdir(parents=True,exist_ok=True)
    traces=[c.trace() for w in deps.wrappers for c in w.completed]
    value=dict(fixture=True,config=cfg,authorization_fixture=approval(cfg),result=result,resource_trace=deps.log,candidates=traces,
               real_ros_calls=0,real_checkpoint_reads=0)
    with (root/(name+'.json')).open('xb') as f: f.write(boot.canonical(value))
    path=Path(cfg['output_dir'])/'events.jsonl'
    if path.is_file():
        with (root/(name+'.jsonl')).open('xb') as f: f.write(path.read_bytes())
    manifest=Path(cfg['output_dir'])/'run_manifest.json'
    if manifest.is_file():
        with (root/(name+'_run_manifest.json')).open('xb') as f: f.write(manifest.read_bytes())


def test_default_main_and_check_config_have_no_real_factories(tmp_path,capsys):
    module=wrapper_module()
    assert module.main([])==2
    cfg=config(tmp_path); path=tmp_path/'config.json'; path.write_bytes(boot.canonical(cfg))
    assert module.main(['--config',str(path),'--check-config'])==2  # Code ID deliberately mismatches.
    report=json.loads(capsys.readouterr().out.splitlines()[-1])
    assert any('code_sha256' in e for e in report['errors'])
    assert all(v=='NOT_OBSERVED' for v in report['live_observation'].values())
    assert not Path(cfg['output_dir']).exists()


def test_distributed_config_exports_unresolved_expectations(capsys):
    module=wrapper_module()
    path=ROOT/'ros2_ws/src/aic_e2e_runtime/config/spatial_path_shadow_v4.param.yaml'
    assert module.main(['--config',str(path),'--check-config'])==2
    report=json.loads(capsys.readouterr().out)
    cfg=boot.load_config(path)
    assert not report['authorization_valid'] and report['unresolved']
    assert all(v=='NOT_OBSERVED' for v in report['live_observation'].values())
    folder=os.environ.get('V4_BOOTSTRAP_TRACE_DIR')
    if folder:
        import inspect
        root=Path(folder); root.mkdir(parents=True,exist_ok=True)
        with (root/'resolved_config.json').open('xb') as f: f.write(boot.canonical(dict(config=cfg,check=report,real_operations_performed=False)))
        rows=[]
        for role in boot.ROLES:
            rows.append(dict(role=role,expected=cfg['bindings'][role],observed='NOT_OBSERVED',
                source_file='src/aic_transfuser_lite/runtime/spatial_bootstrap_v4.py',function='BindingGuard.check',
                line=inspect.getsourcelines(boot.BindingGuard.check)[1],prior_contract_commit='60b3da378b1dee37bfcd3d8841b519b1d07e8d0c',
                prior_source='SpatialPathShadowWrapperV4.receive/tick; SpatialInputV4.append/build',
                unresolved=[v for v in report['unresolved'] if role in v]))
        with (root/'input_binding.json').open('xb') as f: f.write(boot.canonical(dict(rows=rows,synchronization=cfg['synchronization'],state='EXPECTED_CODE_ONLY_NOT_LIVE_OBSERVATION')))


def test_no_approval_and_ros_remaps_reject_real_entry_before_factories(tmp_path,capsys):
    cfg=config(tmp_path); module=wrapper_module()
    cfg['code_sha256']=boot.code_identity(Path(module.__file__))
    path=tmp_path/'unapproved.json'; path.write_bytes(boot.canonical(cfg))
    assert module.main(['--config',str(path)])==2
    assert not Path(cfg['output_dir']).exists()
    assert module.main(['--config',str(path),'--ros-args','-r','/fake/image:=/other'])==2
    assert 'ROS remaps/overrides not authorized' in capsys.readouterr().err


def test_main_cli_reaches_authorized_fake_assembly(tmp_path,capsys):
    cfg=config(tmp_path); deps=Dependencies([complete_input]); module=wrapper_module()
    cfg['code_sha256']=boot.code_identity(Path(module.__file__))
    path=tmp_path/'config.json'; auth_path=tmp_path/'fixture_authorization.json'
    path.write_bytes(boot.canonical(cfg)); auth_path.write_bytes(boot.canonical(approval(cfg)))
    code=boot.main(['--config',str(path),'--authorization',str(auth_path)],wrapper_file=Path(module.__file__),
        wrapper_factory=deps.wrapper,schema_dir=ROOT/'schemas',dependencies=deps)
    result=json.loads(capsys.readouterr().out)
    assert code==0 and result['operations']['fake_forward_calls']==1 and result['fixture'] is True
    evidence('main_cli_fake',cfg,deps,result)


@pytest.mark.parametrize('field,value',[('mode','SYNTHETIC'),('checkpoint_sha256','b'*64),('input_contract_sha256','b'*64),
    ('code_sha256','b'*64),('enabled',False),('control_connection_enabled',True)])
def test_invalid_identity_mode_or_disabled_has_no_effects(tmp_path,field,value):
    cfg=config(tmp_path); cfg[field]=value; deps=Dependencies()
    result=execute(cfg,deps)
    assert result['exit_code']==2 and not deps.log and not Path(cfg['output_dir']).exists()


@pytest.mark.parametrize('value',[True,False,0,-1,1.0,float('nan'),float('inf'),'100',None])
@pytest.mark.parametrize('field',boot.INT_LIMITS)
def test_integer_limits_reject_before_factories(tmp_path,field,value):
    cfg=config(tmp_path); cfg['limits'][field]=value; deps=Dependencies()
    result=execute(cfg,deps,auth={})
    assert result['exit_code']==2 and not deps.log
    assert any('limits.'+field in e for e in result['check']['errors'])


@pytest.mark.parametrize('field',boot.SECOND_LIMITS)
@pytest.mark.parametrize('value',[True,False,0,-1,float('nan'),float('inf'),'1',None])
def test_seconds_reject_before_factories(tmp_path,field,value):
    cfg=config(tmp_path); cfg['limits'][field]=value; deps=Dependencies()
    result=execute(cfg,deps,auth={})
    assert result['exit_code']==2 and not deps.log
    assert any('limits.'+field in e for e in result['check']['errors'])


@pytest.mark.parametrize('defect',['role','topic','alias','producer','beam','grid','authorization','fixture','binding_hash'])
def test_unresolved_binding_and_approval_are_not_boolean_proofs(tmp_path,defect):
    cfg=config(tmp_path)
    if defect=='role': del cfg['bindings']['nominal']
    if defect=='topic': cfg['bindings']['nominal']['topic']=''
    if defect=='alias': cfg['bindings']['nominal']['producer_role']='APPLIED_FINAL_COMMAND'
    if defect=='producer': cfg['bindings']['image']['evidence']='UNKNOWN'
    if defect=='beam': cfg['bindings']['lidar']['sensor']['angle_increment']=None
    if defect=='grid': cfg['synchronization']['grid_phase']='UNKNOWN'
    auth=approval(cfg)
    if defect=='authorization': auth['approved_by']=''
    if defect=='fixture': auth['fixture']=False
    if defect=='binding_hash': auth['binding_sha256']='b'*64
    deps=Dependencies(); result=execute(cfg,deps,auth)
    assert result['exit_code']==2 and not deps.log


def test_full_fake_bootstrap_output_and_owned_cleanup(tmp_path):
    cfg=config(tmp_path); deps=Dependencies([complete_input]); result=execute(cfg,deps)
    assert result['exit_code']==0 and result['reason']=='SESSION_DURATION_LIMIT'
    assert result['operations']==dict(real_ros_init_attempted=False,real_ros_initialized=False,real_checkpoint_read_attempted=False,
                                     real_checkpoint_read=False,fixed_forward_calls=0,fake_forward_calls=1)
    assert result['counters']['accepted']['ids']==[0] and result['counters']['saved']['ids']==[0]
    assert result['counters']['dropped']['ids']==[] and deps.model.calls==1
    w=deps.wrappers[0]; c=w.completed[0]
    assert c.terminal and len(bytes.fromhex(c.event['payload']['output']['float32_le_hex']))==160
    assert c.event['mode']=='LIVE_PASSIVE_FIXTURE' and c.event['passive_scope']['fixture'] is True
    assert c.event['execution']['live_sensor_connection_authorized'] is False
    assert w.stopped and not w.pending and not w.adapter.final_fallback_verified
    for stage in ('executor_shutdown','node_destroy','context_shutdown','context_destroy'): assert deps.log.count(stage)==1
    rows=[json.loads(line) for line in (Path(cfg['output_dir'])/'events.jsonl').read_bytes().splitlines()]
    assert [r['payload']['event_type'] for r in rows][-4:]==['LOGGER_HEALTH','WRITER_RECEIPT','SESSION_END','WRITER_RECEIPT']
    assert all(r['mode']=='LIVE_PASSIVE_FIXTURE' for r in rows)
    evidence('complete',cfg,deps,result)


def test_tick_without_callbacks_releases_m1(tmp_path):
    cfg=config(tmp_path)
    def first(d): d.send('nominal',950_000_000); d.send('image')
    def second(d):
        for role in ('lidar','velocity','steering','image'): d.send(role,1_100_000_000)
    deps=Dependencies([first,second]); result=execute(cfg,deps)
    w=deps.wrappers[0]
    assert [c.event['execution']['forward_calls'] for c in w.completed]==[0,1]
    assert w.completed[0].event['payload']['reason']=='SYNC_DEADLINE'
    assert deps.log.count('spin_once')>2 and not w.pending
    evidence('m0_m1_poll',cfg,deps,result)


@pytest.mark.parametrize('kind',['camera_reset','history_gap','monotonic_reset'])
def test_reset_does_not_reuse_old_command_readiness_or_budget(tmp_path,kind):
    cfg=config(tmp_path)
    def initial(d): complete_input(d,2_000_000_000)
    def next_input(d):
        if kind=='monotonic_reset': d.clock.ns=-1; return
        ns=1_500_000_000 if kind=='camera_reset' else 2_500_000_000
        # No fresh command in the new history/epoch.
        if kind=='camera_reset': d.send('image',ns)
        for role in ('velocity','steering','lidar'): d.send(role,ns)
        if kind=='history_gap': d.send('image',ns)
    deps=Dependencies([initial,next_input]); result=execute(cfg,deps)
    assert deps.model.calls==1 and result['counters']['forward_started']['ids']==[0]
    if kind=='monotonic_reset': assert result['reason']=='MONOTONIC_RESET_STOP'
    else: assert result['counters']['dropped']['ids']==[1]
    evidence(kind,cfg,deps,result)


@pytest.mark.parametrize('scenario',['zero','drop_only','command_absent','wrong_geometry','future_command'])
def test_no_input_or_unusable_input_stops_on_monotonic_time(tmp_path,scenario):
    cfg=config(tmp_path)
    def action(d):
        if scenario=='zero': return
        if scenario=='drop_only': d.node_object.callbacks['image'](SimpleNamespace()); return
        if scenario=='command_absent':
            for role in ('velocity','steering','lidar','image'): d.send(role)
        elif scenario=='wrong_geometry':
            d.send('nominal',950_000_000); d.send('velocity'); d.send('steering')
            msg=message('lidar'); msg.angle_increment=.1; d.node_object.callbacks['lidar'](msg); d.send('image')
        else:
            d.send('nominal',2_000_000_000)
            for role in ('velocity','steering','lidar','image'): d.send(role)
    deps=Dependencies([action]); result=execute(cfg,deps)
    assert result['reason']=='SESSION_DURATION_LIMIT' and deps.model.calls==0
    assert result['input_result']==('NO_INPUT' if scenario=='zero' else 'NO_FORWARD')
    if scenario=='command_absent': assert result['binding_observations']['nominal']=='NOT_OBSERVED'
    evidence(scenario,cfg,deps,result)


@pytest.mark.parametrize('limit',['max_candidates','max_forward_calls','max_file_bytes'])
def test_run_limits_stop_admission_and_forward(tmp_path,limit):
    cfg=config(tmp_path); cfg['limits'][limit]=8192 if limit=='max_file_bytes' else 1
    deps=Dependencies([complete_input,lambda d:complete_input(d,1_100_000_000)])
    result=execute(cfg,deps)
    assert result['counters']['accepted']['count']==1 and deps.model.calls<=1
    assert all(c.terminal for c in deps.wrappers[0].completed)
    assert result['known_saved_bytes']<=cfg['limits']['max_file_bytes']
    assert result['reason']=={'max_candidates':'CANDIDATE_LIMIT','max_forward_calls':'FORWARD_LIMIT','max_file_bytes':'RECORDING_FAILED'}[limit]
    evidence('limit_'+limit,cfg,deps,result)


@pytest.mark.parametrize('stage',['loader','writer_factory','context_factory','context_init','node_factory','message_types','wrapper_factory','subscription','executor_factory','add_node','spin_once'])
def test_initialization_failures_only_cleanup_acquired_resources(tmp_path,stage):
    cfg=config(tmp_path); deps=Dependencies(fail=stage); result=execute(cfg,deps)
    assert result['exit_code']==4 and stage in result['first_error']
    for close in ('executor_shutdown','node_destroy','context_shutdown','context_destroy'): assert deps.log.count(close)<=1
    if deps.writer_object: assert deps.writer_object.fd is None
    if deps.node_object: assert deps.log.count('node_destroy')==1
    if 'context_factory' in deps.log and stage!='context_factory': assert deps.log.count('context_shutdown')==1
    if stage=='context_init': assert 'context_destroy' not in deps.log
    if stage in ('add_node','spin_once'): assert deps.log.count('executor_shutdown')==1
    evidence('failure_'+stage,cfg,deps,result)


@pytest.mark.parametrize('interrupt',[True,False])
def test_interrupt_and_error_terminalize_pending_without_forward(tmp_path,interrupt):
    cfg=config(tmp_path)
    def pending(d): d.send('image')
    def stop(d):
        if interrupt: raise KeyboardInterrupt('FAKE_SIGINT')
        raise RuntimeError('FIRST_RUN_ERROR')
    deps=Dependencies([pending,stop]); result=execute(cfg,deps)
    assert result['exit_code']==(130 if interrupt else 4) and deps.model.calls==0
    assert result['counters']['accepted']['ids']==result['counters']['dropped']['ids']==[0]
    assert deps.wrappers[0].completed[0].terminal and not deps.wrappers[0].pending
    evidence('interrupt' if interrupt else 'run_error',cfg,deps,result)


def test_cleanup_error_does_not_replace_first_error(tmp_path):
    cfg=config(tmp_path)
    def stop(d): raise RuntimeError('ORIGINAL_RUN_ERROR')
    deps=Dependencies([stop],fail='node_destroy'); result=execute(cfg,deps)
    assert 'ORIGINAL_RUN_ERROR' in result['first_error']
    assert any('node_destroy' in x for x in result['cleanup_errors'])
    assert deps.log.count('context_shutdown')==1 and deps.writer_object.fd is None
    evidence('cleanup_error',cfg,deps,result)


def test_missing_dependency_is_distinct_and_closes_writer(tmp_path):
    cfg=config(tmp_path); deps=Dependencies()
    def missing(): raise ImportError('FAKE_MISSING_RCLPY')
    deps.context=missing
    result=execute(cfg,deps)
    assert result['exit_code']==3 and result['reason']=='DEPENDENCY_MISSING'
    assert deps.writer_object.fd is None and deps.model.calls==0
    evidence('dependency_missing',cfg,deps,result)


def test_shutdown_grace_overrun_is_not_silent_success(tmp_path):
    cfg=config(tmp_path); deps=Dependencies(); original=deps.executor
    def executor(context):
        obj=original(context); shutdown=obj.shutdown
        def slow_shutdown(**kwargs):
            deps.clock.advance(200_000_000); return shutdown(**kwargs)
        obj.shutdown=slow_shutdown; return obj
    deps.executor=executor
    result=execute(cfg,deps)
    assert result['shutdown_grace_exceeded'] and result['exit_code']==6
    assert deps.writer_object.fd is None
    evidence('shutdown_grace',cfg,deps,result)


@pytest.mark.parametrize('stage',['loader','context_init','node_factory'])
def test_startup_time_limit_prevents_following_factories(tmp_path,stage):
    cfg=config(tmp_path); deps=Dependencies(); original=deps.step
    def slow_step(name):
        original(name)
        if name==stage: deps.clock.advance(1_000_000_000)
    deps.step=slow_step; result=execute(cfg,deps)
    assert result['reason']=='SESSION_DURATION_LIMIT' and deps.model.calls==0
    if stage=='loader': assert 'context_factory' not in deps.log
    if stage=='context_init': assert 'node_factory' not in deps.log
    assert 'subscription' not in deps.log and 'spin_once' not in deps.log
    evidence('startup_deadline_'+stage,cfg,deps,result)


def test_executor_shutdown_false_is_reported(tmp_path):
    cfg=config(tmp_path); deps=Dependencies(); original=deps.executor
    def executor(context):
        obj=original(context)
        def timeout(**kwargs): deps.step('executor_shutdown'); return False
        obj.shutdown=timeout; return obj
    deps.executor=executor; result=execute(cfg,deps)
    assert result['exit_code']==6 and any('TimeoutError' in e for e in result['cleanup_errors'])
    assert deps.log.count('node_destroy')==1 and deps.writer_object.fd is None
    evidence('executor_shutdown_timeout',cfg,deps,result)


def test_existing_output_is_not_overwritten(tmp_path):
    cfg=config(tmp_path); out=Path(cfg['output_dir']); out.mkdir(); marker=out/'keep.txt'; marker.write_text('USER')
    deps=Dependencies(); result=execute(cfg,deps)
    assert result['exit_code']==4 and not deps.log and marker.read_text()=='USER'
    evidence('output_exists',cfg,deps,result)


@pytest.mark.parametrize('mode',['receipt','closed'])
def test_logger_failure_or_closed_stops_future_forward(tmp_path,mode):
    cfg=config(tmp_path); deps=Dependencies([complete_input,complete_input],writer_mode=mode)
    result=execute(cfg,deps)
    assert deps.model.calls==(1 if mode=='receipt' else 0)
    if mode=='receipt':
        assert result['counters']['saved']['ids']==[0] and result['counters']['dropped']['ids']==[]
        assert result['first_error']=='OSError: FAKE_RECEIPT_FAILURE'
    evidence('writer_'+mode,cfg,deps,result)


def test_modes_are_distinct_and_design_unchanged():
    with pytest.raises(ValueError): Records(ROOT/'schemas',mode='LIVE_PASSIVE')
    old=Records(ROOT/'schemas',mode='SYNTHETIC').event('SESSION_END')
    assert old['schema_version']=='spatial_path_v4_runtime_record_v1' and 'passive_scope' not in old


def test_live_passive_fixture_full_schema_if_available(tmp_path):
    pytest.importorskip('jsonschema',reason='existing environment lacks full Draft2020 validator; no install')
    cfg=config(tmp_path); deps=Dependencies([complete_input]); execute(cfg,deps)
    records=deps.writer_object.records
    for row in (Path(cfg['output_dir'])/'events.jsonl').read_bytes().splitlines(): records.validate_schema(json.loads(row))


def test_static_no_control_and_explicit_owned_ros_factories():
    import ast
    paths=[Path(boot.__file__),ROOT/'ros2_ws/src/aic_e2e_runtime/aic_e2e_runtime/spatial_path_shadow_node_v4.py']
    banned={'create_publisher','create_client','create_service','set_parameters','publish','call_async','create_action_client'}
    for path in paths:
        source=path.read_text(); tree=ast.parse(source)
        assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in banned for n in ast.walk(tree))
        assert 'inference_node_v3' not in source and 'controller' not in source
    source=Path(boot.__file__).read_text()
    assert 'SingleThreadedExecutor' in source and 'use_global_arguments=False' in source
    assert 'enable_rosout=False' in source and 'start_parameter_services=False' in source
