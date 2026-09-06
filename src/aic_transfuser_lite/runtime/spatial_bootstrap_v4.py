"""V4-only guarded assembly. Import/check-config never imports ROS or reads weights.

One owned context/node/executor; bounded spin_once plus monotonic polling. No
hard interruption of synchronous model/I/O calls, no control endpoints or retry.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sys
import time
from typing import Callable

CHECKPOINT_ID = '0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f'
INPUT_ID = '77fff3f9c5875b6f116cebe35fde88d42d459aa04c195676c44e5c6caca5edc7'
ROLES = ('image','lidar','velocity','steering','nominal')
TYPES = dict(image='sensor_msgs/msg/Image',lidar='sensor_msgs/msg/LaserScan',
             velocity='autoware_auto_vehicle_msgs/msg/VelocityReport',
             steering='autoware_auto_vehicle_msgs/msg/SteeringReport',
             nominal='autoware_auto_control_msgs/msg/AckermannControlCommand')
FIELDS = dict(image={'encoding':'rgb8','height':'px','width':'px','step':'bytes','data':'uint8'},
              lidar={'ranges':'m','angle_min':'rad','angle_max':'rad','angle_increment':'rad','range_min':'m','range_max':'m'},
              velocity={'longitudinal_velocity':'m/s','lateral_velocity':'m/s','heading_rate':'rad/s'},
              steering={'steering_tire_angle':'rad'},
              nominal={'lateral.steering_tire_angle':'rad','longitudinal.speed':'m/s','longitudinal.acceleration':'m/s^2'})
INT_LIMITS = ('grid_period_ns','max_sync_wait_ns','candidate_capacity','queue_capacity',
              'max_record_bytes','max_file_bytes','max_candidates','max_forward_calls')
SECOND_LIMITS = ('session_duration_s','poll_period_s','shutdown_grace_s')


def canonical(value: object) -> bytes:
    return json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()


def digest(value: object) -> str:
    return hashlib.sha256(canonical(value)).hexdigest()


def code_identity(wrapper_file: Path) -> str:
    """Identity of explicit assembly boundary files, not checkpoint or dataset paths."""
    root=Path(__file__).resolve().parent
    paths=[root/name for name in ('spatial_bootstrap_v4.py','spatial_input_v4.py','spatial_recording_v4.py','spatial_runtime_v4.py')]
    paths.append(wrapper_file)
    return digest({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})


def load_config(path: Path) -> dict:
    """Only the explicitly selected small config is opened. Referenced paths aren't followed."""
    with path.open('rb') as stream:
        blob=stream.read(65537)
    if len(blob)>65536:
        raise ValueError('config: exceeds 65536 bytes')
    import yaml
    value=yaml.safe_load(blob)
    if not isinstance(value,dict):
        raise ValueError('config: mapping required')
    return value


def check_config(config: dict, actual_code_id: str, authorization: dict | None = None, *, fixture: bool = False) -> dict:
    """Pure config/approval comparison. Expectations are never live observations."""
    errors=[]; unresolved=[]
    def error(field: str, why: str) -> None: errors.append(field+': '+why)
    def text(value: object) -> bool: return isinstance(value,str) and bool(value.strip()) and value not in ('UNKNOWN','MISSING')
    def positive(value: object) -> bool: return type(value) is int and value>0
    def finite(value: object) -> bool:
        try: return type(value) in (int,float) and math.isfinite(value)
        except OverflowError: return False
    def evidence(value: object) -> bool:
        return (isinstance(value,dict) and set(value)=={'file','commit','basis'} and
                text(value.get('file')) and text(value.get('basis')) and isinstance(value.get('commit'),str) and
                re.fullmatch('[0-9a-f]{40}',value['commit']) is not None and
                (fixture or 'FAKE' not in value['basis']))
    if not isinstance(config,dict):
        return dict(valid=False,errors=['config: mapping required'],unresolved=[],authorization_valid=False,status='INVALID_CONFIG')
    allowed={'version','mode','enabled','code_sha256','checkpoint_sha256','input_contract_sha256','ros','output_dir','limits','bindings','synchronization','control_connection_enabled'}
    if set(config)!=allowed: error('config.keys','missing or unknown fields: '+str(sorted(map(str,set(config)^allowed))))
    if config.get('version')!='spatial_bootstrap_v4_v2': error('version','unsupported')
    if config.get('mode')!='LIVE_PASSIVE': error('mode','LIVE_PASSIVE required; fixture is an injected execution fact')
    if type(config.get('enabled')) is not bool: error('enabled','bool required')
    if config.get('control_connection_enabled') is not False: error('control_connection_enabled','must be false')
    for key,expected in (('code_sha256',actual_code_id),('checkpoint_sha256',CHECKPOINT_ID),('input_contract_sha256',INPUT_ID)):
        if not isinstance(config.get(key),str) or not re.fullmatch('[0-9a-f]{64}',config[key]) or config[key]!=expected: error(key,'identity mismatch')
    ros=config.get('ros',{})
    if not isinstance(ros,dict): ros={}; error('ros','mapping required')
    if set(ros)!={'environment','namespace','domain_id'}: error('ros.keys','environment/namespace/domain_id required')
    if not text(ros.get('environment')): unresolved.append('ros.environment')
    if not isinstance(ros.get('namespace'),str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*',ros['namespace']): error('ros.namespace','explicit non-root namespace required')
    if not positive(ros.get('domain_id')): error('ros.domain_id','positive Python int required')
    output=config.get('output_dir')
    if not isinstance(output,str) or not output or not Path(output).is_absolute(): error('output_dir','explicit absolute new directory required')
    limits=config.get('limits',{})
    if not isinstance(limits,dict): limits={}; error('limits','mapping required')
    if set(limits)!=set(INT_LIMITS+SECOND_LIMITS): error('limits.keys','all explicit limits required; no defaults')
    for key in INT_LIMITS:
        if not positive(limits.get(key)): error('limits.'+key,'positive Python int (bool excluded) required')
    for key in SECOND_LIMITS:
        value=limits.get(key)
        if type(value) not in (int,float) or value<=0:
            error('limits.'+key,'finite positive seconds required')
        else:
            try:
                if not math.isfinite(float(value)): error('limits.'+key,'finite positive seconds required')
            except OverflowError: error('limits.'+key,'finite seconds required')
    if limits.get('grid_period_ns')!=100_000_000: error('limits.grid_period_ns','fixed input contract uses 100000000 ns')
    sync=config.get('synchronization',{})
    if not isinstance(sync,dict): sync={}; error('synchronization','mapping required')
    expected_sync=dict(grid_phase='ROS_EPOCH_ZERO_NEAREST',ego='EXACT_CAMERA_HEADER',lidar='NEAREST_WITHIN_30000000_NS',
                       sensor_dt=['camera_header-grid','lidar_header-camera_header'])
    if set(sync)!=set(expected_sync)|{'evidence'}: error('synchronization.keys','unexpected or missing fields')
    for key,value in expected_sync.items():
        if sync.get(key)!=value: unresolved.append('synchronization.'+key)
    if not evidence(sync.get('evidence')): unresolved.append('synchronization.evidence')
    bindings=config.get('bindings',{})
    if not isinstance(bindings,dict): bindings={}; error('bindings','mapping required')
    if set(bindings)!=set(ROLES): error('bindings.roles','exactly image/lidar/velocity/steering/nominal required')
    topics=[]
    for role in ROLES:
        b=bindings.get(role,{})
        if not isinstance(b,dict): error('bindings.'+role,'mapping required'); continue
        prefix='bindings.'+role
        keys={'topic','message_type','fields','frame','source_clock','qos','producer_role','evidence','sensor','interface'}
        if set(b)!=keys: error(prefix+'.keys','missing or unknown field')
        topic=b.get('topic')
        if not isinstance(topic,str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*',topic): unresolved.append(prefix+'.topic')
        else: topics.append(topic)
        if b.get('message_type')!=TYPES[role]: error(prefix+'.message_type','unsupported alias/type')
        if b.get('fields')!=FIELDS[role]: error(prefix+'.fields','field/unit contract mismatch')
        if b.get('source_clock')!='ROS_SIM': unresolved.append(prefix+'.source_clock')
        if b.get('qos')!=('SENSOR_DATA' if role in ('image','lidar') else 'RELIABLE_VOLATILE_DEPTH10'): error(prefix+'.qos','unsupported profile')
        for key in ('frame','producer_role'):
            if not text(b.get(key)): unresolved.append(prefix+'.'+key)
        if not evidence(b.get('evidence')): unresolved.append(prefix+'.evidence')
        interface=b.get('interface')
        if not isinstance(interface,dict) or set(interface)!={'stamp_source','message_frame','evidence'}:
            unresolved.append(prefix+'.interface.MISSING_INTERFACE_DEFINITION')
        else:
            pair=(interface.get('stamp_source'),interface.get('message_frame'))
            allowed_pairs=(('header.stamp','header.frame_id'),) if role in ('image','lidar','velocity') else (('header.stamp','header.frame_id'),('stamp',None))
            if pair not in allowed_pairs: error(prefix+'.interface','unsupported role stamp/frame contract')
            if not evidence(interface.get('evidence')): unresolved.append(prefix+'.interface.evidence')
        if role=='nominal' and b.get('producer_role')!='PASSIVE_NOMINAL_BEFORE_ACTUATION': unresolved.append(prefix+'.producer_role')
        sensor=b.get('sensor')
        if role=='image':
            if not isinstance(sensor,dict) or set(sensor)!={'height','width','step','encoding'}: unresolved.append(prefix+'.sensor')
            elif sensor.get('encoding')!='rgb8' or not all(positive(sensor.get(k)) for k in ('height','width','step')): unresolved.append(prefix+'.sensor')
            elif sensor['step']<sensor['width']*3: error(prefix+'.sensor.step','less than width*3')
        elif role=='lidar':
            if not isinstance(sensor,dict) or set(sensor)!={'beams','angle_min','angle_max','angle_increment','range_min','range_max'}: unresolved.append(prefix+'.sensor')
            elif sensor.get('beams')!=750: error(prefix+'.sensor.beams','750 required, not sufficient alone')
            elif any(not finite(sensor.get(k)) for k in ('angle_min','angle_max','angle_increment','range_min','range_max')): unresolved.append(prefix+'.sensor.geometry')
            elif sensor['angle_increment']<=0 or sensor['range_min']<0 or sensor['range_max']<=sensor['range_min'] or not math.isclose(sensor['angle_max'],sensor['angle_min']+749*sensor['angle_increment'],abs_tol=1e-6,rel_tol=0): error(prefix+'.sensor.geometry','inconsistent geometry')
        elif sensor!={}: error(prefix+'.sensor','must be empty for non-sensors')
    if len(topics)!=len(set(topics)): error('bindings.topic','duplicate topics/role alias not supported')
    try: config_id=digest(config); binding_id=digest(bindings)
    except (ValueError,TypeError): config_id=None; binding_id=None; error('config','noncanonical/nonfinite JSON')
    approval_errors=[]
    expected=dict(version='spatial_v4_run_authorization_v1',code_sha256=actual_code_id,config_sha256=config_id,binding_sha256=binding_id,
                  checkpoint_sha256=CHECKPOINT_ID,ros=ros,output_dir=output,limits=limits,roles=list(ROLES),
                  live_sensor_subscription=True,fixed_checkpoint_inference=True,control_connection_enabled=False,
                  training_authorized=False,runtime_promotion_authorized=False,fixture=fixture)
    if not isinstance(authorization,dict): approval_errors.append('authorization: MISSING')
    else:
        if set(authorization)!=set(expected)|{'approved_by','approved_at','scope_reason'}: approval_errors.append('authorization.keys')
        for key,value in expected.items():
            try: matches=canonical(authorization.get(key))==canonical(value)
            except (ValueError,TypeError): matches=False
            if not matches: approval_errors.append('authorization.'+key)
        for key in ('approved_by','approved_at','scope_reason'):
            if not text(authorization.get(key)): approval_errors.append('authorization.'+key)
    if config.get('enabled') is not True: approval_errors.append('enabled: DEFAULT_DISABLED')
    return dict(valid=not errors,errors=errors,unresolved=unresolved,
                status='INVALID_CONFIG' if errors else ('LIVE_BINDING_UNRESOLVED' if unresolved else 'EXPECTED_BINDING_CONFIGURED_NOT_OBSERVED'),
                config_sha256=config_id,binding_sha256=binding_id,code_sha256=actual_code_id,
                authorization_valid=not approval_errors,authorization_errors=approval_errors,
                live_observation={role:'NOT_OBSERVED' for role in ROLES},fixture=fixture)


def extract_message_stamp(role: str, msg: object, interface: dict) -> tuple[int, str | None]:
    """Explicit, shared extraction; never substitute receipt time or expected frame."""
    source=interface['stamp_source']
    if source=='header.stamp' and interface['message_frame']=='header.frame_id':
        stamp=msg.header.stamp
        frame=msg.header.frame_id
        if not isinstance(frame,str): raise ValueError('frame must be a string')
    elif source=='stamp' and interface['message_frame'] is None and role in ('steering','nominal'):
        stamp=msg.stamp; frame=None
    else:
        raise ValueError('unsupported role stamp/frame contract')
    if type(stamp.sec) is not int or type(stamp.nanosec) is not int or stamp.sec<0 or not 0<=stamp.nanosec<1_000_000_000:
        raise ValueError('header stamp integer/range')
    return stamp.sec*1_000_000_000+stamp.nanosec,frame


class BindingGuard:
    """Bounded latest-message checks, not producer/graph verification or sensor parity."""
    def __init__(self, config: dict, admission_stop: Callable, forward_stop: Callable):
        self.config=config; self.admission_stop=admission_stop; self.forward_stop=forward_stop
        self.observed={role:'NOT_OBSERVED' for role in ROLES}
        self.rejected=0
        self.message_contract_observations={}

    def reset_epoch(self) -> None:
        self.observed={role:'NOT_OBSERVED_CURRENT_EPOCH' for role in ROLES}
        self.message_contract_observations={}

    def check(self, role: str, msg: object) -> int:
        b=self.config['bindings'][role]
        observation=dict(role=role,message_type=b['message_type'],stamp_source=b['interface']['stamp_source'],
                         expected_message_frame_field=b['interface']['message_frame'],
                         message_frame_present=hasattr(getattr(msg,'header',None),'frame_id'),
                         observed_frame=None,expected_semantic_frame=b['frame'],status='NOT_EXTRACTED')
        self.message_contract_observations[role]=observation
        try:
            ns,frame=extract_message_stamp(role,msg,b['interface'])
            observation.update(observed_frame=frame,header_ns=ns,status='EXTRACTED')
            if b['interface']['message_frame'] is not None and frame!=b['frame']: raise ValueError('frame mismatch')
            if role=='image':
                for field,value in b['sensor'].items():
                    if getattr(msg,field)!=value: raise ValueError('image.'+field)
                if len(msg.data)!=msg.height*msg.step: raise ValueError('image stride/data length')
            elif role=='lidar':
                if len(msg.ranges)!=750: raise ValueError('lidar.beams')
                for field,value in b['sensor'].items():
                    if field!='beams' and (not math.isfinite(getattr(msg,field)) or not math.isclose(getattr(msg,field),value,abs_tol=1e-6,rel_tol=0)):
                        raise ValueError('lidar.'+field)
            else:
                for field in FIELDS[role]:
                    value=msg
                    for part in field.split('.'): value=getattr(value,part)
                    if type(value) not in (int,float) or not math.isfinite(value): raise ValueError(role+'.'+field)
            self.observed[role]='MESSAGE_CONTENT_MATCHED_NOT_PRODUCER_PROVEN'
            observation['status']=self.observed[role]
            return ns
        except Exception as exc:
            self.observed[role]='CONTENT_REJECTED:'+str(exc)[:200]
            observation['status']=self.observed[role]
            self.rejected+=1
            raise ValueError('INPUT_BINDING:'+role+':'+str(exc)[:200]) from exc

    def ready(self) -> bool:
        # Content receipt is separate from adapter-owned per-slot eligibility.
        return all(v=='MESSAGE_CONTENT_MATCHED_NOT_PRODUCER_PROVEN' for v in self.observed.values())


class RealDependencies:
    """Only called after authorization. No hidden global ROS context or signal handler."""
    fixture=False
    clock=staticmethod(time.monotonic_ns)

    def load_model(self):
        from .spatial_runtime_v4 import load_fixed
        return load_fixed(device='cpu')

    def context(self):
        from rclpy.context import Context
        return Context()

    def node(self, context, config):
        from rclpy.node import Node
        return Node('spatial_path_shadow_v4',context=context,namespace=config['ros']['namespace'],
                    cli_args=[],use_global_arguments=False,enable_rosout=False,start_parameter_services=False)

    def executor(self, context):
        from rclpy.executors import SingleThreadedExecutor
        return SingleThreadedExecutor(context=context)

    def message_types(self):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, LaserScan
        from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
        from autoware_auto_control_msgs.msg import AckermannControlCommand
        return dict(image=Image,lidar=LaserScan,velocity=VelocityReport,steering=SteeringReport,
                    nominal=AckermannControlCommand,sensor_qos=qos_profile_sensor_data)


def run(config: dict, authorization: dict, *, actual_code_id: str, schema_dir: Path,
        wrapper_factory: Callable, dependencies: object | None = None) -> dict:
    deps=dependencies if dependencies is not None else RealDependencies()
    fixture=deps.fixture is True
    config=deepcopy(config); authorization=deepcopy(authorization)
    checked=check_config(config,actual_code_id,authorization,fixture=fixture)
    result=dict(check=checked,fixture=fixture,reason='NOT_STARTED',first_error=None,cleanup_errors=[],
                operations=dict(real_ros_init_attempted=False,real_ros_initialized=False,real_checkpoint_read_attempted=False,
                                real_checkpoint_read=False,fixed_forward_calls=0,fake_forward_calls=0),
                lifecycle=[],counters=None,binding_observations={},tick_calls=0,exit_code=2,
                stage_boundaries=[],cleanup_attempts=[],timing_errors=[])
    if not checked['valid'] or checked['unresolved'] or not checked['authorization_valid']:
        result['reason']=checked['status'] if checked['unresolved'] or not checked['valid'] else 'AUTHORIZATION_REQUIRED'
        return result
    from .spatial_input_v4 import SpatialInputV4
    from .spatial_recording_v4 import Records, PrivateWriter
    from .spatial_runtime_v4 import SpatialRuntimeV4
    limits=config['limits']; context=node=executor=writer=wrapper=core=records=None
    first_time=None; last_time=None
    def event(name: str) -> None: result['lifecycle'].append(name)
    def fail(exc: BaseException) -> None:
        if result['first_error'] is None: result['first_error']=type(exc).__name__+': '+str(exc)[:500]
    def stopping(include_candidates: bool=True) -> str | None:
        nonlocal last_time
        now=deps.clock()
        previous=last_time; last_time=now
        if now<first_time or (previous is not None and now<previous): return 'MONOTONIC_RESET_STOP'
        if (now-first_time)*1e-9>=limits['session_duration_s']: return 'SESSION_DURATION_LIMIT'
        if writer and writer.state!='OPEN': return 'LOGGER_'+writer.state
        if core and core.forward_calls>=limits['max_forward_calls']: return 'FORWARD_LIMIT'
        if include_candidates and records and records.sequence>=limits['max_candidates']: return 'CANDIDATE_LIMIT'
        return None
    def startup_budget_exhausted(stage: str) -> bool:
        reason=stopping()
        result['stage_boundaries'].append(dict(stage=stage,monotonic_ns=last_time,stop_reason=reason))
        if reason: result.update(reason=reason,exit_code=0)
        return reason is not None
    try:
        first_time=last_time=deps.clock()
        out=Path(config['output_dir'])
        parent=out.parent.resolve(strict=True)
        if shutil.disk_usage(parent).free<limits['max_file_bytes']: raise OSError('OUTPUT_CAPACITY_INSUFFICIENT')
        if startup_budget_exhausted('output_preflight'): return result
        out.mkdir(mode=0o700)  # Exclusive directory acquisition; never overwrite/reuse.
        event('output_directory_owned')
        if startup_budget_exhausted('output_directory'): return result
        metadata=canonical(dict(version='spatial_bootstrap_session_v1',fixture=fixture,config=config,
                                authorization=authorization,resolved_expectations=checked))
        if len(metadata)>limits['max_record_bytes'] or len(metadata)>=limits['max_file_bytes']:
            raise ValueError('OUTPUT_MANIFEST_BYTE_LIMIT')
        if startup_budget_exhausted('manifest_serialized'): return result
        with (out/'run_manifest.json').open('xb') as stream:
            stream.write(metadata)
        result['manifest_bytes']=len(metadata); event('manifest_written_not_durable')
        if startup_budget_exhausted('manifest_written'): return result
        result['operations']['real_checkpoint_read_attempted']=not fixture
        result['operations']['real_checkpoint_read']=False if fixture else None
        model,inventory=deps.load_model(); event('fixed_loader_returned_fake' if fixture else 'fixed_loader_returned')
        result['operations']['real_checkpoint_read']=not fixture
        if inventory.get('before_sha256')!=CHECKPOINT_ID or inventory.get('after_sha256')!=CHECKPOINT_ID:
            raise ValueError('loader inventory identity mismatch')
        if startup_budget_exhausted('loader'): return result
        scope=dict(config_sha256=checked['config_sha256'],binding_sha256=checked['binding_sha256'],code_sha256=actual_code_id,
                   authorization_sha256=digest(authorization),checkpoint_sha256=CHECKPOINT_ID,fixture=fixture,
                   authorized_by=authorization['approved_by'],authorized_at=authorization['approved_at'])
        records=Records(schema_dir,mode='LIVE_PASSIVE_FIXTURE' if fixture else 'LIVE_PASSIVE',clock=deps.clock,passive_scope=scope)
        event('records_owned')
        if startup_budget_exhausted('records'): return result
        writer_factory=getattr(deps,'writer',PrivateWriter)
        writer=writer_factory(out/'events.jsonl',records,capacity=limits['queue_capacity'],
            max_record_bytes=limits['max_record_bytes'],max_file_bytes=limits['max_file_bytes']-len(metadata)); event('writer_owned')
        if startup_budget_exhausted('writer'): return result
        adapter=SpatialInputV4(command_binding_known=True,final_fallback_verified=False)
        if startup_budget_exhausted('adapter'): return result
        core=SpatialRuntimeV4(model,records,writer,checkpoint_hash=CHECKPOINT_ID,forward_limit=limits['max_forward_calls'])
        event('adapter_core_owned')
        if startup_budget_exhausted('core'): return result
        context=deps.context(); event('context_owned')
        if startup_budget_exhausted('context_factory'): return result
        result['operations']['real_ros_init_attempted']=not fixture
        result['operations']['real_ros_initialized']=False if fixture else None
        context.init(args=[],domain_id=config['ros']['domain_id'],initialize_logging=False); event('context_initialized')
        result['operations']['real_ros_initialized']=not fixture
        if startup_budget_exhausted('context_init'): return result
        node=deps.node(context,config); event('node_owned')
        if startup_budget_exhausted('node_factory'): return result
        guard=BindingGuard(config,stopping,lambda:stopping(False))
        message_types=deps.message_types()
        if startup_budget_exhausted('message_types'): return result
        wrapper=wrapper_factory(node,core,adapter,message_types,{r:config['bindings'][r]['topic'] for r in ROLES},
            grid_period_ns=limits['grid_period_ns'],max_sync_wait_ns=limits['max_sync_wait_ns'],candidate_capacity=limits['candidate_capacity'],guard=guard)
        event('wrapper_bound')
        if startup_budget_exhausted('wrapper_factory'): return result
        executor=deps.executor(context); event('executor_owned')
        if startup_budget_exhausted('executor_factory'): return result
        executor.add_node(node); event('executor_node_registered')
        if startup_budget_exhausted('add_node'): return result
        while True:
            reason=stopping()
            if reason: result['reason']=reason; break
            if not context.ok(): result['reason']='CONTEXT_STOPPED'; break
            remaining=limits['session_duration_s']-(deps.clock()-first_time)*1e-9
            executor.spin_once(timeout_sec=max(0.,min(limits['poll_period_s'],remaining)))
            reason=stopping()
            if reason: result['reason']=reason; break
            wrapper.tick()  # Independent of callback arrivals and ROS clock progression.
            result['tick_calls']+=1
        result['exit_code']=0
    except KeyboardInterrupt as exc:
        result['reason']='INTERRUPTED'; result['exit_code']=130; fail(exc)
    except ImportError as exc:
        result['reason']='DEPENDENCY_MISSING'; result['exit_code']=3; fail(exc)
    except Exception as exc:
        result['reason']='STARTUP_OR_RUN_EXCEPTION'; result['exit_code']=4; fail(exc)
    finally:
        result['stop_reason_before_cleanup']=result['reason']
        result['owned_resources']={name:value is not None for name,value in
            (('records',records),('writer',writer),('core',core),('context',context),('node',node),('wrapper',wrapper),('executor',executor))}
        def timing_sample(stage: str) -> int | None:
            try: return deps.clock()
            except Exception as exc:
                fail(exc)
                result['timing_errors'].append(stage+': '+type(exc).__name__+': '+str(exc)[:300])
                return None
        end_start=timing_sample('cleanup_entry')
        cleanup_epoch=records.clock_epoch if records else None
        def cleanup(name: str, callback: Callable) -> None:
            attempt=dict(name=name,status='ATTEMPTED')
            result['cleanup_attempts'].append(attempt)
            try: callback(); event(name); attempt['status']='COMPLETED'
            except Exception as exc:
                fail(exc); attempt['status']='FAILED'
                result['cleanup_errors'].append(name+': '+type(exc).__name__+': '+str(exc)[:300])
        if wrapper:
            cleanup('wrapper_stopped',lambda:wrapper.stop('SESSION_END:'+result['reason']))
            result['binding_observations']=dict(wrapper.guard.observed)
            result['message_contract_observations']=deepcopy(wrapper.guard.message_contract_observations)
        if executor:
            def shutdown_executor():
                if executor.shutdown(timeout_sec=limits['shutdown_grace_s']) is False:
                    raise TimeoutError('executor shutdown grace exhausted')
            cleanup('executor_shutdown',shutdown_executor)
        if node: cleanup('node_destroyed',node.destroy_node)
        if context:
            cleanup('context_shutdown',context.try_shutdown)
            def destroy_context():
                if context.handle is not None: context.destroy()
            cleanup('context_destroy_checked',destroy_context)
        if writer:
            if writer.error and result['first_error'] is None: result['first_error']=writer.error
            if writer.state=='OPEN':
                def end_records():
                    for kind in ('LOGGER_HEALTH','SESSION_END'):
                        if writer.state!='OPEN': break
                        e=records.event(kind); e['payload']['reason']=result['reason']
                        writer.enqueue(e); writer.drain()
                cleanup('health_end_attempted',end_records)
            cleanup('writer_closed',writer.close)
            result['writer_state']=writer.state; result['writer_error']=writer.error
            result['known_saved_bytes']=result.get('manifest_bytes',0)+writer.bytes_written
            if writer.error:
                if result['first_error'] is None: result['first_error']=writer.error
                result['reason']='RECORDING_FAILED'; result['exit_code']=5
        if records: result['counters']={k:dict(ids=sorted(v),count=len(v)) for k,v in records.counts.items()}
        if core:
            result['operations']['fake_forward_calls' if fixture else 'fixed_forward_calls']=core.forward_calls
            result['forward_return_observed']=bool(records.counts['forward_returned'])
        if result['cleanup_errors'] and result['exit_code']==0: result['exit_code']=6
        end_finish=timing_sample('cleanup_end')
        timing_reason=None
        if end_start is None or end_finish is None: timing_reason='CLOCK_READ_FAILED'
        elif end_finish<end_start or (last_time is not None and end_start<last_time): timing_reason='CLOCK_REGRESSION'
        elif records and records.clock_epoch!=cleanup_epoch: timing_reason='CLOCK_EPOCH_CHANGED'
        result['shutdown_timing']=dict(start_ns=end_start,end_ns=end_finish,reason=timing_reason,
            start_epoch=cleanup_epoch,end_epoch=records.clock_epoch if records else None,
            duration_s=None if timing_reason else (end_finish-end_start)*1e-9)
        result['terminal_accounting_status']='UNKNOWN_CLEANUP_FAILED' if any(a['status']=='FAILED' and a['name']=='wrapper_stopped' for a in result['cleanup_attempts']) else ('NOT_CONSTRUCTED' if records is None else 'OBSERVED_COUNTER_SETS_NOT_DURABILITY')
        result['shutdown_grace_exceeded']=None if timing_reason else result['shutdown_timing']['duration_s']>limits['shutdown_grace_s']
        if (timing_reason or result['timing_errors']) and result['exit_code']==0: result['exit_code']=6
        if result['shutdown_grace_exceeded'] and result['exit_code']==0: result['exit_code']=6
        result['input_result']='NO_INPUT' if not records or not records.sequence else ('NO_FORWARD' if not core or not core.forward_calls else 'FORWARD_ATTEMPTED_NOT_PATH_VALIDITY')
    return result


def main(args: list[str] | None, *, wrapper_file: Path, wrapper_factory: Callable, schema_dir: Path,
         dependencies: object | None = None) -> int:
    parser=argparse.ArgumentParser(description='V4 input-only guarded bootstrap; no control outputs')
    parser.add_argument('--config'); parser.add_argument('--authorization'); parser.add_argument('--check-config',action='store_true')
    argv=list(sys.argv[1:] if args is None else args)
    if '--ros-args' in argv:
        split=argv.index('--ros-args'); tail=argv[split+1:]
        # launch_ros may add this redundant fixed node name. All topic/namespace/
        # parameter remaps remain forbidden and cannot evade the bound config.
        if tail not in ([],['-r','__node:=spatial_path_shadow_v4']):
            print(json.dumps(dict(status='CONFIG_ERROR',reason='ROS remaps/overrides not authorized')),file=sys.stderr)
            return 2
        argv=argv[:split]
    parsed=parser.parse_args(argv)
    if not parsed.config:
        print(json.dumps(dict(status='DEFAULT_DISABLED',reason='explicit --config and separate authorization required')))
        return 2
    try:
        config=load_config(Path(parsed.config))
        authorization=load_config(Path(parsed.authorization)) if parsed.authorization else None
        code=code_identity(wrapper_file)
        if parsed.check_config:
            report=check_config(config,code,authorization)
            print(json.dumps(report,ensure_ascii=False)); return 0 if report['valid'] and not report['unresolved'] else 2
        result=run(config,authorization,actual_code_id=code,schema_dir=schema_dir,wrapper_factory=wrapper_factory,dependencies=dependencies)
        print(json.dumps(result,ensure_ascii=False),file=sys.stderr if result['exit_code'] else sys.stdout)
        return result['exit_code']
    except (ValueError,OSError) as exc:
        print(json.dumps(dict(status='CONFIG_ERROR',reason=str(exc))),file=sys.stderr); return 2
