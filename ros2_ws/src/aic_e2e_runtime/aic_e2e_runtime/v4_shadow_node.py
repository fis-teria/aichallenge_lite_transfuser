"""Standard ROS2 entry point: passive subscriptions -> bounded V4 worker.

No vehicle publishers. Parent keeps graph monitoring alive during inference.
"""
from pathlib import Path
import json
import multiprocessing as mp
import queue
import time

# Reuse the package's canonical-source/install-layout resolution (no ROS init).
from . import spatial_path_shadow_node_v4 as _source_layout


def validate_config(c: dict) -> None:
    from aic_transfuser_lite.runtime.publisherless_shadow_v4 import Envelope
    from aic_transfuser_lite.runtime.passive_controller_command_v4 import ControllerCommandBinding
    if c.get('enabled') is not True: raise ValueError('V4_SHADOW_DISABLED')
    gate = c.get('start_gate')
    policy = c.get('inference_start_policy', 'OFFICIAL_HELPER_AND_RACE_ARM' if gate else 'INPUT_READY_SHADOW')
    if policy not in ('INPUT_READY_SHADOW', 'OFFICIAL_HELPER_AND_RACE_ARM'):
        raise ValueError('UNKNOWN_INFERENCE_START_POLICY')
    if (policy == 'OFFICIAL_HELPER_AND_RACE_ARM') != (gate is not None):
        raise ValueError('INFERENCE_START_POLICY_MISMATCH')
    if gate is not None:
        if (gate.get('policy') != 'OFFICIAL_HELPER_AND_RACE_ARM' or
                not Path(gate.get('receipt_file', '')).is_absolute() or
                gate.get('instance_id') in (None, '', 'UNKNOWN') or
                gate.get('expected_node') != '/autostart_orchestrator' or
                gate.get('topics') != {'arm': '/overtake/race_armed',
                                      'initialization': '/autostart/initialization_ready'}):
            raise ValueError('EXPLICIT_START_GATE_REQUIRED')
    Envelope(**c['envelope']).validate(time.time())
    binding=ControllerCommandBinding(**c['command_binding']);binding.validate()
    roles={'image','lidar','velocity','steering','command','odometry','clock'}
    if (set(c['topics'])!=roles or set(c['expected_nodes'])!=roles or
            c['topics']['command']!=binding.topic or c['expected_nodes']['command']!=binding.producer_id or
            not c['pose_frame'] or not c['pose_evidence'] or c['pose_evidence']=='UNKNOWN' or
            not c['output_file']): raise ValueError('EXPLICIT_LIVE_BINDINGS_REQUIRED')


def worker(config: dict, incoming, outgoing) -> None:
    """Owned child: assets loaded here only, finite queue, no ROS context."""
    from aic_transfuser_lite.runtime.publisherless_shadow_v4 import Envelope, ShadowSession, fixed_infer_factory
    from aic_transfuser_lite.runtime.spatial_input_v4 import SpatialInputV4
    from aic_transfuser_lite.runtime.shadow_observation_join_v4 import ShadowObservationJoin
    from aic_transfuser_lite.control.path_control_bridge import ShadowBridge
    from aic_transfuser_lite.runtime.shadow_delivery_v4 import consume_batch
    from aic_transfuser_lite.runtime.shadow_start_gate_v4 import ForwardPermit
    validate_config(config)
    infer,identity=fixed_infer_factory()
    def emit(value): outgoing.put(value,timeout=.1)
    emit(dict(event='MODEL_LOADED',identity=identity))
    adapter=SpatialInputV4(command_binding_known=True,
        final_fallback_verified=config['command_binding']['source']=='final_fallback')
    permit = ForwardPermit(config['envelope']['session_id']) if config.get('start_gate') else None
    session=ShadowSession(Envelope(**config['envelope']),adapter,infer,ShadowBridge(None),emit,
                          forward_permit=permit)
    commands=()
    join=ShadowObservationJoin(session,lambda:commands,emit,clock_id='AWSIM_ROS',
         monotonic_id='HOST_MONOTONIC',pose_frame=config['pose_frame'],pose_evidence=config['pose_evidence'])
    while session.active():
        try: item=incoming.get(timeout=.05)
        except queue.Empty: continue
        if item['kind']!='batch': raise ValueError('DELIVERY_PROTOCOL')
        if permit is not None: permit.update(item.get('forward_permit'))
        commands=item['commands']
        emit(consume_batch(item,join,emit,time.monotonic_ns))
    emit(dict(event='WORKER_FINISHED',reason=session.terminal))


def wait_model_ready(process, outgoing, emit, spin, expired) -> None:
    """No input subscriptions exist during bounded model loading.

    Loading time consumes the parent's original deadline. No old sensor queue
    is replayed or restamped when the worker becomes ready.
    """
    while True:
        if expired(): raise RuntimeError('MODEL_STARTUP_DEADLINE')
        try: record=outgoing.get(timeout=.02)
        except queue.Empty:
            if not process.is_alive():
                raise RuntimeError('MODEL_STARTUP_EXIT:'+str(process.exitcode))
            spin()
            continue
        if expired(): raise RuntimeError('MODEL_STARTUP_DEADLINE')
        if record.get('event')!='MODEL_LOADED':
            raise RuntimeError('MODEL_STARTUP_PROTOCOL')
        emit(record)
        return


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from aic_transfuser_lite.runtime.shadow_ros2_transport_v4 import ShadowROS2Transport,ros2_types
    from aic_transfuser_lite.runtime.passive_controller_command_v4 import ControllerCommandBinding
    from aic_transfuser_lite.runtime.shadow_delivery_v4 import DeliveryPump
    rclpy.init(args=args)
    node=Node('v4_shadow',enable_rosout=False,start_parameter_services=False)
    node.declare_parameter('config_file','')
    process=transport=stream=pump=start_observer=None
    ctx=mp.get_context('spawn');incoming=ctx.Queue(maxsize=1);outgoing=ctx.Queue(maxsize=64)
    terminal=None
    try:
        config=json.loads(Path(node.get_parameter('config_file').value).read_text())
        validate_config(config)
        stream=Path(config['output_file']).open('x',encoding='utf-8')
        used=0;started=time.monotonic();limit=config['envelope']['wall_s']
        def emit(record):
            nonlocal used,terminal
            text=json.dumps(record,allow_nan=False,default=str)+'\n'
            used+=len(text.encode())
            if used>config['envelope']['log_bytes']: raise RuntimeError('LOG_LIMIT')
            stream.write(text);stream.flush()
        pump=DeliveryPump(incoming,emit)
        def receive(role,message,received_ns,epoch):
            nonlocal terminal
            try: pump.accept(role,message,received_ns,epoch)
            except (ValueError,RuntimeError) as exc: terminal=str(exc)
        def on_reset(reason,epoch):
            # Fail closed for this finite session, including clock reset. No old
            # in-flight child result is accepted once parent has invalidated it.
            nonlocal terminal
            terminal=reason
            pump.close(reason)
        types=ros2_types()
        process=ctx.Process(target=worker,args=(config,incoming,outgoing));process.start()
        wait_model_ready(process,outgoing,emit,
            lambda:rclpy.spin_once(node,timeout_sec=.02),
            lambda: not rclpy.ok() or time.monotonic()-started>=limit or
                time.time()>=config['envelope']['authorized_until_unix_s'])
        if config.get('start_gate'):
            from aic_transfuser_lite.runtime.shadow_start_gate_v4 import ROSStartObserver
            start_observer=ROSStartObserver(node,config['start_gate'],config['envelope']['session_id'],emit)
            emit(dict(event='PREPARE_STARTED',forward_enabled=False))
        transport=ShadowROS2Transport(node,types,config['topics'],
            {r:qos_profile_sensor_data if r in ('image','lidar') else 10 for r in types},
            ControllerCommandBinding(**config['command_binding']),config['expected_nodes'],
            receive,on_reset,emit,
            clock_id='AWSIM_ROS',monotonic_id='HOST_MONOTONIC')
        emit(dict(event='SESSION_STARTED',source_verification='GRAPH_SINGLE_PUBLISHER_NOT_PER_MESSAGE',
                  control_publish=False,config=config))
        while rclpy.ok() and terminal is None:
            if time.monotonic()-started>=limit or time.time()>=config['envelope']['authorized_until_unix_s']:
                terminal='SESSION_DEADLINE';break
            rclpy.spin_once(node,timeout_sec=.02)
            if terminal: break
            # Preserve per-loop freshness admission without repeatedly copying
            # the 64-command history while the worker owns the only credit.
            if not transport._fresh(): break
            permit = None
            if start_observer:
                was_open = start_observer.gate.opened
                permit = start_observer.poll(str(transport.epoch))
                if start_observer.gate.fault:
                    terminal=start_observer.gate.fault;break
                if not was_open and permit:
                    emit(dict(event='RUN_SHADOW_STARTED',permit=permit))
            for _ in range(64):
                try: result=outgoing.get_nowait()
                except queue.Empty: break
                # Recheck independently before accepting each in-flight PLAN.
                if start_observer and result.get('event')=='PLAN':
                    if start_observer.poll(str(transport.epoch)) is None:
                        terminal=start_observer.gate.fault or 'START_PERMISSION_MISSING';break
                if result.get('event')=='BATCH_CONSUMED':
                    try: pump.acknowledge(result)
                    except RuntimeError as exc: terminal=str(exc);break
                emit(result)
                if result.get('event')=='WORKER_FINISHED': terminal=result['reason']
            if not process.is_alive() and terminal is None: terminal='WORKER_EXIT:'+str(process.exitcode)
            if terminal: break
            def snapshot():
                commands=transport.command_snapshot()
                if terminal: raise RuntimeError(terminal)
                return commands
            try:
                pump.dispatch(time.monotonic_ns(),
                    None if transport.last_ros_ns is None else transport.last_ros_ns*1e-9,snapshot,
                    forward_permit=permit)
            except (ValueError,RuntimeError) as exc: terminal=str(exc)
        pump.close(terminal or 'SESSION_CLOSED')
        emit(dict(event='SESSION_END',reason=terminal,control_publish=False))
    finally:
        if pump: pump.close(terminal or 'SESSION_CLOSED')
        if transport: transport.close()
        if start_observer: start_observer.close()
        if process:
            process.join(timeout=.2)
            if process.is_alive(): process.terminate();process.join(timeout=2)
            if process.is_alive(): process.kill();process.join(timeout=2)
        for q in (incoming,outgoing): q.cancel_join_thread();q.close()
        if stream: stream.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
