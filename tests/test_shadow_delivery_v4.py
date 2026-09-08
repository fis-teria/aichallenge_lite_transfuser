"""Synthetic traffic only: no ROS, sensor assets, model or controller."""
import json
import multiprocessing as mp
import queue
from types import SimpleNamespace as O

import pytest
from aic_transfuser_lite.runtime.shadow_delivery_v4 import DeliveryPump, consume_batch, INPUT_WAIT_NS


def message(n=1):
    return O(header=O(stamp=O(sec=1,nanosec=n)))


def test_blocked_worker_does_not_accumulate_clock_or_tick_snapshots(tmp_path):
    q=queue.Queue(maxsize=1);events=[];pump=DeliveryPump(q,events.append)
    snapshots=[]
    def commands(): snapshots.append(1);return ('same-source-command',)
    old_depth=0;old_full_at=None;peak=0
    # Artificial 500 ms blocked forward; input frequencies explicitly declared.
    for ms in range(0,501,5):
        roles=['clock']
        if ms%100==0: roles.append('image')
        if ms%50==0: roles.append('lidar')
        if ms%35==0: roles.extend(['velocity','steering'])
        if ms%20==0: roles.append('odometry')
        for role in roles:
            # Old path: one input and one tick after every callback.
            old_depth+=2
            if old_depth>64 and old_full_at is None: old_full_at=ms
            pump.accept(role,message(ms),ms*1_000_000,'0')
            pump.dispatch(ms*1_000_000,ms/1000,commands)
        peak=max(peak,len(pump.pending))
    assert old_full_at is not None and old_full_at<500
    assert pump.sequence==1 and len(snapshots)==1 and q.qsize()==1
    assert peak<=64
    drops=[r for r in events if r['event']=='DELIVERY_DROPPED']
    assert any(r['role']=='image' for r in drops)
    assert all(r['reason']=='DELIVERY_DEADLINE' for r in drops)
    first=q.get_nowait();pump.acknowledge({'batch_id':first['batch_id']})
    assert pump.dispatch(501_000_000,.501,commands)
    second=q.get_nowait()
    assert all(501_000_000-i['received_ns']<INPUT_WAIT_NS for i in second['inputs'])
    assert len(snapshots)==2 and all(i['role']!='clock' for i in second['inputs'])
    trace=dict(kind='SYNTHETIC_NOT_AWSIM',blocked_ms=500,old_full_ms=old_full_at,
               old_enqueues=old_depth,new_batches_before_ack=1,pending_peak=peak,drops=drops)
    (tmp_path/'traffic_trace.json').write_text(json.dumps(trace,indent=2))
    print(json.dumps({k:v for k,v in trace.items() if k!='drops'}))


def test_no_resampling_restamping_or_generation_mixing():
    q=queue.Queue();events=[];pump=DeliveryPump(q,events.append)
    m=message(42);pump.accept('image',m,100,'epoch-A')
    command=O(source='nominal',available_ns=90,header_ns=80)
    pump.dispatch(110,1.,lambda:(command,));batch=q.get_nowait()
    assert batch['inputs'][0]['message'] is m
    assert batch['inputs'][0]['received_ns']==100 and batch['inputs'][0]['epoch']=='epoch-A'
    assert batch['commands']==(command,) and command.available_ns==90
    pump.accept('lidar',message(),120,'epoch-A');pump.close('CLOCK_RESET')
    assert not pump.dispatch(20_000_111,1.,lambda:()) and not pump.pending
    pump.acknowledge({'batch_id':0})  # cannot revive closed session
    assert pump.closed and not pump.dispatch(40_000_111,2.,lambda:())
    assert events[-1]['reason']=='CLOCK_RESET'


def test_rate_credit_and_real_overload_remain_bounded():
    q=queue.Queue(maxsize=1);pump=DeliveryPump(q,lambda r:None)
    pump.dispatch(0,0.,lambda:());batch=q.get_nowait()
    pump.acknowledge({'batch_id':batch['batch_id']})
    assert not pump.dispatch(19_999_999,0.,lambda:())
    assert pump.dispatch(20_000_000,0.,lambda:())
    with pytest.raises(RuntimeError,match='ACK_MISMATCH'): pump.acknowledge({'batch_id':0})
    for i in range(64): pump.accept('lidar',message(i),i,'0')
    with pytest.raises(RuntimeError,match='INPUT_QUEUE_FULL'): pump.accept('lidar',message(),65,'0')
    pump.expire(INPUT_WAIT_NS+65)
    assert not pump.pending


def test_expired_batch_uses_actual_worker_time_not_sent_tick():
    events=[];calls=[];ticks=[]
    join=O(on_input=lambda *a:calls.append(a),tick=lambda *a:ticks.append(a))
    batch=dict(batch_id=4,sent_ns=100,now_s=1.,inputs=[
        dict(role='image',message=message(),received_ns=100,epoch='0')])
    result=consume_batch(batch,join,events.append,lambda:INPUT_WAIT_NS+100)
    assert not calls and events[0]['reason']=='DELIVERY_DEADLINE'
    assert ticks==[(INPUT_WAIT_NS+100,1.)]  # no fabricated simulation time
    assert result['accepted']==0 and result['queue_wait_ns']==INPUT_WAIT_NS


def test_worker_preserves_supported_input_and_rejects_future_receipt():
    calls=[];join=O(on_input=lambda *a:calls.append(a),tick=lambda *a:None)
    item=dict(role='odometry',message=message(),received_ns=100,epoch='0')
    batch=dict(batch_id=0,sent_ns=110,now_s=4.,inputs=[item])
    assert consume_batch(batch,join,lambda r:None,lambda:120)['accepted']==1
    assert calls[0][2:]==(100,'0')
    with pytest.raises(ValueError,match='CLOCK_ORDER'):
        consume_batch(batch,join,lambda r:None,lambda:90)
    batch['sent_ns']=80
    with pytest.raises(ValueError,match='FUTURE_RECEIPT'):
        consume_batch(batch,join,lambda r:None,lambda:90)


def _spawn_receiver(incoming,outgoing,release):
    item=incoming.get(timeout=5)
    if not release.wait(5): raise RuntimeError('fixture release timed out')
    outgoing.put({'batch_id':item['batch_id'],'received_ns':item['inputs'][0]['received_ns'],
                  'header_ns':item['inputs'][0]['message'].header.stamp.nanosec})


def test_real_spawn_ipc_keeps_one_credit_and_original_metadata():
    ctx=mp.get_context('spawn');incoming=ctx.Queue(1);outgoing=ctx.Queue(1);release=ctx.Event()
    process=ctx.Process(target=_spawn_receiver,args=(incoming,outgoing,release))
    pump=DeliveryPump(incoming,lambda r:None)
    try:
        process.start();pump.accept('image',message(42),100,'0')
        assert pump.dispatch(110,1.,lambda:())
        for i in range(100):
            assert not pump.dispatch(1000+i*20_000_000,1.,lambda:())
        release.set();ack=outgoing.get(timeout=5)
        assert ack['received_ns']==100 and ack['header_ns']==42
        pump.acknowledge(ack);process.join(timeout=5);assert process.exitcode==0
    finally:
        release.set()
        if process.is_alive(): process.terminate();process.join(timeout=2)
        for q in (incoming,outgoing): q.close();q.join_thread()
