"""ROS-free, single-in-flight delivery. No inference or control authority.

Only the parent owns this buffer. A worker acknowledgement releases its credit;
there is never a FIFO of obsolete tick/command snapshots behind a slow forward.
"""
from collections import deque
import queue
from typing import Callable

INPUT_WAIT_NS = 300_000_000  # same observation-join deadline, never extended


def dropped(item: dict, emit: Callable, reason: str) -> None:
    message = item['message']
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None) if header is not None else getattr(message, 'stamp', None)
    emit(dict(event='DELIVERY_DROPPED', reason=reason, role=item['role'],
              epoch=item['epoch'], received_ns=item['received_ns'],
              header_ns=None if stamp is None else stamp.sec*1_000_000_000+stamp.nanosec))


class DeliveryPump:
    """64 pending sensor events, at most one outstanding batch, 50 Hz maximum.

    Clock monitoring remains in the parent transport. No latest-only replacement
    of useful scan/pose brackets; only already-expired input is evicted. A burst
    exceeding the finite capacity before expiry remains a terminal fault.
    """
    def __init__(self, incoming, emit: Callable, *, capacity: int = 64,
                 period_ns: int = 20_000_000):
        if type(capacity) is not int or not 1 <= capacity <= 64 or not 0 < period_ns <= 20_000_000:
            raise ValueError('DELIVERY_BOUNDS')
        self.incoming, self.emit = incoming, emit
        self.capacity, self.period_ns = capacity, period_ns
        self.pending = deque()
        self.inflight = None
        self.sequence = 0
        self.last_sent_ns = None
        self.last_received_ns = None
        self.closed = False

    def expire(self, now_ns: int) -> None:
        while self.pending and now_ns-self.pending[0]['received_ns'] >= INPUT_WAIT_NS:
            dropped(self.pending.popleft(), self.emit, 'DELIVERY_DEADLINE')

    def accept(self, role: str, message: object, received_ns: int, epoch: str) -> None:
        if self.closed: return
        if role == 'clock': return  # join ignores this; parent still validates it
        if role not in ('image', 'lidar', 'velocity', 'steering', 'odometry'):
            raise ValueError('DELIVERY_ROLE')
        if self.last_received_ns is not None and received_ns < self.last_received_ns:
            raise ValueError('DELIVERY_RECEIPT_ORDER')
        self.last_received_ns = received_ns
        self.expire(received_ns)
        if len(self.pending) >= self.capacity:
            raise RuntimeError('INPUT_QUEUE_FULL')
        self.pending.append(dict(kind='input', role=role, message=message,
                                 received_ns=received_ns, epoch=epoch))

    def dispatch(self, now_ns: int, now_ros_s: float | None, commands: Callable,
                 *, forward_permit: dict | None = None) -> bool:
        if self.closed: return False
        self.expire(now_ns)
        if (self.inflight is not None or now_ros_s is None or
                (self.last_sent_ns is not None and now_ns-self.last_sent_ns < self.period_ns)):
            return False
        batch = dict(kind='batch', batch_id=self.sequence, inputs=list(self.pending),
                     commands=commands(), sent_ns=now_ns, now_s=now_ros_s,
                     forward_permit=forward_permit)
        try: self.incoming.put_nowait(batch)
        except queue.Full as exc: raise RuntimeError('DELIVERY_CREDIT_MISMATCH') from exc
        self.pending.clear()
        self.inflight = self.sequence
        self.sequence += 1
        self.last_sent_ns = now_ns
        self.emit(dict(event='DELIVERY_SENT', batch_id=self.inflight,
                       inputs=len(batch['inputs']), command_entries=len(batch['commands']), sent_ns=now_ns))
        return True

    def acknowledge(self, record: dict) -> None:
        if self.closed: return
        if self.inflight is None or record.get('batch_id') != self.inflight:
            raise RuntimeError('DELIVERY_ACK_MISMATCH')
        self.inflight = None

    def close(self, reason: str) -> None:
        self.closed = True
        while self.pending: dropped(self.pending.popleft(), self.emit, reason)
        self.inflight = None


def consume_batch(batch: dict, join, emit: Callable, monotonic_ns: Callable) -> dict:
    """Use actual worker time, never a queued tick's historical cutoff.

    The caller binds batch commands before this call. Original sensor receipt,
    header and command availability timestamps remain unchanged.
    """
    started = monotonic_ns()
    if started < batch['sent_ns']: raise ValueError('DELIVERY_CLOCK_ORDER')
    accepted = 0
    for item in batch['inputs']:
        age = monotonic_ns()-item['received_ns']
        if age < 0: raise ValueError('DELIVERY_FUTURE_RECEIPT')
        if age >= INPUT_WAIT_NS:
            dropped(item, emit, 'DELIVERY_DEADLINE'); continue
        join.on_input(item['role'], item['message'], item['received_ns'], item['epoch'])
        accepted += 1
    # One candidate at most: another slow forward must obtain a new cutoff.
    join.tick(monotonic_ns(), batch['now_s'])
    return dict(event='BATCH_CONSUMED', batch_id=batch['batch_id'], accepted=accepted,
                queue_wait_ns=started-batch['sent_ns'], worker_ns=monotonic_ns()-started)
