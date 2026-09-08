"""Shadow inference admission only. Never authorizes or sends vehicle commands.

The helper receipt is trusted local supervisor evidence, NOT authentication.
Bool topics are latched: require a session-observed false -> true edge, then
retain state while graph monitoring is fresh. Do not invent Bool heartbeats.
"""
from pathlib import Path
import json
import os
import time

LEASE_NS = 500_000_000


def save_helper_completion(path: Path, receipt: dict) -> None:
    """Supervisor handoff AFTER wait() returns zero; no helper invocation here.

    Caller supplies actual same-host monotonic times and selected instance ID.
    Unique run path, atomic publication; existing receipts are never overwritten.
    No periodic rewriting of a successful receipt is required or permitted.
    """
    verifier = StartGate(receipt['session_id'], receipt['instance_id'], receipt['helper_started_ns'])
    verifier.complete(receipt, time.monotonic_ns())
    if verifier.fault: raise ValueError(verifier.fault)
    raw = json.dumps(receipt, allow_nan=False).encode()
    if len(raw) > 4096: raise ValueError('HELPER_RECEIPT_SIZE')
    temporary = path.with_name(path.name+'.pending')
    with temporary.open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    # link is atomic and refuses an existing target (unlike os.replace).
    os.link(temporary, path)
    temporary.unlink()


class StartGate:
    def __init__(self, session_id: str, instance_id: str, started_ns: int):
        self.session_id, self.instance_id, self.started_ns = session_id, instance_id, started_ns
        self.false_ns = self.true_ns = self.init_ns = None
        self.initialized = False
        self.receipt = None
        self.opened = False
        self.opened_ns = None
        self.fault = None

    def invalidate(self, reason: str) -> None:
        self.fault = self.fault or reason

    def observe(self, role: str, value: bool, now_ns: int) -> None:
        if self.fault: return
        if type(value) is not bool or now_ns < self.started_ns:
            self.invalidate('START_EVIDENCE_INVALID'); return
        if role == 'initialization':
            self.initialized = value
            if value: self.init_ns = now_ns
            elif self.opened: self.invalidate('INITIALIZATION_REVOKED')
        elif role == 'arm':
            if not value:
                if self.opened or self.true_ns is not None:
                    self.invalidate('RACE_ARM_REVOKED')
                self.false_ns = now_ns
            elif self.false_ns is not None:
                self.true_ns = now_ns
        else: self.invalidate('START_ROLE_INVALID')

    def complete(self, receipt: dict, now_ns: int) -> None:
        """Only a successful existing helper may produce this session receipt."""
        try:
            start, end = receipt['helper_started_ns'], receipt['helper_completed_ns']
            valid = (receipt['session_id'] == self.session_id and
                     receipt['instance_id'] == self.instance_id and receipt['epoch'] == '0' and
                     type(receipt['exit_code']) is int and receipt['exit_code'] == 0 and
                     type(start) is int and type(end) is int and
                     self.started_ns <= start <= end <= now_ns)
            if not valid: raise ValueError('identity/time/status')
            if self.receipt is not None and self.receipt != receipt:
                raise ValueError('receipt changed')
            self.receipt = dict(receipt)
        except (KeyError, TypeError, ValueError):
            self.invalidate('HELPER_RECEIPT_INVALID')

    def permit(self, now_ns: int, epoch: str, graph_checked_ns: int | None) -> dict | None:
        if epoch != '0': self.invalidate('START_EPOCH_CHANGED')
        if graph_checked_ns is None or not 0 <= now_ns-graph_checked_ns < LEASE_NS:
            self.invalidate('START_GRAPH_STALE')
        if self.fault: return None
        if not (self.receipt and self.initialized and self.false_ns is not None and
                self.true_ns is not None and self.init_ns is not None): return None
        start = self.receipt['helper_started_ns']
        if not (self.false_ns <= start <= self.true_ns <= now_ns): return None
        self.opened = True
        if self.opened_ns is None: self.opened_ns = now_ns
        return dict(session_id=self.session_id, epoch=epoch, issued_ns=now_ns,
                    expires_ns=min(now_ns+LEASE_NS, graph_checked_ns+LEASE_NS),
                    not_before_ns=self.opened_ns, phase='RUN_SHADOW')


class ForwardPermit:
    """Worker-side bounded lease. Expiry after opening is terminal, not fallback."""
    def __init__(self, session_id: str, monotonic_ns=time.monotonic_ns):
        self.session_id, self.monotonic_ns = session_id, monotonic_ns
        self.value = None
        self.opened = False
        self.fault = None

    def update(self, value: dict | None) -> None:
        if self.opened and value is None: self.fault = 'START_PERMISSION_REVOKED'
        self.value = value

    def check(self, epoch: str | None = None) -> bool:
        if self.fault: return False
        if self.value is None: return False
        p, now = self.value, self.monotonic_ns()
        try:
            if not (p['session_id'] == self.session_id and p['epoch'] == '0' and
                    (epoch is None or epoch == p['epoch']) and p['phase'] == 'RUN_SHADOW' and
                    type(p['issued_ns']) is int and type(p['expires_ns']) is int and
                    type(p['not_before_ns']) is int and p['not_before_ns'] <= p['issued_ns'] and
                    p['issued_ns'] <= now < p['expires_ns'] <= p['issued_ns']+LEASE_NS):
                raise ValueError('lease')
        except (KeyError, TypeError, ValueError):
            self.fault = 'START_PERMISSION_EXPIRED_OR_INVALID'; return False
        self.opened = True
        return True


class ROSStartObserver:
    """Read-only ROS Bool subscriptions plus bounded local helper receipt read.

    No per-message publisher authentication; isolated instance is a prerequisite.
    Receipt path must be unique and absent at startup, shared with the supervisor.
    """
    def __init__(self, node, config: dict, session_id: str, emit):
        from std_msgs.msg import Bool
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.node, self.config, self.emit = node, config, emit
        self.path = Path(config['receipt_file'])
        if self.path.exists(): raise ValueError('START_RECEIPT_ALREADY_EXISTS')
        self.gate = StartGate(session_id, config['instance_id'], time.monotonic_ns())
        self.checked_ns = None
        self.subscriptions = []
        self.timer = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        try:
            self.check_graph()
            if self.gate.fault: raise ValueError(self.gate.fault)
            for role, topic in config['topics'].items():
                self.subscriptions.append(node.create_subscription(
                    Bool, topic, lambda m, r=role: self.receive(r, m.data), qos))
            self.timer = node.create_timer(.1, self.check_graph)
        except Exception:
            self.close(); raise

    def check_graph(self) -> None:
        now = time.monotonic_ns()
        if self.checked_ns is not None and not 0 <= now-self.checked_ns < LEASE_NS:
            self.gate.invalidate('START_GRAPH_STALE'); return
        try:
            for role, topic in self.config['topics'].items():
                endpoints = self.node.get_publishers_info_by_topic(topic)
                if len(endpoints) != 1: raise ValueError('publisher count:'+role)
                e = endpoints[0]
                if e.node_namespace.rstrip('/')+'/'+e.node_name != self.config['expected_node']:
                    raise ValueError('publisher node:'+role)
            if time.monotonic_ns()-now >= LEASE_NS: raise ValueError('graph query timeout')
            self.checked_ns = now
        except Exception as exc:
            self.gate.invalidate('START_GRAPH_INVALID:'+str(exc))

    def receive(self, role: str, value: bool) -> None:
        now = time.monotonic_ns()
        if self.checked_ns is None or not 0 <= now-self.checked_ns < LEASE_NS:
            self.gate.invalidate('START_GRAPH_STALE'); return
        self.gate.observe(role, value, now)
        self.emit(dict(event='START_EVIDENCE', role=role, value=value, received_ns=now))

    def poll(self, epoch: str) -> dict | None:
        if self.gate.receipt is None:
            try:
                with self.path.open('rb') as stream: raw = stream.read(4097)
                if len(raw) > 4096: raise ValueError('receipt size')
                value = json.loads(raw)
                self.gate.complete(value, time.monotonic_ns())
                self.emit(dict(event='HELPER_RECEIPT', value=value))
            except FileNotFoundError: pass
            except (OSError, ValueError, TypeError): self.gate.invalidate('HELPER_RECEIPT_READ_FAILED')
        return self.gate.permit(time.monotonic_ns(), epoch, self.checked_ns)

    def close(self) -> None:
        if self.timer is not None: self.node.destroy_timer(self.timer)
        for sub in self.subscriptions: self.node.destroy_subscription(sub)
        self.subscriptions.clear()
