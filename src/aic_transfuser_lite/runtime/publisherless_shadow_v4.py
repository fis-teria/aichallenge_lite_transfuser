"""Publisherless V4 observation worker and independent shadow control clock.

Only caller-observed synchronized inputs enter the existing PASSIVE adapter.
No command-history feedback from shadow controls. No ROS imports or asset reads
at import time. Outer process supervision is required for a blocked forward.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
import math
import time
import numpy as np
from aic_transfuser_lite.control.path_control_bridge import Plan, PathPose, Vehicle, ShadowBridge


@dataclass(frozen=True)
class Envelope:
    session_id: str
    wall_s: float
    forward_limit: int | None
    candidate_limit: int
    authorized_until_unix_s: float
    log_bytes: int
    termination_policy: str = 'COUNT_BOUNDED'

    def validate(self, unix_s: float) -> None:
        timed = self.termination_policy == 'TIME_BOUNDED'
        valid_forward = (self.forward_limit is None if timed else
                         type(self.forward_limit) is int and 1 <= self.forward_limit <= 40)
        if (self.termination_policy not in ('COUNT_BOUNDED', 'TIME_BOUNDED') or
                not self.session_id or not valid_forward or type(self.candidate_limit) is not int or
                not 1 <= self.candidate_limit <= (10000 if timed else 200) or
                not math.isfinite(self.wall_s) or not 0 < self.wall_s <= (285 if timed else 115) or
                not math.isfinite(self.authorized_until_unix_s) or
                self.authorized_until_unix_s < unix_s+self.wall_s+5 or
                type(self.log_bytes) is not int or not 1024 <= self.log_bytes <= (64 if timed else 16)*1024**2):
            raise ValueError('FINITE_SHADOW_ENVELOPE_REQUIRED')


class ShadowSession:
    """No transport methods: injected adapter/infer are executed in owned child.

    Count-bounded trials retain the 115s cap. Explicit time-bounded shadow
    sessions allow 285s inside an independently enforced 300s host envelope.
    Count attempted forward BEFORE invoking model, including errors.
    """
    output_points = 20
    output_source = 'FIXED_V4_UNCORRECTED'

    def __init__(self, envelope: Envelope, adapter: object, infer: object,
                 bridge: ShadowBridge, emit: object, *, monotonic=time.monotonic,
                 unix=time.time, forward_permit=None):
        envelope.validate(unix())
        self.envelope, self.adapter, self.infer, self.bridge, self.emit = envelope, adapter, infer, bridge, emit
        self.monotonic, self.unix = monotonic, unix
        self.started = monotonic()
        self.forward_calls = self.candidates = 0
        self.terminal = None
        self.last_epoch = None
        self.seen: set[str] = set()
        self.forward_permit = forward_permit

    def active(self) -> bool:
        if self.terminal:
            return False
        if self.forward_permit is not None:
            self.forward_permit.check()
            if self.forward_permit.fault:
                self.terminal = self.forward_permit.fault
                self.bridge._invalidate(self.terminal)
                return False
        if self.monotonic()-self.started >= self.envelope.wall_s or self.unix() >= self.envelope.authorized_until_unix_s:
            self.terminal = 'SESSION_DEADLINE'
        elif self.envelope.forward_limit is not None and self.forward_calls >= self.envelope.forward_limit:
            self.terminal = 'FORWARD_LIMIT'
        elif self.candidates >= self.envelope.candidate_limit:
            self.terminal = 'CANDIDATE_LIMIT'
        return self.terminal is None

    def observation(self, obs: object, passive_commands: tuple, pose: PathPose | None,
                    *, finalized_ns: int, now_s: float) -> dict:
        if not self.active():
            return self._event('REJECTED', self.terminal)
        self.candidates += 1
        candidate_id = obs.sample_id
        if candidate_id in self.seen:
            return self._event('REJECTED', 'DUPLICATE_CANDIDATE', input_id=candidate_id)
        self.seen.add(candidate_id)
        if len(self.seen) > self.envelope.candidate_limit:
            self.terminal = 'CANDIDATE_LIMIT'
            return self._event('REJECTED', self.terminal)
        epoch = obs.camera.epoch
        if self.last_epoch != epoch:
            self.adapter.reset('EPOCH_CHANGE')
            self.bridge._invalidate('EPOCH_CHANGE')
            self.last_epoch = epoch
        try:
            status = self.adapter.append(obs, finalized_ns)
            if status != 'ACCEPTED':
                raise ValueError('INPUT_'+status)
            # Explicit source: never fabricated zero commands or own shadow result.
            for command in passive_commands:
                if command.source not in ('nominal', 'final_fallback'):
                    raise ValueError('PASSIVE_EXTERNAL_SOURCE_REQUIRED')
                if command.source == 'final_fallback' and not getattr(self.adapter, 'final_fallback_verified', False):
                    raise ValueError('FINAL_COMMAND_BINDING_UNVERIFIED')
                self.adapter.add_command(command)
            if not self.adapter.commands:
                raise ValueError('PASSIVE_COMMAND_MISSING')
            if self.forward_permit is not None and not self.forward_permit.check(epoch):
                return self._event('PREPARED', 'WAITING_FOR_START', input_id=candidate_id)
            if (self.forward_permit is not None and
                    obs.camera.received_ns < self.forward_permit.value['not_before_ns']):
                return self._event('PREPARED', 'PRE_START_OBSERVATION', input_id=candidate_id)
            batch, provenance = self.adapter.build(finalized_ns)
            if self.monotonic()-self.started >= self.envelope.wall_s:
                self.terminal = 'SESSION_DEADLINE'
                return self._event('REJECTED', self.terminal)
            self.forward_calls += 1
            output_id = f'{self.envelope.session_id}:{candidate_id}:forward:{self.forward_calls}'
            self._event('FORWARD_STARTED', None, input_id=candidate_id, output_id=output_id)
            before = self.monotonic()
            raw = np.asarray(self.infer(batch))
            generated = self.monotonic()
            if self.forward_permit is not None and not self.forward_permit.check(epoch):
                self.terminal = self.forward_permit.fault or 'START_PERMISSION_REVOKED'
                self.bridge._invalidate(self.terminal)
                return self._event('REJECTED', self.terminal, output_id=output_id)
            if raw.shape != (self.output_points, 2) or raw.dtype != np.float32 or not np.isfinite(raw).all():
                raise ValueError('OUTPUT_SHAPE_DTYPE_FINITE')
            # Result completed after deadline cannot become a fresh plan.
            if generated-self.started >= self.envelope.wall_s or self.unix() >= self.envelope.authorized_until_unix_s:
                self.terminal = 'LATE_FORWARD'
                return self._event('REJECTED', self.terminal, output_id=output_id)
            source_s = obs.camera.header_ns*1e-9
            clock = obs.camera.clock_id
            # Transport bound pose is observation keyed; re-key only to this same forward.
            if pose is not None:
                if pose.plan_id != candidate_id:
                    raise ValueError('POSE_CANDIDATE_ID_MISMATCH')
                from dataclasses import replace
                pose = replace(pose, plan_id=output_id)
            ttl = self.bridge.limits.path_ttl_s if self.bridge.limits else 0.
            plan = Plan(output_id, self.output_source, source_s, generated,
                        source_s+ttl, clock, epoch, 'base_link', 'BASE_LINK_ORIGIN',
                        tuple(tuple(float(v) for v in xy) for xy in raw))
            accepted = self.bridge.accept(plan, pose, now_s=now_s)
            return self._event('PLAN', self.bridge.reason, input_id=candidate_id, output_id=output_id,
                               accepted=accepted, raw_xy_m=raw.tolist(), source_s=source_s,
                               source=plan.source, clock=clock, epoch=epoch,
                               frame=plan.frame, reference_point=plan.reference_point,
                               expires_s=plan.expires_s,
                               observation_pose_evidence=None if pose is None else pose.evidence,
                               observation_pose_xyyaw=None if pose is None else list(pose.base_in_local),
                               generated_monotonic_s=generated, inference_s=generated-before,
                               command_policy=provenance.get('command_policy'),
                               command_provenance=[asdict(c) if c is not None else None
                                                   for c in provenance.get('commands', [])],
                               control_publish_count=0, gear_publish_count=0, mode_publish_count=0)
        except Exception as exc:
            self.bridge._invalidate('INPUT_OR_FORWARD_REJECTED')
            return self._event('REJECTED', type(exc).__name__+':'+str(exc), input_id=candidate_id)

    def control_tick(self, now_s: float, clock: str, epoch: str, state: Vehicle | None) -> dict:
        if not self.active():
            self.bridge._invalidate(self.terminal)
        value = self.bridge.tick(now_s=now_s, clock=clock, epoch=epoch, state=state)
        return self._event('SHADOW_CONTROL', value.get('reason'), value=value)

    def _event(self, kind, reason, **fields):
        record = dict(event=kind, reason=reason, session_id=self.envelope.session_id,
                      forward_calls=self.forward_calls, candidates=self.candidates, **fields)
        self.emit(record)
        return record


def inference_device_metadata(model):
    """Actual loaded parameter device; no extra forward or GPU synchronization."""
    import torch
    device = next(model.parameters()).device
    return dict(model_device=str(device), torch_version=str(torch.__version__),
                cuda_build=torch.version.cuda,
                gpu_name=torch.cuda.get_device_name(device) if device.type == 'cuda' else None,
                model_loaded_cuda_allocated_bytes=torch.cuda.memory_allocated(device) if device.type == 'cuda' else 0,
                memory_scope='PYTORCH_ALLOCATOR_AFTER_MODEL_LOAD_NOT_PEAK_OR_WHOLE_GPU')


def fixed_infer_factory():
    """Call only inside explicitly admitted real-input child; exact existing loader."""
    import torch
    from .spatial_runtime_v4 import load_fixed, SpatialRuntimeV4
    from .spatial_input_v4 import freeze_batch
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model, identity = load_fixed(device)
    model.eval()
    identity['execution_device'] = inference_device_metadata(model)
    def infer(batch):
        with torch.inference_mode():
            result = model(freeze_batch(batch, device))
        return SpatialRuntimeV4.snapshot(result)
    return infer, identity


def create_input_only_node(node_factory, message_types: dict, topics: dict, callback,
                           *, command_binding=None):
    """Auditable ROS construction boundary. No publishers, clients, actions, timers.

    ROS middleware-internal endpoints still require independent verification.
    Sensor synchronizer is caller-supplied and must provide real provenance.
    """
    command_role = 'command' if command_binding is not None else 'nominal'
    roles = ('image', 'lidar', 'velocity', 'steering', command_role, 'odometry', 'clock')
    if set(topics) != set(roles) or any(not isinstance(t, str) or not t.startswith('/') for t in topics.values()):
        raise ValueError('EXPLICIT_PASSIVE_TOPICS_REQUIRED')
    if command_binding is not None:
        command_binding.validate()
        if topics['command'] != command_binding.topic:
            raise ValueError('COMMAND_TOPIC_BINDING_MISMATCH')
    # Factory must independently attest middleware-internal endpoints: rclpy's
    # parameter-event behavior varies by version. No invented disable keyword.
    node = node_factory('v4_publisherless_shadow', enable_rosout=False,
                        start_parameter_services=False)
    try:
        for role in roles:
            node.create_subscription(message_types[role], topics[role],
                                     lambda message, r=role: callback(r, message),
                                     message_types['qos'][role])
    except Exception:
        node.destroy_node()
        raise
    return node
