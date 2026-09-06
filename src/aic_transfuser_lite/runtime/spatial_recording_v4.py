"""Bounded private recording for V4; no middleware or vehicle control imports."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

import numpy as np
import torch

from .spatial_input_v4 import INPUT_FIELDS, INPUT_ID

GRID = [float(f'{i/10:.1f}') for i in range(1,21)]


def sha(blob: bytes) -> str:
    return hashlib.sha256(blob).hexdigest()


def encoded(value: object) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')


def identity(value: str | None = None, reason: str = 'NOT_OBSERVED') -> dict:
    return {'status':'KNOWN' if value else 'MISSING','sha256':value,'reason':reason}


def stamp(ns: int | None = None, *, domain: str = 'MONOTONIC', clock: str = 'harness_process', epoch: str = '0', source: str = 'harness') -> dict:
    return dict(status='KNOWN' if ns is not None else 'MISSING', ns=str(ns) if ns is not None else None,
                domain=domain if ns is not None else 'UNKNOWN', clock_id=clock if ns is not None else None,
                epoch_id=epoch if ns is not None else None, source=source, reason='OBSERVED' if ns is not None else 'NOT_OBSERVED')


def descriptor_preimage(items: list[dict]) -> bytes:
    if [d['field'] for d in items] != list(INPUT_FIELDS):
        raise ValueError('descriptor order/duplicate')
    return ('v4-input-snapshot-v2\n'+''.join(f"{d['field']}|{d['dtype']}|{','.join(map(str,d['shape']))}|little|{d['bytes_sha256']}\n" for d in items)).encode('ascii')


def input_descriptors(batch: object) -> tuple[list[dict], str]:
    items = []
    for name in INPUT_FIELDS:
        t = getattr(batch,name).detach().cpu().contiguous()
        array = t.numpy()
        blob = array.astype('u1' if t.dtype == torch.bool else '<f4', copy=False).tobytes(order='C')
        items.append(dict(field=name,dtype='bool' if t.dtype == torch.bool else 'float32',shape=list(t.shape),byte_order='little',bytes_sha256=sha(blob)))
    return items, sha(descriptor_preimage(items))


def numeric(array: np.ndarray) -> list:
    def cell(x: float):
        return float(x) if math.isfinite(float(x)) else ('NaN' if math.isnan(float(x)) else ('+Infinity' if x > 0 else '-Infinity'))
    return [[cell(x) for x in row] for row in array]


def geometry(xy: np.ndarray) -> dict:
    """float32 raw [20,2] -> float64 local spacing/prefix/17 four-point chords."""
    if xy.shape != (20,2):
        raise ValueError('geometry shape')
    points = xy.astype(np.float64)
    spacing, cumulative, total = [], [], 0.0
    for k in range(20):
        pair = np.stack((points[k-1] if k else np.zeros(2),points[k]))
        length = float(np.hypot(*(pair[1]-pair[0]))) if np.isfinite(pair).all() else None
        if length is not None and not math.isfinite(length):
            length = None
        spacing.append(length)
        total = total+length if total is not None and length is not None else None
        if total is not None and not math.isfinite(total):
            total = None
        cumulative.append(total)
    chords = []
    for j in range(17):
        status, length, direction = 'UNKNOWN_NONFINITE', None, None
        if np.isfinite(points[j:j+4]).all():
            delta = points[j+3]-points[j]
            length = float(np.hypot(*delta))
            if not math.isfinite(length):
                status, length = 'UNKNOWN_NUMERIC', None
            elif length <= 1e-12:
                status = 'UNKNOWN_DEGENERATE'
            else:
                status, direction = 'DEFINED', float(np.arctan2(delta[1],delta[0]))
        chords.append(dict(j=j,start_s_m=GRID[j],end_s_m=GRID[j+3],length_m=length,direction_rad=direction,status=status,reason=status))
    return dict(derivation_version='raw-f64-four-point-v2',spacing_m=spacing,cumulative_polyline_m=cumulative,
                origin_to_first_index=0,chords=chords,status='COMPUTED' if all(x is not None for x in cumulative) else 'PARTIAL',reason='RAW_ONLY_NOT_PATH_VALIDITY')


class Records:
    """Only uses design-v2 field templates, not its design-only authorization envelope."""
    def __init__(self, schema_dir: Path, *, mode: str, clock=time.monotonic_ns):
        if mode not in ('SYNTHETIC','OFFLINE_TENSOR_REPLAY'):
            raise ValueError('live mode not authorized')
        self.mode, self.clock = mode, clock
        self.design = json.loads((schema_dir/'spatial_path_v4_shadow_record_v1.schema.json').read_text())
        self.schema = json.loads((schema_dir/'spatial_path_v4_runtime_record_v1.schema.json').read_text())
        self.session = str(uuid.uuid4())
        self.clock_epoch = '0'
        self.counts = {k:set() for k in ('accepted','input_built','forward_started','forward_returned','enqueued','saved','dropped')}
        self.sequence = 0

    def blank(self, definition: str) -> dict:
        # Bounded schema template walk: initializes MISSING, never synthetic measured values.
        def visit(s):
            if '$ref' in s:
                return visit(self.design['$defs'][s['$ref'].rsplit('/',1)[-1]])
            if 'const' in s:
                return deepcopy(s['const'])
            if 'oneOf' in s:
                return None if any(x.get('type') == 'null' for x in s['oneOf']) else visit(s['oneOf'][0])
            if 'enum' in s:
                return next((x for x in ('MISSING','UNKNOWN','NOT_COMPUTED','NOT_INFERRED','NOT_BUILT','NONE','NOT_VERIFIED') if x in s['enum']), s['enum'][0])
            t = s.get('type')
            if isinstance(t,list) and 'null' in t:
                return None
            if t == 'object':
                return {k:visit(s['properties'][k]) for k in s.get('required',[])}
            if t == 'array':
                return [visit(s['items']) for _ in range(s.get('minItems',0))]
            if t == 'boolean':
                return False
            if t in ('integer','number'):
                return 0
            return 'NOT_OBSERVED'
        return visit(self.design['$defs'][definition])

    def event(self, kind: str = 'FRAME', *, candidate: int | None = None) -> dict:
        p = self.blank('event')
        p.update(record_id=str(uuid.uuid4()),session_id=self.session,process_boot_id=self.session,reset_id='0',event_type=kind,
                 candidate_sequence=candidate,candidate_reason='CANDIDATE' if candidate is not None else 'NO_CANDIDATE',status='OBSERVED',reason='HARNESS')
        p['timing']['event_observed'] = self.now()
        p['identities']['input_contract'] = identity(INPUT_ID,'FIXED_TRAINING_IDENTITY_NOT_SENSOR_PARITY')
        p['identities']['schema'] = identity(sha(encoded(self.schema)),'canonical runtime schema JSON')
        p['output']['failure_stage'] = 'NONE'
        p['output']['pose_reference_point'] = 'UNKNOWN'
        p['history']['sensor_timing'] = self.blank('sensorTiming')
        p['logger']['drop_candidate_ranges']['status'] = 'UNKNOWN'
        if kind in ('LOGGER_HEALTH','SESSION_END'):
            c = self.blank('stageCounts')
            c.update(scope_session_id=self.session,observed_at=self.now(),observer_id=self.session)
            for key, values in self.counts.items():
                c[key] = dict(status='KNOWN',value=len(values),reason='unique candidate transitions in this harness')
            p['stage_counts'] = c
        return dict(schema_version='spatial_path_v4_runtime_record_v1',mode=self.mode,
                    execution=dict(forward_calls=0,live_sensor_connection_authorized=False,control_connection_enabled=False),payload=p)

    def now(self) -> dict:
        return stamp(self.clock(),clock=self.session,epoch=self.clock_epoch,source='harness_monotonic')

    def accept(self, received: dict | None = None) -> dict:
        i = self.sequence
        self.sequence += 1
        self.counts['accepted'].add(i)
        e = self.event(candidate=i)
        e['payload']['timing']['candidate_received'] = deepcopy(received) if received is not None else self.now()
        return e

    def bind(self, e: dict, batch: object, provenance: dict | None = None, *, finalized: dict | None = None) -> None:
        p, h = e['payload'], self.blank('history')
        p['timing']['input_finalized'] = deepcopy(finalized) if finalized is not None else self.now()
        h.update(status='BUILT',input_builder_id=str(uuid.uuid4()))
        h['input_descriptors'], digest = input_descriptors(batch)
        h['tensor_hash'] = identity(digest,'descriptor preimage v2')
        st = h['sensor_timing'] = self.blank('sensorTiming')
        st.update(status='BUILT',values_s=numeric(batch.sensor_dt_sec[0].cpu().numpy()),input_builder_id=h['input_builder_id'],input_contract_version='spatial_diagnostic_inputs_v4_v1',input_contract_hash=identity(INPUT_ID))
        st['components'] = []
        for j in range(4):
            row = []
            for k, meaning in enumerate(('camera_header-grid','lidar_header-camera_header')):
                row.append(dict(component_index=k,meaning=meaning,operation='UNKNOWN',lhs_time=stamp(),rhs_time=stamp(),reference=f'sensor_dt_sec[0,{j},{k}]; actual snapshot, source timing unavailable',clock_mapping_evidence=identity()))
            st['components'].append(row)
        for role,n,mask in (('camera',4,batch.image_mask[0]),('lidar',4,batch.lidar_mask[0]),('ego',10,batch.ego_feature_mask[0].any(-1)),('command',10,batch.command_mask[0])):
            h[role] = []
            for j in range(n):
                slot = self.blank('slot')
                # design-v2 has only a boolean padding field. With no provenance,
                # false is a compatibility sentinel, NOT evidence of a real slot.
                slot.update(slot_index=j,padding=False,mask_used=bool(mask[j]),source='FIXTURE_METADATA_MISSING',sample_id=None,
                            reason='PADDING_UNKNOWN; boolean false is not evidence; tensor mask retained')
                if provenance:
                    pads=provenance['command_padding'] if role=='command' else [not m for m in provenance['ego_masks' if role=='ego' else 'sensor_masks']]
                    slot['padding']=pads[j]
                    slot['reason']='WARM_UP_PADDING' if pads[j] else ('PRESENT_VALID' if bool(mask[j]) else 'PRESENT_INVALID_OR_MISSING')
                if role == 'ego':
                    slot['feature_mask_used'] = batch.ego_feature_mask[0,j].tolist()
                if role != 'command' or slot['padding']:
                    slot['causality'] = 'NOT_APPLICABLE'
                    for check in ('command_source_past','command_available_before_input'):
                        slot[check]['status'] = 'NOT_APPLICABLE'
                h[role].append(slot)
        p['history'] = h
        self.counts['input_built'].add(p['candidate_sequence'])
        if provenance:
            self.bind_transport(e, provenance)

    def bind_transport(self, e: dict, provenance: dict) -> None:
        p = e['payload']
        def source_stamp(s):
            return stamp(s.header_ns,domain='ROS_SIM',clock=s.clock_id,epoch=s.epoch,source='transport_header')
        def fill(slot,s):
            slot.update(source='PASSIVE_OR_SYNTHETIC_TRANSPORT',header_time=source_stamp(s),
                acquisition_time=stamp(s.acquisition_ns,domain='DEVICE',clock=s.clock_id,epoch=s.epoch,source='transport_acquisition'),
                receipt_monotonic_time=stamp(s.received_ns,clock=s.monotonic_id,epoch=s.monotonic_epoch),available_monotonic_time=stamp(s.available_ns,clock=s.monotonic_id,epoch=s.monotonic_epoch))
            finalized=p['timing']['input_finalized']
            same=finalized['status']=='KNOWN' and finalized['clock_id']==s.monotonic_id and finalized['epoch_id']==s.monotonic_epoch
            age=int(finalized['ns'])-s.available_ns if same else None
            slot['age_at_input']=dict(status='KNOWN' if age is not None and age>=0 else 'UNKNOWN',ns=str(age) if age is not None and age>=0 else None,
                basis='input finalized minus available',reason='transport availability age' if same else 'CLOCK_MAPPING_UNKNOWN')
        for role in ('camera','lidar','ego'):
            frames = provenance['ego_frames' if role == 'ego' else 'sensor_frames']
            for slot,f in zip(p['history'][role],frames):
                fill(slot, getattr(f,'ego_stamp' if role=='ego' else role))
                slot['sample_id'] = f.sample_id
        reference = provenance['sensor_frames'][-1].camera
        p['output']['t_obs'] = source_stamp(reference)
        p['reset_id'] = str(provenance['reset_count'])
        finalized = p['timing']['input_finalized']
        p['history']['reason']='selection_cutoff_ns='+str(provenance['selection_cutoff_ns'])+'; clock_id='+reference.monotonic_id+'; epoch='+reference.monotonic_epoch+'; not input_finalized'
        for j,f in enumerate(provenance['sensor_frames']):
            for k,(left,right) in enumerate(((source_stamp(f.camera),stamp(f.grid_ns,domain='ROS_SIM',clock=f.camera.clock_id,epoch=f.camera.epoch,source='grid')), (source_stamp(f.lidar),source_stamp(f.camera)))):
                p['history']['sensor_timing']['components'][j][k].update(operation='LHS_MINUS_RHS_SECONDS',lhs_time=left,rhs_time=right,reference='transport grid/camera/lidar; ns -> s',clock_mapping_evidence=identity(sha(encoded([left,right])),'same clock/epoch transport'))
        for slot,c,frame in zip(p['history']['command'],provenance['commands'],provenance['command_frames']):
            if frame is not None:
                slot['sample_id']=frame.sample_id
            if c is None:
                if not slot['padding']:
                    slot['source']='COMMAND_MISSING_OR_INVALID'
                continue
            fill(slot,c.stamp)
            slot['source'] = c.source
            for name,lhs,rhs,ok in (
                ('command_source_past',source_stamp(c.stamp),source_stamp(reference),c.stamp.header_ns<reference.header_ns),
                ('command_available_before_input',stamp(c.stamp.available_ns,clock=c.stamp.monotonic_id,epoch=c.stamp.monotonic_epoch),finalized,finalized['ns'] is not None and c.stamp.available_ns<=int(finalized['ns']))):
                same=lhs['status']==rhs['status']=='KNOWN' and (lhs['clock_id'],lhs['epoch_id'],lhs['domain'])==(rhs['clock_id'],rhs['epoch_id'],rhs['domain'])
                slot[name].update(status=('PROVEN' if ok else 'VIOLATION') if same else 'UNKNOWN',lhs_time=lhs,rhs_time=rhs,lhs_reference=name,rhs_reference='t_obs' if name=='command_source_past' else 'input_finalized',clock_mapping_evidence=identity(sha(encoded([lhs,rhs])) if same else None,'same clock/epoch' if same else 'CLOCK_MAPPING_UNKNOWN'))
            statuses=[slot[k]['status'] for k in ('command_source_past','command_available_before_input')]
            slot['causality'] = 'PROVEN_PAST_AND_AVAILABLE' if statuses==['PROVEN','PROVEN'] else ('VIOLATION' if 'VIOLATION' in statuses else 'UNKNOWN')

    def validate(self, e: dict) -> None:
        # Full draft2020 validation is an explicit optional verification stage;
        # absence must not masquerade as validation success or block pure core.
        self.validate_semantics(e)

    def validate_schema(self, e: dict) -> None:
        from jsonschema import Draft202012Validator
        from referencing import Registry, Resource
        registry = Registry().with_resource(self.design['$id'],Resource.from_contents(self.design))
        Draft202012Validator(self.schema,registry=registry).validate(e)

    def validate_semantics(self, e: dict) -> None:
        p, out = e['payload'], e['payload']['output']
        if e['mode'] not in ('SYNTHETIC','OFFLINE_TENSOR_REPLAY') or e['execution']['live_sensor_connection_authorized'] or e['execution']['control_connection_enabled']:
            raise ValueError('unauthorized execution')
        if out['status']=='NOT_INFERRED' and any(out[k] is not None for k in ('forward_invocation_id','output_id','actual_shape','dtype','source_device','model_xy_m','float32_le_hex')):
            raise ValueError('unobserved metadata fabricated')
        if out['status']=='FORWARD_EXCEPTION' and (out['forward_invocation_id'] is None or out['output_id'] is not None):
            raise ValueError('forward exception identity')
        if out['status'] in ('SNAPSHOT_ERROR','OUTPUT_CONTRACT_ERROR') and (out['forward_invocation_id'] is None or out['output_id'] is None or out['model_xy_m'] is not None):
            raise ValueError('returned failure identity/payload')
        if p['history']['status'] == 'BUILT':
            if sha(descriptor_preimage(p['history']['input_descriptors'])) != p['history']['tensor_hash']['sha256']:
                raise ValueError('input descriptor hash')
            if p['history']['sensor_timing']['input_builder_id'] != p['history']['input_builder_id']:
                raise ValueError('snapshot identity')
        if out['status'] in ('NONFINITE','SHAPE_FINITE_ONLY'):
            if out['actual_shape'] != [1,20,2] or out['dtype'] != 'float32' or out['forward_invocation_id'] is None or out['output_id'] is None:
                raise ValueError('output contract/identity')
            raw = bytes.fromhex(out['float32_le_hex'])
            xy = np.frombuffer(raw,dtype='<f4').reshape(20,2)
            if sha(raw) != out['tensor_hash']['sha256'] or numeric(xy) != out['model_xy_m']:
                raise ValueError('output bits/tag/hash mismatch')
            if (not bool(np.isfinite(xy).all())) != (out['status']=='NONFINITE'):
                raise ValueError('finite status mismatch')
            if geometry(xy) != out['geometry']:
                raise ValueError('geometry semantic mismatch')
        for role in ('camera','lidar','ego','command'):
            for j,s in enumerate(p['history'][role]):
                if s['slot_index'] != j:
                    raise ValueError('slot order')
                if s['causality']=='PROVEN_PAST_AND_AVAILABLE' and any(s[k]['status']!='PROVEN' for k in ('command_source_past','command_available_before_input')):
                    raise ValueError('causality summary')
                for field,strict in (('command_source_past',True),('command_available_before_input',False)):
                    check=s[field]
                    if check['status']=='PROVEN':
                        l,r=check['lhs_time'],check['rhs_time']
                        if l['status']!='KNOWN' or r['status']!='KNOWN' or (l['clock_id'],l['epoch_id'],l['domain']) != (r['clock_id'],r['epoch_id'],r['domain']):
                            raise ValueError('unproven clock comparison')
                        if not (int(l['ns'])<int(r['ns']) if strict else int(l['ns'])<=int(r['ns'])):
                            raise ValueError('causality violation')
        for key,left,right in (('inference_duration','inference_start','inference_api_return'),('end_to_end_age','candidate_received','snapshot_ready')):
            duration=p['timing'][key]
            if duration['status']=='KNOWN':
                l,r=p['timing'][left],p['timing'][right]
                if l['status']!='KNOWN' or r['status']!='KNOWN' or (l['clock_id'],l['epoch_id'],l['domain']) != (r['clock_id'],r['epoch_id'],r['domain']) or int(duration['ns'])!=int(r['ns'])-int(l['ns']) or int(duration['ns'])<0:
                    raise ValueError('duration/clock mismatch')


class PrivateWriter:
    """Bounded queue; OPEN/CLOSED/FAILED, no recursive receipts or async workers."""
    def __init__(self, path: Path, records: Records, *, capacity: int=4, max_record_bytes: int=131072, max_file_bytes: int=8388608):
        if min(capacity,max_record_bytes,max_file_bytes)<=0:
            raise ValueError('finite positive recording budgets required')
        self.records, self.capacity, self.max_record_bytes, self.max_file_bytes = records,capacity,max_record_bytes,max_file_bytes
        self.queue: deque = deque()
        self.fd = os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        self.bytes_written = 0
        self.state = 'OPEN'
        self.error: str | None = None
        self.log_writes = 0

    @property
    def failed(self) -> bool:
        return self.state=='FAILED'

    def fail(self, reason: str, candidate: int | None = None) -> None:
        self.state='FAILED'
        if self.error is None:
            self.error=reason
        if candidate is not None:
            self.records.counts['dropped'].add(candidate)

    def enqueue(self, event: dict, outcome: dict | None = None) -> bool:
        outcome=outcome if outcome is not None else dict(data_write='NOT_ATTEMPTED',receipt_write='NOT_ATTEMPTED',error=None)
        candidate=event['payload']['candidate_sequence']
        if self.state!='OPEN' or len(self.queue)>=self.capacity:
            reason='LOGGER_'+self.state if self.state!='OPEN' else 'QUEUE_FULL'
            if self.state=='OPEN':
                self.fail(reason,candidate)
            outcome['error']=outcome['error'] or self.error or reason
            if candidate is not None:
                self.records.counts['dropped'].add(candidate)
            return False
        try:
            event=deepcopy(event)
            p=event['payload']
            p['timing']['record_enqueued']=self.records.now()
            p['logger'].update(state='HEALTHY',queue_capacity=self.capacity,queue_depth=len(self.queue)+1)
            self.records.validate(event)
            blob=encoded(event)
            if len(blob)>self.max_record_bytes:
                raise ValueError('RECORD_BYTE_LIMIT')
        except Exception as exc:
            self.fail(type(exc).__name__+': '+str(exc)[:500],candidate)
            outcome['error']=outcome['error'] or self.error
            return False
        self.queue.append((event,blob,outcome))
        if candidate is not None:
            self.records.counts['enqueued'].add(candidate)
        return True

    def _write(self, blob: bytes) -> None:
        if self.fd is None or self.state!='OPEN':
            raise OSError('WRITER_NOT_OPEN')
        if self.bytes_written+len(blob)+1>self.max_file_bytes:
            raise OSError('FILE_BYTE_LIMIT')
        data=memoryview(blob+b'\n')
        while data:
            written=os.write(self.fd,data)
            if written<=0:
                raise OSError('ZERO_WRITE')
            self.bytes_written+=written
            data=data[written:]
        self.log_writes+=1

    def drain(self) -> None:
        while self.queue and self.state=='OPEN':
            event,blob,outcome=self.queue.popleft()
            p=event['payload']
            try:
                outcome['data_write']='UNKNOWN'  # May partially write before failing.
                self._write(blob)
                outcome['data_write']='KNOWN_WRITE_COMPLETED_NOT_DURABLE'
                if p['candidate_sequence'] is not None:
                    self.records.counts['saved'].add(p['candidate_sequence'])
                confirmed=self.records.now()
                receipt=self.records.event('WRITER_RECEIPT')
                receipt['payload']['commit_receipt']=dict(target_record_id=p['record_id'],target_session_id=p['session_id'],
                    target_candidate_sequence=p['candidate_sequence'],original_bytes_hash=sha(blob),writer_id=self.records.session,
                    confirmed_at=confirmed,commit_level='WRITE_COMPLETED_NOT_DURABLE',reason='OS write completed; no fsync')
                self.records.validate(receipt)
                receipt_blob=encoded(receipt)
                if len(receipt_blob)>self.max_record_bytes:
                    raise OSError('RECEIPT_BYTE_LIMIT')
                outcome['receipt_write']='UNKNOWN'  # Only after preparation, before possible partial write.
                self._write(receipt_blob)
                outcome['receipt_write']='KNOWN_WRITE_COMPLETED_NOT_DURABLE'
            except Exception as exc:
                # Receipt failure does not lose an already completed main record.
                # Do not remove any pre-existing processing DROP membership.
                lost=p['candidate_sequence'] if outcome['data_write']!='KNOWN_WRITE_COMPLETED_NOT_DURABLE' else None
                self.fail(type(exc).__name__+': '+str(exc)[:500],lost)
                outcome['error']=outcome['error'] or self.error
        if self.state=='FAILED':
            for event,_,outcome in self.queue:
                i=event['payload']['candidate_sequence']
                if i is not None:
                    self.records.counts['dropped'].add(i)
                outcome['error']=outcome['error'] or self.error
            self.queue.clear()

    def close(self) -> None:
        if self.fd is None:
            return
        try:
            self.drain()
        finally:
            fd,self.fd=self.fd,None
            try:
                os.close(fd)
            except OSError as exc:
                self.fail('CLOSE_ERROR:'+str(exc))
            if self.state=='OPEN':
                self.state='CLOSED'
