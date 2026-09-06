"""Fixed step500 path-only inference. No controller, optimizer or ROS dependency."""
from __future__ import annotations

import io
from pathlib import Path

import numpy as np
import torch

from aic_transfuser_lite.models.spatial_path_diagnostic_v4 import SpatialPathDiagnosticV4
from .spatial_input_v4 import freeze_batch
from .spatial_recording_v4 import Records, PrivateWriter, geometry, identity, numeric, sha

CHECKPOINT = Path('/home/thistle/e2e_autonomous/runs/spatial_diagnostic_v4_20260906_f33b197/checkpoints/final.pt')
CHECKPOINT_SHA = '0316692543a901d9d6b96718c5651d6739367aa851f83fb3a133c126ccc8919f'


def state_inventory(model: torch.nn.Module) -> dict:
    parameters=dict(model.named_parameters())
    return {name:dict(shape=list(t.shape),dtype=str(t.dtype),finite=bool(torch.isfinite(t).all()),
                     kind='parameter' if name in parameters else 'buffer',sha256=sha(t.detach().cpu().contiguous().numpy().tobytes()))
            for name,t in model.state_dict().items()}


def load_fixed(device: str = 'cpu') -> tuple[torch.nn.Module, dict]:
    """Exactly the user allowlisted path; weights_only strict all-state load."""
    if CHECKPOINT.is_symlink():
        raise ValueError('checkpoint symlink forbidden')
    blob=CHECKPOINT.read_bytes()
    before=sha(blob)
    if before!=CHECKPOINT_SHA:
        raise ValueError('checkpoint identity mismatch')
    payload=torch.load(io.BytesIO(blob),map_location='cpu',weights_only=True)
    if set(payload)!={'format','state_dict'} or payload['format']!='SPATIAL_DIAGNOSTIC_NOT_RUNTIME':
        raise ValueError('checkpoint format')
    model=SpatialPathDiagnosticV4(image_height=224,image_width=384,lidar_points=750,ego_dim=4)
    expected=model.state_dict()
    state=payload['state_dict']
    if set(state)!=set(expected):
        raise ValueError('strict key mismatch')
    for name,t in state.items():
        if not isinstance(t,torch.Tensor) or t.shape!=expected[name].shape or t.dtype!=expected[name].dtype or not bool(torch.isfinite(t).all()):
            raise ValueError('invalid state: '+name)
    model.load_state_dict(state,strict=True)
    model.eval().requires_grad_(False)
    model.to(device)
    after=sha(CHECKPOINT.read_bytes())
    if before!=after:
        raise ValueError('checkpoint changed during read')
    return model, dict(path=str(CHECKPOINT),expected_sha256=CHECKPOINT_SHA,before_sha256=before,after_sha256=after,
                      weights_only=True,strict=True,loaded=list(state),missing=[],unexpected=[],state=state_inventory(model))


class SpatialRuntimeV4:
    def __init__(self, model: torch.nn.Module, records: Records, writer: PrivateWriter | None = None,
                 *, device: str = 'cpu', checkpoint_hash: str | None = None, forward_limit: int = 6):
        self.model=model.eval().requires_grad_(False)
        self.records,self.writer,self.device=records,writer,device
        self.checkpoint_hash=checkpoint_hash
        self.forward_calls=0
        self.forward_limit=forward_limit

    def infer(self, batch: object, provenance: dict | None = None) -> dict:
        event=self.records.accept()
        p=event['payload']
        if self.writer and self.writer.failed:
            p.update(event_type='DROP',status='DROPPED',reason='LOGGER_FAILED_NEW_INFERENCE_STOPPED')
            self.records.counts['dropped'].add(p['candidate_sequence'])
            return event
        if self.forward_calls>=self.forward_limit:
            p.update(event_type='DROP',status='DROPPED',reason='FORWARD_BUDGET')
            self.records.counts['dropped'].add(p['candidate_sequence'])
            return event
        try:
            frozen=freeze_batch(batch,self.device)
            self.records.bind(event,frozen,provenance)
        except (ValueError,RuntimeError) as exc:
            p.update(event_type='DROP',status='DROPPED',reason=str(exc))
            self.records.counts['dropped'].add(p['candidate_sequence'])
            return event
        out=p['output']
        out['forward_invocation_id']=p['record_id']+':forward'
        p['identities']['checkpoint']=identity(self.checkpoint_hash,'fixed file verified' if self.checkpoint_hash else 'synthetic model')
        self.forward_calls+=1
        event['execution']['forward_calls']=1
        self.records.counts['forward_started'].add(p['candidate_sequence'])
        p['timing']['inference_start']=self.records.now()
        try:
            with torch.inference_mode():
                result=self.model(frozen)
            p['timing']['inference_api_return']=self.records.now()
            out['output_id']=p['record_id']+':output'
            self.records.counts['forward_returned'].add(p['candidate_sequence'])
        except Exception as exc:
            p['timing']['inference_failed_at']=self.records.now()
            out.update(status='FORWARD_EXCEPTION',failure_stage='FORWARD',reason=type(exc).__name__+': '+str(exc)[:500])
            p.update(status='EXCEPTION',reason=out['reason'])
        else:
            try:
                if not isinstance(result,torch.Tensor):
                    out.update(status='OUTPUT_CONTRACT_ERROR',failure_stage='OUTPUT_CONTRACT',reason='NOT_TENSOR')
                else:
                    out.update(actual_shape=list(result.shape) if result.ndim<=8 else None,dtype=str(result.dtype).removeprefix('torch.'),
                               source_device=str(result.device),metadata_reason='observed returned tensor metadata')
                    if result.shape!=(1,20,2) or result.dtype!=torch.float32:
                        out.update(status='OUTPUT_CONTRACT_ERROR',failure_stage='OUTPUT_CONTRACT',reason='SHAPE_OR_DTYPE; no cast/trim')
                    else:
                        xy=self.snapshot(result)
                        p['timing']['snapshot_ready']=self.records.now()
                        raw=xy.astype('<f4',copy=False).tobytes(order='C')
                        out.update(status='SHAPE_FINITE_ONLY' if np.isfinite(xy).all() else 'NONFINITE',failure_stage='NONE',reason='RAW_SNAPSHOT_NOT_VALIDITY',
                                   model_xy_m=numeric(xy),float32_le_hex=raw.hex(),tensor_hash=identity(sha(raw),'160 raw little-endian float32 bytes'),
                                   frame='base_link',pose_reference_point='BASE_LINK_ORIGIN',geometry=geometry(xy))
            except Exception as exc:
                p['timing']['snapshot_failed_at']=self.records.now()
                out.update(status='SNAPSHOT_ERROR',failure_stage='SNAPSHOT',reason=type(exc).__name__+': '+str(exc)[:500],
                           model_xy_m=None,float32_le_hex=None,geometry=None,tensor_hash=identity())
            for field,left,right in (('inference_duration','inference_start','inference_api_return'),('end_to_end_age','candidate_received','snapshot_ready')):
                l,r=p['timing'][left],p['timing'][right]
                if l['status']==r['status']=='KNOWN' and (l['clock_id'],l['epoch_id'])==(r['clock_id'],r['epoch_id']):
                    p['timing'][field]=dict(status='KNOWN',ns=str(int(r['ns'])-int(l['ns'])),basis=right+' minus '+left,reason='harness monotonic')
        self.records.validate(event)
        if self.writer:
            self.writer.enqueue(event)
            self.writer.drain()
        return event

    @staticmethod
    def snapshot(result: torch.Tensor) -> np.ndarray:
        # cpu copy synchronizes this output's GPU work. No model re-entry or numeric cast.
        return result.detach().cpu().contiguous().numpy()[0].copy()
