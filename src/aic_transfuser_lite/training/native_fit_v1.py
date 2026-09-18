"""Train-only native teacher geometry and bounded comparison sampling."""
from __future__ import annotations
from typing import Any, Sequence
import math
import numpy as np
import torch
from ..data.time_dataset_v1 import TimeSample
from ..data.time_split_v1 import assert_split_membership
from .time_recovery_geometry_v1 import fixed_time_pp_angle


def comparison_schedule(*, old_count: int, native_count: int, front: Sequence[int],
                        steps: int, focused: bool, seed: int = 42) -> list[list[tuple[str, int]]]:
    """Fixed 32 presentations/update; focus changes sampling, never the split.

    Control: 3380/63988 native fraction. Focus: 8/32 native, half explicitly
    front-approach, half uniform native. Old replay keeps its original sampler.
    This is a short diagnostic draw, not a replacement full-corpus epoch.
    """
    if (any(type(n) is not int or n <= 0 for n in (old_count,native_count,steps))
            or type(focused) is not bool or not front or len(set(front))!=len(front)
            or any(type(i) is not int or not 0 <= i < native_count for i in front)):
        raise ValueError('nonempty counts and unique in-range front indices required')
    old_rng=np.random.default_rng(seed); native_rng=np.random.default_rng(seed+1)
    old_order=old_rng.permutation(old_count); cursor=0; schedule=[]
    for step in range(steps):
        count=8 if focused else ((step+1)*32*3380//63988-step*32*3380//63988)
        batch=[]
        for _ in range(32-count):
            if cursor==len(old_order):old_order=old_rng.permutation(old_count);cursor=0
            batch.append(('old',int(old_order[cursor])));cursor+=1
        for j in range(count):
            index=int(native_rng.choice(front)) if focused and j<4 else int(native_rng.integers(native_count))
            batch.append(('native',index))
        schedule.append(batch)
    return schedule


class NativeGeometryObjective:
    """PP tire-angle [rad] and far lateral [m] sum on explicit train anchors.

    Prediction/teacher [B,30,2] metres; support [B,30] bool. Teacher-selected
    horizon and response length are labels only. Caller divides by all support.
    This supplements the unchanged old recovery objective; no inference input
    or source-run prefix is disguised as a recovery run.
    """
    def __init__(self, rows: Sequence[dict[str, Any]], *, split_manifest: dict[str, Any]) -> None:
        if not rows or len({r['anchor_id'] for r in rows})!=len(rows):
            raise ValueError('nonempty unique native targets required')
        assert_split_membership(split_manifest,[r['run_id'] for r in rows],split='train')
        for r in rows:
            if (not r['run_id'].startswith('lidar-v45-pc10-')
                    or not math.isfinite(r['horizon_s']) or not .1<=r['horizon_s']<=3.
                    or not math.isfinite(r['response_length_m']) or r['response_length_m']<=0):
                raise ValueError('invalid native teacher geometry')
        self.rows={r['anchor_id']:dict(r) for r in rows}

    def validate_samples(self, samples: Sequence[TimeSample]) -> None:
        for s in samples:
            r=self.rows.get(s.anchor_id)
            if r is not None and (s.run!=r['run_id'] or s.inputs is None or s.teacher is None
                    or not s.teacher.xy_mask.all() or not np.isfinite(s.teacher.xy_m).all()):
                raise ValueError('native geometry needs matching run and complete teacher')

    def __call__(self, prediction: torch.Tensor, target: torch.Tensor,
                 support: torch.Tensor, samples: Sequence[TimeSample]) -> torch.Tensor:
        if (prediction.shape!=(len(samples),30,2) or target.shape!=prediction.shape
                or support.shape!=(len(samples),30) or support.dtype!=torch.bool):
            raise ValueError('expected [B,30,2] metre XY and [B,30] bool support')
        self.validate_samples(samples)
        chosen=[i for i,s in enumerate(samples) if s.anchor_id in self.rows]
        if not chosen:return prediction.sum()*0.
        if not support[chosen].all():raise ValueError('complete native support required')
        p=prediction[chosen].float();t=target[chosen].detach().float()
        h=p.new_tensor([self.rows[samples[i].anchor_id]['horizon_s'] for i in chosen])
        length=p.new_tensor([self.rows[samples[i].anchor_id]['response_length_m'] for i in chosen])
        offset=(0.0010000169277191162,0.)
        near=(fixed_time_pp_angle(p,h,length,offset)-fixed_time_pp_angle(t,h,length,offset)).abs()
        far=(p[:,19:,1]-t[:,19:,1]).abs().mean(1)
        return (near+.5*far).sum()


class AddNativeObjective:
    """Keep old recovery loss identical; optionally add selected native geometry."""
    def __init__(self, recovery: Any, native: NativeGeometryObjective | None) -> None:
        self.recovery,self.native=recovery,native

    def validate_samples(self, samples: Sequence[TimeSample]) -> None:
        self.recovery.validate_samples(samples)
        if self.native is not None:self.native.validate_samples(samples)

    def __call__(self, prediction: torch.Tensor, target: torch.Tensor,
                 support: torch.Tensor, samples: Sequence[TimeSample]) -> torch.Tensor:
        result=self.recovery(prediction,target,support,samples)
        return result if self.native is None else result+self.native(prediction,target,support,samples)
