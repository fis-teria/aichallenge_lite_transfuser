"""Train-only launch sampling with explicit simulator drive windows.

Sensor inputs and measured [30,2] metre teachers are never rewritten. The drive
window is a dataset eligibility annotation, not an additional model feature.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np
from torch.utils.data import Dataset

from .time_dataset_v1 import TimeSample
from .time_split_v1 import assert_split_membership, content_sha256


@dataclass(frozen=True)
class DriveWindow:
    ready_sim_ns: int
    ready_available_ns: int
    external_brake_sim_ns: int

    def __post_init__(self) -> None:
        if (any(type(v) is not int or v < 0 for v in
                (self.ready_sim_ns, self.ready_available_ns, self.external_brake_sim_ns))
                or self.external_brake_sim_ns <= self.ready_sim_ns):
            raise ValueError('ordered finite integer drive-window nanoseconds required')

    def exclusion(self, observation_ns: int, freeze_ns: int, *,
                  teacher_span_ns: int = 3_050_000_000) -> str | None:
        """Require Ready at capture/availability and future before external brake.

        3.05 s covers the 3 s teacher plus its 50 ms interpolation tolerance.
        The brake cutoff excludes an external intervention, not an inferred stop.
        """
        if any(type(v) is not int or v < 0 for v in (observation_ns, freeze_ns)):
            raise ValueError('integer observation and receipt nanoseconds required')
        if type(teacher_span_ns) is not int or teacher_span_ns <= 0:
            raise ValueError('positive integer teacher span required')
        if observation_ns < self.ready_sim_ns:
            return 'SIMULATOR_NOT_READY_AT_CAPTURE'
        if freeze_ns < self.ready_available_ns:
            return 'READY_NOT_AVAILABLE_AT_FREEZE'
        if observation_ns + teacher_span_ns >= self.external_brake_sim_ns:
            return 'TEACHER_CROSSES_EXTERNAL_BRAKE'
        return None


class LaunchQuotaDataset(Dataset[TimeSample]):
    """Fixed-budget sampling: retain all eligible anchors and protect launch.

    Common to both arms: excluded protocol slots are filled from eligible,
    non-launch nominal observations. Treatment replaces only surplus recovery
    repetitions with run-balanced launch observations. Every eligible unique
    observation remains represented; no validation/test sample may enter.
    """

    def __init__(self, dataset: Any, reference_indices: Sequence[int], *,
                 eligible_indices: Sequence[int], launch_indices: Sequence[int],
                 recovery_run_ids: Sequence[str], launch_fraction: float | None,
                 seed: int) -> None:
        assert_split_membership(dataset.split_manifest, dataset.run_ids, split='train')
        if type(seed) is not int or seed < 0:
            raise ValueError('nonnegative integer seed required')
        if launch_fraction is not None and (not np.isfinite(launch_fraction) or not 0 < launch_fraction < 1):
            raise ValueError('launch fraction must be finite and between zero and one')
        if len(set(dataset.anchor_ids)) != len(dataset.anchor_ids):
            raise ValueError('underlying anchor IDs must be unique')
        eligible, launches = set(eligible_indices), set(launch_indices)
        reference = list(reference_indices)
        if (not eligible or not launches or len(eligible) != len(eligible_indices)
                or len(launches) != len(launch_indices) or not launches <= eligible
                or any(type(i) is not int or not 0 <= i < len(dataset)
                       for i in [*reference, *eligible, *launches])
                or not eligible <= set(reference)):
            raise ValueError('unique supported eligible/launch indices in the reference required')
        recovery = set(recovery_run_ids)
        if (not recovery or not recovery <= set(dataset.run_ids)
                or any(dataset.run_ids[i] in recovery for i in launches)
                or any(dataset.run_ids[i] in recovery for i in set(reference)-eligible)):
            raise ValueError('only nominal protocol exclusions and nominal launch targets allowed')
        if not dataset.input_valid[list(launches)].all() or not dataset.xy_mask[list(launches)].all():
            raise ValueError('launch targets require valid inputs and full measured teachers')
        if not np.isfinite(dataset.targets[list(launches)]).all():
            raise ValueError('launch teachers must be finite [N,30,2] metres')
        if dataset.targets.shape != (len(dataset),30,2) or dataset.xy_mask.shape != (len(dataset),30):
            raise ValueError('expected [N,30,2] metre targets and [N,30] support')
        filler = sorted(i for i in eligible if dataset.run_ids[i] not in recovery and i not in launches)
        if not filler:
            raise ValueError('active non-launch nominal refill observations required')
        rng = np.random.default_rng(seed)
        slots = [j for j,i in enumerate(reference) if i not in eligible]
        refill = []
        while len(refill) < len(slots):
            refill.extend(map(int,rng.permutation(filler)))
        common = reference.copy()
        for j,i in zip(slots,refill):
            common[j]=i
        self.indices=common.copy()
        initial_count=sum(i in launches for i in common)
        quota=initial_count if launch_fraction is None else round(len(common)*launch_fraction)
        if quota < initial_count:
            raise ValueError('launch quota cannot remove existing launch observations')
        required=quota-initial_count
        counts=Counter(common)
        donors=[]
        for j in map(int,rng.permutation(len(common))):
            i=common[j]
            if dataset.run_ids[i] in recovery and counts[i]>1:
                donors.append(j);counts[i]-=1
                if len(donors)==required:
                    break
        if required==0:
            donors=[]
        if len(donors)<required:
            raise ValueError('not enough surplus recovery repetitions for launch quota')
        groups={r:sorted(i for i in launches if dataset.run_ids[i]==r)
                for r in sorted({dataset.run_ids[i] for i in launches})}
        selected=[]
        queues={r:[] for r in groups}
        # Replenish the least-presented launch run, balancing final counts.
        run_counts=Counter(dataset.run_ids[i] for i in common if i in launches)
        for _ in range(required):
            r=min(groups,key=lambda r:(run_counts[r],r))
            if not queues[r]:queues[r]=list(map(int,rng.permutation(groups[r])))
            selected.append(queues[r].pop());run_counts[r]+=1
        for j,i in zip(donors,selected,strict=True):self.indices[j]=i
        self.dataset=dataset
        self.run_ids=[dataset.run_ids[i] for i in self.indices]
        self.anchor_ids=[dataset.anchor_ids[i] for i in self.indices]
        if len(self.indices)!=len(reference) or set(self.indices)!=eligible:
            raise ValueError('fixed budget or complete eligible coverage violated')
        self.audit=dict(presentations=len(self.indices),eligible_unique=len(eligible),
            protocol_excluded_unique=len(set(reference)-eligible),refilled_slots=len(slots),
            launch_unique=len(launches),launch_presentations=quota,
            launch_run_presentations=dict(Counter(dataset.run_ids[i] for i in self.indices if i in launches)),
            surplus_recovery_slots_replaced=required,
            recovery_presentations=sum(r in recovery for r in self.run_ids),
            common_order_sha256=content_sha256([dataset.anchor_ids[i] for i in common]),
            presentation_order_sha256=content_sha256(self.anchor_ids),
            all_eligible_unique_preserved=True,inputs_and_teachers_unchanged=True)

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self,index: int) -> TimeSample:
        return self.dataset[self.indices[index]]
