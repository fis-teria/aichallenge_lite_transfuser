from collections import Counter
from types import SimpleNamespace

import numpy as np
import pytest

from aic_transfuser_lite.data.time_launch_training_v1 import DriveWindow, LaunchQuotaDataset
from test_time_recovery_matched_mix_v1 import setup


class Samples:
    def __init__(self):
        original,_=setup()
        self.__dict__.update(vars(original))
        nominal=self.run_ids[0]
        self.run_ids += [nominal,nominal]
        self.anchor_ids += ['countdown','second_launch']
        self.input_valid=np.ones(7,bool)
        self.xy_mask=np.ones((7,30),bool)
        self.targets=np.ones((7,30,2),np.float32)
        self.samples=[object() for _ in self.anchor_ids]

    def __len__(self):return len(self.anchor_ids)
    def __getitem__(self,index):return self.samples[index]


def dataset(fraction=None, **changes):
    full=Samples()
    kwargs=dict(eligible_indices=[0,1,2,3,4,6],launch_indices=[0,6],
        recovery_run_ids=['recovery_train','new_recovery'],launch_fraction=fraction,seed=42)
    kwargs.update(changes)
    # Two nominal launches, one normal observation, a countdown observation,
    # and eight recovery slots containing only three unique recovery anchors.
    reference=[0,1,1,2,3,3,4,4,1,3,5,6]
    return full,LaunchQuotaDataset(full,reference,**kwargs)


def test_countdown_and_external_stop_are_not_drive_teachers():
    window=DriveWindow(10_000_000_000,500_000_000_000,30_000_000_000)
    assert window.exclusion(9_999_999_999,501_000_000_000)=='SIMULATOR_NOT_READY_AT_CAPTURE'
    assert window.exclusion(10_000_000_000,499_999_999_999)=='READY_NOT_AVAILABLE_AT_FREEZE'
    assert window.exclusion(10_000_000_000,500_000_000_000) is None
    assert window.exclusion(26_950_000_000,510_000_000_000)=='TEACHER_CROSSES_EXTERNAL_BRAKE'
    with pytest.raises(ValueError):DriveWindow(10,100,10)
    with pytest.raises(ValueError):window.exclusion(1.2,4)


def test_fixed_budget_preserves_every_eligible_teacher_without_rewriting():
    _,control=dataset()
    full,treatment=dataset(.5)
    assert len(control)==len(treatment)==12
    assert set(control.indices)==set(treatment.indices)=={0,1,2,3,4,6}
    assert 5 not in treatment.indices
    assert treatment.audit['launch_presentations']==6
    assert treatment.audit['surplus_recovery_slots_replaced']==4
    assert treatment.audit['common_order_sha256']==control.audit['common_order_sha256']
    assert Counter(treatment.indices)[2]>=1
    assert all(treatment[j] is full.samples[i] for j,i in enumerate(treatment.indices))
    np.testing.assert_array_equal(full.targets,np.ones((7,30,2),np.float32))
    assert dataset(.5)[1].indices==treatment.indices


@pytest.mark.parametrize('changes',[
    {'launch_indices':[1]}, {'eligible_indices':[0,2,3,4,6]},
    {'launch_indices':[0,0]}, {'launch_indices':[5]},
    {'launch_fraction':.95}, {'launch_fraction':float('nan')}, {'seed':True},
])
def test_invalid_or_uncoverable_quota_fails(changes):
    with pytest.raises(ValueError):dataset(**changes)


def test_validation_or_unsupported_launch_cannot_enter_training():
    full=Samples();full.input_valid[0]=False
    kwargs=dict(eligible_indices=[0,1,2,3,4,6],launch_indices=[0,6],
        recovery_run_ids=['recovery_train','new_recovery'],launch_fraction=None,seed=42)
    with pytest.raises(ValueError,match='valid inputs'):
        LaunchQuotaDataset(full,list(range(7)),**kwargs)
    full.input_valid[0]=True
    full.run_ids[0]='recovery_val'
    with pytest.raises(ValueError):LaunchQuotaDataset(full,list(range(7)),**kwargs)
