from __future__ import annotations

import pytest

from aic_transfuser_lite.data.clock_segments import ClockSample, segment_clock_epochs
from tools.audit_lidar_v2x_obstacles import coalesce_sim_clock_receipts, overlap, windows


def test_sim_time_recorder_duplicate_receipts_do_not_create_false_resets():
    raw=[ClockSample(35,35),ClockSample(35,40),ClockSample(45,45),ClockSample(45,50)]
    values=coalesce_sim_clock_receipts(raw)
    assert [(v.bag_stamp_ns,v.sim_stamp_ns) for v in values]==[(35,40),(45,50)]
    assert len(segment_clock_epochs(values,max_forward_jump_ns=100))==1
    assert len(raw)==4


@pytest.mark.parametrize('samples',[
    [ClockSample(40,40),ClockSample(40,0)],
    [ClockSample(40,40),ClockSample(50,30)],
    [ClockSample(40,40),ClockSample(30,50)],
])
def test_real_clock_reversal_is_not_hidden_by_duplicate_receipt_handling(samples):
    with pytest.raises(ValueError,match='reversal'):
        coalesce_sim_clock_receipts(samples)


def test_forward_jump_remains_visible_to_epoch_validation():
    values=coalesce_sim_clock_receipts([ClockSample(1,1),ClockSample(2,1000)])
    assert len(segment_clock_epochs(values,max_forward_jump_ns=100))==2


def test_reverse_window_excludes_touching_history_or_future_endpoints():
    reverse=windows([10,11,11,12,30],max_gap_ns=2)
    assert reverse==[(10,12),(30,30)]
    assert overlap(2,10,reverse)
    assert overlap(12,20,reverse)
    assert not overlap(13,29,reverse)
    with pytest.raises(ValueError):overlap(20,10,reverse)
