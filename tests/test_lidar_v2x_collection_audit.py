from __future__ import annotations


def test_driving_start_is_case_insensitive_but_excludes_playstart():
    from tools.audit_lidar_v2x_obstacles import driving_start_stamps
    assert driving_start_stamps([(1,'PlayStart'),(2,'playstart'),(3,'WaitStart'),
        (4,'Start'),(5,'start'),(6,' finish ')]) == [4,5]


def test_monitor_race_start_uses_simulation_stamp_not_wall_time_or_vehicle_ready():
    from tools.audit_lidar_v2x_obstacles import resolve_driving_start
    samples=[dict(time=-1,awsim_state='start',ego=dict(stamp=2)),
             dict(time=100,awsim_state='playstart',ego=dict(stamp=3)),
             dict(time=101,awsim_state='start',ego=dict(stamp=7.2))]
    assert resolve_driving_start([(5,'Ready')],{'scenario_start_observed':True},samples) == (
        7_200_000_000,'monitor:/admin/awsim/state + ego simulation stamp')
    assert resolve_driving_start([(6,'Start')],{},[]) == (6,'/awsim/state')
    with pytest.raises(ValueError,match='No verified'):
        resolve_driving_start([(5,'Ready')],{'scenario_start_observed':False},samples)

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
