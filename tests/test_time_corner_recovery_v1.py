from copy import deepcopy
from dataclasses import asdict, replace
import math

import numpy as np
import pytest

from aic_transfuser_lite.data.time_corner_recovery_v1 import (
    corner_coverage, partition_corner_sites, pending_corner_laps,
)
from aic_transfuser_lite.data.time_large_recovery_v1 import (
    LargeRecoverySite, LargeRecoveryConfig, LargeRecoveryState, propose_large_recovery,
    large_recovery_events,
)
from aic_transfuser_lite.data.time_large_recovery_reference_v1 import preparation_lateral
from test_time_large_recovery_v1 import synthetic_run


def sites():
    return [LargeRecoverySite(f'C{i+1:02}', s, .2, settle_distance_m=4.,
                target_heading_rad=math.radians(4.), heading_tolerance_rad=math.radians(1.),
                corner_id=f'C{i+1:02}')
            for i, s in enumerate((24., 73., 134., 161., 182., 211., 235., 248., 264., 296., 315.))]


def test_every_corner_is_assigned_once_including_last_corner_and_close_pairs():
    laps = partition_corner_sites(sites())
    assert len(laps) == 4
    assert sorted(s.corner_id for lap in laps for s in lap.sites) == [s.corner_id for s in sites()]
    assert all(len(lap.sites) <= 3 for lap in laps)
    assert all(b.start_s_m-a.start_s_m >= 40. for lap in laps for a, b in zip(lap.sites, lap.sites[1:]))
    with pytest.raises(ValueError):
        partition_corner_sites([sites()[0], sites()[0]])


def test_heading_goal_cannot_be_satisfied_by_old_aligned_state():
    site = sites()[4]
    cfg = LargeRecoveryConfig((site,))
    state = LargeRecoveryState(stage='preparing', event_id=1, approach_seen=True,
        start_ns=1, start_wall_ns=1, target_since_ns=500_000_000,
        last_sim_ns=950_000_000, last_wall_ns=950_000_000)
    values = dict(sim_ns=1_000_000_000, wall_ns=1_000_000_000, s_m=site.release_s_m,
        speed_mps=1.3, lateral_m=.2, nominal_stamp_ns=1_000_000_000, entry_clear=True)
    assert propose_large_recovery(cfg, state, heading_rad=0., **values).transition is None
    assert propose_large_recovery(cfg, state, heading_rad=math.radians(4.), **values).transition == 'request'
    with pytest.raises(ValueError):
        replace(site, target_heading_rad=math.radians(12.))
    with pytest.raises(ValueError):
        replace(site, heading_tolerance_rad=float('nan'))


def test_heading_profile_has_correct_release_position_and_slope_and_bounded_preview():
    site = sites()[0]
    x = np.array([site.start_s_m, site.release_s_m-1e-4, site.release_s_m,
                  site.release_s_m+1e-4, site.release_s_m+6., site.release_s_m+14.])
    y = preparation_lateral(x, site)
    assert y[0] == 0. and y[2] == pytest.approx(.2)
    assert (y[3]-y[1])/2e-4 == pytest.approx(math.tan(math.radians(4.)), abs=1e-5)
    assert y[4] == y[5] and y[5] < .5
    with pytest.raises(ValueError, match='SHAPE'):
        preparation_lateral(np.zeros((3, 2)), site)


def test_short_preparation_changes_start_without_moving_corner_goal_or_relaxing_spacing():
    site=replace(sites()[4],approach_distance_m=4.)
    assert site.start_s_m==site.release_s_m-8.
    y=preparation_lateral(np.array([site.start_s_m,site.release_s_m]),site)
    np.testing.assert_allclose(y,[0.,.2],atol=1e-10)
    # Different approach lengths must still be spaced by start, not release.
    other=replace(site,site_id='C12',corner_id='C12',release_s_m=site.release_s_m+40.,approach_distance_m=8.)
    assert len(partition_corner_sites([site,other]))==2
    with pytest.raises(ValueError):
        replace(site,approach_distance_m=2.)


def test_offline_boundary_uses_requested_heading_and_preserves_legacy_zero_goal():
    _, rows = synthetic_run(LargeRecoveryConfig((LargeRecoverySite('C01',60.,.2),)))
    assert large_recovery_events(rows)[0]['recovery_confirmed']
    wrong = deepcopy(rows)
    for r in wrong:
        r['large_recovery']['config']['sites'][0].update(target_heading_rad=math.radians(4.), corner_id='C01')
    assert not large_recovery_events(wrong)[0]['recovery_confirmed']
    for r in wrong:
        if r['large_recovery']['command_source'] == 'preparation':
            r['large_recovery']['heading_error_rad'] = math.radians(4.)
    assert large_recovery_events(wrong)[0]['recovery_confirmed']


def audited_run(name='r1', split='train'):
    site = sites()[0]
    event = dict(event_id=1, corner_id=site.corner_id, target_offset_m=.2,
        target_heading_rad=site.target_heading_rad, recovery_confirmed=True, completed=True, stable_at_end=True)
    anchors = [dict(anchor_id=str(i), event_id=1, site_id=site.site_id, base_s_m=24.+i*.05,
        lateral_m=.2 if i < 4 else 0., heading_rad=site.target_heading_rad if i < 4 else 0., speed_mps=1.3)
        for i in range(70)]
    return dict(run_id=name, split=split, result_status='COMPLETE_LAP', fault=None, stop_confirmed=True,
        closed_bag=True, accepted=len(anchors), events=[event], anchor_states=anchors)


def test_coverage_requires_valid_entry_state_in_both_whole_run_splits():
    a = audited_run()
    c = corner_coverage(sites(), [a])
    assert c['missing']['train'] == [s.corner_id for s in sites()[1:]]
    assert len(c['missing']['validation']) == 11 and not c['complete']
    laps = pending_corner_laps(sites(), c, split='train')
    assert {s.corner_id for lap in laps for s in lap.sites} == set(c['missing']['train'])
    wrong = deepcopy(a)
    for row in wrong['anchor_states']:
        row['heading_rad'] = 0.
    assert len(corner_coverage(sites(), [wrong])['missing']['train']) == 11
    wrong = deepcopy(a); wrong['result_status'] = 'STOPPED'
    assert len(corner_coverage(sites(), [wrong])['missing']['train']) == 11
    with pytest.raises(ValueError, match='SPLIT_IDENTITY'):
        corner_coverage(sites(), [a, a])
    c['missing']['train'] = []
    with pytest.raises(ValueError, match='PENDING_IDENTITY'):
        pending_corner_laps(sites(), c, split='train')


def test_entry_coverage_keeps_overspeed_anchors_only_under_recorded_policy():
    audit = audited_run()
    for row in audit['anchor_states']:
        row['speed_mps'] = 1.8
    assert 'C01' in corner_coverage(sites(), [audit])['missing']['train']
    audit['events'][0]['speed_policy'] = 'record_actual_v1'
    assert 'C01' not in corner_coverage(sites(), [audit])['missing']['train']
    audit['anchor_states'][0]['speed_mps'] = float('nan')
    with pytest.raises(ValueError, match='FINITE'):
        corner_coverage(sites(), [audit])
