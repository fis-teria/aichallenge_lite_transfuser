from copy import deepcopy

import pytest

from aic_transfuser_lite.runtime.recovery_disturbance_markers import DisturbanceLocations


def row(sign=1):
    return dict(annotation_schema='measured_random_steering_pulse_v1', phase='hold',
                publication=dict(sim_ns=1_100_000_000, sequence=20),
                pulse=dict(applied=True, requested_rad=.1*sign, effective_rad=.02*sign),
                random_pulse=dict(config=dict(sites=[dict(site_id='S00', start_s_m=116., sign=sign)]),
                                  state=dict(event_id=1, active_site_index=0)),
                current_pose=dict(x_m=4., y_m=5., yaw_rad=.3, stamp_ns=1_000_000_000),
                projection=dict(s_m=116.2))


@pytest.mark.parametrize('sign,label', [(1, 'S00 LEFT'), (-1, 'S00 RIGHT')])
def test_actual_map_position_and_direction_are_frozen_at_first_nonzero_publication(sign, label):
    trace = DisturbanceLocations(); first = row(sign)
    assert trace.add(first)
    later = deepcopy(first); later['current_pose']['x_m'] = 10.
    assert not trace.add(later)
    event, = trace.report()['events']
    assert event['label'] == label and (event['x_m'], event['y_m']) == (4., 5.)
    assert event['actual_s_m'] == 116.2 and event['planned_s_m'] == 116.
    assert event['effective_rad'] == .02*sign
    assert trace.report()['frame_id'] == 'map'


@pytest.mark.parametrize('key,value', [('phase', 'recovery'), ('publication', None), ('annotation_schema', None)])
def test_non_injection_rows_do_not_create_markers(key, value):
    sample = row(); sample[key] = value
    trace = DisturbanceLocations(); assert not trace.add(sample) and not trace.events


@pytest.mark.parametrize('key,value', [('applied', False), ('requested_rad', 0.), ('effective_rad', 0.), ('effective_rad', -.01)])
def test_rejected_zero_ramp_and_ineffective_commands_are_not_marked(key, value):
    sample = row(); sample['pulse'][key] = value
    trace = DisturbanceLocations(); assert not trace.add(sample)
    assert trace.add(row())


def test_actual_site_index_after_a_skipped_site_is_used():
    sample = row(); sites = sample['random_pulse']['config']['sites']
    sites.append(dict(site_id='R07', start_s_m=233., sign=1))
    sample['random_pulse']['state']['active_site_index'] = 1
    sample['projection']['s_m'] = 233.5
    trace = DisturbanceLocations(); assert trace.add(sample)
    assert trace.events[1].site_id == 'R07'


def test_legacy_unnamed_collections_are_not_relabelled():
    sample = row(); sample['random_pulse']['config']['sites'] = []
    assert not DisturbanceLocations().add(sample)


@pytest.mark.parametrize('field,value', [('x_m', float('nan')), ('y_m', float('inf')), ('yaw_rad', '0')])
def test_invalid_pose_is_explicitly_rejected(field, value):
    sample = row(); sample['current_pose'][field] = value
    with pytest.raises(ValueError, match='DISTURBANCE_MAP_POSE_FINITE'):
        DisturbanceLocations().add(sample)


@pytest.mark.parametrize('event_id,index', [(0, 0), (4, 0), (1, None), (1, 1)])
def test_invalid_event_or_site_index_is_rejected(event_id, index):
    sample = row(); sample['random_pulse']['state'].update(event_id=event_id, active_site_index=index)
    with pytest.raises(ValueError, match='DISTURBANCE_EVENT_OR_SITE_INDEX'):
        DisturbanceLocations().add(sample)


def test_marker_identifiers_cannot_be_reused_for_another_site():
    trace = DisturbanceLocations(); assert trace.add(row())
    sample = row(); sample['random_pulse']['config']['sites'][0]['site_id'] = 'R01'
    with pytest.raises(ValueError, match='DISTURBANCE_EVENT_ID_REUSED'):
        trace.add(sample)
