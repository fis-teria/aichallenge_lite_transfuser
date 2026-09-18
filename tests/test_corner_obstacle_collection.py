import math

import pytest

from tools.prepare_corner_obstacle_collection import collection_document, required_finish_progress
from tools.audit_native_corner_collection import corner_coverage


def point(i):
    return dict(site=f'corner_{i:02d}', map_pose=[10., 20., math.pi/2])


def plan():
    return dict(seed=1,sim_timeout_s=780,image='test')


def test_all_twelve_corners_are_native_objects_with_no_vehicle_domains():
    doc=collection_document(plan(),'case',[point(i) for i in range(1,13)],340.547)
    assert doc['expect']['finish'] == {'ego_reference_s_greater_than':400.547}
    assert len(doc['objects']) == 12
    assert [o['type'] for o in doc['objects']] == ['box','cone']*6
    assert doc['objects'][0]['pose'] == dict(map_xy=[10.,20.], yaw_deg=90.)
    assert 'actors' not in doc and 'clearance' not in doc['expect']
    assert doc['simulator']['collisions'] == 'on'
    assert doc['runtime']['rosbag'] is True


@pytest.mark.parametrize('members,length', [([],340),([point(1)]*2,340),
    ([point(i) for i in range(33)],340),([point(1)],float('nan')),([point(1)],60),
    ([dict(site='a',map_pose=[1,2])],340),([dict(site='a',map_pose=[1,2,float('inf')])],340)])
def test_bad_count_duplicate_shape_and_units_rejected(members,length):
    with pytest.raises(ValueError):
        collection_document(plan(),'case',members,length)


def test_thirty_two_objects_allowed_and_yaw_normalized():
    points=[point(i) for i in range(32)]
    points[0]['map_pose'][2]=3*math.pi
    doc=collection_document(plan(),'case',points,340)
    assert len(doc['objects'])==32
    assert abs(doc['objects'][0]['pose']['yaw_deg']) == pytest.approx(180)


def location(s=10):
    return dict(site='corner_01',object_id='corner_01_box',object_type='box',
                monitor_s_m=s,map_pose=[10.,0.,0.])


def sample(s,t,truth=True):
    return dict(time=t,ego=dict(stamp=t+7),ego_gt=dict(progress_m=s,x=s,y=2.,source='gnss' if truth else 'estimate'))


def test_native_coverage_waits_for_next_lap_when_corner_is_behind_grid():
    rows=[sample(i, (i-24)*.1) for i in range(24,141)]
    result=corner_coverage(rows,[location()],100)[0]
    assert result['first_encounter_progress_m']==110
    assert result['passed'] and result['post_distance_observed']
    assert result['crossing_sim_s']==pytest.approx(15.6)
    assert not corner_coverage(rows[:70],[location()],100)[0]['passed']


def test_native_coverage_does_not_infer_pass_from_endpoint_or_teleport():
    for rows in ([sample(1,0),sample(40,.1)], [sample(1,0),sample(9,.8),sample(11,3)],
                 [sample(11,0),sample(40,3)], [sample(i,i*.1,False) for i in range(40)]):
        assert not corner_coverage(rows,[location()],100)[0]['passed']
    with pytest.raises(ValueError):
        corner_coverage([],[location(-1)],100)


def test_finish_covers_wrapped_first_corner_without_requiring_the_next_obstacle():
    length = 340.5467525855492
    stations = [7.578035117890783, 38.45644825210893, 319.1095321865732]
    finish = required_finish_progress(24., stations, length, 25.)
    assert finish == pytest.approx(373.125)
    assert finish >= length + stations[0] + 25.
    assert finish < length + stations[1]
    members = [dict(point(i), monitor_s_m=s) for i, s in enumerate(stations, 1)]
    document = collection_document(dict(plan(), initial_progress_m=24.), 'case', members, length)
    assert document['expect']['finish']['ego_reference_s_greater_than'] == finish


def test_finish_without_wrap_preserves_full_post_distance():
    assert required_finish_progress(1., [10., 40.0001], 100., 25.) == pytest.approx(65.001)


@pytest.mark.parametrize('start,stations,length,post', [
    (24., [], 100., 25.), (24., [float('nan')], 100., 25.),
    (100., [10.], 100., 25.), (0., [-1.], 100., 25.),
    (0., [100.], 100., 25.), (0., [10.], 100., -1.), (0., [0.], 0., 25.)])
def test_finish_rejects_invalid_station_units_and_nonfinite_inputs(start, stations, length, post):
    with pytest.raises(ValueError):
        required_finish_progress(start, stations, length, post)
