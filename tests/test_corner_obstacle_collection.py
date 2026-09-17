import math

import pytest

from tools.prepare_corner_obstacle_collection import collection_document


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
