import numpy as np
import pytest
from tools.curate_native_teacher_data import placement_distances


def test_single_cone_absent_box_and_rotated_pose():
    hull=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
    b,c,g=placement_distances([dict(object_type='cone',map_pose=[3.,0.,0.])],
        np.array([[0.,0.,0.],[1.,0.,np.pi/2]]),hull)
    np.testing.assert_array_equal(b,[1e6,1e6])
    np.testing.assert_allclose(c,[3.,2.]);np.testing.assert_allclose(g,[1.825,.825])


def test_repeated_six_by_six_preserves_nearest_geometry():
    hull=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
    places=[dict(object_type='box',map_pose=[4.,0.,0.]),dict(object_type='cone',map_pose=[3.,0.,0.])]
    a=placement_distances(places,np.zeros((1,3)),hull)
    b=placement_distances(places*6,np.zeros((1,3)),hull)
    np.testing.assert_array_equal(a,b)
    boxes=placement_distances(places[:1],np.zeros((1,3)),hull)
    np.testing.assert_array_equal(boxes,np.array([[4.],[1e6],[1e6]]))


@pytest.mark.parametrize('placements',[[],[dict(object_type='cart',map_pose=[1.,0.,0.])],
    [dict(object_type='cone',map_pose=[float('nan'),0.,0.])]])
def test_invalid_native_placements_fail(placements):
    with pytest.raises(ValueError):placement_distances(placements,np.zeros((1,3)),np.array([[0.,0.],[1.,0.],[0.,1.]]))
