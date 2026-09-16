import math
import numpy as np
import pytest

from aic_transfuser_lite.data.recovery_reference_v3 import OccupancyMapV3
from aic_transfuser_lite.data.time_recovery_map_body_v1 import body_path_is_free, body_polyline_is_free


def grid() -> OccupancyMapV3:
    return OccupancyMapV3(np.ones((401,401),dtype=bool),.05,0.,0.)


def occupy(occupancy: OccupancyMapV3, x: float, y: float) -> None:
    occupancy.free[400-round(y/.05),round(x/.05)] = False


def test_oriented_body_fits_corridor_without_shrinking_standard_half_width():
    occupancy=grid()
    occupancy.free[:,round(10.95/.05):] = False
    xy=np.array([[10.,5.],[10.,7.]])
    assert not occupancy.footprint_is_free(xy[:,0],xy[:,1],1.4)
    assert body_polyline_is_free(occupancy,xy)
    # A wall inside the same standard .85 m half width must still reject.
    occupy(occupancy,10.8,6.)
    assert not body_polyline_is_free(occupancy,xy)


@pytest.mark.parametrize('point',[(11.9,10.),(9.5,10.),(10.,10.8),(10.2,10.2)])
def test_front_rear_side_and_interior_occupied_cells_reject(point):
    occupancy=grid();occupy(occupancy,*point)
    assert not body_path_is_free(occupancy,np.array([[10.,10.,0.],[10.01,10.,0.]]))


def test_obstacle_between_distant_vertices_is_not_skipped():
    occupancy=grid();occupy(occupancy,10.,10.)
    assert not body_path_is_free(occupancy,np.array([[5.,10.,0.],[15.,10.,0.]]))


def test_rotation_sweeps_corners_even_when_both_endpoint_bodies_are_free():
    occupancy=grid();occupy(occupancy,11.2,11.2)
    a=np.array([10.,10.,0.]);b=np.array([10.,10.,math.pi/2.])
    assert body_path_is_free(occupancy,np.stack((a,a)))
    assert body_path_is_free(occupancy,np.stack((b,b)))
    assert not body_path_is_free(occupancy,np.stack((a,b)))


def test_heading_wrap_uses_short_arc_and_map_boundary_is_occupied():
    occupancy=grid();occupy(occupancy,11.5,10.)
    assert body_path_is_free(occupancy,np.array([[10.,10.,math.pi-.01],[10.,10.,-math.pi+.01]]))
    assert not body_path_is_free(occupancy,np.array([[.1,.1,0.],[.2,.1,0.]]))


@pytest.mark.parametrize('bad',[np.zeros((2,2)),np.zeros((1,3)),np.full((2,3),np.nan)])
def test_invalid_pose_shape_and_nonfinite_values_are_explicit(bad):
    with pytest.raises(ValueError,match='MAP_BODY_POSES_SHAPE'):
        body_path_is_free(grid(),bad)


def test_duplicate_vertices_are_not_given_an_arbitrary_heading():
    with pytest.raises(ValueError,match='MAP_BODY_DUPLICATE_VERTEX'):
        body_polyline_is_free(grid(),np.array([[10.,10.],[10.,10.]]))
