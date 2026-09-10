import math
import pytest
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import lidar_to_root,display_points


def test_rotated_offset():
    x,y,a=lidar_to_root(2.,4.,math.pi/2,1.)
    assert x==pytest.approx(2.) and y==pytest.approx(3.)


def test_observation_pose_display_preserves_raw():
    raw=[[1.,0.] for _ in range(20)]
    record=dict(event='PLAN',raw_xy_m=raw,observation_pose_xyyaw=[2.,3.,math.pi/2])
    assert display_points(record)[0]==pytest.approx((2.,4.))
    assert raw==[[1.,0.] for _ in range(20)]


def test_invalid_output():
    with pytest.raises(ValueError):display_points(dict(event='PLAN',raw_xy_m=[],observation_pose_xyyaw=[0.,0.,0.]))
    with pytest.raises(ValueError):lidar_to_root(float('nan'),0.,0.,1.)
