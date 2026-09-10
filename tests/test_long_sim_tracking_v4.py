import numpy as np
import pytest
from aic_transfuser_lite.control.long_sim_tracking_v4 import tracking_command,check_scan
from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID

def test_pp_forward_and_frame_transform():
    raw=np.c_[LONG_GRID,np.zeros(46)]
    a=tracking_command(raw,(0.,0.,0.),(0.,0.,0.),0.)
    b=tracking_command(raw,(5.,7.,np.pi/2),(5.,7.,np.pi/2),0.)
    assert abs(a['steer_rad'])<1e-6 and b['steer_rad']==pytest.approx(a['steer_rad'],abs=1e-6)
    assert a['acceleration_mps2']>0 and a['prefix_points']<46
    with pytest.raises(ValueError):tracking_command(raw,(0.,0.,0.),(0.,0.,0.),.6)
    raw[0,0]=np.nan
    with pytest.raises(ValueError):tracking_command(raw,(0.,0.,0.),(0.,0.,0.),0.)

def test_lidar_stopping_and_unknown():
    r=np.full(750,20.)
    args=(-np.pi/2,np.pi/749,.1,30.,.25)
    assert check_scan(r,*args)>1.
    r[375]=.5
    with pytest.raises(ValueError,match='OCCUPIED'):check_scan(r,*args)
    r[375]=np.nan
    with pytest.raises(ValueError,match='UNKNOWN'):check_scan(r,*args)
