import numpy as np
from aic_transfuser_lite.data.spatial_long_view_v4 import LONG_GRID,long_spatial_target


def future(speed):
    x=np.zeros((30,8));x[:,0]=np.arange(1,31)/10;x[:,1]=x[:,0]*speed
    x[:,4]=speed;x[:,7]=1
    return x


def test_long_straight_and_input_unchanged():
    f=future(8);copy=f.copy();r=long_spatial_target(f)
    assert len(LONG_GRID)==46 and LONG_GRID[-1]==20
    assert r['mask'].all() and r['xy'].shape==(46,2)
    np.testing.assert_allclose(r['xy'][:,0],LONG_GRID)
    np.testing.assert_array_equal(f,copy)


def test_short_recovery_is_not_extended():
    r=long_spatial_target(future(.4))
    assert not r['mask'][20:].any()
    assert np.all(r['xy'][~r['mask']]==0)


def test_invalid_prefix_cannot_resume():
    f=future(8);f[3,7]=0;r=long_spatial_target(f)
    assert r['observed_length_m']<2.5 and not r['mask'][20:].any()
