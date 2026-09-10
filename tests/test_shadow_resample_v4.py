import numpy as np
import pytest
from aic_transfuser_lite.control.shadow_resample_v4 import resample_shadow


def test_straight_preserve_and_finite_horizon():
    xy=np.c_[np.arange(20)/10,np.zeros(20)];before=xy.copy()
    out,bound=resample_shadow(xy,.3,.03)
    np.testing.assert_array_equal(xy,before)
    np.testing.assert_array_equal(out[[0,-1]],xy[[0,-1]])
    assert len(out)==8 and bound<.002


def test_large_corner_cut_rejected():
    xy=np.c_[np.arange(20)/10,np.zeros(20)];xy[10,1]=.2
    with pytest.raises(ValueError,match='DEVIATION'):resample_shadow(xy,.3,.03)
