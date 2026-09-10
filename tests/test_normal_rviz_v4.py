import numpy as np
import pytest
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import lidar_display_points
import importlib.util
from pathlib import Path
spec=importlib.util.spec_from_file_location('normal_rviz',Path(__file__).parents[1]/'tools/integrate_normal_rviz_v4.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
integrate,DISPLAY=module.integrate,module.DISPLAY


def test_preserve_normal_rviz_and_refuse_duplicate():
    original='Panels: []\nVisualization Manager:\n  Class: ""\n  Displays:\n    - Name: Existing\n  Global Options:\n    Fixed Frame: map\n'
    result=integrate(original)
    assert result.replace(DISPLAY,'')==original
    with pytest.raises(ValueError):integrate(result)
    with pytest.raises(ValueError):integrate('unknown')


def test_display_mount_is_rigid_and_keeps_full_raw():
    raw=np.c_[np.arange(46)*.2,np.sin(np.arange(46))]
    record=dict(event='PLAN',source='LONG_V4_20M_EPOCH12_UNCORRECTED',raw_xy_m=raw.tolist(),observation_pose_xyyaw=[0.,0.,0.])
    xyz=np.array(lidar_display_points(record))
    assert xyz.shape==(46,3)
    assert np.allclose(np.diff(xyz[:,:2],axis=0),np.diff(raw,axis=0))
    assert np.allclose(xyz[:,0]+1.1649999618530273,raw[:,0])
    assert np.array_equal(record['raw_xy_m'],raw)
