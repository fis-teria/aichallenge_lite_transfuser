"""Known geometry validates direction, units and unobservable scan motion."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('motion_diagnostic',Path(__file__).parents[1]/'tools/diagnose_lidar_wheel_motion.py')
module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=module
spec.loader.exec_module(module)


def room() -> np.ndarray:
    corners=np.array([[-5,-4],[9,-4],[9,6],[-5,6],[-5,-4]])
    return np.concatenate([a+(b-a)*np.linspace(0,1,200)[:,None] for a,b in zip(corners,corners[1:])])


def test_scan_motion_direction_and_reverse() -> None:
    reference=room(); truth=np.array([.2,-.025,.015])
    current=module.transform(reference,module.inverse(truth))
    result=module.register(reference,current)
    assert result.supported and result.weak_ratio > .001
    np.testing.assert_allclose(result.pose,truth,atol=.002)
    reverse=module.register(current,reference)
    np.testing.assert_allclose(module.compose(result.pose,reverse.pose),np.zeros(3),atol=.002)


def test_mount_lever_and_base_units() -> None:
    base=np.array([.21,.01,-.03])
    lidar=module.compose(module.compose(module.inverse(module.MOUNT),base),module.MOUNT)
    assert abs(lidar[1]-base[1]) > .04
    np.testing.assert_allclose(module.base_motion(lidar),base,atol=1e-12)


def test_corridor_is_not_longitudinal_ground_truth() -> None:
    x=np.linspace(-12,12,350)
    reference=np.concatenate([np.column_stack((x,np.full_like(x,-4))),[[np.nan,np.nan]],np.column_stack((x,np.full_like(x,4)))])
    current=reference.copy()
    current[np.isfinite(reference).all(axis=1)]-=np.array([.20,.02])
    first=module.register(reference,current)
    seeded=module.register(reference,current,np.array([.2,0,0]))
    assert first.weak_ratio < 1e-8
    assert abs(first.pose[0]-seeded.pose[0]) > .15
    assert first.median_m < .001 and seeded.median_m < .001


def test_invalid_beams_keep_adjacency_and_metre_units() -> None:
    scan=SimpleNamespace(ranges=[1,np.nan,np.inf,16,0,.2,2],angle_min=0,angle_increment=.1,range_min=.1,range_max=30)
    result=module.cloud_from_scan(scan)
    assert result.shape==(7,2)
    assert np.isnan(result[1:6]).all()
    np.testing.assert_allclose(np.linalg.norm(result[[0,6]],axis=1),[1,2])


def test_missing_surface_is_explicit() -> None:
    with pytest.raises(ValueError,match='CLOUD_SHAPE'):
        module.register(np.zeros((9,3)),room())
    with pytest.raises(ValueError,match='INSUFFICIENT_SURFACE'):
        module.register(np.full((100,2),np.nan),room())
