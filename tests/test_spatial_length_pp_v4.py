"""Synthetic saved-reference contract; no saved inputs or controller execution."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from evaluate_spatial_length_pp_v4 import prepared


def record():
    return dict(new_reason=None,reference_xy_m=[[0.,0.],[1.,0.]],diagnostics=dict(
        length_policy='ENDPOINT_NORMALIZED_LENGTH_V1',reference_frame='base_link@t_obs',fit_success=True,
        maximum_deviation_bound_m=.08,reference_actual_s_m=[0.,1.],reference_yaw_rad=[0.,0.]))


def test_accepted_shape():
    assert prepared(record()).world_xy.shape==(2,2)


@pytest.mark.parametrize('key,value',[('maximum_deviation_bound_m',.101),('maximum_deviation_bound_m',float('nan')),
    ('reference_actual_s_m',[0.,.5]),('reference_yaw_rad',[0.]),('fit_success',False),('reference_frame','world')])
def test_invalid_saved_reference(key,value):
    r=record();r['diagnostics'][key]=value
    with pytest.raises(ValueError):prepared(r)


def test_rejected_case_never_promoted():
    r=record();r['new_reason']='BOUNDED_FIT_INFEASIBLE'
    with pytest.raises(ValueError):prepared(r)
