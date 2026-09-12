import numpy as np
import pytest
import json
from tools.mppi_cma.geometry import closed_path_offsets,fit_similarity,convex_overlap,vehicle_polygon
from tools.mppi_cma.verify_tracks import verify


def test_periodic_offset_metres_and_identity():
    theta=np.linspace(0,2*np.pi,128,endpoint=False)
    xy=10*np.column_stack([np.cos(theta),np.sin(theta)])
    np.testing.assert_array_equal(closed_path_offsets(xy,np.zeros(16)),xy)
    moved=closed_path_offsets(xy,np.ones(16))
    np.testing.assert_allclose(np.linalg.norm(moved,axis=1),9,atol=1e-8)
    assert moved.shape==(128,2)


def test_offset_invalid_shapes_and_nonfinite():
    with pytest.raises(ValueError):closed_path_offsets(np.zeros((4,3)),np.ones(4))
    with pytest.raises(ValueError):closed_path_offsets(np.ones((5,2)),np.ones(4))
    with pytest.raises(ValueError):closed_path_offsets(np.ones((5,2)),np.array([0,0,np.nan,0]))


def test_body_overlap_includes_edge_crossing_and_margin():
    body=vehicle_polygon(0,0,0)
    crossing=np.array([[.5,-2],[.6,-2],[.6,2],[.5,2]])
    assert convex_overlap(body,crossing)
    remote=np.array([[1.7,-.5],[2,-.5],[2,.5],[1.7,.5]])
    assert not convex_overlap(body,remote)
    assert convex_overlap(body,remote,.1)
    assert not convex_overlap(vehicle_polygon(0,0,np.pi/2),remote,.1)


def test_similarity_coordinate_direction_and_holdout():
    source=np.array([[0,0],[30,0],[0,40],[30,40]])
    matrix=np.array([[0,1],[-1,0]])
    translation=np.array([89638,43503])
    fit=fit_similarity(source,source@matrix.T+translation)
    np.testing.assert_allclose(fit['matrix'],matrix,atol=1e-9)
    np.testing.assert_allclose(fit['translation'],translation,atol=1e-8)
    assert fit['max_residual_m']<1e-7
    with pytest.raises(ValueError):fit_similarity(np.zeros((4,2)),np.zeros((4,2)))


def test_interpolation_detects_between_sample_lane_crossing(tmp_path):
    episode=tmp_path/'episodes'/'crossing';episode.mkdir(parents=True)
    (tmp_path/'calibration.json').write_text(json.dumps({'ot_lane_polygon_map_m':[[-.1,-2],[.1,-2],[.1,2],[-.1,2]]}))
    (episode/'track.csv').write_text('stamp_s,x_m,y_m,yaw_rad,ot_overlap\n0,-3,0,0,0\n1,3,0,0,0\n')
    result=verify(tmp_path,'crossing')
    assert not result['passed'] and result['overlap_s']>0 and result['samples_tested']>=120
    (episode/'track.csv').write_text('stamp_s,x_m,y_m,yaw_rad,ot_overlap\n0,-3,0,0,0\n1,30,0,0,0\n')
    with pytest.raises(ValueError,match='jump'):verify(tmp_path,'crossing')
