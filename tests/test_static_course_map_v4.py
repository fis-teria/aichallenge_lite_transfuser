from pathlib import Path
import numpy as np
import pytest
import yaml
from aic_transfuser_lite.control.static_course_map_v4 import classify,StaticCourseMap,load_map


def test_trinary_no_smoothing_and_negate():
    p=np.array([[0,205,255]],np.uint8)
    assert classify(p,negate=0,free_thresh=.196,occupied_thresh=.65).tolist()==[[100,-1,0]]
    assert classify(p,negate=1,free_thresh=.196,occupied_thresh=.65).tolist()==[[0,100,100]]
    p=np.array([[0,255,0],[255,255,255],[0,255,0]],np.uint8)
    assert (classify(p,negate=0,free_thresh=.196,occupied_thresh=.65)==100).sum()==4


def test_exact_threshold_unknown():
    assert classify(np.array([[0,255]],np.uint8),negate=0,free_thresh=0.,occupied_thresh=1.).tolist()==[[-1,-1]]


def test_pixel_centres_and_outside_not_clipped():
    m=StaticCourseMap(np.array([[100,-1],[0,100]],np.int8),1.,(10.,20.,0.),{})
    assert m.query(np.array([[10.5,21.5],[10.5,20.5],[11.5,21.5],[9.99,20.5],[12.,20.5]])).tolist()==[100,0,-1,-1,-1]


def test_rotated_map_and_bad_query():
    m=StaticCourseMap(np.array([[100]],np.int8),1.,(10.,20.,np.pi/2),{})
    assert m.query(np.array([[9.5,20.5]])).tolist()==[100]
    with pytest.raises(ValueError):m.query(np.array([[np.nan,0]]))
    with pytest.raises(ValueError):m.query(np.zeros(2))


@pytest.mark.parametrize('kwargs',[dict(negate=2,free_thresh=.2,occupied_thresh=.6),
    dict(negate=0,free_thresh=.7,occupied_thresh=.6),dict(negate=0,free_thresh=.2,occupied_thresh=np.nan)])
def test_bad_thresholds(kwargs):
    with pytest.raises(ValueError):classify(np.zeros((1,1),np.uint8),**kwargs)


def test_explicit_sibling_only_and_immutable_inputs(tmp_path):
    from PIL import Image
    p=tmp_path/'map.yaml';img=tmp_path/'map.pgm'
    Image.fromarray(np.array([[0,255]],np.uint8)).save(img)
    config=dict(image='map.pgm',resolution=.1,origin=[1,2,0],negate=0,free_thresh=.196,occupied_thresh=.65)
    p.write_text(yaml.safe_dump(config)); before=(p.read_bytes(),img.read_bytes())
    m=load_map(p)
    assert m.grid.tolist()==[[100,0]] and not m.grid.flags.writeable
    assert not m.provenance['runtime_permission'] and (p.read_bytes(),img.read_bytes())==before
    config['image']='../elsewhere.pgm';p.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):load_map(p)
