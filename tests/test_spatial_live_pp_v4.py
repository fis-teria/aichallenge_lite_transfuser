"""Synthetic-only PP connection and cumulative authorization checks."""
import json
from pathlib import Path
import sys
import numpy as np
import pytest
import yaml
from aic_transfuser_lite.control.spatial_tracking_contracts_v4 import PreparedPath
from aic_transfuser_lite.control.spatial_live_pp_v4 import propose

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from spatial_pp_budget_v4 import PPBudget, LIMITS


def fixture():
    cfg = yaml.safe_load((ROOT/'configs/control/spatial_sim_e2e_v4.yaml').read_text())
    cfg['command_schedule_s'] = .05
    s = np.linspace(0, 1.5, 76)
    return PreparedPath(np.c_[s, s*0], s, np.arange(len(s)), np.zeros(len(s)-1),
                        np.zeros(len(s)), {}, None), cfg


def test_straight_hold_and_run():
    path, cfg = fixture()
    a = propose(path,np.zeros(5),0.,cfg,permission='HOLD')
    b = propose(path,np.zeros(5),0.,cfg,permission='RUN')
    assert a['first_control'][0] == 0 and a['stop']
    assert b['first_control'][0] == .1 and b['first_control'][1] == 0
    assert b['states'].shape == (16,5) and not b['mpc_used']
    assert np.isfinite(b['states']).all()


def test_new_path_changes_target_not_old_progress():
    path,cfg=fixture(); z=np.zeros(5)
    raw=path.world_xy.copy()
    straight=propose(path,z,0.,cfg,permission='RUN')
    path.world_xy[:,1]=.1*path.world_xy[:,0]
    left=propose(path,z,0.,cfg,permission='RUN')
    path.world_xy[:,1]*=-1
    right=propose(path,z,0.,cfg,permission='RUN')
    assert left['first_control'][1]>0>right['first_control'][1]
    assert straight['progress_s_m']==left['progress_s_m']==right['progress_s_m']==0
    assert np.array_equal(raw[:,0],path.world_xy[:,0])


@pytest.mark.parametrize('bad',[np.full(5,np.nan),np.zeros(4)])
def test_invalid_state(bad):
    path,cfg=fixture()
    with pytest.raises(ValueError):propose(path,bad,0.,cfg,permission='RUN')


def test_endpoint_and_limit():
    path,cfg=fixture()
    r=propose(path,np.array([1.39,0,0,.2,0]),0.,cfg,permission='RUN')
    assert r['stop'] and r['first_control'][0]<=0


def budget(tmp_path):
    p=tmp_path/'budget.json'
    used=dict(wall_s=1538.,forward=124,mpc=1,snapshots=11,powered=7,powered_s=270.,
              log_bytes=88578313,tiny_forward=5711)
    p.write_text(json.dumps(dict(used=used,active=None,attempts=[],tiny_authorized_limits=dict(LIMITS,powered=7))))
    return PPBudget(p)


def reservation():
    return dict(wall_s=170.,forward=100,mpc=0,snapshots=0,powered=1,powered_s=10.,log_bytes=1000)


def test_authorization_once_preserves_counts_and_caps_four(tmp_path):
    b=budget(tmp_path)
    try:
        before=dict(b.value['used']); a=b.authorize(); assert b.authorize()==a
        assert b.value['used']==before and b.limits['powered']==11
        for i in range(4):
            r=reservation();b.reserve(str(i),r);b.finish(r,exact=True)
        with pytest.raises(ValueError,match='FOUR_PP'):b.reserve('fifth',reservation())
        assert b.value['used']['tiny_forward']==5711 and b.value['used']['forward']==524
    finally:b.close()


def test_deadline_and_tiny_common_cap(tmp_path,monkeypatch):
    b=budget(tmp_path)
    try:
        a=b.authorize()
        b.value['used']['tiny_forward']=7000
        with pytest.raises(ValueError,match='COMMON_FORWARD'):b.reserve('over',reservation())
        monkeypatch.setattr('spatial_pp_budget_v4.time.time',lambda:a['driving_cutoff_unix_s'])
        with pytest.raises(ValueError,match='CUTOFF'):b.reserve('late',reservation())
    finally:b.close()
