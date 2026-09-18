from copy import deepcopy
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from mppi_collection_isolation import validate_plan, validate_holder, patch_generated, validate_teacher_actors


def test_domains_services_and_vehicle_identity():
    services={k:dict(environment={}) for k in ('scn-car1','scn-rosbag1','scn-simulator','scn-monitor')}
    monitor=dict(ego_domain=1,ego_ground_truth_domain=2,vehicles=[dict(domain=1,awsim_index=1,v2x_id='d1'),dict(domain=2,awsim_index=2)])
    patch_generated(services,monitor,domain=2,holder='codex-avoidance-b',cpus='4-7')
    assert monitor['ego_domain']==2 and monitor['ego_ground_truth_domain']==3
    assert monitor['vehicles'][0]==dict(domain=2,awsim_index=1,v2x_id='d1')
    assert all(s['network_mode']=='container:codex-avoidance-b' for s in services.values())
    assert services['scn-car1']['environment']['ROS_DOMAIN_ID']=='2'
    assert services['scn-rosbag1']['environment']['ROS_DOMAIN_ID']=='2'


def test_reject_shared_or_unowned_namespaces():
    plan=dict(instances=[dict(domain=1,holder='codex-avoidance-a'),dict(domain=2,holder='codex-avoidance-b')])
    assert validate_plan(plan,2)['holder']=='codex-avoidance-b'
    bad=deepcopy(plan);bad['instances'][1]['holder']='codex-avoidance-a'
    with pytest.raises(ValueError):validate_plan(bad,2)
    holder=dict(Name='/codex-avoidance-a',State=dict(Running=True),HostConfig=dict(NetworkMode='none',Privileged=False),Config=dict(Labels={'codex.owner':'avoidance-collection'}))
    validate_holder(holder,'codex-avoidance-a')
    for key,value in [('NetworkMode','host'),('Privileged',True)]:
        bad=deepcopy(holder);bad['HostConfig'][key]=value
        with pytest.raises(ValueError):validate_holder(bad,'codex-avoidance-a')


def test_physical_slow_actor_only():
    validate_teacher_actors([dict(profile='static_physical'),dict(profile='line_trace',speed_mps=3/3.6)])
    for actor in [dict(profile='v2x_ghost'),dict(profile='line_trace',speed_mps=float('nan')),
                  dict(profile='line_trace',speed_mps=3.),dict(profile='line_trace',speed_mps=0)]:
        with pytest.raises(ValueError):validate_teacher_actors([actor])
