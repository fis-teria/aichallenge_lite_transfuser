from copy import deepcopy

import pytest

from aic_transfuser_lite.runtime.recovery_parallel_v1 import (
    cpu_ids, new_rviz_window, validate_domain, validate_parallel_containers, validate_parallel_plan,
)


def plan():
    return dict(schema='awsim_recovery_parallel_v1', scope='time_recovery_parallel_test', instances=[
        dict(run_id='codex-time-recovery-a', ros_domain_id=1, network_container='codex-time-recovery-a-net',
             node_cpus='0-3', simulation_cpus='8-9,12-15'),
        dict(run_id='codex-time-recovery-b', ros_domain_id=2, network_container='codex-time-recovery-b-net',
             node_cpus='4-7', simulation_cpus='10-11,16-19')])


def inspections():
    result = []
    for row in plan()['instances']:
        name = row['run_id']
        result.append(dict(Name='/'+name+'-net', Id=name+'-id',
            Config=dict(Labels={'aic.recovery.scope':plan()['scope'], 'aic.recovery.run_id':name}),
            HostConfig=dict(NetworkMode='none', Privileged=False)))
        result.append(dict(Name='/'+name+'-simulator-1', Id=name+'-sim-id',
            Config=dict(Labels={'com.docker.compose.project':name}),
            HostConfig=dict(NetworkMode='container:'+name+'-id')))
    return result


def test_two_private_namespaces_and_domains():
    p = plan()
    assert validate_parallel_plan(p, scope=p['scope'], run_id=p['instances'][1]['run_id'])['ros_domain_id'] == 2
    validate_parallel_containers(p, inspections())
    validate_domain(2, {'ROS_DOMAIN_ID':'2'})
    assert cpu_ids('0-3,8') == {0,1,2,3,8}


@pytest.mark.parametrize('change', ['domain', 'cpu', 'network', 'duplicate', 'count'])
def test_conflicting_plans_rejected(change):
    p = deepcopy(plan())
    if change == 'domain': p['instances'][1]['ros_domain_id'] = 1
    if change == 'cpu': p['instances'][1]['node_cpus'] = '2-5'
    if change == 'network': p['instances'][1]['network_container'] = p['instances'][0]['network_container']
    if change == 'duplicate': p['instances'][1]['run_id'] = p['instances'][0]['run_id']
    if change == 'count': p['instances'].pop()
    with pytest.raises(ValueError): validate_parallel_plan(p, scope=p['scope'], run_id=p['instances'][0]['run_id'])


@pytest.mark.parametrize('change', ['host', 'other_namespace', 'unrelated', 'missing_holder', 'wrong_scope'])
def test_cross_environment_access_rejected(change):
    items = inspections()
    if change == 'host': items[1]['HostConfig']['NetworkMode'] = 'host'
    if change == 'other_namespace': items[1]['HostConfig']['NetworkMode'] = 'container:codex-time-recovery-b-id'
    if change == 'unrelated': items[1]['Config']['Labels']['com.docker.compose.project'] = 'unrelated'
    if change == 'missing_holder': items.pop(0)
    if change == 'wrong_scope': items[0]['Config']['Labels']['aic.recovery.scope'] = 'another'
    with pytest.raises(ValueError): validate_parallel_containers(plan(), items)


@pytest.mark.parametrize('domain', [0, -1, 102, True, 1.0])
def test_invalid_domains(domain):
    with pytest.raises(ValueError): validate_domain(domain)


def test_domain_environment_mismatch():
    with pytest.raises(ValueError): validate_domain(2, {'ROS_DOMAIN_ID':'1'})


def test_rviz_belongs_to_new_instance():
    assert new_rviz_window({'0x1'}, ['0x1 "autoware.rviz"', '0x2 "autoware.rviz"']) == '0x2'
    with pytest.raises(ValueError): new_rviz_window({'0x1'}, ['0x1 "autoware.rviz"'])
    with pytest.raises(ValueError): new_rviz_window(set(), ['0x1 "autoware.rviz"', '0x2 "autoware.rviz"'])
