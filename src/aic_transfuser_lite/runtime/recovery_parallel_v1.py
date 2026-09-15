"""External isolation for two unchanged AWSIM processes.

Vehicle ROS domains differ. Each instance also owns a Docker network namespace,
so the simulator's existing domain-0 administration remains private.
"""
from __future__ import annotations

import re
from typing import Any, Sequence


def validate_domain(domain: int, environment: dict[str, str] | None = None) -> None:
    if type(domain) is not int or not 1 <= domain <= 101:
        raise ValueError('vehicle ROS domain must be an integer in 1..101')
    if environment is not None and environment.get('ROS_DOMAIN_ID') != str(domain):
        raise ValueError('ROS_DOMAIN_ID does not match the declared instance')


def single_vehicle_node_name(domain: int) -> str:
    """AWSIM names nodes by vehicle index, independently of the DDS base domain.

    These launchers explicitly set AWSIM_VEHICLES=1. The observed domain-2
    endpoint and MultiDomainROS2Manager both identify vehicle 1 as awsim_d1.
    """
    validate_domain(domain)
    return 'awsim_d1'


def cpu_ids(value: str) -> set[int]:
    if not re.fullmatch(r'\d+(?:-\d+)?(?:,\d+(?:-\d+)?)*', value):
        raise ValueError('explicit CPU list required')
    result: set[int] = set()
    for token in value.split(','):
        ends = list(map(int, token.split('-')))
        first, last = ends[0], ends[-1]
        if first > last or last > 4095:
            raise ValueError('invalid CPU range')
        result.update(range(first, last + 1))
    return result


def validate_parallel_plan(plan: dict[str, Any], *, scope: str, run_id: str) -> dict[str, Any]:
    if plan.get('schema') != 'awsim_recovery_parallel_v1' or plan.get('scope') != scope:
        raise ValueError('parallel scope mismatch')
    instances = plan.get('instances', [])
    if len(instances) != 2:
        raise ValueError('exactly two bounded instances required')
    domains: set[int] = set()
    names: set[str] = set()
    assigned: set[int] = set()
    for row in instances:
        name = row['run_id']
        if not re.fullmatch(r'codex-time-recovery-[a-z0-9-]+', name) or name in names:
            raise ValueError('unique owned run IDs required')
        validate_domain(row['ros_domain_id'])
        if row['ros_domain_id'] in domains or row['network_container'] != name + '-net':
            raise ValueError('distinct domains and owned network containers required')
        nodes, sim = cpu_ids(row['node_cpus']), cpu_ids(row['simulation_cpus'])
        if nodes & sim or assigned & (nodes | sim):
            raise ValueError('parallel CPU allocations overlap')
        assigned |= nodes | sim
        names.add(name)
        domains.add(row['ros_domain_id'])
    matches = [row for row in instances if row['run_id'] == run_id]
    if len(matches) != 1:
        raise ValueError('run is not in the parallel plan')
    return matches[0]


def validate_parallel_containers(plan: dict[str, Any], inspections: Sequence[dict[str, Any]]) -> None:
    """Reject unrelated containers and any instance outside its private network."""
    rows = {row['run_id']: row for row in plan['instances']}
    by_name = {item['Name'].lstrip('/'): item for item in inspections}
    holders = {}
    for name, row in rows.items():
        holder = by_name.get(row['network_container'])
        if holder is None:
            raise ValueError('owned network namespace is absent')
        labels = holder['Config'].get('Labels') or {}
        if (labels.get('aic.recovery.scope') != plan['scope']
                or labels.get('aic.recovery.run_id') != name
                or holder['HostConfig']['NetworkMode'] != 'none'
                or holder['HostConfig'].get('Privileged', False)):
            raise ValueError('invalid network namespace owner')
        holders[name] = {holder['Id'], row['network_container']}
    for item in inspections:
        labels = item['Config'].get('Labels') or {}
        name = labels.get('aic.recovery.run_id') or labels.get('com.docker.compose.project')
        if name not in rows:
            raise ValueError('unrelated active container present')
        if item['Name'].lstrip('/') == rows[name]['network_container']:
            continue
        mode = item['HostConfig']['NetworkMode']
        if not mode.startswith('container:') or mode.removeprefix('container:') not in holders[name]:
            raise ValueError('container is outside its assigned network namespace')


def new_rviz_window(before: set[str], lines: Sequence[str]) -> str:
    windows = [line.strip().split()[0] for line in lines if 'autoware.rviz' in line]
    candidates = [window for window in windows if window not in before]
    if len(candidates) != 1:
        raise ValueError('exactly one new ordinary RViz window required')
    return candidates[0]
