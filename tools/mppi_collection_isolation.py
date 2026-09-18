"""Per-environment network/domain isolation for two MPPI teacher recorders.

Only generated launch artifacts change. Domain 0 remains inside each isolated
network namespace. The official AWSIM executable and scene are never modified.
"""
from __future__ import annotations
import fcntl
import json
from pathlib import Path
import re
import subprocess
from typing import Any


def validate_plan(plan: dict[str, Any], domain: int) -> dict[str, Any]:
    slots = plan['instances']
    if (len(slots) != 2 or {s['domain'] for s in slots} != {1,2}
            or len({s['holder'] for s in slots}) != 2 or domain not in (1,2)
            or any(not re.fullmatch(r'codex-avoidance-[a-z0-9-]+',s['holder']) for s in slots)):
        raise ValueError('two distinct owned namespaces with domains 1/2 required')
    return next(s for s in slots if s['domain'] == domain)


def validate_holder(row: dict[str, Any], name: str) -> None:
    if (row['Name'].lstrip('/') != name or not row['State']['Running']
            or row['HostConfig']['NetworkMode'] != 'none'
            or row['HostConfig'].get('Privileged',False)
            or row['Config'].get('Labels',{}).get('codex.owner') != 'avoidance-collection'):
        raise ValueError('holder must be owned, running, unprivileged and network none')


def patch_generated(services: dict[str, Any], monitor: dict[str, Any], *,
                    domain: int, holder: str, cpus: str) -> None:
    if domain not in (1,2) or not re.fullmatch(r'[0-9,-]+',cpus):
        raise ValueError('invalid domain or CPU set')
    if not {'scn-car1','scn-rosbag1','scn-simulator','scn-monitor'} <= services.keys():
        raise ValueError('expected one teacher vehicle')
    for service in services.values():
        service['network_mode'] = 'container:'+holder
        service['cpuset'] = cpus
        service.setdefault('environment',{}).update(
            RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file:///opt/autoware/cyclonedds.xml')
    for key in ('scn-car1','scn-rosbag1'):
        services[key]['environment']['ROS_DOMAIN_ID'] = str(domain)
    monitor['ego_domain'] = domain
    # Vehicle index d1 stays d1: it is not the DDS domain number.
    for vehicle in monitor['vehicles']:
        if vehicle.get('domain') is not None:
            vehicle['domain'] += domain-1
    if monitor.get('ego_ground_truth_domain') is not None:
        monitor['ego_ground_truth_domain'] += domain-1


def install_runner_isolation(plan: dict[str, Any], domain: int, lock_root: Path) -> dict[str, Any]:
    from scenario_tool import runner
    slot = validate_plan(plan,domain)
    holders = {}
    for s in plan['instances']:
        row = json.loads(subprocess.check_output(['docker','inspect',s['holder']],text=True))[0]
        validate_holder(row,s['holder']);holders[s['holder']] = row['Id']
    original_release = runner.ScenarioRunner.release_lock
    def acquire(instance: Any) -> None:
        # Shared official lock excludes an ordinary host-network scenario run.
        instance._isolation_shared_lock = open(instance.context.tool_dir/'.run.lock','a')
        fcntl.flock(instance._isolation_shared_lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
        instance.lock_handle = open(lock_root/(slot['holder']+'.lock'),'a')
        fcntl.flock(instance.lock_handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    def release(instance: Any) -> None:
        original_release(instance)
        handle = getattr(instance,'_isolation_shared_lock',None)
        if handle is not None:
            fcntl.flock(handle,fcntl.LOCK_UN);handle.close();instance._isolation_shared_lock=None
    original_competing = runner.active_competing_simulators
    def competing(project: str) -> tuple[str, ...]:
        conflicts=[]
        for name in original_competing(project):
            row=json.loads(subprocess.check_output(['docker','inspect',name],text=True))[0]
            mode=row['HostConfig']['NetworkMode']
            allowed={v for key,v in holders.items() if key!=slot['holder']}
            allowed.update(key for key in holders if key!=slot['holder'])
            if not mode.startswith('container:') or mode.split(':',1)[1] not in allowed:conflicts.append(name)
        return tuple(conflicts)
    runner.ScenarioRunner.acquire_lock=acquire
    runner.ScenarioRunner.release_lock=release
    runner.active_competing_simulators=competing
    return slot
