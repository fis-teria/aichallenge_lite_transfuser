"""One finite, isolated AWSIM input-only probe. Preserve prior containers/assets."""
import json
import subprocess
import time
from pathlib import Path
from spatial_dev_host_v4 import AttemptBudget


def run(args, timeout=15):
    return subprocess.run(args,check=True,text=True,capture_output=True,timeout=timeout)


def main():
    root=Path(__file__).resolve().parent
    out=root/'evidence';out.mkdir()
    start=time.monotonic();name='codex-'+root.name.replace('_','-')
    old=Path('/home/graneple/e2e_autonomous/static_map_awsim_20260907/stationary_bfd7263_01/compose.json')
    budget=AttemptBudget(Path('/home/graneple/e2e_autonomous/spatial_run_continuation_20260907/budget.json'))
    budget.limits=dict(wall_s=3600.,forward=7100,mpc=6000,snapshots=16,powered=11,powered_s=320.,log_bytes=536870912)
    reservation=dict(wall_s=115.,forward=0,mpc=0,snapshots=0,powered=0,powered_s=0.,log_bytes=16*1024**2)
    existing=run(['docker','ps','--format','{{.Names}}']).stdout
    if existing.strip(): raise ValueError('ACTIVE_CONTAINERS_REQUIRE_CLASSIFICATION')
    template=json.loads(old.read_text());sim=template['services']['simulator']
    sim['environment']['V4_PROJECT']=name
    for mount in sim['volumes']:
        if mount['target']=='/evidence': mount['source']=str(out)
        if mount['target']=='/tmp/.X11-unix':
            (out/'x11').mkdir();mount['source']=str(out/'x11')
    compose={'name':name,'services':{'simulator':sim}}
    path=out/'compose.json';path.write_text(json.dumps(compose,indent=2))
    cmd=['docker','compose','-p',name,'-f',str(path)]
    budget.reserve(name,reservation)
    result={'control_publish':False,'forward':0,'powered':0,'cleanup':[]}
    cid=None
    try:
        run(cmd+['up','-d','--no-build'],timeout=25)
        cid=run(cmd+['ps','-q','simulator']).stdout.strip()
        info=json.loads(run(['docker','inspect',cid]).stdout)[0]
        if info['HostConfig']['NetworkMode']!='none' or info['HostConfig']['Privileged'] or info['HostConfig'].get('Devices'):
            raise ValueError('ISOLATION_FAILED')
        result['container_id']=cid
        result['network_mode']=info['HostConfig']['NetworkMode']
        result['mounts']=[dict(destination=m['Destination'],rw=m['RW']) for m in info['Mounts']]
        run(['docker','cp',str(root/'probe_shadow_ros2_graph_v4.py'),cid+':/evidence/probe.py'])
        r=run(['docker','exec',cid,'bash','-c',
               'source /autoware/install/setup.bash && export ROS_DOMAIN_ID=1 && timeout -k 2 35 python3 /evidence/probe.py'],timeout=40)
        (out/'stdout.log').write_text(r.stdout);(out/'stderr.log').write_text(r.stderr)
        result['status']='PROBE_COMPLETED_NOT_DRIVING'
    except Exception as exc:
        result['status']='FAILED';result['error']=str(exc)
        if isinstance(exc,subprocess.CalledProcessError):
            (out/'stdout.log').write_text(exc.stdout or '');(out/'stderr.log').write_text(exc.stderr or '')
    finally:
        if cid:
            for args in (['docker','pause',cid],['docker','kill','--signal','KILL',cid]):
                try: run(args,timeout=6)
                except Exception as exc: result['cleanup'].append(str(exc))
        try: run(cmd+['down','--timeout','2'],timeout=10)
        except Exception as exc: result['cleanup'].append(str(exc))
        result['wall_s']=time.monotonic()-start
        result['final_containers']=run(['docker','ps','-a','--filter','label=com.docker.compose.project='+name,'--format','{{.ID}} {{.Status}}']).stdout
        used=sum(p.stat().st_size for p in out.rglob('*') if p.is_file())
        budget.finish(dict(reservation,wall_s=result['wall_s'],log_bytes=used),exact=True)
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result))


if __name__=='__main__': main()
