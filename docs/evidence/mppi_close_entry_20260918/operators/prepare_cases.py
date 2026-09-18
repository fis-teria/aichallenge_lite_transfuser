from pathlib import Path
import json,sys,hashlib
root=Path('/home/graneple/e2e_autonomous/mppi_close_adjust_20260918')
old=root.with_name('mppi_v45_collection_fix_20260918')
repo=Path('/home/graneple/git/autononous_ai/aichallenge-racingkart')
sys.path.insert(0,str(repo/'scenario_tool'))
from scenario_tool import yamlio
(root/'scenarios').mkdir(exist_ok=True)
for suffix,source in [('close-entry-a01','cone-close6-a02'),('close-entry-control-a01','cone-close6-a02'),('entry-normal-a01','cone-b01')]:
    name='lidar-v45-pc10-front-'+suffix
    source='lidar-v45-pc10-front-'+source
    p=old/'scenarios'/(source+'.yaml')
    scenario=yamlio.load_file(p)
    scenario['name']=name
    dest=root/'scenarios'/(name+'.yaml');assert not dest.exists()
    yamlio.dump_file(dest,scenario)
    metadata=json.loads((old/'scenarios'/(source+'.json')).read_text())
    metadata.update(run_id=name,source_scenario=str(p),source_scenario_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                    scope='Teacher early-entry A/B only; not admitted as moving teacher until audit')
    (root/'scenarios'/(name+'.json')).write_text(json.dumps(metadata,indent=2)+'\n')
    print(name)
