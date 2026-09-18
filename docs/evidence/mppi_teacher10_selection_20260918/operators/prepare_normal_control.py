from pathlib import Path
import json,sys
root=Path('/home/graneple/e2e_autonomous/mppi_close_adjust_20260918/scenarios')
sys.path.insert(0,'/home/graneple/git/autononous_ai/aichallenge-racingkart/scenario_tool')
from scenario_tool import yamlio
old='lidar-v45-pc10-front-normal10-on-a01';new='lidar-v45-pc10-front-normal10-control-a01'
p=yamlio.load_file(root/(old+'.yaml'));p['name']=new
assert not (root/(new+'.yaml')).exists()
yamlio.dump_file(root/(new+'.yaml'),p)
m=json.loads((root/(old+'.json')).read_text());m['run_id']=new
(root/(new+'.json')).write_text(json.dumps(m,indent=2))
