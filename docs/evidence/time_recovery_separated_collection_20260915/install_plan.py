import json
import manage as m

m.remote((m.HERE/'plan_native.py').read_text(),native=True,lock=True,timeout=240)
m.copy_to_host([m.UNC/'selected_site_plan.json'])
m.copy_to_host(sorted((m.UNC/'planned_references').glob('*.json')),m.ROOT+'/planned_references')
m.remote(rf'''
from pathlib import Path
import hashlib,json
root=Path({m.ROOT!r});plan=json.loads((root/'selected_site_plan.json').read_text())
ledger=json.loads((root/'campaign_20260914.json').read_text())
assert not ledger['attempts'] and not ledger['planned_runs'] and len(plan['runs'])==10
assert sum(len(r['sites']) for r in plan['runs'])==22
for r in plan['runs']:
 assert hashlib.sha256((root/'planned_references'/r['reference']).read_bytes()).hexdigest()==r['reference_sha256']
ledger['planned_runs']=plan['runs'];ledger['selection_sha256']=hashlib.sha256((root/'selected_site_plan.json').read_bytes()).hexdigest()
(root/'campaign_20260914.json').write_text(json.dumps(ledger,indent=2))
print('PLAN_INSTALLED_WITH_S00_EXPANSION_GATE')
''')
