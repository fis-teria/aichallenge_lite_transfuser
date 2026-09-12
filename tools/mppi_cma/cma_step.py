"""One pycma ask/tell checkpoint step in the existing, pinned optimizer image."""
from __future__ import annotations
import json
from pathlib import Path
import pickle
import sys
import cma


def main() -> None:
    root=Path(sys.argv[1]);command=sys.argv[2]
    state=root/'optimizer.pkl'
    if command=='init':
        if state.exists():raise FileExistsError(state)
        center=json.loads((root/'seed_anchors.json').read_text())
        es=cma.CMAEvolutionStrategy(center,.25,{'bounds':[-1.2,3.5],'popsize':6,'seed':20260912,'verbose':-9})
    else:
        with state.open('rb') as stream:es=pickle.load(stream)
    if command=='tell':
        values=json.loads((root/'evaluated.json').read_text())
        es.tell(values['candidates'],values['objectives'])
    if command in {'init','ask'}:
        (root/'proposals.json').write_text(json.dumps([list(map(float,x)) for x in es.ask()],indent=2))
    with state.open('wb') as stream:pickle.dump(es,stream)
    print(json.dumps({'step':command,'generation':es.countiter,'sigma':es.sigma}))


if __name__=='__main__':main()
