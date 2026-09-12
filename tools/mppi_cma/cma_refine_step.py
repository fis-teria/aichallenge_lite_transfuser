"""Atomic, repeatable CMA ask/tell transactions in the pinned optimizer image."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import pickle
import sys

import cma
import numpy as np


def atomic_json(path: Path, data: object) -> None:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, indent=2)); temporary.replace(path)


def step(root: Path, command: str) -> dict:
    if command not in ('init', 'ask', 'tell'):
        raise ValueError('Unknown CMA transaction')
    path = root / 'optimizer.pkl'
    if command == 'init':
        if path.exists():
            raise FileExistsError(path)
        config = json.loads((root / 'optimizer_config.json').read_text())
        center = json.loads((root / 'seed_anchors.json').read_text())
        if np.asarray(center, dtype=float).shape != (16,) or not np.isfinite(center).all():
            raise ValueError('CMA center must contain 16 finite lateral anchors in metres')
        if not np.isfinite(config['sigma_m']) or not 0 < config['sigma_m'] <= .5:
            raise ValueError('CMA sigma must be in (0, 0.5] metres')
        rng = np.random.RandomState(config['seed'])
        es = cma.CMAEvolutionStrategy(center, config['sigma_m'], {
            'bounds': [-1.2, 3.5], 'popsize': 8, 'seed': config['seed'],
            'randn': rng.randn, 'verbose': -9})
        bundle = {'optimizer': es, 'pending': None, 'last_tell_sha256': None,
                  'last_tell_generation': None}
    else:
        with path.open('rb') as stream:
            bundle = pickle.load(stream)
        es = bundle['optimizer']
    if command in ('init', 'ask'):
        if bundle['pending'] is None:
            bundle['pending'] = [list(map(float, candidate)) for candidate in es.ask()]
    else:
        raw = (root / 'evaluated.json').read_bytes()
        values = json.loads(raw)
        digest = hashlib.sha256(raw).hexdigest()
        if bundle['last_tell_generation'] == values['generation']:
            if digest != bundle['last_tell_sha256']:
                raise ValueError('A committed CMA evaluation was changed')
        else:
            if values['generation'] != es.countiter or values['candidates'] != bundle['pending']:
                raise ValueError('CMA generation or candidate ordering does not match the checkpoint')
            es.tell(values['candidates'], values['objectives'])
            bundle.update(pending=None, last_tell_sha256=digest,
                          last_tell_generation=values['generation'])
    temporary = path.with_suffix('.tmp')
    with temporary.open('wb') as stream:
        pickle.dump(bundle, stream)
    temporary.replace(path)
    if bundle['pending'] is not None:
        atomic_json(root / 'proposals.json', bundle['pending'])
    status = {'generation': es.countiter, 'sigma_m': es.sigma, 'pending': bundle['pending'] is not None}
    atomic_json(root / 'optimizer_status.json', status)
    return status


if __name__ == '__main__':
    print(json.dumps(step(Path(sys.argv[1]), sys.argv[2])))
