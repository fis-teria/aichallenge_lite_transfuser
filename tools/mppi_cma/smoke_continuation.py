"""Native proof that a copied pycma checkpoint yields identical future candidates."""
from pathlib import Path
import hashlib
import json
import tempfile

from cma_refine_step import step, atomic_json
from continuation_state import clone_checkpoint


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary); source = root/'source'; source.mkdir()
        atomic_json(source/'seed_anchors.json', [0.]*16)
        atomic_json(source/'optimizer_config.json', {'sigma_m': .16, 'seed': 202609120})
        step(source, 'init')
        for generation in range(3):
            step(source, 'ask')
            proposals = json.loads((source/'proposals.json').read_text())
            atomic_json(source/'evaluated.json', {'generation': generation, 'candidates': proposals,
                         'objectives': [sum(x*x for x in row) for row in proposals]})
            step(source, 'tell')
        before = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()}
        left, right = root/'left', root/'right'
        clone_checkpoint(source, left, 3); clone_checkpoint(source, right, 3)
        for directory in (left, right):
            assert step(directory, 'ask')['generation'] == 3
        next_proposals = json.loads((left/'proposals.json').read_text())
        assert next_proposals == json.loads((right/'proposals.json').read_text())
        atomic_json(left/'evaluated.json', {'generation': 3, 'candidates': next_proposals,
                    'objectives': [sum(x*x for x in row) for row in next_proposals]})
        assert step(left, 'tell')['generation'] == 4
        assert step(left, 'tell')['generation'] == 4
        step(left, 'ask')
        assert json.loads((left/'proposals.json').read_text()) != next_proposals
        assert {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()} == before
    print('CMA_CONTINUATION_DISTRIBUTION_AND_RNG_SMOKE_OK')


if __name__ == '__main__':
    main()
