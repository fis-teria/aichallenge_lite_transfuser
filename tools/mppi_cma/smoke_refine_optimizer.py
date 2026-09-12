"""Exercise native pycma checkpoint ordering/retries inside the pinned image."""
from pathlib import Path
import hashlib
import json
import tempfile

from cma_refine_step import step, atomic_json


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        atomic_json(root/'seed_anchors.json', [0.] * 16)
        atomic_json(root/'optimizer_config.json', {'sigma_m': .16, 'seed': 202609120})
        assert step(root, 'init')['generation'] == 0
        proposals = json.loads((root/'proposals.json').read_text())
        assert len(proposals) == 8 and all(len(candidate) == 16 for candidate in proposals)
        assert all(-1.2 <= value <= 3.5 for candidate in proposals for value in candidate)
        step(root, 'ask')
        assert json.loads((root/'proposals.json').read_text()) == proposals
        evaluated = {'generation': 0, 'candidates': proposals,
                     'objectives': [sum(x*x for x in candidate) for candidate in proposals]}
        atomic_json(root/'evaluated.json', evaluated)
        assert step(root, 'tell')['generation'] == 1
        assert step(root, 'tell')['generation'] == 1
        step(root, 'ask')
        assert json.loads((root/'proposals.json').read_text()) != proposals
        before = hashlib.sha256((root/'optimizer.pkl').read_bytes()).hexdigest()
        evaluated['objectives'][0] += 1
        atomic_json(root/'evaluated.json', evaluated)
        try:
            step(root, 'tell')
        except ValueError:
            pass
        else:
            raise AssertionError('Changed committed evaluation must be rejected')
        assert hashlib.sha256((root/'optimizer.pkl').read_bytes()).hexdigest() == before
    print('CMA_REFINEMENT_TRANSACTION_SMOKE_OK')


if __name__ == '__main__':
    main()
