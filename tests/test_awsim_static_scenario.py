from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from aic_transfuser_lite.runtime.awsim_static_scenario import load_static_scenario, static_scenario_startup

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('kind', ['cone', 'box'])
def test_fixed_fixture_preserves_native_pose_units_and_spawn_checks(kind):
    path = ROOT / f'configs/scenarios/time_avoidance_single_{kind}.yaml'
    data, meta = load_static_scenario(path)
    doc = yaml.safe_load(data)
    assert 'vehicles' not in doc and meta['object_count'] == 1
    assert doc['objects'][kind][0]['at'] == [359.3672, -15.9015]
    assert doc['objects'][kind][0]['yaw'] == -126.5927
    block = (f"[Scenario] {kind} 'avoidance_{kind}': renderer=mesh enabled=True off=False mat=HDRP/Lit supported=True\n"
             'Spawned 1 object(s) from /output/static_obstacles.yaml\n')
    # Existing AWSIM initializes the scene twice; repeated identical spawns
    # are valid. Missing or mismatched actual creation must not authorize Start.
    log = block * 2 + 'Applied race settings: ranking=True, collisions=True, wallRecovery=True\n'
    assert static_scenario_startup(log, meta)['spawn_passes'] == 2
    if kind == 'cone':
        aggregate_only = ('Spawned 1 object(s) from /output/static_obstacles.yaml\n' * 2
                          + 'Applied race settings: collisions=True, wallRecovery=True\n')
        result = static_scenario_startup(aggregate_only, meta)
        assert result['named_renderer_evidence'] == []
        assert result['names_not_logged_by_simulator'] == [('cone', 'avoidance_cone')]
    else:
        with pytest.raises(ValueError, match='STATIC_SCENARIO_RENDER_IDENTITIES'):
            static_scenario_startup(log.replace(block, 'Spawned 1 object(s) from /output/static_obstacles.yaml\n'), meta)
    for bad in (log.replace('Spawned 1', 'Spawned 2'), log.replace('collisions=True', 'collisions=False'),
                log.replace('enabled=True', 'enabled=False'), log.replace(f"'avoidance_{kind}'", "'other'")):
        with pytest.raises(ValueError, match='STATIC_SCENARIO'):
            static_scenario_startup(bad, meta)


@pytest.mark.parametrize('mutation', ['ego', 'time', 'bool_time', 'collision', 'shape', 'nan', 'bool_pose', 'duplicate', 'unknown'])
def test_malformed_or_unbounded_fixtures_are_rejected(tmp_path, mutation):
    doc = yaml.safe_load((ROOT / 'configs/scenarios/time_avoidance_single_box.yaml').read_text())
    obj = doc['objects']['box'][0]
    if mutation == 'ego': doc['vehicles'] = {'1': {'at': [0., 0.]}}
    elif mutation == 'time': doc['fail']['timeoutSeconds'] = 10000
    elif mutation == 'bool_time': doc['fail']['timeoutSeconds'] = True
    elif mutation == 'collision': doc['fail']['collision'] = 0
    elif mutation == 'shape': obj['at'] = [0., 1., 2.]
    elif mutation == 'nan': obj['yaw'] = float('nan')
    elif mutation == 'bool_pose': obj['at'][0] = True
    elif mutation == 'duplicate': doc['objects']['box'].append(deepcopy(obj))
    elif mutation == 'unknown': doc['objects']['vehicle'] = [deepcopy(obj)]
    path = tmp_path / 'fixture.yaml'
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match='STATIC_SCENARIO'):
        load_static_scenario(path)
