"""Fixed-object AWSIM fixtures: setup-only Unity x/z metres and yaw degrees."""
from __future__ import annotations

import math
from pathlib import Path
import re
from typing import Any

import yaml


def load_static_scenario(path: Path) -> tuple[bytes, dict[str, Any]]:
    """Validate a bounded native fixture without changing the normal ego spawn.

    Object ``at`` has shape [2] in Unity x/z metres; ``yaw`` is the native
    scenario format's degrees. These poses never become model/control inputs.
    """
    data = path.read_bytes()
    doc = yaml.safe_load(data)
    allowed = {'schemaVersion', 'name', 'description', 'fail', 'objects'}
    if (not isinstance(doc, dict) or set(doc) - allowed or doc.get('schemaVersion') != 2
            or not isinstance(doc.get('name'), str)
            or not re.fullmatch(r'[a-z0-9_-]+', doc['name'])):
        raise ValueError('STATIC_SCENARIO_SCHEMA_OR_EGO_OVERRIDE')
    fail = doc.get('fail')
    if (not isinstance(fail, dict) or set(fail) != {'collision', 'timeoutSeconds'}
            or type(fail['collision']) is not int or fail['collision'] != 1
            or type(fail['timeoutSeconds']) not in (int, float)
            or not math.isfinite(fail['timeoutSeconds']) or not 30 <= fail['timeoutSeconds'] <= 660):
        raise ValueError('STATIC_SCENARIO_FINITE_FAILURE_LIMITS')
    groups = doc.get('objects')
    if not isinstance(groups, dict) or not groups or set(groups) - {'box', 'cone'}:
        raise ValueError('STATIC_SCENARIO_OBJECT_TYPES')
    identities = []
    for kind, objects in groups.items():
        if not isinstance(objects, list) or not objects:
            raise ValueError('STATIC_SCENARIO_OBJECT_LIST')
        for obj in objects:
            if not isinstance(obj, dict) or set(obj) != {'name', 'at', 'yaw'}:
                raise ValueError('STATIC_SCENARIO_OBJECT_FIELDS')
            if not isinstance(obj['name'], str) or not re.fullmatch(r'[a-z0-9_-]+', obj['name']):
                raise ValueError('STATIC_SCENARIO_OBJECT_NAME')
            xy = obj['at']
            if not isinstance(xy, list) or len(xy) != 2:
                raise ValueError('STATIC_SCENARIO_POSE_SHAPE')
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in [*xy, obj['yaw']]):
                raise ValueError('STATIC_SCENARIO_POSE_FINITE')
            identities.append((kind, obj['name']))
    if not 1 <= len(identities) <= 12 or len({name for _, name in identities}) != len(identities):
        raise ValueError('STATIC_SCENARIO_OBJECT_COUNT_OR_DUPLICATE')
    return data, {'name': doc['name'], 'object_count': len(identities),
                  'identities': identities, 'scope': 'SETUP_ONLY_NOT_MODEL_INPUT'}


def static_scenario_startup(log_text: str, expected: dict[str, Any]) -> dict[str, Any]:
    """Require visible named objects and collisions in the actual Unity log."""
    spawns = re.findall(r'^Spawned (\d+) object\(s\) from /output/static_obstacles.yaml\s*$',
                        log_text, re.MULTILINE)
    if not spawns or any(int(count) != expected['object_count'] for count in spawns):
        raise ValueError('STATIC_SCENARIO_SPAWN_COUNT')
    objects = set(re.findall(r"^\[Scenario\] (box|cone) '([^']+)':.*enabled=True off=False.*supported=True",
                             log_text, re.MULTILINE))
    if objects != {tuple(identity) for identity in expected['identities']}:
        raise ValueError('STATIC_SCENARIO_RENDER_IDENTITIES')
    settings = re.findall(r'^Applied race settings:.*$', log_text, re.MULTILINE)
    if not settings or not re.search(r'\bcollisions=True(?:,|\s|$)', settings[-1]):
        raise ValueError('STATIC_SCENARIO_COLLISIONS_DISABLED')
    return {**expected, 'spawn_passes': len(spawns), 'collisions_enabled': True,
            'scope': 'SPAWN_CHECK_ONLY_NOT_AVOIDANCE_ACCEPTANCE'}
