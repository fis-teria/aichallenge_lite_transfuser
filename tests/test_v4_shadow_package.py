"""Package registration and disabled-by-default startup; no model assets."""
import ast
import importlib
import json
from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).parents[1]
PACKAGE=ROOT/'ros2_ws/src/aic_e2e_runtime'


def test_disabled_config_does_not_load_model():
    sys.path.insert(0,str(PACKAGE))
    module=importlib.import_module('aic_e2e_runtime.v4_shadow_node')
    config=json.loads((PACKAGE/'config/v4_shadow.example.json').read_text())
    with pytest.raises(ValueError,match='V4_SHADOW_DISABLED'): module.validate_config(config)


def test_registered_node_launch_and_no_actuator_api():
    assert 'v4_shadow_node = aic_e2e_runtime.v4_shadow_node:main' in (PACKAGE/'setup.py').read_text()
    assert "executable='v4_shadow_node'" in (PACKAGE/'launch/v4_shadow.launch.py').read_text()
    tree=ast.parse((PACKAGE/'aic_e2e_runtime/v4_shadow_node.py').read_text())
    attrs={n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)}
    assert not attrs.intersection({'create_publisher','publish','create_client','send_goal_async'})
    assert 'Process' in attrs and 'spin_once' in attrs
