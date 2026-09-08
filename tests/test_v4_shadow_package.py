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


def test_node_uses_single_credit_batch_and_not_per_callback_ticks():
    source=(PACKAGE/'aic_e2e_runtime/v4_shadow_node.py').read_text()
    assert 'incoming=ctx.Queue(maxsize=1)' in source
    assert 'pump.dispatch(' in source and 'pump.acknowledge(result)' in source
    assert "kind='tick'" not in source
    assert 'consume_batch(item,join,emit,time.monotonic_ns)' in source


def test_loading_finishes_before_any_transport_is_created():
    sys.path.insert(0,str(PACKAGE))
    module=importlib.import_module('aic_e2e_runtime.v4_shadow_node')
    import queue
    from types import SimpleNamespace
    messages=queue.Queue();emitted=[];spins=[]
    def spin():
        spins.append(1)
        if len(spins)==3: messages.put({'event':'MODEL_LOADED'})
    module.wait_model_ready(SimpleNamespace(is_alive=lambda:True),messages,
                            emitted.append,spin,lambda:False)
    assert len(spins)==3 and emitted==[{'event':'MODEL_LOADED'}]
    source=(PACKAGE/'aic_e2e_runtime/v4_shadow_node.py').read_text()
    assert source.index('wait_model_ready(process,outgoing') < source.index('transport=ShadowROS2Transport(')


@pytest.mark.parametrize('condition,reason',[
    ('deadline','MODEL_STARTUP_DEADLINE'),('exit','MODEL_STARTUP_EXIT:1'),
    ('protocol','MODEL_STARTUP_PROTOCOL'),('late_ready','MODEL_STARTUP_DEADLINE')])
def test_loading_failure_never_admits_inputs(condition,reason):
    sys.path.insert(0,str(PACKAGE))
    module=importlib.import_module('aic_e2e_runtime.v4_shadow_node')
    import queue
    from types import SimpleNamespace
    messages=queue.Queue();emitted=[];checks=[]
    if condition in ('protocol','late_ready'):
        messages.put({'event':'MODEL_LOADED' if condition=='late_ready' else 'OTHER'})
    def expired():
        checks.append(1)
        return condition=='deadline' or (condition=='late_ready' and len(checks)>1)
    with pytest.raises(RuntimeError,match=reason):
        module.wait_model_ready(SimpleNamespace(is_alive=lambda:False,exitcode=1),
            messages,emitted.append,lambda:None,expired)
    assert not emitted
