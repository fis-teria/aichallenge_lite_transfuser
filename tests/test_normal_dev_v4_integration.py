"""Synthetic launch integration only; no ROS, model or simulator."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

ROOT=Path(__file__).parents[1]
spec=importlib.util.spec_from_file_location('normal_dev',ROOT/'tools/normal_dev_v4_integration.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def fixture():
    return ('#!/bin/bash\n# USER CHANGE\nexport ROS_DOMAIN_ID=$id\nros2 launch existing system.xml\n',
            '<launch><node pkg="existing" exec="controller"/></launch>',
            'environment:\n    - CONTROL_METHOD=${CONTROL_METHOD:-}\n# USER CHANGE\n')


def test_preserves_controller_and_start_and_no_runner():
    runtime,launch,compose=m.integrate(*fixture())
    assert '# USER CHANGE' in runtime and '# USER CHANGE' in compose
    tree=ET.fromstring(launch)
    assert tree.find('node').attrib=={'pkg':'existing','exec':'controller'}
    assert tree.find('group').attrib['if']=='$(env V4_SHADOW_ENABLED false)'
    assert runtime.count('ros2 launch')==1
    assert 'source "${V4_SHADOW_SETUP}"' in runtime
    assert 'v4_shadow_prefix=$(ros2 pkg prefix aic_e2e_runtime)' in runtime
    assert runtime.index('source "${V4_SHADOW_SETUP}"') < runtime.index('v4_shadow_prefix=')
    assert '"${mode}" == "awsim"' in runtime
    assert 'CONTROL_METHOD=${CONTROL_METHOD:-}' in compose
    assert not any(x in runtime+launch+compose for x in ('official_start','race_armed','AWSIM_START_MODE','helper_complete'))


def test_duplicate_and_unknown_layout_rejected():
    with pytest.raises(ValueError,match='ALREADY'): m.integrate(*m.integrate(*fixture()))
    a,b,c=fixture()
    with pytest.raises(ValueError,match='UNKNOWN'): m.integrate(a.replace('export','other'),b,c)


def test_example_is_input_ready_but_disabled():
    import json
    config=json.loads((ROOT/'ros2_ws/src/aic_e2e_runtime/config/v4_shadow.example.json').read_text())
    assert config['inference_start_policy']=='INPUT_READY_SHADOW'
    assert config['start_gate'] is None and config['enabled'] is False
