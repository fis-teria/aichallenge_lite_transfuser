from copy import deepcopy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

from aic_transfuser_lite.runtime.awsim_traffic import DomainLapJudge, background_launch, traffic_domains

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from time_dev_runner import make_command
from aic_transfuser_lite.control.time_dev_v1 import TimeDevSpeeds


def row(index, lap=0, section=0, elapsed=0.):
    return dict(domain_id=1, publisher_count=1, monotonic_ns=index*100_000_000,
                data=[660-index*.1, lap, elapsed, section, 1., 2., 0.])


def test_domain_lap_ignores_background_progress_and_requires_full_ego_lap():
    judge = DomainLapJudge()
    for record in [row(0), row(1), row(2, 1, 1), row(3, 1, 1, .1),
                   row(4, 1, 2, .2), row(5, 1, 3, .3)]:
        judge.feed(record)
        assert not judge.completed
    judge.feed(row(6, 2, 1))
    assert judge.completed
    assert [e['next'] for e in judge.section_events] == [0, 1, 2, 0]
    assert judge.laps[0]['lap_seconds_lower_bound'] == .3
    assert judge.laps[0]['lap_seconds_upper_bound'] == pytest.approx(.4)
    assert judge.laps[0]['domain_id'] == 1


@pytest.mark.parametrize('bad', [
    dict(domain_id=2), dict(publisher_count=2), dict(publisher_count=0),
    dict(monotonic_ns=0), dict(data=[0.]*6), dict(data=[0.]*8),
    dict(data=[659., 1.5, 0., 1., 1., 2., 0.]),
    dict(data=[661., 0., 0., 0., 1., 2., 0.]),
    dict(data=[659., 0., float('nan'), 0., 1., 2., 0.]),
    dict(data=[659., 0., float('inf'), 0., 1., 2., 0.]),
    dict(data=[659., 0., -1., 0., 1., 2., 0.]),
    dict(data=[659., 2., 0., 1., 1., 2., 0.]),
])
def test_ambiguous_or_reset_status_is_never_a_lap(bad):
    judge = DomainLapJudge()
    judge.feed(row(0))
    with pytest.raises(ValueError, match='EGO_STATUS_JUDGE_CONTRACT'):
        judge.feed({**row(1), **bad})
    assert judge.invalid and not judge.completed


@pytest.mark.parametrize('lap,section,elapsed', [(0, 0, 0.), (1, 3, .2), (2, 1, 0.), (1, 1, 0.)])
def test_missing_section_or_backward_time_fails_closed(lap, section, elapsed):
    judge = DomainLapJudge()
    judge.feed(row(0))
    judge.feed(row(1, 1, 1, .1))
    with pytest.raises(ValueError):
        judge.feed(row(2, lap, section, elapsed))
    assert not judge.completed


def test_cannot_attach_mid_lap_and_claim_complete():
    with pytest.raises(ValueError):
        DomainLapJudge().feed(row(5, 1, 2, 8.))


def test_background_launch_preserves_initialization_and_guards():
    original = '''<launch><arg name="simulation"/><include file="$(find-pkg-share aichallenge_submit_launch)/launch/aichallenge_submit.launch.xml"><arg name="control_method" value="$(var control_method)"/></include><include file="awsim.launch.xml"><arg name="race_arm_on_vehicle_state" value="Start"/></include></launch>'''
    expected = ET.fromstring(original)
    actual = ET.fromstring(background_launch(original, 10/3.6))
    added = actual.find('include').findall('arg')[-1]
    assert added.attrib == dict(name='reference_execution_speed_cap_mps', value=str(10/3.6))
    actual.find('include').remove(added)
    assert ET.tostring(actual) == ET.tostring(expected)
    for cap in [0., -1., 30., float('nan')]:
        with pytest.raises(ValueError):
            background_launch(original, cap)
    with pytest.raises(ValueError):
        background_launch('<launch/>', 1.)


def test_owned_pp_domains_and_command():
    assert traffic_domains(2) == (1, 2, 3)
    for count in [True, -1, 4, 2.0]:
        with pytest.raises(ValueError):
            traffic_domains(count)
    with pytest.raises(ValueError):
        traffic_domains(2, 2)
    command = make_command(source=Path('/source'), deployment=Path('/deployment'),
        run_id='codex-time-traffic-test', display=':0', speeds=TimeDevSpeeds(), record_video=True,
        pp_vehicles=2)
    assert command[command.index('--pp-vehicles')+1] == '2'
    assert command[command.index('--npcs')+1] == '0'
