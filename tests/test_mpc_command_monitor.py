"""Synthetic checks; no ROS or AWSIM."""
import importlib.util
import math
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / 'integrations/mpc_solution_guard/command_monitor.py'
spec = importlib.util.spec_from_file_location('command_monitor', PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('final', [False, True])
def test_bound_and_domain(final):
    limit = math.radians(32)
    gain = 1.639
    boundary = limit * (gain if final else 1.)
    for sign in (-1, 1):
        assert module.command_fault([sign*boundary, 0., -.9], limit, gain, final=final) is None
        assert module.command_fault([sign*(boundary+.001), 0., -.9], limit, gain, final=final)


@pytest.mark.parametrize('values', [[float('nan'), 0., 0.], [0., float('inf'), 0.],
                                  [0., 0., float('nan')], []])
def test_nonfinite_or_shape(values):
    assert module.command_fault(values, .5, 1.639, final=False) == 'COMMAND_NONFINITE_OR_SHAPE'


def test_run10_excess_is_rejected():
    assert module.command_fault([1.0088, 0., 0.], math.radians(32), 1.639, final=False)
    assert module.command_fault([1.6534, 0., 0.], math.radians(32), 1.639, final=True)
