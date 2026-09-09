"""Synthetic solver/ROS-free tests of the actual patched integration methods."""
import ast
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as S, ModuleType

import numpy as np
import pytest
from scipy import sparse

ROOT = Path(__file__).resolve().parents[1] / 'integrations/mpc_solution_guard'
spec = importlib.util.spec_from_file_location('guarded_mpc', ROOT / 'MPC.py')
module = importlib.util.module_from_spec(spec)
HAS_OSQP = importlib.util.find_spec('osqp') is not None
if not HAS_OSQP:
    # Explicit test double ONLY for synthetic acceptance tests; never deployment.
    stub = ModuleType('osqp')
    stub.OSQPException = type('SyntheticOSQPException', (Exception,), {})
    sys.modules['osqp'] = stub
try:
    spec.loader.exec_module(module)
finally:
    if not HAS_OSQP:
        del sys.modules['osqp']


def fixture(status='solved', status_val=1, vector=None, error=None):
    m = module.MPC.__new__(module.MPC)
    m.N, m.nx, m.nu = 2, 3, 2
    m.infeasibility_counter = 0
    m.previous_steering = 0.0
    m.max_steering_rate = .2
    m.input_constraints = {'umin': np.array([0., -.5]), 'umax': np.array([5., .5])}
    m.model = S(length=1., Ts=.025, wp_id=0, safety_margin=.25,
                reference_path=S(circular=True), temporal_state=None, current_waypoint=None,
                get_current_waypoint=lambda: None, t2s=lambda **kw: None)
    m.current_control = np.array([4., .4, 4., .4])
    m.current_prediction = ['OLD']
    m.current_prediction_trajectory = ['OLD']
    m.current_prediction_contract = (1, 2, True, True)
    m._last_problem_prediction_contract = (0, 0, False, False)
    lo = np.r_[np.full(9, -10.), 0., -.5, 0., -.5]
    hi = np.r_[np.full(9, 10.), 5., .5, 5., .5]
    m._solution_constraints = (sparse.eye(13), lo, hi)
    m._init_problem = lambda *args: None
    m.update_prediction = lambda *args: ([0.], [0.])
    m.update_prediction_trajectory = lambda *args: [(0., 0., 0., 1.)]
    vector = np.r_[np.zeros(9), 1., 0., 1., 0.] if vector is None else vector
    def solve():
        if error:
            raise error
        return S(info=S(status=status, status_val=status_val), x=vector)
    m.optimizer = S(solve=solve)
    return m


def test_zero_steering_is_valid_without_margin_retry():
    m = fixture()
    calls = []
    m._init_problem = lambda n, margin: calls.append(margin)
    u, _ = m.get_control()
    assert u.tolist() == [1., 0.]
    assert m.last_control_valid
    assert calls == [.25]


@pytest.mark.parametrize('status,val', [('primal infeasible', 3), ('solved inaccurate', 2),
                                       ('maximum iterations reached', 7)])
def test_reject_status_without_old_controls(status, val):
    m = fixture(status, val)
    u, _ = m.get_control()
    assert u.tolist() == [0., 0.]
    assert not m.last_control_valid
    assert m.current_prediction is None and m.current_prediction_trajectory is None
    assert not m.current_control.any()


@pytest.mark.parametrize('vector', [None, [1.], np.full(13, np.nan),
                                    np.full(13, np.inf), np.r_[np.zeros(9), 1., 2., 1., 0.]])
def test_bad_vectors(vector):
    m = fixture(vector=vector if vector is not None else np.array(None))
    assert m.get_control()[0][0] == 0
    assert not m.last_control_valid


@pytest.mark.parametrize('error', [TypeError('bad'), ValueError('bad'), RuntimeError('bad')])
def test_solver_and_setup_exception(error):
    for setup in (False, True):
        m = fixture(error=error)
        if setup:
            def fail(*args):
                raise error
            m._init_problem = fail
        assert m.get_control()[0][0] == 0
        assert type(error).__name__ in m.last_rejection_reason


@pytest.mark.parametrize('sign', [-1, 1])
def test_rate_and_absolute_bounds(sign):
    m = fixture(vector=np.r_[np.zeros(9), 1., sign*.5, 1., 0.])
    for _ in range(200):
        before = m.previous_steering
        u, _ = m.get_control()
        assert abs(u[1]-before) <= .005+1e-12
        assert abs(u[1]) <= np.arctan(.5)
    assert m.last_control_valid


def test_failure_then_success_and_state_constraint():
    m = fixture(vector=np.r_[11., np.zeros(8), 1., 0., 1., 0.])
    assert m.get_control()[0][0] == 0
    assert 'CONSTRAINT_VIOLATION' in m.last_rejection_reason
    m.optimizer = fixture().optimizer
    assert m.get_control()[0][0] == 1
    assert m.infeasibility_counter == 0


def publisher():
    # Compile the actual method, without importing or starting ROS.
    tree = ast.parse((ROOT/'mpc_controller.py').read_text(encoding='utf-8'))
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                  and n.name == '_guard_publish_values')
    ns = {'np': np}
    exec(compile(ast.Module(body=[method], type_ignores=[]), 'actual_publish_guard', 'exec'), ns)
    instance = S(_mpc_cfg=S(delta_max=.5, steer_rate_max=.35,
                           steering_tire_angle_gain_var=1.639, control_rate=40.,
                           a_min=-.9, a_max=3., v_max=5.555),
                 get_logger=lambda: S(error=lambda text: None))
    return instance, ns['_guard_publish_values']


@pytest.mark.parametrize('u,acc', [([1., 1.01], 3.), ([1., np.nan], 3.),
                                  ([1., 0.], np.inf), ([1.], 3.), ([6., 0.], 3.)])
def test_publish_boundary_rejects_and_brakes(u, acc):
    n, guard = publisher()
    values, a, boost = guard(n, u, acc, True)
    assert values.tolist() == [0., 0.]
    assert a == -.9 and not boost


def test_publish_rate_and_no_input_mutation():
    n, guard = publisher()
    raw = np.array([1., .4])
    values, a, _ = guard(n, raw, 1., False)
    assert values[1]*1.639 == pytest.approx(.35/40)
    assert raw.tolist() == [1., .4]
    assert a == 1.


@pytest.mark.skipif(not HAS_OSQP, reason='OSQP unavailable; synthetic tests are not real solver validation')
def test_actual_osqp_solved_vector_is_accepted():
    m = fixture()
    A, lo, hi = m._solution_constraints
    solver = module.osqp.OSQP()
    solver.setup(P=sparse.eye(13, format='csc'), q=-np.r_[np.zeros(9), 1., 0., 1., 0.],
                 A=A.tocsc(), l=lo, u=hi, verbose=False)
    m.optimizer = solver
    assert m.get_control()[0][0] == pytest.approx(1., abs=1e-4)
    assert m.last_control_valid


def test_invalid_previous_steering_cannot_extend_bound():
    m = fixture()
    m.previous_steering = 1.0
    assert m.get_control()[0].tolist() == [0., 0.]
    assert not m.last_control_valid


def test_missing_solver_metadata_is_rejected():
    m = fixture()
    m.optimizer = S(solve=lambda: S(x=np.zeros(13)))
    assert m.get_control()[0][0] == 0
    assert 'AttributeError' in m.last_rejection_reason


def test_actual_publish_method_keeps_raw_gain_contract():
    import copy
    n, guard = publisher()
    n._guard_publish_values = lambda *args: guard(n, *args)
    recorded = []
    n._create_ackerman_control_command = lambda stamp, u, acc, boost: S(
        lateral=S(steering_tire_angle=u[1]), longitudinal=S(speed=u[0], acceleration=acc))
    n._command_raw_pub = S(publish=lambda msg: recorded.append(copy.deepcopy(msg)))
    n._command_pub = S(publish=lambda msg: recorded.append(copy.deepcopy(msg)))
    tree = ast.parse((ROOT/'mpc_controller.py').read_text(encoding='utf-8'))
    method = next(x for x in ast.walk(tree) if isinstance(x, ast.FunctionDef)
                  and x.name == '_publish_control_command')
    ns = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), 'actual_publish', 'exec'), ns)
    ns['_publish_control_command'](n, None, [1., .4], 1., False)
    assert recorded[1].lateral.steering_tire_angle == pytest.approx(
        recorded[0].lateral.steering_tire_angle*1.639)
    ns['_publish_control_command'](n, None, [1., 1.01], 3., True)
    assert recorded[-1].longitudinal.speed == 0
    assert recorded[-1].longitudinal.acceleration == -.9
