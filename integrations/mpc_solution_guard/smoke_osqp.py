"""No pytest/ROS required: real OSQP smoke using synthetic QP constraints."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as S

import numpy as np
import osqp
from scipy import sparse

spec = importlib.util.spec_from_file_location('actual_mpc_guard', Path(__file__).with_name('MPC.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
m = mod.MPC.__new__(mod.MPC)
m.N, m.nx, m.nu = 2, 3, 2
m.previous_steering, m.max_steering_rate, m.infeasibility_counter = 0., .2, 0
m.input_constraints = {'umin': np.array([0., -.5]), 'umax': np.array([5., .5])}
m.model = S(length=1., Ts=.025, wp_id=0, safety_margin=.25,
            reference_path=S(circular=True), temporal_state=None, current_waypoint=None,
            get_current_waypoint=lambda: None, t2s=lambda **kwargs: None)
m._last_problem_prediction_contract = (0, 0, False, False)
m._init_problem = lambda *args: None
m.update_prediction = lambda *args: ([0.], [0.])
m.update_prediction_trajectory = lambda *args: [(0., 0., 0., 1.)]
A = sparse.eye(13, format='csc')
lo = np.r_[np.full(9, -10.), 0., -.5, 0., -.5]
hi = np.r_[np.full(9, 10.), 5., .5, 5., .5]
m._solution_constraints = A, lo, hi
m.optimizer = osqp.OSQP()
m.optimizer.setup(P=A, q=-np.r_[np.zeros(9), 1., 0., 1., 0.], A=A, l=lo, u=hi, verbose=False)
u, _ = m.get_control()
assert m.last_control_valid and abs(u[0]-1.) < 1e-4 and u[1] == 0
# Infeasible equality rows with finite bounds, using the SAME real OSQP.
bad_A = sparse.vstack([A, A[0]], format='csc')
bad_lo = np.r_[lo, 20.]
bad_hi = np.r_[hi, 20.]
m.optimizer = osqp.OSQP()
m.optimizer.setup(P=A, q=np.zeros(13), A=bad_A, l=bad_lo, u=bad_hi, verbose=False)
m._solution_constraints = bad_A, bad_lo, bad_hi
u, _ = m.get_control()
assert not m.last_control_valid and u.tolist() == [0., 0.]
assert m.current_prediction is None and m.current_prediction_trajectory is None
print(json.dumps({'osqp_version': osqp.__version__, 'passed': 2,
                  'rejected_status': m.last_solver_status, 'control': u.tolist(),
                  'ros_started': False, 'control_published': False}))
