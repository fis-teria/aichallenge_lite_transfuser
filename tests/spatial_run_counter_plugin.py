"""Opt-in unit-only method counters, no model/solver invocation by this plugin."""
from collections import Counter
import json
import os
from pathlib import Path
import sys

COUNTS=Counter()
def trace(frame,event,arg):
    if event!='call':return
    name=frame.f_code.co_name
    file=frame.f_code.co_filename.replace('\\','/')
    if name=='forward' and ('/tests/' in file or '/aic_transfuser_lite/' in file):
        COUNTS['forward_methods:'+file.split('/')[-1]]+=1
    elif name=='solve' and file.endswith('/control/spatial_mpc_v4.py'):
        COUNTS['mpc_solve_methods']+=1
    elif name=='constrained_reference' and file.endswith('/control/constrained_reference_v4.py'):
        COUNTS['reference_adapter_calls']+=1
    elif name=='minimize' and '/scipy/optimize/' in file:
        COUNTS['scipy_minimize_calls']+=1

def pytest_sessionstart(session):
    sys.setprofile(trace)

def pytest_sessionfinish(session,exitstatus):
    sys.setprofile(None)
    path=os.environ.get('V4_UNIT_COUNTER_OUTPUT')
    if path:
        Path(path).write_text(json.dumps(dict(exitstatus=int(exitstatus),counts=dict(COUNTS),
            scope='UNIT_TEST_METHOD_CALLS_NOT_SIMULATOR_EXECUTION',
            fixed_checkpoint_forward=0,note='Named forward methods include fake fixtures; no claim for uninstrumented callables'),indent=2))
