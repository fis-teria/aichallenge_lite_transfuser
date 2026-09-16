import json
from pathlib import Path
import sqlite3
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from train_time_launch_protection import ready_window,stage_errors


def test_ready_uses_recorded_transition_and_next_clock_not_countdown_guess(tmp_path):
    from rosbags.typesys import Stores,get_typestore
    store=get_typestore(Stores.ROS2_HUMBLE)
    clock_type='rosgraph_msgs/msg/Clock';state_type='std_msgs/msg/String'
    Time=store.types['builtin_interfaces/msg/Time']
    Clock=store.types[clock_type];String=store.types[state_type]
    (tmp_path/'bag').mkdir()
    with sqlite3.connect(tmp_path/'bag/bag_0.db3') as db:
        db.execute('CREATE TABLE topics (id INTEGER,name TEXT,type TEXT)')
        db.executemany('INSERT INTO topics VALUES (?,?,?)',[(1,'/clock',clock_type),(2,'/awsim/state',state_type)])
        db.execute('CREATE TABLE messages (id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        rows=[(1,Clock(Time(9,0))),(2,String('Start')),(1,Clock(Time(10,0))),
              (2,String('Ready')),(1,Clock(Time(10,5_000_000)))]
        for i,(topic,message) in enumerate(rows,1):
            db.execute('INSERT INTO messages VALUES (?,?,?,?)',(i,topic,100+i,
                bytes(store.serialize_cdr(message,clock_type if topic==1 else state_type))))
    (tmp_path/'probe_summary.json').write_text(json.dumps(dict(fault=None,stop_confirmed=True,lap_completed_sim=19.,brake_sim=20.)))
    window,proof=ready_window(tmp_path)
    assert window.ready_sim_ns==10_005_000_000
    assert window.ready_available_ns==104
    assert proof['state_transitions'][-1]['row_id']==4
    assert window.exclusion(10_000_000_000,105)=='SIMULATOR_NOT_READY_AT_CAPTURE'
    assert window.exclusion(10_005_000_000,105) is None


def test_stage_errors_are_run_equal_and_invalid_predictions_do_not_shrink_support():
    ds=SimpleNamespace(targets=np.zeros((4,30,2),np.float32),run_ids=['a','a','b','c'])
    values=np.zeros_like(ds.targets);values[:, :,0]=np.array([1.,1.,3.,2.])[:,None]
    stages={'nominal':[0,1,2],'recovery':[3]}
    result=stage_errors(values,ds,[0,1,2,3],stages)
    assert result['nominal']['ade_m']==2. and result['nominal']['endpoint_3s_m']==2.
    assert result['nominal']['anchors']==3 and result['nominal']['runs']==2
    values[0,0,0]=float('nan')
    result=stage_errors(values,ds,[0,1,2,3],stages)
    assert result['nominal']['ade_m'] is None and result['nominal']['anchors']==3
    assert result['nominal']['invalid_predictions']==1
    with pytest.raises(ValueError):stage_errors(values[:3],ds,[0,1,2,3],stages)
