from threading import Barrier, Event, Lock, get_ident

import pytest

from tools.mppi_cma.parallel_dispatch import dispatch


def test_four_concurrent_jobs_preserve_result_order_and_serial_callbacks():
    barrier=Barrier(4,timeout=5)
    lock=Lock()
    active=0
    maximum=0
    caller=get_ident()
    def run(index):
        nonlocal active,maximum
        with lock:
            active+=1;maximum=max(maximum,active)
        barrier.wait()
        with lock:active-=1
        return index*10
    def callback(*_args):assert get_ident()==caller
    assert dispatch(list(range(8)),run,callback,callback,4)==[i*10 for i in range(8)]
    assert maximum==4


def test_failure_stops_new_jobs_but_preserves_other_running_results():
    ready=Event();release=Event();started_jobs=[];finished_jobs=[]
    def started(index,_job):
        started_jobs.append(index)
        if len(started_jobs)==4:ready.set()
    def run(index):
        assert ready.wait(5)
        if index==0:raise ValueError('simulation failure')
        assert release.wait(5)
        return index
    def finished(index,_job,result):
        finished_jobs.append((index,result))
        if isinstance(result,Exception):release.set()
    with pytest.raises(RuntimeError,match='drained'):
        dispatch(list(range(8)),run,started,finished,4)
    assert started_jobs==[0,1,2,3]
    assert sorted(i for i,result in finished_jobs if not isinstance(result,Exception))==[1,2,3]


def test_budget_reservation_failure_starts_no_unreserved_job():
    actual=[]
    def started(index,_job):
        if index>=2:raise ValueError('finite budget reached')
    def run(index):actual.append(index);return index
    with pytest.raises(RuntimeError,match='drained'):
        dispatch(list(range(7)),run,started,lambda *_args:None,4)
    assert sorted(actual)==[0,1]
    with pytest.raises(ValueError):dispatch([],run,started,lambda *_args:None,5)
