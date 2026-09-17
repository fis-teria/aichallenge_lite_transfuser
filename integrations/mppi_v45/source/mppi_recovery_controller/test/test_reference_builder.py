from concurrent.futures import Future
from dataclasses import replace
import os
import pickle
import time

from mppi_recovery_controller.geometry import WallGrid, build_reference
from mppi_recovery_controller.reference_builder import ReferenceBuilder


def wall():
    return WallGrid(.5, 80, 40, -10., -10., 1., 0.,
                    bytes(100 if row in (0, 39) else 0
                          for row in range(40) for _ in range(80)), 'map')


def samples(y=0.):
    return tuple((float(x), y, 0., 3.) for x in range(12))


class ControlledExecutor:
    def __init__(self):
        self.jobs = []

    def submit(self, function, *args):
        future = Future()
        self.jobs.append((function, args, future))
        return future

    def finish(self, index):
        function, args, future = self.jobs[index]
        future.set_result(function(*args))

    def shutdown(self, **_kwargs):
        pass


def test_cold_build_runs_in_child_and_result_remains_usable_after_worker_exit():
    builder = ReferenceBuilder(6.)
    try:
        assert builder.request(samples(), wall()) is None
        deadline = time.monotonic() + 10
        result = None
        while time.monotonic() < deadline and result is None:
            result = builder.poll()
            time.sleep(.01)
        assert result is not None
        assert builder.worker_pid != os.getpid()
        assert result.points == build_reference(samples(), wall(), 6.).points
        assert result.is_free(0., 0.)
        assert not result.is_free(0., -10.)
    finally:
        builder.close()
    restored = pickle.loads(pickle.dumps(result))
    assert restored.occupied_body_samples((0., -10., 0.), [(0., 0.)]) == 1


def test_return_to_A_while_B_builds_reuses_A_and_warms_B_cache():
    pool = ControlledExecutor()
    builder = ReferenceBuilder(6., executor=pool)
    assert builder.request(samples(), wall()) is None
    pool.finish(0)
    a = builder.poll()
    assert a is not None
    assert builder.request(samples(1.), wall()) is None
    assert builder.request(samples(), wall()) is a
    pool.finish(1)
    assert builder.poll() is a
    b = builder.request(samples(1.), wall())
    assert b.points[0].y == 1.
    assert builder.submitted == 2
    assert builder.cache_hits == 2


def test_pending_requests_coalesce_and_old_map_result_is_not_adopted():
    pool = ControlledExecutor()
    builder = ReferenceBuilder(6., executor=pool)
    builder.request(samples(), wall())
    builder.request(samples(1.), wall())
    builder.request(samples(2.), wall())
    newer = replace(wall(), cells=bytes([255])*len(wall().cells))
    assert builder.request(samples(3.), newer) is None
    pool.finish(0)
    assert builder.poll() is None
    assert builder.stale_results == 1
    assert len(pool.jobs) == 2
    assert pool.jobs[1][1][0] == samples(3.)
    pool.finish(1)
    result = builder.poll()
    assert result.points[0].y == 3.
    assert not result.is_free(0., 0.)
    assert all(p.lower == p.upper == 0. for p in result.points)


def test_failed_reference_does_not_spin_or_replace_current_generation():
    pool = ControlledExecutor()
    builder = ReferenceBuilder(6., executor=pool)
    builder.request(samples(), wall())
    pool.jobs[0][2].set_exception(ValueError('invalid map'))
    assert builder.poll() is None
    assert builder.error == 'invalid map'
    for _ in range(10):
        assert builder.poll() is None
    assert builder.submitted == 1
    builder.request(samples(1.), wall())
    pool.finish(1)
    assert builder.poll().points[0].y == 1.
