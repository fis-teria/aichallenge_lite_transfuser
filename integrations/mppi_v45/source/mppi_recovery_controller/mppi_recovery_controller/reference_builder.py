"""One background geometry build, latest requested reference, and a small cache."""

from collections import OrderedDict
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
import multiprocessing
import os
import time

from .geometry import build_reference


def build_in_worker(samples, wall, max_width):
    started = time.monotonic()
    geometry = build_reference(samples, wall, max_width)
    return geometry, os.getpid(), (time.monotonic() - started) * 1000


class ReferenceBuilder:
    def __init__(self, max_width, *, cache_size=8, executor=None):
        self.max_width = max_width
        self.cache_size = cache_size
        self.executor = executor if executor is not None else self._new_executor()
        self.wall = None
        self.map_generation = 0
        self.requested = None
        self.pending = None
        self.cache = OrderedDict()
        self.failed = None
        self.error = None
        self.submitted = self.completed = self.cache_hits = self.stale_results = 0
        self.worker_pid = None
        self.build_ms = None

    @staticmethod
    def _new_executor():
        # Never fork a running ROS process and inherit its DDS/executor threads.
        return ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))

    def request(self, samples, wall):
        if wall != self.wall:
            self.wall = wall
            self.map_generation += 1
            self.cache.clear()
            self.failed = None
        key = (self.map_generation, samples) if wall is not None and samples is not None else None
        if key != self.requested and key in self.cache:
            self.cache_hits += 1
            self.cache.move_to_end(key)
        self.requested = key
        return self.poll()

    def poll(self):
        if self.pending is not None and self.pending[1].done():
            key, future = self.pending
            self.pending = None
            try:
                geometry, self.worker_pid, self.build_ms = future.result()
            except Exception as error:
                self.failed = key
                self.error = str(error)
                if isinstance(error, BrokenProcessPool):
                    self.executor.shutdown(wait=False, cancel_futures=True)
                    self.executor = self._new_executor()
            else:
                self.completed += 1
                if key[0] == self.map_generation:
                    # A result for A still warms the cache while B is requested.
                    self.cache[key] = geometry
                    self.cache.move_to_end(key)
                    while len(self.cache) > self.cache_size:
                        self.cache.popitem(last=False)
                else:
                    self.stale_results += 1
        key = self.requested
        if key is None:
            return None
        if key in self.cache:
            return self.cache[key]
        if self.pending is None and key != self.failed:
            try:
                future = self.executor.submit(build_in_worker, key[1], self.wall, self.max_width)
            except Exception as error:
                self.failed = key
                self.error = str(error)
            else:
                self.pending = key, future
                self.submitted += 1
        return None

    def close(self):
        self.executor.shutdown(wait=True, cancel_futures=True)
