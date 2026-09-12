"""Bounded concurrent evaluations with progress callbacks on the caller thread."""
from __future__ import annotations

from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from collections.abc import Callable, Sequence
from typing import TypeVar

Job = TypeVar('Job')
Result = TypeVar('Result')


def dispatch(jobs: Sequence[Job], run: Callable[[Job], Result],
             started: Callable[[int, Job], None],
             finished: Callable[[int, Job, Result | Exception], None],
             workers: int) -> list[Result]:
    """Keep at most workers jobs running; drain running jobs after any failure.

    The started callback reserves the finite budget before submission. A failed
    reservation starts no job. Results retain input order regardless of finish
    order. Callbacks never run on a worker thread.
    """
    if workers not in range(1, 5):
        raise ValueError('workers must be between 1 and 4')
    results: dict[int, Result] = {}
    errors: list[Exception] = []
    next_index = 0
    with ThreadPoolExecutor(max_workers=workers) as pool:
        running = {}
        while running or (next_index < len(jobs) and not errors):
            while len(running) < workers and next_index < len(jobs) and not errors:
                index = next_index
                job = jobs[index]
                try:
                    started(index, job)
                    running[pool.submit(run, job)] = index
                    next_index += 1
                except Exception as exc:
                    errors.append(exc)
            if not running:
                break
            done, _ = wait(running, return_when=FIRST_COMPLETED)
            for future in sorted(done, key=lambda item: running[item]):
                index = running.pop(future)
                try:
                    value = future.result()
                    results[index] = value
                except Exception as exc:
                    value = exc
                    errors.append(exc)
                try:
                    finished(index, jobs[index], value)
                except Exception as exc:
                    errors.append(exc)
    if errors:
        raise RuntimeError('Evaluation batch failed; running jobs were drained and remaining jobs were not started') from errors[0]
    return [results[index] for index in range(len(jobs))]
