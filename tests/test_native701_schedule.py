"""The focused arm must not displace any nominal/recovery presentation."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from compare_native701_retention import comparison_schedule


def test_identical_old_presentations_and_complete_native_coverage():
    args = dict(old_count=91, native_count=701, front=[0, 8, 70], steps=200)
    uniform = comparison_schedule(**args, focused=False)
    focused = comparison_schedule(**args, focused=True)
    assert uniform == comparison_schedule(**args, focused=False)
    assert all(len(batch) == 40 for batch in uniform + focused)
    assert [b[:32] for b in uniform] == [b[:32] for b in focused]
    for schedule in (uniform, focused):
        assert {i for b in schedule for kind, i in b if kind == 'native'} == set(range(701))
        assert all(all(kind == 'old' for kind, _ in b[:32]) for b in schedule)
    assert all(all(i in args['front'] for _, i in b[32:36]) for b in focused)


@pytest.mark.parametrize('front', [[], [1, 1], [-1], [4]])
def test_invalid_membership(front):
    with pytest.raises(ValueError):
        comparison_schedule(old_count=10, native_count=4, front=front, steps=2, focused=True)
