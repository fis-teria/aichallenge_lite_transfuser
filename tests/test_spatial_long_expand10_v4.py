import numpy as np
import pytest

from aic_transfuser_lite.training.spatial_long_expand10_v4 import ten_metre_candidates, merge_train
from aic_transfuser_lite.training.spatial_long_full_v4 import epoch_batches


def test_distance_boundary_unknown_and_split_isolation():
    rows = [dict(sample_id=str(i), split=split, h30_noise_filtered_arc_m_provisional=value)
            for i, (split, value) in enumerate([
                ('train', '9.999'), ('train', '10'), ('train', '20'),
                ('train', ''), ('train', None), ('val', '20'), ('test', '20')])]
    assert [r['sample_id'] for r in ten_metre_candidates(rows)] == ['1', '2']
    with pytest.raises(ValueError, match='duplicate'):
        ten_metre_candidates(rows + rows)
    for bad in ('nan', 'inf', '-1'):
        with pytest.raises(ValueError, match='distance'):
            ten_metre_candidates([dict(sample_id='bad', split='train', h30_noise_filtered_arc_m_provisional=bad)])


def row(name, support=10., eligible=True, split='train'):
    return dict(sample_id=name, split=split, run_id='run', stamp_ns=0, row_index=0,
                future_sha256=name, diagnostic_eligible=eligible, reasons=[] if eligible else ['bad'],
                geometry={'covered_grid_m': support})


def test_union_preserves_short_teachers_deduplicates_and_rejects_drift():
    old = [row('recovery', 1.3), row('existing'), row('validation', split='validation')]
    expanded = [row('existing'), row('new'), row('rejected', eligible=False)]
    merged = merge_train(old, expanded)
    assert {r['sample_id'] for r in merged} == {'recovery', 'existing', 'new'}
    with pytest.raises(ValueError, match='non-train'):
        merge_train(old, [row('test', split='test')])
    with pytest.raises(ValueError, match='changed'):
        merge_train(old, [row('existing', eligible=False)])
    with pytest.raises(ValueError, match='contradiction'):
        merge_train(old, [row('short', support=9.5)])
    with pytest.raises(ValueError, match='duplicate'):
        merge_train(old, expanded + expanded)


def test_expanded_epoch_includes_all_new_teachers_and_tail():
    count = 12698 + 1786 - 376
    batches = epoch_batches(count, 1)
    ids = np.array([i for batch in batches for i in batch])
    np.testing.assert_array_equal(np.sort(ids), np.arange(count))
    assert len(batches[-1]) == count % 8
