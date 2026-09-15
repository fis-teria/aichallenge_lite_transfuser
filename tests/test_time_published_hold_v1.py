import pytest

from aic_transfuser_lite.data.time_published_hold_v1 import longest_published_hold_s


def test_same_clock_stamp_keeps_continuity_without_double_counting_time():
    # ROS may publish twice against the same /clock sample after a delayed tick.
    samples = [(0, 100, 1, True), (50_000_000, 50_000_100, 2, True),
               (50_000_000, 57_300_100, 3, True), (100_000_000, 100_000_100, 4, False)]
    assert longest_published_hold_s(samples) == .1


def test_same_timestamp_replacement_that_releases_does_interrupt_the_hold():
    samples = [(0, 100, 1, True), (50_000_000, 50_000_100, 2, False),
               (50_000_000, 57_300_100, 3, True), (100_000_000, 100_000_100, 4, False)]
    assert longest_published_hold_s(samples) == .05


@pytest.mark.parametrize('gap_kind', ['sim', 'wall', 'sequence'])
def test_missing_evidence_cannot_prove_a_continuous_hold(gap_kind):
    sim = [0, 50_000_000, 100_000_000, 150_000_000]
    wall = [100, 50_000_100, 100_000_100, 150_000_100]
    sequence = [1, 2, 3, 4]
    if gap_kind == 'sim':
        sim[2:] = [250_000_000, 300_000_000]
    elif gap_kind == 'wall':
        wall[2:] = [250_000_100, 300_000_100]
    else:
        sequence[2:] = [4, 5]
    assert longest_published_hold_s(list(zip(sim, wall, sequence, [True, True, True, False]))) == .05


def test_final_command_is_not_extrapolated_and_empty_input_has_no_hold():
    assert longest_published_hold_s([(0, 100, 1, True)]) == 0.
    assert longest_published_hold_s([]) == 0.


@pytest.mark.parametrize('last', [(-1, 2, 2, True), (1, 1, 2, True), (1, 2, 1, True),
                                  (1., 2, 2, True), (1, 2, 2, 1), (1, 2, 2)])
def test_invalid_shape_units_and_time_order_are_rejected(last):
    with pytest.raises(ValueError, match='PUBLISHED_HOLD'):
        longest_published_hold_s([(0, 1, 1, True), last])
