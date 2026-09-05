from __future__ import annotations

import numpy as np
import pytest

from aic_transfuser_lite.data.spatial_diagnostic_view_v4 import GRID, spatial_target, self_intersects


def future(kind: str = "straight") -> np.ndarray:
    t = np.arange(1, 31, dtype=np.float64) * .1
    value = np.zeros((30, 8), dtype=np.float64)
    value[:, 0], value[:, 4], value[:, 7] = t, 1., 1.
    value[:, 1] = t
    if kind in ("left", "right", "nonmonotonic_x"):
        angle = t * (1.5 if kind == "nonmonotonic_x" else .5)
        value[:, 1] = np.sin(angle)
        value[:, 2] = (1-np.cos(angle)) * (-1 if kind == "right" else 1)
    elif kind == "slow":
        value[:, 1], value[:, 4] = t*.03, .03
    elif kind == "stopped":
        value[:, 1:7] = 0
    return value


@pytest.mark.parametrize("kind", ["straight", "left", "right", "nonmonotonic_x", "slow", "stopped"])
def test_geometry_contract(kind: str) -> None:
    data = future(kind)
    old = data.copy()
    result = spatial_target(data)
    np.testing.assert_array_equal(data, old)
    assert result["xy"].shape == (20, 2) and result["mask"].shape == (20,)
    assert result["tier"] == "OBSERVED_DIAGNOSTIC_ONLY"
    assert np.isfinite(result["xy"]).all()
    if kind == "straight":
        np.testing.assert_allclose(result["xy"][:, 0], GRID)
        assert result["mask"].all()
        assert result["xy"][0, 0] == pytest.approx(.1)
    elif kind == "slow":
        assert result["processed_length_m"] > .08  # Last-adopted-point accumulation, not per-frame deadband.
    elif kind == "stopped":
        assert result["processed_length_m"] == 0 and not result["mask"].any()
        assert result["distance_status"] == "KNOWN_OBSERVED"
    elif kind == "nonmonotonic_x":
        assert np.any(np.diff(data[:, 1]) < 0)
        assert result["mask"].all()  # X nonmonotonicity is not a rejection criterion.


@pytest.mark.parametrize("first", [0, 5, 13, 29])
def test_missing_is_not_bridged(first: int) -> None:
    data = future()
    data[first, 7] = 0
    data[first, 1:7] = np.nan
    result = spatial_target(data)
    assert result["prefix_count"] == first
    if first == 0:
        assert result["raw_length_m"] is None and not result["mask"].any()
    else:
        assert result["raw_length_m"] == pytest.approx(first*.1)
    assert (result["xy"][~result["mask"]] == 0).all()


@pytest.mark.parametrize("fault", ["valid_nan", "time_reverse", "time_gap", "jump", "badmask", "frame", "shape"])
def test_prefix_or_structure_faults(fault: str) -> None:
    data = future()
    if fault == "valid_nan":
        data[5, 1] = np.nan
    elif fault == "time_reverse":
        data[5, 0] = .1
    elif fault == "time_gap":
        data[5, 0] = 1.2
    elif fault == "jump":
        data[5, 1] = 20
    elif fault == "badmask":
        data[5, 7] = 2
    if fault in ("badmask", "frame", "shape"):
        with pytest.raises(ValueError):
            spatial_target(data[:29] if fault == "shape" else data, frame="UNKNOWN" if fault == "frame" else "base_link@t_obs")
    else:
        assert spatial_target(data)["prefix_count"] == 5


def test_duplicates_short_path_no_extrapolation() -> None:
    data = future()
    data[:, 1] = np.minimum(data[:, 1], .35)
    result = spatial_target(data)
    assert result["raw_length_m"] == pytest.approx(.35)
    assert result["mask"].sum() == 3
    assert (result["xy"][3:] == 0).all()


def test_noise_hold_and_reverse_flags() -> None:
    data = future("stopped")
    data[:, 1] = np.sin(np.arange(30)) * .003
    result = spatial_target(data)
    assert result["raw_length_m"] > 0 and result["processed_length_m"] == 0
    assert "long_hold_not_intent" in result["flags"]
    data = future()
    data[:, 4] = -1
    assert "reverse_motion" in spatial_target(data)["flags"]


def test_self_intersection() -> None:
    assert self_intersects(np.array([[1, 1], [0, 1], [1, 0]], dtype=float))
    assert not self_intersects(np.array([[1, 0], [2, 0]], dtype=float))
