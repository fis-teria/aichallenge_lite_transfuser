from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from aic_transfuser_lite.data.canonical_schema_v3 import (
    DATASET_SCHEMA_VERSION_V3,
    AssetReferenceV3,
    DatasetManifestV3,
    DenseFutureStateV3,
    MissingReason,
    OptionalNumericV3,
    RunRecordV3,
    make_sample_id,
)


def _run() -> RunRecordV3:
    return RunRecordV3(
        run_id="run01",
        scenario_id="scenario01",
        segment_id="epoch000",
        source_uri="file:///bags/run01",
        source_hash="a" * 64,
        topic_profile_id="awsim_v3",
        start_stamp_ns=1,
        end_stamp_ns=2,
        capabilities=("trajectory",),
        conversion_status="complete",
    )


def test_dataset_v3_manifest_has_versioned_si_contract() -> None:
    manifest = DatasetManifestV3(
        dataset_id="dataset01", topic_profile_id="awsim_v3", runs=(_run(),)
    )
    manifest.validate()
    assert manifest.schema_version == DATASET_SCHEMA_VERSION_V3
    assert manifest.coordinate_frame == "base_link@t_obs"
    assert (manifest.distance_unit, manifest.angle_unit, manifest.time_unit) == (
        "m",
        "rad",
        "s",
    )
    assert make_sample_id("run01", "epoch000", 123) == "run01__epoch000__123"


def test_json_schema_fixes_version_backend_and_si_units() -> None:
    schema_path = Path(__file__).parents[1] / "schemas" / "dataset_v3.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    properties = schema["properties"]
    assert properties["schema_version"]["const"] == DATASET_SCHEMA_VERSION_V3
    assert properties["storage_backend"]["const"] == "csv_npy_jpeg"
    assert properties["coordinate_frame"]["const"] == "base_link@t_obs"
    assert properties["distance_unit"]["const"] == "m"
    assert properties["angle_unit"]["const"] == "rad"
    assert properties["time_unit"]["const"] == "s"


@pytest.mark.parametrize(
    ("value", "valid", "reason"),
    [
        (float("nan"), True, MissingReason.NOT_MISSING),
        (0.0, False, MissingReason.NOT_RECORDED),
        (float("nan"), False, MissingReason.NOT_MISSING),
        (1.0, True, MissingReason.UNKNOWN),
    ],
)
def test_optional_numeric_rejects_valid_missing_inconsistency(
    value: float, valid: bool, reason: MissingReason
) -> None:
    with pytest.raises(ValueError):
        OptionalNumericV3(value, valid, reason).validate(field_name="speed_mps")


def test_asset_missingness_never_uses_a_path_as_zero_fill() -> None:
    missing = AssetReferenceV3(
        path=None,
        valid=False,
        source_stamp_ns=None,
        source_age_ms=None,
        missing_reason=MissingReason.NOT_RECORDED,
    )
    missing.validate(field_name="camera")
    with pytest.raises(ValueError, match="invalid but has an asset path"):
        AssetReferenceV3(
            path="images/fake.jpg",
            valid=False,
            source_stamp_ns=None,
            source_age_ms=None,
            missing_reason=MissingReason.NOT_RECORDED,
        ).validate(field_name="camera")


def test_dense_future_state_validates_shape_and_nan_mask_contract() -> None:
    future = DenseFutureStateV3(
        relative_time_sec=np.array([0.1, 0.2], dtype=np.float32),
        x_m=np.array([1.0, np.nan], dtype=np.float32),
        y_m=np.array([0.0, np.nan], dtype=np.float32),
        yaw_rad=np.array([0.1, np.nan], dtype=np.float32),
        longitudinal_speed_mps=np.array([2.0, np.nan], dtype=np.float32),
        lateral_speed_mps=np.array([0.0, np.nan], dtype=np.float32),
        yaw_rate_rps=np.array([0.0, np.nan], dtype=np.float32),
        valid=np.array([True, False]),
    )
    future.validate()

    with pytest.raises(ValueError, match="must use NaN"):
        DenseFutureStateV3(
            **{
                **future.__dict__,
                "x_m": np.array([1.0, 0.0], dtype=np.float32),
            }
        ).validate()


def test_manifest_rejects_version_unit_hash_and_duplicate_drift() -> None:
    with pytest.raises(ValueError, match="schema version"):
        DatasetManifestV3(
            dataset_id="dataset01",
            topic_profile_id="awsim_v3",
            runs=(_run(),),
            schema_version="v2",
        ).validate()
    with pytest.raises(ValueError, match="SI/frame"):
        DatasetManifestV3(
            dataset_id="dataset01",
            topic_profile_id="awsim_v3",
            runs=(_run(),),
            distance_unit="cm",
        ).validate()
    with pytest.raises(ValueError, match="SHA-256"):
        RunRecordV3(**{**_run().__dict__, "source_hash": "bad"}).validate()
    with pytest.raises(ValueError, match="duplicate run segment"):
        DatasetManifestV3(
            dataset_id="dataset01",
            topic_profile_id="awsim_v3",
            runs=(_run(), _run()),
        ).validate()
