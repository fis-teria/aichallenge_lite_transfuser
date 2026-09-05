"""Annotation-only postprocessing; no model/optimizer execution."""
import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location("annotation", Path(__file__).parents[1]/"tools/annotate_spatial_validation_results_v4.py")
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


@pytest.mark.parametrize("spelling", ["val", "validation"])
def test_annotation_alias_preserves_fixed_selection(spelling: str) -> None:
    selected = [dict(sample_id="id", run_id="run", role="validation_main", normal_recovery="UNKNOWN", collection_slice="UNKNOWN")]
    annotations = [dict(sample_id="id", run_id="run", split=spelling, normal_recovery="recovery", collection_case="offset_left_far")]
    result = module.join_annotations(selected, annotations)
    assert result[0]["sample_id"] == "id" and result[0]["role"] == "validation_main"
    assert selected[0]["normal_recovery"] == "UNKNOWN" and result[0]["normal_recovery"] == "recovery"


@pytest.mark.parametrize("fault", ["missing", "test", "run", "duplicate"])
def test_bad_annotation_rejected(fault: str) -> None:
    selected = [dict(sample_id="id", run_id="run")]
    annotations = [dict(sample_id="id", run_id="run", split="val", normal_recovery="normal", collection_case="unknown")]
    if fault == "missing": annotations = []
    if fault == "test": annotations[0]["split"] = "test"
    if fault == "run": annotations[0]["run_id"] = "other"
    if fault == "duplicate": annotations *= 2
    with pytest.raises(ValueError): module.join_annotations(selected, annotations)
