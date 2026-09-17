"""Pin the collection variant while retaining original upstream source hashes."""
import hashlib
import json
from pathlib import Path


def test_pinned_mppi_v45_source_and_runtime_dependency() -> None:
    root = Path(__file__).resolve().parents[1] / "integrations/mppi_v45"
    manifest = json.loads((root / "source_manifest.json").read_text())
    paths = set()
    for row in manifest["files"]:
        path = root / row["path"]
        assert path.resolve().is_relative_to(root.resolve())
        assert row["path"] not in paths
        paths.add(row["path"])
        data = path.read_bytes()
        assert len(data) == row["bytes"], row["path"]
        assert hashlib.sha256(data).hexdigest() == row["sha256"], row["path"]
    assert "support/multi_purpose_mpc_ros/boost_logic.py" in paths
    assert manifest["teacher"] == "MPPI_SIM_V45"
    assert manifest["collection_revision"] == "lidar-motion-intent-r2"
    changes = {r["path"] for r in manifest["files"] if "upstream_sha256" in r}
    assert changes == set(manifest["collection_changed_files"])
    assert "source/reference_space_mppi_planner/src/reference_space_mppi_node.cpp" in changes
    for row in manifest["files"]:
        if "upstream_sha256" in row:
            assert row["upstream_sha256"] is None or len(row["upstream_sha256"]) == 64
            assert row["upstream_sha256"] != row["sha256"]
