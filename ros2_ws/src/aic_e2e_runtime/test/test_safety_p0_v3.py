from pathlib import Path


def test_safety_node_uses_strict_stamps_and_deadline() -> None:
    source = (Path(__file__).parents[1] / "aic_e2e_runtime" / "safety_supervisor_node.py").read_text()
    assert "from .runtime_adapter import stamp_to_seconds" not in source
    assert "strict_message_stamp_to_seconds" in source
    assert "now > self.nominal_valid_until_sec" in source
