from __future__ import annotations

from pathlib import Path
import sys


def prefer_canonical_source() -> Path:
    """Put the committed canonical package before the frozen ROS vendor copy."""
    source_checkout = Path(__file__).resolve().parents[4] / "src"
    candidates = [source_checkout]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("aic_e2e_runtime")) / "python_src"
        )
    except (ImportError, LookupError):
        pass
    for candidate in reversed(candidates):
        if (candidate / "aic_transfuser_lite" / "contracts" / "model_batch_v3.py").is_file():
            value = str(candidate)
            if value in sys.path:
                sys.path.remove(value)
            sys.path.insert(0, value)
            return candidate
    raise RuntimeError("canonical AIC TransFuser V3 source is not installed")
