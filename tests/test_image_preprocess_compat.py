from __future__ import annotations

import numpy as np
from PIL import Image
import torch

from aic_transfuser_lite.data import image_preprocess

PIL_IMAGE_MODULE = Image


class LegacyPillowImageProxy:
    """Expose the Pillow <9.1 API without altering Pillow's own internals."""

    BILINEAR = getattr(PIL_IMAGE_MODULE, "Resampling", PIL_IMAGE_MODULE).BILINEAR
    Image = PIL_IMAGE_MODULE.Image
    fromarray = staticmethod(PIL_IMAGE_MODULE.fromarray)


def test_preprocess_supports_pillow_without_resampling_enum(monkeypatch) -> None:
    monkeypatch.setattr(image_preprocess, "Image", LegacyPillowImageProxy)
    rgb = np.arange(12 * 20 * 3, dtype=np.uint8).reshape(12, 20, 3)
    output = image_preprocess.preprocess_image(rgb, height=6, width=10)
    assert output.shape == (3, 6, 10)
    assert output.dtype == torch.float32
    assert torch.isfinite(output).all()
