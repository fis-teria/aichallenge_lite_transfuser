from __future__ import annotations

from PIL import Image
import numpy as np
import torch


def preprocess_image(
    image: Image.Image | np.ndarray,
    *,
    height: int,
    width: int,
    mean: tuple[float, float, float] = (0.485, 0.456, 0.406),
    std: tuple[float, float, float] = (0.229, 0.224, 0.225),
) -> torch.Tensor:
    """Resize and normalize an RGB image to a CHW float tensor."""
    pil = image if isinstance(image, Image.Image) else Image.fromarray(np.asarray(image))
    # Pillow < 9.1 exposes filters directly on Image. Both branches select the
    # same bilinear algorithm, so this is a runtime-compatibility boundary and
    # does not change the Dataset-v2 preprocessing contract.
    resampling = getattr(Image, "Resampling", Image)
    pil = pil.convert("RGB").resize((width, height), resampling.BILINEAR)
    array = np.asarray(pil, dtype=np.float32) / 255.0
    tensor = torch.from_numpy(array).permute(2, 0, 1)
    mean_t = torch.tensor(mean, dtype=tensor.dtype).view(3, 1, 1)
    std_t = torch.tensor(std, dtype=tensor.dtype).view(3, 1, 1)
    return (tensor - mean_t) / std_t
