from __future__ import annotations

from typing import Any, Callable, Generic, TypeVar


T = TypeVar("T")


class Registry(Generic[T]):
    """Small explicit factory registry with duplicate/unknown-name rejection."""

    def __init__(self, category: str) -> None:
        if not category:
            raise ValueError("registry category must be non-empty")
        self.category = category
        self._factories: dict[str, Callable[..., T]] = {}

    def register(self, name: str, factory: Callable[..., T]) -> None:
        normalized = str(name).strip()
        if not normalized:
            raise ValueError(f"{self.category} registry name must be non-empty")
        if not callable(factory):
            raise TypeError(f"{self.category} factory must be callable")
        if normalized in self._factories:
            raise ValueError(f"duplicate {self.category} registry entry: {normalized!r}")
        self._factories[normalized] = factory

    def build(self, name: str, **kwargs: Any) -> T:
        if name not in self._factories:
            raise KeyError(
                f"unknown {self.category} {name!r}; available={list(self.names())}"
            )
        return self._factories[name](**kwargs)

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._factories))


MODEL_REGISTRY: Registry[Any] = Registry("model")
ENCODER_REGISTRY: Registry[Any] = Registry("encoder")
TEMPORAL_REGISTRY: Registry[Any] = Registry("temporal")
FUSION_REGISTRY: Registry[Any] = Registry("fusion")
HEAD_REGISTRY: Registry[Any] = Registry("head")


def register_v1_factory(registry: Registry[Any] | None = None) -> Registry[Any]:
    """Register the frozen V1 constructor without changing its existing factory."""

    from .transfuser_lite_v1 import AICTransFuserLiteV1

    target = registry or MODEL_REGISTRY
    if "transfuser_lite_v1" not in target.names():
        target.register(
            "transfuser_lite_v1", lambda config: AICTransFuserLiteV1(config)
        )
    return target
