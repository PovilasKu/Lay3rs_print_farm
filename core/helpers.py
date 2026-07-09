"""Small helper functions used across core robot modules."""

import json
import math
from pathlib import Path
from typing import Any


def load_json(path: str | Path) -> Any:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path: str | Path, data: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def validate_vector(name: str, values: Any, length: int) -> list[float]:
    if not isinstance(values, list):
        raise RuntimeError(f"{name} must be a list, got: {type(values)}")
    if len(values) != length:
        raise RuntimeError(f"{name} must contain exactly {length} values, got {len(values)}: {values}")
    if not all(is_number(v) for v in values):
        raise RuntimeError(f"{name} contains non-numeric values: {values}")
    return [float(v) for v in values]


def add_vectors(a: list[float], b: list[float]) -> list[float]:
    if len(a) != len(b):
        raise RuntimeError(f"Cannot add vectors of different lengths: {len(a)} and {len(b)}")
    return [float(x) + float(y) for x, y in zip(a, b)]


def add_axis_step(pose: list[float], axis: int, step: float) -> list[float]:
    new_pose = list(pose)
    new_pose[int(axis)] += float(step)
    return new_pose


def get_number(params: dict | None, key: str, default: Any) -> float:
    params = params or {}
    return float(params.get(key, default))


def truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ["1", "true", "yes", "y", "ok", "stereo", "fused"]
    return False
