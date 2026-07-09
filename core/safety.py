"""Safety checks for planned and commanded robot poses."""

from pathlib import Path
from core.helpers import load_json
import config as cfg


def load_safe_limits() -> dict | None:
    """
    Load optional pose limits from data/safety/safe_limits.json.

    Expected format:
    {
      "pose_axes": {
        "0": [-500, 500],
        "1": [-700, 700],
        "2": [100, 900],
        "3": [-180, 180],
        "4": [-180, 180],
        "5": [-180, 180]
      }
    }
    """
    path = Path(cfg.SAFE_LIMITS_FILE)
    if not path.exists():
        return None
    data = load_json(path)
    return data.get("pose_axes")


def check_safe_limits(pose: list[float], safe_limits: dict | None = None) -> None:
    limits = safe_limits if safe_limits is not None else load_safe_limits()
    if not limits:
        return

    for axis_key, axis_limits in limits.items():
        axis = int(axis_key)
        low, high = axis_limits
        value = pose[axis]
        if value < low or value > high:
            raise RuntimeError(
                f"Refusing move. Pose axis {axis} value {value:.2f} is outside safe range {axis_limits}"
            )
