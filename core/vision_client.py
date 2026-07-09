"""HTTP client and normalizer for the Vision Pi AprilTag JSON output."""

import json
import math
import statistics
import time
import urllib.request

import config as cfg
from core.helpers import truthy


def normalize_family_name(value) -> str | None:
    """Return a comparable AprilTag family name, e.g. tag36h11 or tag25h9."""
    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    text = text.replace("_", "").replace("-", "").replace(" ", "")
    if text in ["369", "36h11", "tag36", "tag36h11", "3611"]:
        return "tag36h11"
    if text in ["259", "25h9", "tag25", "tag25h9"]:
        return "tag25h9"
    if text.startswith("36") and "11" in text:
        return "tag36h11"
    if text.startswith("25") and "9" in text:
        return "tag25h9"
    return text


def read_tag_once() -> dict:
    with urllib.request.urlopen(cfg.VISION_URL, timeout=1.0) as response:
        return json.loads(response.read().decode("utf-8"))


def first_present(maps: list[dict], keys: list[str]):
    for data in maps:
        if not isinstance(data, dict):
            continue
        for key in keys:
            if key in data and data[key] is not None:
                return data[key]
    return None


def collect_vision_maps(raw: dict) -> list[dict]:
    maps = []
    for key in ["stereo", "fused", "pose", "tag", "result"]:
        value = raw.get(key)
        if isinstance(value, dict):
            maps.append(value)
    maps.append(raw)
    return maps


def normalize_vision_tag(raw: dict) -> dict:
    """Normalize new stereo JSON and older mono-style JSON into one common format."""
    if not isinstance(raw, dict):
        raise RuntimeError(f"Vision Pi returned non-dict JSON: {raw}")

    maps = collect_vision_maps(raw)
    tag = dict(raw)

    # Prefer nested stereo/fused values, while keeping top-level metadata.
    for item in reversed(maps):
        if isinstance(item, dict):
            tag.update(item)

    ok_value = first_present(maps, ["ok", "detected", "valid"])
    if ok_value is not None:
        tag["ok"] = truthy(ok_value)

    aliases = {
        "tag_id": ["tag_id", "id", "tagId"],
        "tag_family": ["tag_family", "family", "tagFamily", "tag_family_name", "apriltag_family"],
        "x_mm": ["x_mm", "x", "X_mm", "camera_x_mm"],
        "y_mm": ["y_mm", "y", "Y_mm", "camera_y_mm"],
        "z_mm": ["z_mm", "z", "Z_mm", "depth_mm", "camera_z_mm"],
        "distance_mm": ["distance_mm", "distance", "range_mm", "depth_mm", "z_mm"],
        "cx": ["cx", "center_x", "u", "pixel_x"],
        "cy": ["cy", "center_y", "v", "pixel_y"],
        "age_s": ["age_s", "age", "data_age_s"],
        "timestamp": ["timestamp", "time", "t"],
        "tag_pitch_deg": ["tag_pitch_deg", "pitch_deg", "pitch", "tag_tilt_x_deg", "tilt_x_deg", "tilt_x"],
        "tag_yaw_deg": ["tag_yaw_deg", "yaw_deg", "yaw", "tag_tilt_y_deg", "tilt_y_deg", "tilt_y"],
        "tag_roll_deg": ["tag_roll_deg", "roll_deg", "roll"],
        "tag_tilt_x_deg": ["tag_tilt_x_deg", "tilt_x_deg", "tilt_x", "tag_pitch_deg", "pitch_deg"],
        "tag_tilt_y_deg": ["tag_tilt_y_deg", "tilt_y_deg", "tilt_y", "tag_yaw_deg", "yaw_deg"],
        "source": ["source", "pose_source", "mode", "vision_mode"],
        "left_ok": ["left_ok", "left_detected"],
        "right_ok": ["right_ok", "right_detected"],
        "stereo_ok": ["stereo_ok", "fused_ok"],
        "disparity_px": ["disparity_px", "mean_disparity_px", "disparity"],
        "rectified_y_error_px": ["rectified_y_error_px", "mean_rectified_y_error_px"],
        "tag_size_error_mm": ["tag_size_error_mm"],
        "left_margin": ["left_margin"],
        "right_margin": ["right_margin"],
        "frame_id": ["frame_id"],
    }

    for normalized_key, possible_keys in aliases.items():
        value = first_present(maps, possible_keys)
        if value is not None:
            tag[normalized_key] = value

    if "age_s" not in tag and "timestamp" in tag:
        try:
            tag["age_s"] = max(0.0, time.time() - float(tag["timestamp"]))
        except Exception:
            pass

    if "distance_mm" not in tag and "z_mm" in tag:
        tag["distance_mm"] = tag["z_mm"]

    return tag


def is_stereo_confirmed(tag: dict) -> bool:
    source = str(tag.get("source", "")).strip().lower()
    if "stereo" in source or "fused" in source:
        return True
    if truthy(tag.get("stereo_ok")):
        return True
    if truthy(tag.get("left_ok")) and truthy(tag.get("right_ok")):
        return True
    if tag.get("disparity_px") is not None:
        return True
    return False


def vision_source_policy_from_params(params: dict | None) -> str:
    params = params or {}
    if bool(params.get("require_stereo", False)):
        return "require_stereo"
    return str(params.get("vision_source_policy", cfg.VISION_SOURCE_POLICY)).strip().lower()


def describe_vision_source(tag: dict) -> str:
    source = str(tag.get("source", "unknown"))
    parts = [f"source={source}"]
    if "tag_family" in tag:
        parts.append(f"family={tag.get('tag_family')}")

    for key in ["left_ok", "right_ok", "stereo_ok"]:
        if key in tag:
            parts.append(f"{key}={truthy(tag.get(key))}")

    if "disparity_px" in tag:
        try:
            parts.append(f"disparity={float(tag['disparity_px']):.1f}px")
        except Exception:
            parts.append(f"disparity={tag['disparity_px']}")

    if "rectified_y_error_px" in tag:
        try:
            parts.append(f"rect_y={float(tag['rectified_y_error_px']):.2f}px")
        except Exception:
            pass

    return ", ".join(parts)


def validate_tag_for_localize(tag, required_tag_id=None, enable_rotation=True, params=None, quiet=False) -> bool:
    if not tag.get("ok"):
        if not quiet:
            print("Waiting for AprilTag detection...")
        return False

    if required_tag_id is not None:
        seen_tag_id = tag.get("tag_id")
        if seen_tag_id is None or int(seen_tag_id) != int(required_tag_id):
            if not quiet:
                print(f"Waiting for tag_id={required_tag_id}; currently seeing tag_id={seen_tag_id}...")
            return False

    params = params or {}
    required_family = params.get("tag_family", params.get("required_tag_family"))
    require_family_check = bool(params.get("require_tag_family", required_family is not None))
    if require_family_check and required_family is not None:
        expected_family = normalize_family_name(required_family)
        seen_family_raw = tag.get("tag_family")
        seen_family = normalize_family_name(seen_family_raw)
        if seen_family != expected_family:
            if not quiet:
                print(
                    f"Waiting for AprilTag family={expected_family}; "
                    f"currently seeing family={seen_family_raw!r} normalized={seen_family!r}..."
                )
            return False

    age_s = float(tag.get("age_s", 999.0))
    if age_s > cfg.MAX_VISION_AGE_S:
        if not quiet:
            print(f"Waiting for fresh tag data, age={age_s:.2f}s...")
        return False

    source_policy = vision_source_policy_from_params(params)
    stereo_confirmed = is_stereo_confirmed(tag)

    if source_policy in ["require_stereo", "stereo", "stereo_only"] and not stereo_confirmed:
        if not quiet:
            print("Waiting for confirmed stereo/fused AprilTag pose...")
            print(f"Vision status: {describe_vision_source(tag)}")
        return False

    if source_policy == "prefer_stereo" and not stereo_confirmed and cfg.PRINT_VISION_SOURCE and not quiet:
        print("WARNING: using AprilTag pose, but stereo/fused source was not confirmed.")
        print(f"Vision status: {describe_vision_source(tag)}")

    needed = ["x_mm", "y_mm", "z_mm", "distance_mm", "cx", "cy"]
    if enable_rotation:
        needed += ["tag_pitch_deg", "tag_yaw_deg", "tag_roll_deg"]

    missing = [key for key in needed if key not in tag]
    if missing:
        if not quiet:
            print("Tag detected, but some required values are missing from Vision Pi output.")
            print(f"Missing keys: {missing}")
        return False

    return True


def wait_for_fresh_tag(required_tag_id=None, enable_rotation=True, params=None, quiet=False) -> dict:
    for _ in range(25):
        try:
            raw_tag = read_tag_once()
            tag = normalize_vision_tag(raw_tag)

            if validate_tag_for_localize(tag, required_tag_id, enable_rotation, params, quiet=quiet):
                if cfg.PRINT_VISION_SOURCE and not quiet:
                    print(f"Vision status: {describe_vision_source(tag)}")
                return tag

        except Exception as e:
            if not quiet:
                print(f"Vision read failed: {e}")

        time.sleep(0.2)

    raise RuntimeError("No fresh AprilTag pose received from Vision Pi.")


def median_or_none(values):
    if not values:
        return None
    return float(statistics.median(values))


def spread_or_none(values):
    if not values:
        return None
    return float(max(values) - min(values))


def collect_stable_tag(required_tag_id=None, enable_rotation=True, params=None, samples=7, delay_s=None) -> dict:
    delay_s = cfg.VISION_SAMPLE_DELAY_S if delay_s is None else float(delay_s)
    samples = max(1, int(samples))
    min_valid = max(3, int(math.ceil(samples * cfg.VISION_MIN_VALID_RATIO))) if samples >= 3 else 1

    valid_tags = []
    last_error = None

    for _ in range(samples):
        try:
            tag = wait_for_fresh_tag(
                required_tag_id=required_tag_id,
                enable_rotation=enable_rotation,
                params=params,
                quiet=True,
            )
            valid_tags.append(tag)
        except Exception as e:
            last_error = e

        time.sleep(delay_s)

    if len(valid_tags) < min_valid:
        raise RuntimeError(
            f"Could not collect enough valid vision samples: {len(valid_tags)}/{samples}. Last error: {last_error}"
        )

    numeric_keys = [
        "x_mm", "y_mm", "z_mm", "distance_mm", "cx", "cy",
        "tag_pitch_deg", "tag_yaw_deg", "tag_roll_deg",
        "tag_tilt_x_deg", "tag_tilt_y_deg",
        "disparity_px", "rectified_y_error_px", "tag_size_error_mm",
        "left_margin", "right_margin", "age_s",
    ]

    out = dict(valid_tags[-1])
    out["sample_count"] = len(valid_tags)
    out["requested_sample_count"] = samples

    for key in numeric_keys:
        values = []
        for tag in valid_tags:
            if key in tag and tag[key] is not None:
                try:
                    values.append(float(tag[key]))
                except Exception:
                    pass

        if values:
            out[key] = median_or_none(values)
            out[key + "_spread"] = spread_or_none(values)

    newest = max(valid_tags, key=lambda t: float(t.get("timestamp", 0.0)))
    if "timestamp" in newest:
        out["timestamp"] = newest["timestamp"]
        out["age_s"] = max(0.0, time.time() - float(newest["timestamp"]))
    if "frame_id" in newest:
        out["frame_id"] = newest["frame_id"]

    return out


def check_tag_visible(tag_id=None, tag_family=None, timeout_s=1.5, require_tag_family=True, require_stereo=False) -> bool:
    """Return True if a matching tag is visible within timeout_s.

    This is intended for logic checks, e.g. after opening the door the 25h9
    door tag should no longer be visible. It does not require full pose values.
    """
    deadline = time.time() + float(timeout_s)
    params = {
        "tag_family": tag_family,
        "require_tag_family": require_tag_family,
        "vision_source_policy": "require_stereo" if require_stereo else "allow_any",
    }

    while time.time() < deadline:
        try:
            raw = read_tag_once()
            tag = normalize_vision_tag(raw)

            if not tag.get("ok"):
                time.sleep(0.15)
                continue

            if tag_id is not None:
                seen_id = tag.get("tag_id")
                if seen_id is None or int(seen_id) != int(tag_id):
                    time.sleep(0.15)
                    continue

            expected_family = normalize_family_name(tag_family)
            if require_tag_family and expected_family is not None:
                seen_family = normalize_family_name(tag.get("tag_family"))
                if seen_family != expected_family:
                    time.sleep(0.15)
                    continue

            if require_stereo and not is_stereo_confirmed(tag):
                time.sleep(0.15)
                continue

            return True
        except Exception:
            time.sleep(0.15)

    return False
