"""Load printer definitions, printer groups, and rack metadata."""

from pathlib import Path

import config as cfg
from core.helpers import load_json
from planner.variable_resolver import resolve_value


def _load_optional_json(path: Path, default):
    if not path.exists():
        return default
    return load_json(path)


def load_printers() -> dict:
    data = load_json(cfg.PRINTERS_FILE)
    if not isinstance(data, dict) or "printers" not in data:
        raise RuntimeError("printers.json must contain a top-level 'printers' object.")
    return data["printers"]


def load_printer_groups() -> dict:
    data = _load_optional_json(cfg.PRINTER_GROUPS_FILE, {"groups": {}})
    return data.get("groups", {})


def load_rack_layout() -> dict:
    return _load_optional_json(cfg.RACK_LAYOUT_FILE, {"racks": {}})


def list_printers() -> dict:
    return load_printers()


def get_printer(printer_id: str) -> dict:
    printers = load_printers()
    if printer_id not in printers:
        available = ", ".join(sorted(printers.keys()))
        raise RuntimeError(f"Unknown printer '{printer_id}'. Available printers: {available}")

    raw_printer = dict(printers[printer_id])
    raw_printer["id"] = printer_id

    groups = load_printer_groups()
    group_name = raw_printer.get("group")

    merged = {}
    if group_name:
        group = groups.get(group_name, {})
        merged.update(group.get("defaults", {}))
        merged.update(group)
        merged.pop("defaults", None)

    merged.update(raw_printer)

    # Allow printer data itself to contain placeholders, for example:
    # "approach_sequence": "printer_specific/${printer.id}/approach_staging_pose.json"
    context = {"printer": merged}
    merged = resolve_value(merged, context)

    return merged
