"""Load buildplate rack slot definitions."""

import config as cfg
from core.helpers import load_json


def load_buildplate_rack() -> dict:
    data = load_json(cfg.BUILDPLATE_SLOTS_FILE)
    if not isinstance(data, dict) or "rack" not in data:
        raise RuntimeError("buildplate_slots.json must contain a top-level 'rack' object.")
    return data["rack"]


def load_buildplate_slots() -> dict:
    data = load_json(cfg.BUILDPLATE_SLOTS_FILE)
    if not isinstance(data, dict) or "slots" not in data:
        raise RuntimeError("buildplate_slots.json must contain a top-level 'slots' object.")
    return data["slots"]


def list_slots() -> dict:
    return load_buildplate_slots()


def get_slot(slot_id: str) -> dict:
    slots = load_buildplate_slots()
    if slot_id not in slots:
        available = ", ".join(sorted(slots.keys()))
        raise RuntimeError(f"Unknown buildplate rack slot '{slot_id}'. Available slots: {available}")
    slot = dict(slots[slot_id])
    slot["id"] = slot_id
    return slot
