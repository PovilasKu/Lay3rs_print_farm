"""Build an executable command list from a job name and optional printer/rack slot IDs."""

from typing import Any

import config as cfg
from core.helpers import load_json, save_json
from planner.printer_registry import get_printer, list_printers
from planner.rack_registry import get_slot, list_slots, load_buildplate_rack
from planner.sequence_loader import load_sequence_steps
from planner.variable_resolver import resolve_value
from planner.plan_validator import validate_plan


def load_job_catalog() -> dict:
    data = load_json(cfg.JOB_CATALOG_FILE)
    if not isinstance(data, dict) or "jobs" not in data:
        raise RuntimeError("job_catalog.json must contain a top-level 'jobs' object.")
    return data["jobs"]


def list_jobs() -> dict:
    return load_job_catalog()


def print_available_jobs() -> None:
    jobs = list_jobs()
    print()
    print("Available jobs:")
    print("---------------")
    for name, info in jobs.items():
        print(f"{name:34s} {info.get('description', '')}")
    print()


def print_available_printers() -> None:
    printers = list_printers()
    print()
    print("Available printers:")
    print("-------------------")
    for printer_id, info in printers.items():
        display = info.get("display_name", "")
        family = info.get("family", "")
        enabled = info.get("enabled", True)
        print(f"{printer_id:16s} family={family:12s} enabled={enabled!s:5s} {display}")
    print()


def print_available_slots() -> None:
    slots = list_slots()
    print()
    print("Available buildplate rack slots:")
    print("--------------------------------")
    for slot_id, info in slots.items():
        display = info.get("display_name", "")
        status = info.get("status", "unknown")
        print(f"{slot_id:16s} status={status:12s} {display}")
    print()


def build_context(
    job_name: str,
    job_info: dict,
    printer_id: str | None,
    drop_slot_id: str | None = None,
    pick_slot_id: str | None = None,
    extra_vars: dict | None = None,
) -> tuple[dict, dict | None]:
    requires_printer = bool(job_info.get("requires_printer", False))
    requires_drop_slot = bool(job_info.get("requires_drop_slot", False))
    requires_pick_slot = bool(job_info.get("requires_pick_slot", False))

    printer = None
    if requires_printer:
        if not printer_id:
            raise RuntimeError(f"Job '{job_name}' requires --printer.")
        printer = get_printer(printer_id)
        if not printer.get("enabled", True):
            raise RuntimeError(f"Printer '{printer_id}' is marked enabled=false in printers.json.")
    elif printer_id:
        printer = get_printer(printer_id)

    drop_slot = None
    if requires_drop_slot:
        if not drop_slot_id:
            raise RuntimeError(f"Job '{job_name}' requires --drop-slot.")
        drop_slot = get_slot(drop_slot_id)
    elif drop_slot_id:
        drop_slot = get_slot(drop_slot_id)

    pick_slot = None
    if requires_pick_slot:
        if not pick_slot_id:
            raise RuntimeError(f"Job '{job_name}' requires --pick-slot.")
        pick_slot = get_slot(pick_slot_id)
    elif pick_slot_id:
        pick_slot = get_slot(pick_slot_id)

    context = {
        "job": {"name": job_name, **job_info},
        "printer": printer or {},
        "rack": load_buildplate_rack(),
        "drop_slot": drop_slot or {},
        "pick_slot": pick_slot or {},
        "vars": extra_vars or {},
    }
    return context, printer


def build_plan(
    job_name: str,
    printer_id: str | None = None,
    drop_slot_id: str | None = None,
    pick_slot_id: str | None = None,
    extra_vars: dict | None = None,
    save_plan: bool = True,
) -> list[dict]:
    jobs = load_job_catalog()
    if job_name not in jobs:
        available = ", ".join(sorted(jobs.keys()))
        raise RuntimeError(f"Unknown job '{job_name}'. Available jobs: {available}")

    job_info = jobs[job_name]
    context, printer = build_context(job_name, job_info, printer_id, drop_slot_id, pick_slot_id, extra_vars=extra_vars)

    raw_steps = job_info.get("steps")
    if not isinstance(raw_steps, list):
        raise RuntimeError(f"Job '{job_name}' must contain a list called 'steps'.")

    resolved_steps = resolve_value(raw_steps, context)
    commands = load_sequence_steps(resolved_steps, context, add_markers=True)

    validate_plan(job_name, printer, commands)

    if save_plan:
        plan_name = job_name
        if printer_id:
            plan_name += f"__{printer_id}"
        if drop_slot_id:
            plan_name += f"__drop_{drop_slot_id}"
        if pick_slot_id:
            plan_name += f"__pick_{pick_slot_id}"
        plan_path = cfg.LOGS_DIR / "plans" / f"{plan_name}.planned.json"
        save_json(plan_path, commands)
        print(f"Saved planned command list to: {plan_path}")

    return commands
