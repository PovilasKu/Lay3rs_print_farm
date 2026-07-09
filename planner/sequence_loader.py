"""Load and merge JSON command sequence files."""

from pathlib import Path
from typing import Any

import config as cfg
from core.helpers import load_json
from planner.variable_resolver import resolve_value


def sequence_path(relative_path: str | Path) -> Path:
    return cfg.SEQUENCES_DIR / Path(relative_path)


def load_sequence_file(relative_path: str | Path) -> list[dict]:
    path = sequence_path(relative_path)
    if not path.exists():
        raise RuntimeError(f"Sequence file does not exist: {path}")

    commands = load_json(path)
    if not isinstance(commands, list):
        raise RuntimeError(f"Sequence file must contain a list of commands: {path}")

    for i, command in enumerate(commands, start=1):
        if not isinstance(command, dict):
            raise RuntimeError(f"Command {i} in {path} is not an object/dict: {command}")

    return commands


def load_and_resolve_sequence(relative_path: str | Path, context: dict) -> list[dict]:
    commands = load_sequence_file(relative_path)
    return resolve_value(commands, context)


def load_sequence_steps(step_paths: list[Any], context: dict, add_markers: bool = True) -> list[dict]:
    """
    Load a list of sequence path strings and merge them into one command list.

    step_paths are usually already resolved by job_planner. They may also contain
    dictionaries of this form:
      {"file": "path/to/file.json", "optional": true}
    """
    all_commands = []

    for raw_step in step_paths:
        if raw_step is None or raw_step == "":
            continue

        optional = False
        step = raw_step
        if isinstance(raw_step, dict):
            step = raw_step.get("file")
            optional = bool(raw_step.get("optional", False))

        if not step:
            continue

        path = sequence_path(step)
        if optional and not path.exists():
            print(f"Skipping optional missing sequence: {step}")
            continue

        if add_markers:
            all_commands.append({
                "type": "_marker",
                "name": f"Starting sequence: {step}",
                "sequence_file": str(step),
            })

        all_commands.extend(load_and_resolve_sequence(step, context))

    return all_commands
