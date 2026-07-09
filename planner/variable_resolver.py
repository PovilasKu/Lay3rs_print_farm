"""Resolve ${...} placeholders inside job catalogs and command JSON files."""

import re
from typing import Any

PLACEHOLDER_RE = re.compile(r"\$\{([^}]+)\}")


def get_path_value(context: dict, dotted_path: str) -> Any:
    current = context
    for part in dotted_path.split("."):
        if isinstance(current, dict) and part in current:
            current = current[part]
        else:
            raise RuntimeError(f"Could not resolve placeholder '${{{dotted_path}}}'. Missing part: {part}")
    return current


def resolve_string(value: str, context: dict) -> Any:
    """
    Resolve a string containing ${...} placeholders.

    If the whole string is one placeholder, return the original value type.
    Example:
        "${printer.tag_id}" -> 11 as int

    If the placeholder is part of a larger string, return a string.
    Example:
        "printer_family/${printer.family}/open.json" -> "printer_family/bambu_x1/open.json"
    """
    full_match = PLACEHOLDER_RE.fullmatch(value.strip())
    if full_match:
        return get_path_value(context, full_match.group(1).strip())

    def replace(match):
        resolved = get_path_value(context, match.group(1).strip())
        return str(resolved)

    return PLACEHOLDER_RE.sub(replace, value)


def resolve_value(value: Any, context: dict) -> Any:
    if isinstance(value, str):
        return resolve_string(value, context)
    if isinstance(value, list):
        return [resolve_value(item, context) for item in value]
    if isinstance(value, dict):
        return {key: resolve_value(item, context) for key, item in value.items()}
    return value
