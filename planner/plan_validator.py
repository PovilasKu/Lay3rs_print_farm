"""Basic checks for assembled job plans before robot execution."""

KNOWN_COMMAND_TYPES = {
    "_marker",
    "move_cart", "line_cart", "arc_cart", "move_joint",
    "local_cart", "line_tcp", "arc_tcp", "local_joint",
    "localize", "wait",
    "gripper_activate", "gripper", "gripper_status",
    "check_tag_visible", "check_tag_absent", "retry_until_tag_absent",
}


def walk_commands(commands: list[dict]):
    for cmd in commands:
        yield cmd
        nested = cmd.get("commands") if isinstance(cmd, dict) else None
        if isinstance(nested, list):
            yield from walk_commands(nested)


def command_types(commands: list[dict]) -> list[str]:
    return [cmd.get("type") for cmd in walk_commands(commands) if isinstance(cmd, dict) and cmd.get("type") != "_marker"]


def validate_plan(job_name: str, printer: dict | None, commands: list[dict]) -> None:
    if not commands:
        raise RuntimeError(f"Job '{job_name}' produced no commands.")

    types = command_types(commands)

    if printer is not None and "localize" not in types:
        raise RuntimeError(f"Printer job '{job_name}' has no localize command.")

    for i, cmd in enumerate(walk_commands(commands), start=1):
        if not isinstance(cmd, dict):
            raise RuntimeError(f"Plan entry {i} is not a dict: {cmd}")
        if "type" not in cmd:
            raise RuntimeError(f"Plan entry {i} has no 'type': {cmd}")
        if cmd["type"] not in KNOWN_COMMAND_TYPES:
            raise RuntimeError(f"Plan entry {i} uses unknown command type: {cmd['type']}")

    if job_name in ["remove_plate", "insert_plate", "full_buildplate_change", "trial_full_swap_legacy"]:
        if "gripper" not in types and "gripper_activate" not in types:
            print(f"WARNING: Job '{job_name}' does not contain a gripper command.")
