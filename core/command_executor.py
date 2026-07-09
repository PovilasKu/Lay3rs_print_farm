"""Execute one JSON command by dispatching it to the correct core function."""

import time

import config as cfg
from core.helpers import add_vectors, validate_vector
from core.math_utils import tcp_delta_to_global_delta, tcp_delta_to_pose
from core.robot_client import get_actual_joints, get_actual_pose
from core.motion import move_arc, move_cart, move_joint, move_line
from core.localization import execute_localize
from core.gripper import activate_gripper, move_gripper, print_gripper_status
from core.vision_client import check_tag_visible


def execute_move_cart(robot, command):
    move_cart(robot, command["pose"], vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"))


def execute_line_cart(robot, command):
    move_line(robot, command["pose"], vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"))


def execute_arc_cart(robot, command):
    via_pose = command.get("via_pose", command.get("via"))
    end_pose = command.get("end_pose", command.get("pose"))
    if via_pose is None or end_pose is None:
        raise RuntimeError("arc_cart needs via_pose/end_pose or old-style via/pose.")
    move_arc(robot, via_pose, end_pose, vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"), fallback_segments=command.get("fallback_segments", 12))


def execute_move_joint(robot, command):
    move_joint(robot, command["joints"], pose=command.get("pose"), vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"))


def execute_local_cart(robot, command):
    """
    Relative cartesian movement.

    Default frame is TCP/tool-local:
      {"type": "local_cart", "delta": [dx, dy, dz, dRX, dRY, dRZ]}

    Use {"frame": "robot"} to add the delta directly to the robot/base pose.
    Use {"motion": "line"} to force MoveL instead of MoveCart.
    """
    current_pose = get_actual_pose(robot)
    delta = validate_vector("local_cart delta", command["delta"], 6)

    x_sign = float(command.get("local_cart_x_sign", cfg.LOCAL_CART_X_SIGN))
    y_sign = float(command.get("local_cart_y_sign", cfg.LOCAL_CART_Y_SIGN))
    z_sign = float(command.get("local_cart_z_sign", cfg.LOCAL_CART_Z_SIGN))

    applied_delta = list(delta)
    applied_delta[0] *= x_sign
    applied_delta[1] *= y_sign
    applied_delta[2] *= z_sign

    frame = str(command.get("frame", command.get("reference_frame", "tcp"))).strip().lower()
    motion = str(command.get("motion", "cart")).strip().lower()

    if frame in ["tcp", "tool", "local"]:
        global_delta = tcp_delta_to_global_delta(current_pose, applied_delta)
        next_pose = add_vectors(current_pose, global_delta)
        print(f"Current pose:           {[round(v, 3) for v in current_pose]}")
        print(f"Requested TCP delta:    {[round(v, 3) for v in delta]}")
        print(f"Applied TCP delta:      {[round(v, 3) for v in applied_delta]}")
        print(f"Converted global delta: {[round(v, 3) for v in global_delta]}")
    elif frame in ["robot", "base", "global", "pose"]:
        next_pose = add_vectors(current_pose, applied_delta)
        print(f"Current pose:           {[round(v, 3) for v in current_pose]}")
        print(f"Requested robot delta:  {[round(v, 3) for v in delta]}")
        print(f"Applied robot delta:    {[round(v, 3) for v in applied_delta]}")
    else:
        raise RuntimeError(f"Unknown local_cart frame: {frame}. Use 'tcp' or 'robot'.")

    print(f"Local cart signs:       x={x_sign:+.1f}, y={y_sign:+.1f}, z={z_sign:+.1f}")
    print(f"Next pose:              {[round(v, 3) for v in next_pose]}")
    print(f"Motion type:            {motion}")

    if motion in ["line", "straight", "movel"]:
        move_line(robot, next_pose, vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"))
    else:
        move_cart(robot, next_pose, vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"))


def execute_line_tcp(robot, command):
    current_pose = get_actual_pose(robot)
    delta = validate_vector("line_tcp delta", command["delta"][:6], 6)
    next_pose = tcp_delta_to_pose(current_pose, delta)
    move_line(robot, next_pose, vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"))


def execute_arc_tcp(robot, command):
    current_pose = get_actual_pose(robot)
    via_delta_raw = command.get("via_delta")
    end_delta_raw = command.get("end_delta", command.get("delta"))
    if via_delta_raw is None or end_delta_raw is None:
        raise RuntimeError("arc_tcp needs via_delta and end_delta, or old-style via_delta and delta.")

    via_delta = validate_vector("arc_tcp via_delta", via_delta_raw, 6)
    end_delta = validate_vector("arc_tcp end_delta", end_delta_raw, 6)

    via_pose = tcp_delta_to_pose(current_pose, via_delta)
    end_pose = tcp_delta_to_pose(current_pose, end_delta)

    move_arc(robot, via_pose, end_pose, vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"), fallback_segments=command.get("fallback_segments", 12))


def execute_local_joint(robot, command):
    current_joints = get_actual_joints(robot)
    delta_raw = command.get("delta", command.get("delta_joints"))
    if delta_raw is None:
        raise RuntimeError("local_joint needs delta or old-style delta_joints.")
    delta = validate_vector("local_joint delta", delta_raw, 6)
    target_joints = add_vectors(current_joints, delta)
    move_joint(robot, target_joints, vel=command.get("vel"), acc=command.get("acc"), ovl=command.get("ovl"))


def execute_wait(robot, command):
    seconds = float(command.get("seconds", command.get("time", 1.0)))
    print(f"Waiting {seconds:.2f} seconds...")
    time.sleep(seconds)


def execute_gripper_activate(robot, command):
    activate_gripper(robot, device_id=command.get("device_id", cfg.GRIPPER_ID))


def execute_gripper(robot, command):
    move_gripper(
        robot,
        position=command["position"],
        speed=command.get("speed", cfg.GRIPPER_DEFAULT_SPEED),
        torque=command.get("torque", cfg.GRIPPER_DEFAULT_TORQUE),
        max_time_ms=command.get("max_time_ms", cfg.GRIPPER_MAX_TIME_MS),
        device_id=command.get("device_id", cfg.GRIPPER_ID),
        block=command.get("block", cfg.GRIPPER_BLOCK),
        gripper_type=command.get("gripper_type", cfg.GRIPPER_TYPE),
        activate=command.get("activate", False),
    )


def execute_gripper_status(robot, command):
    print_gripper_status(robot)


def execute_check_tag_visible(robot, command):
    tag_id = command.get("tag_id")
    tag_family = command.get("tag_family")
    timeout_s = float(command.get("timeout_s", 2.0))
    require_family = bool(command.get("require_tag_family", tag_family is not None))
    require_stereo = bool(command.get("require_stereo", False))

    visible = check_tag_visible(
        tag_id=tag_id,
        tag_family=tag_family,
        timeout_s=timeout_s,
        require_tag_family=require_family,
        require_stereo=require_stereo,
    )
    print(f"Tag visible check: tag_id={tag_id}, family={tag_family}, visible={visible}")
    if not visible:
        raise RuntimeError(f"Expected tag to be visible, but it was not found: id={tag_id}, family={tag_family}")


def execute_check_tag_absent(robot, command):
    tag_id = command.get("tag_id")
    tag_family = command.get("tag_family")
    timeout_s = float(command.get("timeout_s", 2.0))
    require_family = bool(command.get("require_tag_family", tag_family is not None))
    require_stereo = bool(command.get("require_stereo", False))

    visible = check_tag_visible(
        tag_id=tag_id,
        tag_family=tag_family,
        timeout_s=timeout_s,
        require_tag_family=require_family,
        require_stereo=require_stereo,
    )
    print(f"Tag absent check: tag_id={tag_id}, family={tag_family}, visible={visible}")
    if visible:
        raise RuntimeError(f"Expected tag to be absent, but it was still visible: id={tag_id}, family={tag_family}")


def execute_command_list(robot, commands, start_index=1, total=None):
    total = len(commands) if total is None else total
    for offset, nested in enumerate(commands, start=start_index):
        if nested.get("type") == "_marker":
            print()
            print("############################################################")
            print(nested.get("name", "Starting nested sequence"))
            print("############################################################")
            continue
        execute_command(robot, nested, offset, total)


def execute_retry_until_tag_absent(robot, command):
    """Run nested commands until the selected tag is no longer visible.

    Use this for the door opening operation: localize on the door tag, execute
    the door-opening movements, scan again, and retry if the door tag is still
    visible.
    """
    attempts = int(command.get("max_attempts", 2))
    commands = command.get("commands", [])
    if not isinstance(commands, list) or not commands:
        raise RuntimeError("retry_until_tag_absent requires a non-empty 'commands' list.")

    check = command.get("check", {})
    tag_id = check.get("tag_id", command.get("tag_id"))
    tag_family = check.get("tag_family", command.get("tag_family"))
    timeout_s = float(check.get("timeout_s", command.get("timeout_s", 2.0)))
    require_family = bool(check.get("require_tag_family", command.get("require_tag_family", tag_family is not None)))
    require_stereo = bool(check.get("require_stereo", command.get("require_stereo", False)))

    for attempt in range(1, attempts + 1):
        print()
        print(f"Retry block attempt {attempt}/{attempts}: {command.get('name', 'unnamed retry block')}")
        execute_command_list(robot, commands, start_index=1, total=len(commands))

        visible = check_tag_visible(
            tag_id=tag_id,
            tag_family=tag_family,
            timeout_s=timeout_s,
            require_tag_family=require_family,
            require_stereo=require_stereo,
        )
        print(f"Post-attempt tag visibility: id={tag_id}, family={tag_family}, visible={visible}")
        if not visible:
            print("Retry block succeeded: tag is absent.")
            return

        if attempt < attempts:
            print("Tag is still visible; retrying nested sequence.")

    raise RuntimeError(
        f"Retry block failed: tag remained visible after {attempts} attempts. "
        f"id={tag_id}, family={tag_family}"
    )


COMMAND_HANDLERS = {
    "move_cart": execute_move_cart,
    "line_cart": execute_line_cart,
    "arc_cart": execute_arc_cart,
    "move_joint": execute_move_joint,
    "local_cart": execute_local_cart,
    "line_tcp": execute_line_tcp,
    "arc_tcp": execute_arc_tcp,
    "local_joint": execute_local_joint,
    "localize": execute_localize,
    "wait": execute_wait,
    "gripper_activate": execute_gripper_activate,
    "gripper": execute_gripper,
    "gripper_status": execute_gripper_status,
    "check_tag_visible": execute_check_tag_visible,
    "check_tag_absent": execute_check_tag_absent,
    "retry_until_tag_absent": execute_retry_until_tag_absent,
}


def execute_command(robot, command, index, total):
    if not isinstance(command, dict):
        raise RuntimeError(f"Command {index} must be an object/dict, got: {command}")

    command_type = command.get("type")
    name = command.get("name", "unnamed")

    print()
    print("============================================================")
    print(f"COMMAND {index}/{total}: {command_type} - {name}")
    print("============================================================")

    handler = COMMAND_HANDLERS.get(command_type)
    if handler is None:
        raise RuntimeError(f"Unknown command type: {command_type}")

    handler(robot, command)
