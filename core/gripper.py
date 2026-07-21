"""Fairino gripper helper functions."""

import time

import config as cfg


def clamp_percent_int(name: str, value) -> int:
    """Convert a value to an integer percentage and validate 0–100."""
    value = int(round(float(value)))

    if value < 0 or value > 100:
        raise RuntimeError(
            f"{name} must be from 0 to 100, got: {value}"
        )

    return value


def validate_int_range(
    name: str,
    value,
    minimum: int,
    maximum: int,
) -> int:
    """Convert a value to int and check that it is within a range."""
    value = int(value)

    if value < minimum or value > maximum:
        raise RuntimeError(
            f"{name} must be from {minimum} to {maximum}, "
            f"got: {value}"
        )

    return value


def activate_gripper(
    robot,
    device_id=cfg.GRIPPER_ID,
):
    """
    Reset and activate the configured gripper.

    Fairino ActGripper:
        action=0: reset
        action=1: activate
    """
    device_id = int(device_id)

    print(f"Activating gripper device_id={device_id}...")

    if cfg.DRY_RUN:
        print("[DRY RUN] Would reset and activate gripper.")
        return

    reset_err = robot.ActGripper(device_id, 0)
    print(f"ActGripper reset returned: {reset_err}")

    if reset_err != 0:
        raise RuntimeError(
            f"ActGripper reset failed with error code: {reset_err}"
        )

    time.sleep(0.5)

    activate_err = robot.ActGripper(device_id, 1)
    print(f"ActGripper activate returned: {activate_err}")

    if activate_err != 0:
        raise RuntimeError(
            f"ActGripper activation failed with error code: "
            f"{activate_err}"
        )

    time.sleep(0.5)


def move_gripper(
    robot,
    position,
    speed=None,
    torque=None,
    max_time_ms=None,
    device_id=cfg.GRIPPER_ID,
    block=cfg.GRIPPER_BLOCK,
    gripper_type=cfg.GRIPPER_TYPE,
    rot_num=0.0,
    rot_speed=0,
    rot_torque=0,
    activate=False,
):
    """
    Move the Fairino gripper.

    Fairino SDK signature:

        MoveGripper(
            index,
            pos,
            vel,
            force,
            maxtime,
            block,
            type,
            rotNum,
            rotVel,
            rotTorque,
        )

    For an ordinary parallel gripper:
        gripper_type = 0
        rot_num = 0
        rot_speed = 0
        rot_torque = 0
    """
    device_id = int(device_id)

    position = clamp_percent_int(
        "Gripper position",
        position,
    )

    speed = clamp_percent_int(
        "Gripper speed",
        cfg.GRIPPER_DEFAULT_SPEED
        if speed is None
        else speed,
    )

    torque = clamp_percent_int(
        "Gripper torque",
        cfg.GRIPPER_DEFAULT_TORQUE
        if torque is None
        else torque,
    )

    max_time_ms = validate_int_range(
        "Gripper max_time_ms",
        cfg.GRIPPER_MAX_TIME_MS
        if max_time_ms is None
        else max_time_ms,
        0,
        30000,
    )

    block = validate_int_range(
        "Gripper block",
        block,
        0,
        1,
    )

    gripper_type = validate_int_range(
        "Gripper type",
        gripper_type,
        0,
        1,
    )

    rot_num = float(rot_num)

    rot_speed = clamp_percent_int(
        "Gripper rotation speed",
        rot_speed,
    )

    rot_torque = clamp_percent_int(
        "Gripper rotation torque",
        rot_torque,
    )

    if activate:
        activate_gripper(
            robot,
            device_id=device_id,
        )

    print(
        f"Moving gripper: "
        f"device={device_id}, "
        f"position={position}, "
        f"speed={speed}, "
        f"torque={torque}, "
        f"max_time_ms={max_time_ms}, "
        f"block={block}, "
        f"type={gripper_type}, "
        f"rot_num={rot_num}, "
        f"rot_speed={rot_speed}, "
        f"rot_torque={rot_torque}"
    )

    if cfg.DRY_RUN:
        print("[DRY RUN] Would move gripper.")
        return

    err = robot.MoveGripper(
        device_id,
        position,
        speed,
        torque,
        max_time_ms,
        block,
        gripper_type,
        rot_num,
        rot_speed,
        rot_torque,
    )

    print(f"MoveGripper returned: {err}")

    if err != 0:
        raise RuntimeError(
            f"MoveGripper failed with error code: {err}"
        )

    time.sleep(0.2)


def print_gripper_status(robot):
    """
    Print all gripper status values supported by the installed SDK.

    Unsupported status methods are reported without stopping the program.
    """
    print("Gripper status:")

    status_methods = [
        "GetGripperMotionDone",
        "GetGripperActivateStatus",
        "GetGripperCurPosition",
        "GetGripperCurSpeed",
        "GetGripperCurCurrent",
        "GetGripperVoltage",
        "GetGripperTemp",
    ]

    for method_name in status_methods:
        try:
            method = getattr(robot, method_name)
            result = method()
            print(f"  {method_name}: {result}")

        except AttributeError:
            print(
                f"  {method_name}: unsupported by installed SDK"
            )

        except Exception as exc:
            print(
                f"  {method_name}: query failed: {exc}"
            )