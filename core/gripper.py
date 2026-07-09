"""Fairino gripper helper functions."""

import time

import config as cfg


def clamp_percent_int(name, value) -> int:
    value = int(round(float(value)))
    if value < 0 or value > 100:
        raise RuntimeError(f"{name} must be from 0 to 100, got: {value}")
    return value


def activate_gripper(robot, device_id=cfg.GRIPPER_ID):
    device_id = int(device_id)
    print(f"Activating gripper device_id={device_id}...")

    if cfg.DRY_RUN:
        print("[DRY RUN] Would activate gripper.")
        return

    try:
        err = robot.ActGripper(device_id, 0)
        print(f"ActGripper reset returned: {err}")
        time.sleep(0.5)
    except Exception as e:
        print(f"Gripper reset failed/ignored: {e}")

    err = robot.ActGripper(device_id, 1)
    print(f"ActGripper activate returned: {err}")

    if err != 0:
        raise RuntimeError(f"ActGripper failed with error code: {err}")

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
    activate=False,
):
    device_id = int(device_id)
    position = clamp_percent_int("Gripper position", position)
    speed = clamp_percent_int("Gripper speed", cfg.GRIPPER_DEFAULT_SPEED if speed is None else speed)
    torque = clamp_percent_int("Gripper torque", cfg.GRIPPER_DEFAULT_TORQUE if torque is None else torque)
    max_time_ms = int(cfg.GRIPPER_MAX_TIME_MS if max_time_ms is None else max_time_ms)
    block = int(block)
    gripper_type = int(gripper_type)

    if activate:
        activate_gripper(robot, device_id=device_id)

    print(
        f"Moving gripper: device={device_id}, position={position}, speed={speed}, "
        f"torque={torque}, max_time_ms={max_time_ms}, block={block}"
    )

    if cfg.DRY_RUN:
        print("[DRY RUN] Would move gripper.")
        return

    try:
        err = robot.MoveGripper(device_id, position, speed, torque, max_time_ms, block, gripper_type)
    except TypeError:
        err = robot.MoveGripper(device_id, position, speed, torque, max_time_ms, block)

    print(f"MoveGripper returned: {err}")

    if err != 0:
        raise RuntimeError(f"MoveGripper failed with error code: {err}")

    time.sleep(0.2)


def print_gripper_status(robot):
    print("Gripper status:")
    for name in [
        "GetGripperMotionDone",
        "GetGripperActivateStatus",
        "GetGripperCurPosition",
        "GetGripperCurSpeed",
        "GetGripperCurCurrent",
        "GetGripperVoltage",
        "GetGripperTemp",
    ]:
        try:
            fn = getattr(robot, name)
            result = fn()
            print(f"  {name}: {result}")
        except Exception as e:
            print(f"  {name}: failed/unsupported: {e}")
