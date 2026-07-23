#!/usr/bin/env python3
"""
Interactive FAIRINO robot test utility.

Functions:
    1. Put the robot into manual mode.
    2. Move to absolute Cartesian or joint coordinates.
    3. Move the configured gripper to a requested position.
    4. Read the current TCP pose and joint coordinates.

This script is intended to be placed in the ROOT of fairino_farm_framework,
next to config.py and run_job.py.

Important:
    FAIRINO Mode(1) = manual mode.
    FAIRINO Mode(0) = automatic mode.

    Programmed MoveCart, MoveL and MoveJ commands are performed in automatic
    mode. This script asks for confirmation, temporarily switches to automatic
    mode, performs the move, and then returns the robot to manual mode.
"""

from __future__ import annotations

import sys
import time
from collections.abc import Callable

import config as cfg
from core.gripper import (
    activate_gripper,
    move_gripper,
    print_gripper_status,
)
from core.motion import move_cart, move_joint, move_line
from core.robot_client import (
    connect_robot,
    get_actual_joints,
    get_actual_pose,
)


# ---------------------------------------------------------------------------
# Test settings
# ---------------------------------------------------------------------------

# Deliberately low test speeds.
TEST_VEL = 10.0
TEST_ACC = 10.0
TEST_OVL = 10.0

# Return to controller manual mode after each programmed robot move.
RETURN_TO_MANUAL_AFTER_MOVE = True


def check_return_code(action: str, result) -> None:
    """Raise a clear exception for a non-zero FAIRINO return code."""
    try:
        code = int(result)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            f"{action} returned an unexpected result: {result!r}"
        ) from exc

    if code != 0:
        raise RuntimeError(f"{action} failed with FAIRINO error code {code}.")


def reset_and_enable(robot) -> None:
    """Clear robot errors and enable the robot."""
    print("Clearing robot errors...")
    check_return_code("ResetAllError", robot.ResetAllError())
    time.sleep(0.2)

    print("Enabling robot...")
    check_return_code("RobotEnable(1)", robot.RobotEnable(1))
    time.sleep(0.2)


def set_manual_mode(robot) -> None:
    """
    Put the controller into manual mode.

    This is NOT drag-teach mode. Manual mode allows manual/jog operation
    according to the controller's safety and key-switch state.
    """
    # Exit drag teaching before changing normal control mode.
    try:
        result = robot.DragTeachSwitch(0)
        if int(result) != 0:
            print(
                "Warning: DragTeachSwitch(0) returned "
                f"{result}; continuing with Mode(1)."
            )
    except Exception:
        # Some SDK/controller versions may not expose this call reliably.
        pass

    check_return_code("Mode(1)", robot.Mode(1))
    time.sleep(0.3)
    print("Robot is now in MANUAL mode.")


def set_automatic_mode(robot) -> None:
    """Put the controller into automatic mode for programmed movements."""
    try:
        result = robot.DragTeachSwitch(0)
        if int(result) != 0:
            print(
                "Warning: DragTeachSwitch(0) returned "
                f"{result}; continuing with Mode(0)."
            )
    except Exception:
        pass

    check_return_code("Mode(0)", robot.Mode(0))
    time.sleep(0.3)
    print("Robot is now in AUTOMATIC mode.")


def enable_drag_teach(robot) -> None:
    """Enable hand-guiding/drag-teach mode."""
    set_manual_mode(robot)
    check_return_code("DragTeachSwitch(1)", robot.DragTeachSwitch(1))
    time.sleep(0.3)
    print("Drag-teach mode enabled.")


def disable_drag_teach(robot) -> None:
    """Disable hand-guiding/drag-teach mode."""
    check_return_code("DragTeachSwitch(0)", robot.DragTeachSwitch(0))
    time.sleep(0.3)
    print("Drag-teach mode disabled.")


def parse_six_values(text: str, name: str) -> list[float]:
    """Parse six comma-separated or space-separated numbers."""
    cleaned = text.replace(",", " ")
    parts = [part for part in cleaned.split() if part]

    if len(parts) != 6:
        raise ValueError(
            f"{name} requires exactly 6 numbers; received {len(parts)}."
        )

    try:
        return [float(value) for value in parts]
    except ValueError as exc:
        raise ValueError(f"{name} contains a non-numeric value.") from exc


def read_six_values(prompt: str, name: str) -> list[float]:
    """Keep asking until six valid values are entered or the user cancels."""
    while True:
        value = input(prompt).strip()

        if value.lower() in {"q", "quit", "cancel"}:
            raise KeyboardInterrupt

        try:
            return parse_six_values(value, name)
        except ValueError as exc:
            print(f"Invalid input: {exc}")
            print("Enter six numbers separated by spaces or commas.")


def confirm(message: str) -> bool:
    """Return True only for an explicit yes."""
    answer = input(f"{message} [y/N]: ").strip().lower()
    return answer in {"y", "yes"}


def print_current_position(robot) -> None:
    """Read and print both absolute TCP and joint coordinates."""
    pose = get_actual_pose(robot)
    joints = get_actual_joints(robot)

    print()
    print("Current absolute TCP pose")
    print("-------------------------")
    print(
        "X={:.3f}, Y={:.3f}, Z={:.3f}, "
        "RX={:.3f}, RY={:.3f}, RZ={:.3f}".format(*pose)
    )

    print()
    print("Current joint coordinates")
    print("-------------------------")
    print(
        "J1={:.3f}, J2={:.3f}, J3={:.3f}, "
        "J4={:.3f}, J5={:.3f}, J6={:.3f}".format(*joints)
    )


def execute_programmed_move(
    robot,
    description: str,
    target: list[float],
    movement: Callable[[], None],
) -> None:
    """
    Confirm and execute a programmed move.

    The controller is switched to automatic mode for the movement and,
    by default, returned to manual mode afterward.
    """
    print()
    print(description)
    print(f"Target: {[round(value, 3) for value in target]}")
    print(
        f"Test speed: vel={TEST_VEL}, acc={TEST_ACC}, ovl={TEST_OVL}"
    )

    if not confirm("Execute this robot movement?"):
        print("Movement cancelled.")
        return

    try:
        reset_and_enable(robot)
        set_automatic_mode(robot)
        movement()
        print("Movement completed.")
    finally:
        if RETURN_TO_MANUAL_AFTER_MOVE:
            try:
                set_manual_mode(robot)
            except Exception as exc:
                print(
                    "WARNING: Could not return to manual mode after movement: "
                    f"{exc}"
                )


def move_absolute_cartesian(robot, linear: bool) -> None:
    """
    Move to an absolute Cartesian TCP pose.

    Input order:
        X Y Z RX RY RZ
    Units:
        millimetres and degrees
    """
    current = get_actual_pose(robot)
    print(f"Current TCP pose: {[round(v, 3) for v in current]}")

    target = read_six_values(
        "Enter X Y Z RX RY RZ (or 'q' to cancel): ",
        "Cartesian pose",
    )

    if linear:
        execute_programmed_move(
            robot,
            "Absolute Cartesian straight-line MoveL",
            target,
            lambda: move_line(
                robot,
                target,
                vel=TEST_VEL,
                acc=TEST_ACC,
                ovl=TEST_OVL,
            ),
        )
    else:
        execute_programmed_move(
            robot,
            "Absolute Cartesian point-to-point MoveCart",
            target,
            lambda: move_cart(
                robot,
                target,
                vel=TEST_VEL,
                acc=TEST_ACC,
                ovl=TEST_OVL,
            ),
        )


def move_absolute_joints(robot) -> None:
    """
    Move to absolute joint coordinates.

    Input order:
        J1 J2 J3 J4 J5 J6
    Units:
        degrees
    """
    current = get_actual_joints(robot)
    print(f"Current joints: {[round(v, 3) for v in current]}")

    target = read_six_values(
        "Enter J1 J2 J3 J4 J5 J6 in degrees (or 'q' to cancel): ",
        "Joint position",
    )

    execute_programmed_move(
        robot,
        "Absolute joint MoveJ",
        target,
        lambda: move_joint(
            robot,
            target,
            vel=TEST_VEL,
            acc=TEST_ACC,
            ovl=TEST_OVL,
        ),
    )


def set_gripper_position(robot, gripper_is_active: bool) -> bool:
    """
    Move the gripper to a position from 0 to 100.

    The direction represented by 0 and 100 depends on the configured gripper.
    On many parallel grippers, 0 is closed and 100 is open. Verify this using
    a safe test with no object between the fingers.
    """
    raw = input(
        "Enter gripper position 0-100 (or 'q' to cancel): "
    ).strip()

    if raw.lower() in {"q", "quit", "cancel"}:
        print("Gripper command cancelled.")
        return gripper_is_active

    try:
        position = int(round(float(raw)))
    except ValueError:
        print("Invalid gripper position.")
        return gripper_is_active

    if not 0 <= position <= 100:
        print("Gripper position must be between 0 and 100.")
        return gripper_is_active

    print(
        f"Device ID: {cfg.GRIPPER_ID}; requested position: {position}; "
        f"speed: {cfg.GRIPPER_DEFAULT_SPEED}; "
        f"torque: {cfg.GRIPPER_DEFAULT_TORQUE}"
    )

    if not confirm("Execute this gripper movement?"):
        print("Gripper command cancelled.")
        return gripper_is_active

    reset_and_enable(robot)

    if not gripper_is_active:
        activate_gripper(robot, device_id=cfg.GRIPPER_ID)
        gripper_is_active = True

    move_gripper(
        robot,
        position=position,
        speed=cfg.GRIPPER_DEFAULT_SPEED,
        torque=cfg.GRIPPER_DEFAULT_TORQUE,
        max_time_ms=cfg.GRIPPER_MAX_TIME_MS,
        device_id=cfg.GRIPPER_ID,
        block=cfg.GRIPPER_BLOCK,
        gripper_type=cfg.GRIPPER_TYPE,
        activate=False,
    )

    print("Gripper command completed.")
    return gripper_is_active


def print_menu() -> None:
    print()
    print("FAIRINO manual test utility")
    print("===========================")
    print("1. Set controller MANUAL mode")
    print("2. Set controller AUTOMATIC mode")
    print("3. Show current TCP pose and joints")
    print("4. Move to absolute Cartesian pose (MoveCart/PTP)")
    print("5. Move to absolute Cartesian pose (MoveL/straight)")
    print("6. Move to absolute joint coordinates (MoveJ)")
    print("7. Move gripper to position 0-100")
    print("8. Show gripper status")
    print("9. Enable drag-teach/hand-guiding")
    print("10. Disable drag-teach/hand-guiding")
    print("0. Exit")


def main() -> int:
    robot = None
    gripper_is_active = False

    try:
        print("Connecting to FAIRINO robot...")
        print(f"Robot IP: {cfg.ROBOT_IP}")
        robot = connect_robot()

        reset_and_enable(robot)
        set_manual_mode(robot)

        while True:
            print_menu()
            choice = input("Select an option: ").strip()

            try:
                if choice == "1":
                    reset_and_enable(robot)
                    set_manual_mode(robot)

                elif choice == "2":
                    reset_and_enable(robot)
                    set_automatic_mode(robot)

                elif choice == "3":
                    print_current_position(robot)

                elif choice == "4":
                    move_absolute_cartesian(robot, linear=False)

                elif choice == "5":
                    move_absolute_cartesian(robot, linear=True)

                elif choice == "6":
                    move_absolute_joints(robot)

                elif choice == "7":
                    gripper_is_active = set_gripper_position(
                        robot,
                        gripper_is_active,
                    )

                elif choice == "8":
                    print_gripper_status(robot)

                elif choice == "9":
                    reset_and_enable(robot)
                    enable_drag_teach(robot)

                elif choice == "10":
                    disable_drag_teach(robot)
                    set_manual_mode(robot)

                elif choice == "0":
                    print("Exiting.")
                    break

                else:
                    print("Unknown menu option.")

            except KeyboardInterrupt:
                print("\nOperation cancelled.")

            except Exception as exc:
                print(f"\nERROR: {exc}")
                print(
                    "The requested operation was not completed. "
                    "Check the robot WebApp, safety state, and surroundings."
                )

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130

    except Exception as exc:
        print(f"FATAL ERROR: {exc}", file=sys.stderr)
        return 1

    finally:
        if robot is not None:
            try:
                disable_drag_teach(robot)
            except Exception:
                pass

            try:
                set_manual_mode(robot)
            except Exception:
                pass

            close_rpc = getattr(robot, "CloseRPC", None)
            if callable(close_rpc):
                try:
                    close_rpc()
                except Exception:
                    pass


if __name__ == "__main__":
    raise SystemExit(main())
