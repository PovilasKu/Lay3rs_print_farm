"""Low-level robot movement commands."""

import time

import config as cfg
from core.helpers import validate_vector
from core.safety import check_safe_limits
from core.robot_client import (
    get_actual_pose,
    get_actual_joints,
    pose_close,
    joints_close,
    print_robot_debug,
    reset_robot_errors_quiet,
)


def move_cart(robot, pose, vel=None, acc=None, ovl=None):
    pose = validate_vector("Cartesian pose", pose[:6], 6)
    check_safe_limits(pose)

    vel = float(cfg.VEL if vel is None else vel)
    acc = float(cfg.ACC if acc is None else acc)
    ovl = float(cfg.OVL if ovl is None else ovl)

    if cfg.DRY_RUN:
        print(f"[DRY RUN] Would MoveCart to: {[round(v, 3) for v in pose]}  vel={vel}, acc={acc}, ovl={ovl}")
        return

    reset_robot_errors_quiet(robot)
    print(f"Sending MoveCart: {[round(v, 3) for v in pose]}  vel={vel}, acc={acc}, ovl={ovl}")

    err = robot.MoveCart(
        desc_pos=pose,
        tool=cfg.TOOL,
        user=cfg.USER,
        vel=vel,
        acc=acc,
        ovl=ovl,
        blendT=-1.0,
        config=-1,
    )

    time.sleep(0.5)

    try:
        actual_pose = get_actual_pose(robot)
        print(f"Actual pose after MoveCart: {[round(v, 3) for v in actual_pose]}")
        if pose_close(actual_pose, pose):
            if err != 0:
                print(f"WARNING: MoveCart returned {err}, but target pose was reached. Continuing.")
            else:
                print("MoveCart reached target.")
            return
    except Exception as e:
        print(f"Could not verify MoveCart target: {e}")

    if err != 0:
        print_robot_debug(robot)
        raise RuntimeError(f"MoveCart failed with error code: {err}")

    raise RuntimeError("MoveCart returned 0 but target pose was not reached.")


def move_line(robot, pose, vel=None, acc=None, ovl=None):
    pose = validate_vector("Line Cartesian pose", pose[:6], 6)
    check_safe_limits(pose)

    vel = float(cfg.VEL if vel is None else vel)
    acc = float(cfg.ACC if acc is None else acc)
    ovl = float(cfg.OVL if ovl is None else ovl)

    if cfg.DRY_RUN:
        print(f"[DRY RUN] Would MoveL to: {[round(v, 3) for v in pose]}  vel={vel}, acc={acc}, ovl={ovl}")
        return

    reset_robot_errors_quiet(robot)
    print(f"Sending MoveL: {[round(v, 3) for v in pose]}  vel={vel}, acc={acc}, ovl={ovl}")

    err = robot.MoveL(
        desc_pos=pose,
        tool=cfg.TOOL,
        user=cfg.USER,
        vel=vel,
        acc=acc,
        ovl=ovl,
        blendR=-1.0,
        search=0,
        offset_flag=0,
        offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )

    time.sleep(0.5)

    try:
        actual_pose = get_actual_pose(robot)
        print(f"Actual pose after MoveL: {[round(v, 3) for v in actual_pose]}")
        if pose_close(actual_pose, pose):
            if err != 0:
                print(f"WARNING: MoveL returned {err}, but target pose was reached. Continuing.")
            else:
                print("MoveL reached target.")
            return
    except Exception as e:
        print(f"Could not verify MoveL target: {e}")

    if err != 0:
        print_robot_debug(robot)
        raise RuntimeError(f"MoveL failed with error code: {err}")

    raise RuntimeError("MoveL returned 0 but target pose was not reached.")


def move_joint(robot, joints, pose=None, vel=None, acc=None, ovl=None):
    joints = validate_vector("Joint position", joints[:6], 6)

    desc_pos = None
    if pose is not None:
        desc_pos = validate_vector("MoveJ desc pose", pose[:6], 6)

    vel = float(cfg.VEL if vel is None else vel)
    acc = float(cfg.ACC if acc is None else acc)
    ovl = float(cfg.OVL if ovl is None else ovl)

    if cfg.DRY_RUN:
        print(f"[DRY RUN] Would MoveJ to joints: {[round(v, 3) for v in joints]}  vel={vel}, acc={acc}, ovl={ovl}")
        return

    reset_robot_errors_quiet(robot)
    print(f"Sending MoveJ: {[round(v, 3) for v in joints]}  vel={vel}, acc={acc}, ovl={ovl}")

    attempts = []
    if desc_pos is not None:
        attempts.append((
            "MoveJ with desc_pos keyword",
            lambda: robot.MoveJ(
                joint_pos=joints,
                tool=cfg.TOOL,
                user=cfg.USER,
                desc_pos=desc_pos,
                vel=vel,
                acc=acc,
                ovl=ovl,
                exaxis_pos=[0.0, 0.0, 0.0, 0.0],
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ),
        ))

    attempts.append((
        "MoveJ simple keyword form",
        lambda: robot.MoveJ(
            joint_pos=joints,
            tool=cfg.TOOL,
            user=cfg.USER,
            vel=vel,
            acc=acc,
            ovl=ovl,
        ),
    ))

    last_error = None

    for label, fn in attempts:
        print(f"Trying {label}...")
        try:
            err = fn()
        except Exception as e:
            print(f"{label} exception: {e}")
            last_error = e
            continue

        time.sleep(0.5)

        try:
            actual_joints = get_actual_joints(robot)
            print(f"Actual joints after MoveJ: {[round(v, 3) for v in actual_joints]}")
            if joints_close(actual_joints, joints):
                if err != 0:
                    print(f"WARNING: {label} returned {err}, but joint target was reached. Continuing.")
                    reset_robot_errors_quiet(robot)
                else:
                    print("MoveJ reached target.")
                return
        except Exception as e:
            print(f"Could not verify MoveJ target: {e}")

        last_error = RuntimeError(f"{label} returned error code: {err}")

    print_robot_debug(robot)
    raise RuntimeError(f"MoveJ failed. Last error: {last_error}")


def quadratic_bezier_pose(p0, p1, p2, t):
    a = (1.0 - t) * (1.0 - t)
    b = 2.0 * (1.0 - t) * t
    c = t * t
    return [a * p0[i] + b * p1[i] + c * p2[i] for i in range(6)]


def move_arc_fallback_segments(robot, via_pose, end_pose, vel=None, acc=None, ovl=None, segments=12):
    segments = max(3, int(segments))
    start_pose = get_actual_pose(robot)

    print(f"Using fallback arc approximation with {segments} MoveL segments.")

    for i in range(1, segments + 1):
        t = i / segments
        p = quadratic_bezier_pose(start_pose, via_pose, end_pose, t)
        print()
        print(f"Arc fallback segment {i}/{segments}")
        move_line(robot, p, vel=vel, acc=acc, ovl=ovl)


def move_arc(robot, via_pose, end_pose, vel=None, acc=None, ovl=None, fallback_segments=12):
    via_pose = validate_vector("Arc via pose", via_pose[:6], 6)
    end_pose = validate_vector("Arc end pose", end_pose[:6], 6)
    check_safe_limits(via_pose)
    check_safe_limits(end_pose)

    vel = float(cfg.VEL if vel is None else vel)
    acc = float(cfg.ACC if acc is None else acc)
    ovl = float(cfg.OVL if ovl is None else ovl)
    fallback_segments = int(fallback_segments)

    if cfg.DRY_RUN:
        print(f"[DRY RUN] Would arc via: {[round(v, 3) for v in via_pose]}")
        print(f"[DRY RUN] Would arc end: {[round(v, 3) for v in end_pose]}")
        return

    reset_robot_errors_quiet(robot)

    if not hasattr(robot, "MoveC"):
        print("This SDK does not expose MoveC. Falling back to segmented MoveL arc.")
        move_arc_fallback_segments(robot, via_pose, end_pose, vel=vel, acc=acc, ovl=ovl, segments=fallback_segments)
        return

    print("Sending MoveC arc:")
    print(f"  via: {[round(v, 3) for v in via_pose]}")
    print(f"  end: {[round(v, 3) for v in end_pose]}")

    try:
        err = robot.MoveC(
            desc_pos_p=via_pose,
            tool_p=cfg.TOOL,
            user_p=cfg.USER,
            desc_pos_t=end_pose,
            tool_t=cfg.TOOL,
            user_t=cfg.USER,
            vel_p=vel,
            acc_p=acc,
            ovl_p=ovl,
            vel_t=vel,
            acc_t=acc,
            ovl_t=ovl,
            blendR=-1.0,
            offset_flag=0,
            offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

        time.sleep(0.5)
        actual_pose = get_actual_pose(robot)
        print(f"Actual pose after MoveC: {[round(v, 3) for v in actual_pose]}")

        if pose_close(actual_pose, end_pose):
            if err != 0:
                print(f"WARNING: MoveC returned {err}, but end pose was reached. Continuing.")
            else:
                print("MoveC reached end pose.")
            return

        if err != 0:
            print(f"MoveC returned {err} and end pose was not reached.")
        else:
            print("MoveC returned 0 but end pose was not reached.")

    except Exception as e:
        print(f"MoveC call failed: {e}")

    print_robot_debug(robot)
    print("Falling back to segmented MoveL arc approximation.")
    move_arc_fallback_segments(robot, via_pose, end_pose, vel=vel, acc=acc, ovl=ovl, segments=fallback_segments)
