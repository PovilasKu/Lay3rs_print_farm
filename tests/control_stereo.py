#!/usr/bin/env python3

"""
Fairino robot JSON job runner with precise stereo AprilTag localization.

This is a full replacement for the Control Pi runner. It keeps the earlier command
style but changes the localization logic:

- uses median-filtered batches of fresh stereo samples instead of one frame
- uses coarse and fine localization stages
- uses minimum useful correction steps so the robot does not command endless
  0.02 mm / 0.01 deg moves near the target
- controls explicit pitch/yaw/roll values from the updated stereo server
- keeps compatibility with older tag_tilt_x_deg/tag_tilt_y_deg fields

Run on the Control Pi:

    source ~/fairino-venv/bin/activate
    python ~/fairino_control_precise_stereo.py

Edit JOB_FILE below to choose the JSON file.
"""

import json
import time
import math
import statistics
import urllib.request

import Robot


# ============================================================
# JOB FILE SETTINGS
# ============================================================

JOB_FILE = "/home/fairino/test_sequence.json"
DRY_RUN = False


# ============================================================
# NETWORK SETTINGS
# ============================================================

ROBOT_IP = "192.168.58.2"
VISION_PI_IP = "10.255.2.97"
VISION_PORT = 5005
VISION_ENDPOINT = "/tag"
VISION_URL = f"http://{VISION_PI_IP}:{VISION_PORT}{VISION_ENDPOINT}"


# ============================================================
# CAMERA / DEFAULT TARGET SETTINGS
# ============================================================

WIDTH = 1280
HEIGHT = 800

TARGET_U = WIDTH / 2.0
TARGET_V = HEIGHT / 2.0
TARGET_DISTANCE_MM = 500.0

# Usually keep this False for stereo. The metric x_mm/y_mm/z_mm values are more
# important than pixel center once stereo calibration is working.
USE_PIXEL_FINISH_CHECK = False
PIXEL_TOL = 10.0


# ============================================================
# VISION SETTINGS
# ============================================================

# "prefer_stereo"  -> use stereo when available, but accept older mono-style JSON
# "require_stereo" -> refuse to localize unless stereo/fused is confirmed
# "allow_any"      -> accept any valid AprilTag pose JSON
VISION_SOURCE_POLICY = "require_stereo"
PRINT_VISION_SOURCE = True
MAX_VISION_AGE_S = 0.7

# Stable sampling. The localization stages can override these.
VISION_SAMPLE_DELAY_S = 0.05
VISION_MIN_VALID_RATIO = 0.55


# ============================================================
# ROBOT MOVEMENT SETTINGS
# ============================================================

TOOL = 1
USER = 0

# With vision_fixed.py, the Vision Pi already subtracts the measured X zero bias.
# Therefore a physically centered tag should report x_mm ~= 0.0.
# If you use the unmodified vision_new.py instead, set TARGET_X_MM back to
# 7.15591365291823 and do NOT also apply the vision-side zero calibration.
TARGET_X_MM = 0.0
TARGET_Y_MM = 0.0

VEL = 80.0
ACC = 60.0
OVL = 20.0

# Slower motion during vision correction gives better repeatability.
LOCALIZE_VEL = 20.0
LOCALIZE_ACC = 30.0
LOCALIZE_OVL = 20.0

MOVE_TO_START_POSE = False
TEST_START_POSE = [400.0, 0.0, 350.0, 0.0, 0.0, 0.0]

ROBOT_PREFLIGHT = True

POSE_XYZ_REACHED_TOL_MM = 2.0
POSE_ROT_REACHED_TOL_DEG = 2.0
JOINT_REACHED_TOL_DEG = 0.5

WAIT_AFTER_MOVE_S = 0.9


# ============================================================
# LOCALIZATION STAGES
# ============================================================

# The idea:
# - coarse stage: fast, large corrections, relaxed tolerance
# - fine stage: slower, smaller corrections, stricter tolerance, more samples
#
# Final tolerance should be larger than the actual stationary vision noise.
LOCALIZE_STAGES = [
    {
        "name": "coarse",
        "max_iters": 12,
        "samples": 5,
        "center_tol_mm": 4.0,
        "distance_tol_mm": 4.0,
        "rot_tol_deg": 4.0,
        "gain_xy": 0.75,
        "gain_z": 0.65,
        "gain_pitch": 0.55,
        "gain_yaw": 0.55,
        "gain_roll": 0.45,
        "max_xy_step_mm": 50.0,
        "max_z_step_mm": 50.0,
        "max_rot_step_deg": 20.0,
        "min_xy_step_mm": 2.0,
        "min_z_step_mm": 2.0,
        "min_rot_step_deg": 2.0,
    },
    {
        "name": "fine",
        "max_iters": 10,
        "samples": 9,
        "center_tol_mm": 0.5,
        "distance_tol_mm": 0.5,
        "rot_tol_deg": 1.0,
        "gain_xy": 0.45,
        "gain_z": 0.40,
        "gain_pitch": 0.35,
        "gain_yaw": 0.35,
        "gain_roll": 0.30,
        "max_xy_step_mm": 8.0,
        "max_z_step_mm": 12.0,
        "max_rot_step_deg": 1.0,
        "min_xy_step_mm": 0.2,
        "min_z_step_mm": 0.2,
        "min_rot_step_deg": 0.1,
    },
]


# ============================================================
# ORIENTATION TARGETS AND ZERO OFFSETS
# ============================================================

ENABLE_ROTATION_3AXIS = True

TARGET_PITCH_DEG = 0.0
TARGET_YAW_DEG = 0.0
TARGET_ROLL_DEG = 0.0

PITCH_ZERO_OFFSET_DEG = 0.0
YAW_ZERO_OFFSET_DEG = 0.0
ROLL_ZERO_OFFSET_DEG = 0.0

# Robot pose indices:
#   0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
PITCH_TO_ROBOT_ROT_AXIS = 3
YAW_TO_ROBOT_ROT_AXIS = 5
ROLL_TO_ROBOT_ROT_AXIS = 4

PITCH_SIGN = -1.0
ROLL_SIGN = +1.0
YAW_SIGN = +1.0

# Legacy aliases retained for old JSON commands.
TARGET_TILT_X_DEG = TARGET_PITCH_DEG
TARGET_TILT_Y_DEG = TARGET_YAW_DEG
ROT_TOL_DEG = 1.0


# ============================================================
# CAMERA-TO-ROBOT POSITION AXIS MAPPING
# ============================================================

# Existing mapping from your previous setup.
CAMERA_X_TO_ROBOT_AXIS = 1
CAMERA_Y_TO_ROBOT_AXIS = 2
CAMERA_Z_TO_ROBOT_AXIS = 0

CAMERA_X_SIGN = +1.0
CAMERA_Y_SIGN = -1.0
CAMERA_Z_SIGN = -1.0

# local_cart is TCP-local by default, like the old working script.
# Signs are kept as optional per-command escape hatches, but by default Y is NOT inverted.
# Example override only if needed: {"local_cart_y_sign": -1.0}
LOCAL_CART_X_SIGN = +1.0
LOCAL_CART_Y_SIGN = +1.0
LOCAL_CART_Z_SIGN = +1.0


# Optional safety limits for robot XYZ/RX/RY/RZ.
SAFE_LIMITS = None


# ============================================================
# GRIPPER SETTINGS
# ============================================================

GRIPPER_ID = 2
GRIPPER_TYPE = 0
GRIPPER_BLOCK = 0
GRIPPER_MAX_TIME_MS = 2000
GRIPPER_DEFAULT_SPEED = 50
GRIPPER_DEFAULT_TORQUE = 100


# ============================================================
# BASIC HELPERS
# ============================================================


def clamp(value, low, high):
    return max(low, min(high, value))


def is_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def validate_vector(name, values, length):
    if not isinstance(values, list):
        raise RuntimeError(f"{name} must be a list, got: {type(values)}")

    if len(values) != length:
        raise RuntimeError(
            f"{name} must contain exactly {length} values, got {len(values)}: {values}"
        )

    if not all(is_number(v) for v in values):
        raise RuntimeError(f"{name} contains non-numeric values: {values}")

    return [float(v) for v in values]


def check_safe_limits(pose):
    if SAFE_LIMITS is None:
        return

    for axis, limits in SAFE_LIMITS.items():
        low, high = limits
        value = pose[axis]

        if value < low or value > high:
            raise RuntimeError(
                f"Refusing move. Axis {axis} value {value:.2f} is outside safe range {limits}"
            )


def add_axis_step(pose, axis, step):
    new_pose = list(pose)
    new_pose[int(axis)] += float(step)
    return new_pose


def add_vectors(a, b):
    if len(a) != len(b):
        raise RuntimeError(
            f"Cannot add vectors of different lengths: {len(a)} and {len(b)}"
        )
    return [float(x) + float(y) for x, y in zip(a, b)]


def load_job_file(filename):
    with open(filename, "r", encoding="utf-8") as f:
        job = json.load(f)

    if not isinstance(job, list):
        raise RuntimeError("The JSON job file must contain a list of commands.")

    return job


def get_number(params, key, default):
    return float(params.get(key, default))


def truthy(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in [
            "1",
            "true",
            "yes",
            "y",
            "ok",
            "stereo",
            "fused",
        ]
    return False


def wrap_angle_deg(angle):
    """Wrap angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def angle_error_deg(measured, target):
    return wrap_angle_deg(float(measured) - float(target))


def correction_step(error, gain, max_step, min_step, tolerance):
    """
    Proportional correction with a minimum useful step.

    This prevents endless tiny commands near the target. If the error is already
    within tolerance, the step is exactly zero. Otherwise the step is clamped by
    max_step and forced to at least min_step.
    """
    error = float(error)
    gain = float(gain)
    max_step = abs(float(max_step))
    min_step = abs(float(min_step))
    tolerance = abs(float(tolerance))

    if abs(error) <= tolerance:
        return 0.0

    step = clamp(gain * error, -max_step, max_step)

    if abs(step) < min_step:
        step = min_step if error > 0 else -min_step

    return step


def merged_stage_params(stage, command):
    """Allow JSON command values to override a stage's tuning values."""
    out = dict(stage)

    # Stage-specific overrides may be in command["coarse"] or command["fine"].
    nested = command.get(stage["name"])
    if isinstance(nested, dict):
        out.update(nested)

    # Global localization overrides.
    for key in [
        "center_tol_mm",
        "distance_tol_mm",
        "rot_tol_deg",
        "samples",
        "max_iters",
        "gain_xy",
        "gain_z",
        "gain_pitch",
        "gain_yaw",
        "gain_roll",
        "max_xy_step_mm",
        "max_z_step_mm",
        "max_rot_step_deg",
        "min_xy_step_mm",
        "min_z_step_mm",
        "min_rot_step_deg",
    ]:
        if key in command:
            out[key] = command[key]

    return out


# ============================================================
# TCP-LOCAL COORDINATE HELPERS
# ============================================================


def rpy_deg_to_matrix(rx_deg, ry_deg, rz_deg):
    """
    Convert RX/RY/RZ degrees to a rotation matrix.

    Convention used here:
        R = Rz * Ry * Rx

    Intended for TCP-local XYZ offsets. Test with small 5 mm moves first.
    """
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    cx = math.cos(rx)
    sx = math.sin(rx)
    cy = math.cos(ry)
    sy = math.sin(ry)
    cz = math.cos(rz)
    sz = math.sin(rz)

    return [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]


def tcp_delta_to_global_delta(current_pose, tcp_delta):
    dx_tcp, dy_tcp, dz_tcp, drx, dry, drz = tcp_delta
    rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]

    R = rpy_deg_to_matrix(rx, ry, rz)

    dx_global = R[0][0] * dx_tcp + R[0][1] * dy_tcp + R[0][2] * dz_tcp
    dy_global = R[1][0] * dx_tcp + R[1][1] * dy_tcp + R[1][2] * dz_tcp
    dz_global = R[2][0] * dx_tcp + R[2][1] * dy_tcp + R[2][2] * dz_tcp

    return [dx_global, dy_global, dz_global, drx, dry, drz]


def tcp_delta_to_pose(current_pose, tcp_delta):
    global_delta = tcp_delta_to_global_delta(current_pose, tcp_delta)
    return add_vectors(current_pose, global_delta)


# ============================================================
# ROBOT FUNCTIONS
# ============================================================


def connect_robot():
    robot = Robot.RPC(ROBOT_IP)
    print(robot)
    print(f"Connected to robot using Fairino SDK: {ROBOT_IP}")
    return robot


def try_robot_call(robot, name, *args):
    try:
        fn = getattr(robot, name)
    except AttributeError:
        print(f"{name} not available in this SDK")
        return None

    try:
        result = fn(*args)
        print(f"{name}{args} returned: {result}")
        return result
    except Exception as e:
        print(f"{name}{args} failed/ignored: {e}")
        return None


def robot_preflight(robot):
    print()
    print("Robot preflight")
    print("---------------")

    try_robot_call(robot, "ResetAllError")
    time.sleep(0.2)
    try_robot_call(robot, "RobotEnable", 1)
    time.sleep(0.2)
    try_robot_call(robot, "Mode", 0)
    time.sleep(0.2)
    try_robot_call(robot, "GetRobotErrorCode")

    print("---------------")
    print()


def get_actual_pose(robot):
    err, pose = robot.GetActualTCPPose()

    if err != 0:
        raise RuntimeError(f"GetActualTCPPose failed with error code: {err}")

    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"Unexpected pose format: {pose}")

    return [float(v) for v in pose[:6]]


def get_actual_joints(robot):
    err, joints = robot.GetActualJointPosDegree(0)

    if err != 0:
        raise RuntimeError(f"GetActualJointPosDegree failed with error code: {err}")

    if not isinstance(joints, list) or len(joints) < 6:
        raise RuntimeError(f"Unexpected joint format: {joints}")

    return [float(v) for v in joints[:6]]


def pose_close(
    actual, target, xyz_tol=POSE_XYZ_REACHED_TOL_MM, rot_tol=POSE_ROT_REACHED_TOL_DEG
):
    xyz_errors = [abs(float(actual[i]) - float(target[i])) for i in range(3)]
    rot_errors = [
        abs(wrap_angle_deg(float(actual[i + 3]) - float(target[i + 3])))
        for i in range(3)
    ]

    print(f"Pose XYZ errors: {[round(e, 3) for e in xyz_errors]} mm")
    print(f"Pose rot errors: {[round(e, 3) for e in rot_errors]} deg")

    return max(xyz_errors) <= xyz_tol and max(rot_errors) <= rot_tol


def joints_close(actual, target, tol_deg=JOINT_REACHED_TOL_DEG):
    errors = [abs(float(actual[i]) - float(target[i])) for i in range(6)]
    max_error = max(errors)

    print(f"Joint errors: {[round(e, 4) for e in errors]} deg")
    print(f"Max joint error: {max_error:.4f} deg")

    return max_error <= tol_deg


def print_robot_debug(robot):
    try:
        err2, joints = robot.GetActualJointPosDegree(0)
        print(f"Current joint query return: {err2}")
        print(f"Current joints: {joints}")
    except Exception as e:
        print(f"Could not query current joints: {e}")

    try:
        err3, robot_codes = robot.GetRobotErrorCode()
        print(f"Robot error query return: {err3}")
        print(f"Robot main/sub error code: {robot_codes}")
    except Exception as e:
        print(f"Could not query robot error code: {e}")


def reset_robot_errors_quiet(robot):
    try:
        robot.ResetAllError()
        time.sleep(0.15)
    except Exception:
        pass


# ============================================================
# MOTION FUNCTIONS
# ============================================================


def move_cart(robot, pose, vel=None, acc=None, ovl=None):
    pose = validate_vector("Cartesian pose", pose[:6], 6)
    check_safe_limits(pose)

    vel = float(VEL if vel is None else vel)
    acc = float(ACC if acc is None else acc)
    ovl = float(OVL if ovl is None else ovl)

    if DRY_RUN:
        print(f"[DRY RUN] Would MoveCart to: {[round(v, 3) for v in pose]}")
        return

    reset_robot_errors_quiet(robot)
    print(
        f"Sending MoveCart: {[round(v, 3) for v in pose]}  vel={vel}, acc={acc}, ovl={ovl}"
    )

    err = robot.MoveCart(
        desc_pos=pose,
        tool=TOOL,
        user=USER,
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
                print(
                    f"WARNING: MoveCart returned {err}, but target pose was reached. Continuing."
                )
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

    vel = float(VEL if vel is None else vel)
    acc = float(ACC if acc is None else acc)
    ovl = float(OVL if ovl is None else ovl)

    if DRY_RUN:
        print(f"[DRY RUN] Would MoveL straight line to: {[round(v, 3) for v in pose]}")
        return

    reset_robot_errors_quiet(robot)
    print(
        f"Sending MoveL straight line: {[round(v, 3) for v in pose]}  vel={vel}, acc={acc}, ovl={ovl}"
    )

    err = robot.MoveL(
        desc_pos=pose,
        tool=TOOL,
        user=USER,
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
                print(
                    f"WARNING: MoveL returned {err}, but target pose was reached. Continuing."
                )
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

    vel = float(VEL if vel is None else vel)
    acc = float(ACC if acc is None else acc)
    ovl = float(OVL if ovl is None else ovl)

    if DRY_RUN:
        print(f"[DRY RUN] Would MoveJ to joints: {[round(v, 3) for v in joints]}")
        return

    reset_robot_errors_quiet(robot)
    print(
        f"Sending MoveJ: {[round(v, 3) for v in joints]}  vel={vel}, acc={acc}, ovl={ovl}"
    )

    attempts = []

    if desc_pos is not None:
        attempts.append(
            (
                "MoveJ with desc_pos keyword",
                lambda: robot.MoveJ(
                    joint_pos=joints,
                    tool=TOOL,
                    user=USER,
                    desc_pos=desc_pos,
                    vel=vel,
                    acc=acc,
                    ovl=ovl,
                    exaxis_pos=[0.0, 0.0, 0.0, 0.0],
                    blendT=-1.0,
                    offset_flag=0,
                    offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ),
            )
        )

    attempts.append(
        (
            "MoveJ simple keyword form",
            lambda: robot.MoveJ(
                joint_pos=joints,
                tool=TOOL,
                user=USER,
                vel=vel,
                acc=acc,
                ovl=ovl,
            ),
        )
    )

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
                    print(
                        f"WARNING: {label} returned {err}, but joint target was reached. Continuing."
                    )
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


def move_arc_fallback_segments(
    robot, via_pose, end_pose, vel=None, acc=None, ovl=None, segments=12
):
    segments = max(3, int(segments))
    start_pose = get_actual_pose(robot)

    print(f"Using fallback arc approximation with {segments} MoveL segments.")

    for i in range(1, segments + 1):
        t = i / segments
        p = quadratic_bezier_pose(start_pose, via_pose, end_pose, t)
        print()
        print(f"Arc fallback segment {i}/{segments}")
        move_line(robot, p, vel=vel, acc=acc, ovl=ovl)


def move_arc(
    robot, via_pose, end_pose, vel=None, acc=None, ovl=None, fallback_segments=12
):
    via_pose = validate_vector("Arc via pose", via_pose[:6], 6)
    end_pose = validate_vector("Arc end pose", end_pose[:6], 6)
    check_safe_limits(via_pose)
    check_safe_limits(end_pose)

    vel = float(VEL if vel is None else vel)
    acc = float(ACC if acc is None else acc)
    ovl = float(OVL if ovl is None else ovl)
    fallback_segments = int(fallback_segments)

    if DRY_RUN:
        print(f"[DRY RUN] Would arc via: {[round(v, 3) for v in via_pose]}")
        print(f"[DRY RUN] Would arc end: {[round(v, 3) for v in end_pose]}")
        return

    reset_robot_errors_quiet(robot)

    if not hasattr(robot, "MoveC"):
        print("This SDK does not expose MoveC. Falling back to segmented MoveL arc.")
        move_arc_fallback_segments(
            robot,
            via_pose,
            end_pose,
            vel=vel,
            acc=acc,
            ovl=ovl,
            segments=fallback_segments,
        )
        return

    print("Sending MoveC arc:")
    print(f"  via: {[round(v, 3) for v in via_pose]}")
    print(f"  end: {[round(v, 3) for v in end_pose]}")

    try:
        err = robot.MoveC(
            desc_pos_p=via_pose,
            tool_p=TOOL,
            user_p=USER,
            desc_pos_t=end_pose,
            tool_t=TOOL,
            user_t=USER,
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
                print(
                    f"WARNING: MoveC returned {err}, but end pose was reached. Continuing."
                )
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
    move_arc_fallback_segments(
        robot, via_pose, end_pose, vel=vel, acc=acc, ovl=ovl, segments=fallback_segments
    )


# ============================================================
# VISION FUNCTIONS
# ============================================================


def read_tag_once():
    with urllib.request.urlopen(VISION_URL, timeout=1.0) as response:
        return json.loads(response.read().decode("utf-8"))


def first_present(maps, keys):
    for data in maps:
        if not isinstance(data, dict):
            continue
        for key in keys:
            if key in data and data[key] is not None:
                return data[key]
    return None


def collect_vision_maps(raw):
    maps = []

    for key in ["stereo", "fused", "pose", "tag", "result"]:
        value = raw.get(key)
        if isinstance(value, dict):
            maps.append(value)

    maps.append(raw)
    return maps


def normalize_vision_tag(raw):
    """
    Normalize both the new stereo JSON and older mono-style JSON into the keys
    used by localization.
    """
    if not isinstance(raw, dict):
        raise RuntimeError(f"Vision Pi returned non-dict JSON: {raw}")

    maps = collect_vision_maps(raw)
    tag = dict(raw)

    # Prefer nested stereo/fused pose values, but keep top-level metadata too.
    for m in reversed(maps):
        if isinstance(m, dict):
            tag.update(m)

    ok_value = first_present(maps, ["ok", "detected", "valid"])
    if ok_value is not None:
        tag["ok"] = truthy(ok_value)

    aliases = {
        "tag_id": ["tag_id", "id", "tagId"],
        "x_mm": ["x_mm", "x", "X_mm", "camera_x_mm"],
        "y_mm": ["y_mm", "y", "Y_mm", "camera_y_mm"],
        "z_mm": ["z_mm", "z", "Z_mm", "depth_mm", "camera_z_mm"],
        "distance_mm": ["distance_mm", "distance", "range_mm", "depth_mm", "z_mm"],
        "cx": ["cx", "center_x", "u", "pixel_x"],
        "cy": ["cy", "center_y", "v", "pixel_y"],
        "age_s": ["age_s", "age", "data_age_s"],
        "timestamp": ["timestamp", "time", "t"],
        # New explicit fields.
        "tag_pitch_deg": [
            "tag_pitch_deg",
            "pitch_deg",
            "pitch",
            "tag_tilt_x_deg",
            "tilt_x_deg",
            "tilt_x",
        ],
        "tag_yaw_deg": [
            "tag_yaw_deg",
            "yaw_deg",
            "yaw",
            "tag_tilt_y_deg",
            "tilt_y_deg",
            "tilt_y",
        ],
        "tag_roll_deg": ["tag_roll_deg", "roll_deg", "roll"],
        # Legacy fields retained for old checks/prints.
        "tag_tilt_x_deg": [
            "tag_tilt_x_deg",
            "tilt_x_deg",
            "tilt_x",
            "tag_pitch_deg",
            "pitch_deg",
        ],
        "tag_tilt_y_deg": [
            "tag_tilt_y_deg",
            "tilt_y_deg",
            "tilt_y",
            "tag_yaw_deg",
            "yaw_deg",
        ],
        "source": ["source", "pose_source", "mode", "vision_mode"],
        "left_ok": ["left_ok", "left_detected"],
        "right_ok": ["right_ok", "right_detected"],
        "stereo_ok": ["stereo_ok", "fused_ok"],
        "disparity_px": ["disparity_px", "mean_disparity_px", "disparity"],
        "rectified_y_error_px": ["rectified_y_error_px", "mean_rectified_y_error_px"],
        "tag_size_error_mm": ["tag_size_error_mm"],
        "left_margin": ["left_margin"],
        "right_margin": ["right_margin"],
        "frame_id": ["frame_id"],
    }

    for normalized_key, possible_keys in aliases.items():
        value = first_present(maps, possible_keys)
        if value is not None:
            tag[normalized_key] = value

    if "age_s" not in tag and "timestamp" in tag:
        try:
            tag["age_s"] = max(0.0, time.time() - float(tag["timestamp"]))
        except Exception:
            pass

    if "distance_mm" not in tag and "z_mm" in tag:
        tag["distance_mm"] = tag["z_mm"]

    return tag


def is_stereo_confirmed(tag):
    source = str(tag.get("source", "")).strip().lower()

    if "stereo" in source or "fused" in source:
        return True
    if truthy(tag.get("stereo_ok")):
        return True
    if truthy(tag.get("left_ok")) and truthy(tag.get("right_ok")):
        return True
    if tag.get("disparity_px") is not None:
        return True

    return False


def vision_source_policy_from_params(params):
    if params is None:
        params = {}

    if bool(params.get("require_stereo", False)):
        return "require_stereo"

    return str(params.get("vision_source_policy", VISION_SOURCE_POLICY)).strip().lower()


def describe_vision_source(tag):
    source = str(tag.get("source", "unknown"))
    parts = [f"source={source}"]

    for key in ["left_ok", "right_ok", "stereo_ok"]:
        if key in tag:
            parts.append(f"{key}={truthy(tag.get(key))}")

    if "disparity_px" in tag:
        try:
            parts.append(f"disparity={float(tag['disparity_px']):.1f}px")
        except Exception:
            parts.append(f"disparity={tag['disparity_px']}")

    if "rectified_y_error_px" in tag:
        try:
            parts.append(f"rect_y={float(tag['rectified_y_error_px']):.2f}px")
        except Exception:
            pass

    return ", ".join(parts)


def validate_tag_for_localize(
    tag, required_tag_id=None, enable_rotation=True, params=None, quiet=False
):
    if not tag.get("ok"):
        if not quiet:
            print("Waiting for AprilTag detection...")
        return False

    if required_tag_id is not None:
        seen_tag_id = tag.get("tag_id")
        if seen_tag_id is None or int(seen_tag_id) != int(required_tag_id):
            if not quiet:
                print(
                    f"Waiting for tag_id={required_tag_id}; currently seeing tag_id={seen_tag_id}..."
                )
            return False

    age_s = float(tag.get("age_s", 999.0))
    if age_s > MAX_VISION_AGE_S:
        if not quiet:
            print(f"Waiting for fresh tag data, age={age_s:.2f}s...")
        return False

    source_policy = vision_source_policy_from_params(params)
    stereo_confirmed = is_stereo_confirmed(tag)

    if (
        source_policy in ["require_stereo", "stereo", "stereo_only"]
        and not stereo_confirmed
    ):
        if not quiet:
            print("Waiting for confirmed stereo/fused AprilTag pose...")
            print(f"Vision status: {describe_vision_source(tag)}")
        return False

    if (
        source_policy == "prefer_stereo"
        and not stereo_confirmed
        and PRINT_VISION_SOURCE
        and not quiet
    ):
        print(
            "WARNING: using AprilTag pose, but stereo/fused source was not confirmed."
        )
        print(f"Vision status: {describe_vision_source(tag)}")

    needed = ["x_mm", "y_mm", "z_mm", "distance_mm", "cx", "cy"]
    if enable_rotation:
        needed += ["tag_pitch_deg", "tag_yaw_deg", "tag_roll_deg"]

    missing = [k for k in needed if k not in tag]
    if missing:
        if not quiet:
            print(
                "Tag detected, but some required values are missing from Vision Pi output."
            )
            print(f"Missing keys: {missing}")
        return False

    return True


def wait_for_fresh_tag(
    required_tag_id=None, enable_rotation=True, params=None, quiet=False
):
    for _ in range(25):
        try:
            raw_tag = read_tag_once()
            tag = normalize_vision_tag(raw_tag)

            if validate_tag_for_localize(
                tag, required_tag_id, enable_rotation, params, quiet=quiet
            ):
                if PRINT_VISION_SOURCE and not quiet:
                    print(f"Vision status: {describe_vision_source(tag)}")
                return tag

        except Exception as e:
            if not quiet:
                print(f"Vision read failed: {e}")

        time.sleep(0.2)

    raise RuntimeError("No fresh AprilTag pose received from Vision Pi.")


def median_or_none(values):
    if not values:
        return None
    return float(statistics.median(values))


def spread_or_none(values):
    if not values:
        return None
    return float(max(values) - min(values))


def collect_stable_tag(
    required_tag_id=None,
    enable_rotation=True,
    params=None,
    samples=7,
    delay_s=VISION_SAMPLE_DELAY_S,
):
    """
    Collect several fresh valid vision samples and return a median-filtered tag.
    """
    samples = max(1, int(samples))
    min_valid = (
        max(3, int(math.ceil(samples * VISION_MIN_VALID_RATIO))) if samples >= 3 else 1
    )

    valid_tags = []
    last_error = None

    for _ in range(samples):
        try:
            tag = wait_for_fresh_tag(
                required_tag_id=required_tag_id,
                enable_rotation=enable_rotation,
                params=params,
                quiet=True,
            )
            valid_tags.append(tag)
        except Exception as e:
            last_error = e

        time.sleep(float(delay_s))

    if len(valid_tags) < min_valid:
        raise RuntimeError(
            f"Could not collect enough valid vision samples: {len(valid_tags)}/{samples}. Last error: {last_error}"
        )

    numeric_keys = [
        "x_mm",
        "y_mm",
        "z_mm",
        "distance_mm",
        "cx",
        "cy",
        "tag_pitch_deg",
        "tag_yaw_deg",
        "tag_roll_deg",
        "tag_tilt_x_deg",
        "tag_tilt_y_deg",
        "disparity_px",
        "rectified_y_error_px",
        "tag_size_error_mm",
        "left_margin",
        "right_margin",
        "age_s",
    ]

    out = dict(valid_tags[-1])
    out["sample_count"] = len(valid_tags)
    out["requested_sample_count"] = samples

    for key in numeric_keys:
        values = []
        for tag in valid_tags:
            if key in tag and tag[key] is not None:
                try:
                    values.append(float(tag[key]))
                except Exception:
                    pass

        if values:
            out[key] = median_or_none(values)
            out[key + "_spread"] = spread_or_none(values)

    # Use newest timestamp/frame id for freshness, but median values for pose.
    newest = max(valid_tags, key=lambda t: float(t.get("timestamp", 0.0)))
    if "timestamp" in newest:
        out["timestamp"] = newest["timestamp"]
        out["age_s"] = max(0.0, time.time() - float(newest["timestamp"]))
    if "frame_id" in newest:
        out["frame_id"] = newest["frame_id"]

    return out


# ============================================================
# GRIPPER FUNCTIONS
# ============================================================


def clamp_percent_int(name, value):
    value = int(round(float(value)))
    if value < 0 or value > 100:
        raise RuntimeError(f"{name} must be from 0 to 100, got: {value}")
    return value


def activate_gripper(robot, device_id=GRIPPER_ID):
    device_id = int(device_id)
    print(f"Activating gripper device_id={device_id}...")

    if DRY_RUN:
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
    device_id=GRIPPER_ID,
    block=GRIPPER_BLOCK,
    gripper_type=GRIPPER_TYPE,
    activate=False,
):
    device_id = int(device_id)
    position = clamp_percent_int("Gripper position", position)
    speed = clamp_percent_int(
        "Gripper speed", GRIPPER_DEFAULT_SPEED if speed is None else speed
    )
    torque = clamp_percent_int(
        "Gripper torque", GRIPPER_DEFAULT_TORQUE if torque is None else torque
    )
    max_time_ms = int(GRIPPER_MAX_TIME_MS if max_time_ms is None else max_time_ms)
    block = int(block)
    gripper_type = int(gripper_type)

    if activate:
        activate_gripper(robot, device_id=device_id)

    print(
        f"Moving gripper: device={device_id}, position={position}, speed={speed}, "
        f"torque={torque}, max_time_ms={max_time_ms}, block={block}"
    )

    if DRY_RUN:
        print("[DRY RUN] Would move gripper.")
        return

    try:
        err = robot.MoveGripper(
            device_id, position, speed, torque, max_time_ms, block, gripper_type
        )
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


# ============================================================
# LOCALIZATION LOGIC
# ============================================================


def get_targets(params):
    target_x_mm = get_number(params, "target_x_mm", TARGET_X_MM)
    target_y_mm = get_number(params, "target_y_mm", TARGET_Y_MM)
    target_distance_mm = get_number(params, "target_distance_mm", TARGET_DISTANCE_MM)

    target_pitch_deg = get_number(
        params,
        "target_pitch_deg",
        params.get("target_tilt_x_deg", TARGET_PITCH_DEG),
    )
    target_yaw_deg = get_number(
        params,
        "target_yaw_deg",
        params.get("target_tilt_y_deg", TARGET_YAW_DEG),
    )
    target_roll_deg = get_number(params, "target_roll_deg", TARGET_ROLL_DEG)

    pitch_zero = get_number(params, "pitch_zero_offset_deg", PITCH_ZERO_OFFSET_DEG)
    yaw_zero = get_number(params, "yaw_zero_offset_deg", YAW_ZERO_OFFSET_DEG)
    roll_zero = get_number(params, "roll_zero_offset_deg", ROLL_ZERO_OFFSET_DEG)

    return {
        "target_x_mm": target_x_mm,
        "target_y_mm": target_y_mm,
        "target_distance_mm": target_distance_mm,
        "target_pitch_deg": target_pitch_deg,
        "target_yaw_deg": target_yaw_deg,
        "target_roll_deg": target_roll_deg,
        "pitch_zero_offset_deg": pitch_zero,
        "yaw_zero_offset_deg": yaw_zero,
        "roll_zero_offset_deg": roll_zero,
    }


def get_corrected_angles(tag, params):
    targets = get_targets(params)

    pitch = float(tag["tag_pitch_deg"]) - targets["pitch_zero_offset_deg"]
    yaw = float(tag["tag_yaw_deg"]) - targets["yaw_zero_offset_deg"]
    roll = float(tag["tag_roll_deg"]) - targets["roll_zero_offset_deg"]

    return wrap_angle_deg(pitch), wrap_angle_deg(yaw), wrap_angle_deg(roll)


def build_corrected_pose(current_pose, tag, params, stage):
    """
    Converts camera-relative AprilTag position + orientation error into robot pose correction.

    Important:
        x_mm/y_mm from the Vision Pi are the tag position relative to the stereo
        camera frame. TARGET_X_MM/TARGET_Y_MM define where the tag should appear
        in that camera frame when the real robot TCP/tool is physically centered.
    """
    targets = get_targets(params)
    target_x_mm = targets["target_x_mm"]
    target_y_mm = targets["target_y_mm"]
    target_distance_mm = targets["target_distance_mm"]
    enable_rotation = bool(params.get("enable_rotation", ENABLE_ROTATION_3AXIS))

    center_tol_mm = float(stage["center_tol_mm"])
    distance_tol_mm = float(stage["distance_tol_mm"])
    rot_tol_deg = float(stage["rot_tol_deg"])

    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    x_error_mm = tag_x_mm - target_x_mm
    y_error_mm = tag_y_mm - target_y_mm
    distance_error_mm = tag_z_mm - target_distance_mm

    cam_x_step = correction_step(
        x_error_mm,
        gain=stage["gain_xy"],
        max_step=stage["max_xy_step_mm"],
        min_step=stage["min_xy_step_mm"],
        tolerance=center_tol_mm,
    )
    cam_y_step = correction_step(
        y_error_mm,
        gain=stage["gain_xy"],
        max_step=stage["max_xy_step_mm"],
        min_step=stage["min_xy_step_mm"],
        tolerance=center_tol_mm,
    )
    cam_z_step = correction_step(
        distance_error_mm,
        gain=stage["gain_z"],
        max_step=stage["max_z_step_mm"],
        min_step=stage["min_z_step_mm"],
        tolerance=distance_tol_mm,
    )

    robot_x_step = CAMERA_X_SIGN * cam_x_step
    robot_y_step = CAMERA_Y_SIGN * cam_y_step
    robot_z_step = CAMERA_Z_SIGN * cam_z_step

    next_pose = list(current_pose)
    next_pose = add_axis_step(next_pose, CAMERA_X_TO_ROBOT_AXIS, robot_x_step)
    next_pose = add_axis_step(next_pose, CAMERA_Y_TO_ROBOT_AXIS, robot_y_step)
    next_pose = add_axis_step(next_pose, CAMERA_Z_TO_ROBOT_AXIS, robot_z_step)

    info = {
        "target_x_mm": target_x_mm,
        "target_y_mm": target_y_mm,
        "target_distance_mm": target_distance_mm,
        "tag_x_mm": tag_x_mm,
        "tag_y_mm": tag_y_mm,
        "tag_z_mm": tag_z_mm,
        "x_error_mm": x_error_mm,
        "y_error_mm": y_error_mm,
        "distance_error_mm": distance_error_mm,
        "cam_x_step": cam_x_step,
        "cam_y_step": cam_y_step,
        "cam_z_step": cam_z_step,
        "robot_x_step": robot_x_step,
        "robot_y_step": robot_y_step,
        "robot_z_step": robot_z_step,
        "tag_pitch_deg": 0.0,
        "tag_yaw_deg": 0.0,
        "tag_roll_deg": 0.0,
        "pitch_error_deg": 0.0,
        "yaw_error_deg": 0.0,
        "roll_error_deg": 0.0,
        "robot_pitch_step": 0.0,
        "robot_yaw_step": 0.0,
        "robot_roll_step": 0.0,
    }

    if enable_rotation:
        pitch_deg, yaw_deg, roll_deg = get_corrected_angles(tag, params)

        pitch_error_deg = angle_error_deg(pitch_deg, targets["target_pitch_deg"])
        yaw_error_deg = angle_error_deg(yaw_deg, targets["target_yaw_deg"])
        roll_error_deg = angle_error_deg(roll_deg, targets["target_roll_deg"])

        pitch_step = correction_step(
            pitch_error_deg,
            gain=stage["gain_pitch"],
            max_step=stage["max_rot_step_deg"],
            min_step=stage["min_rot_step_deg"],
            tolerance=rot_tol_deg,
        )
        yaw_step = correction_step(
            yaw_error_deg,
            gain=stage["gain_yaw"],
            max_step=stage["max_rot_step_deg"],
            min_step=stage["min_rot_step_deg"],
            tolerance=rot_tol_deg,
        )
        roll_step = correction_step(
            roll_error_deg,
            gain=stage["gain_roll"],
            max_step=stage["max_rot_step_deg"],
            min_step=stage["min_rot_step_deg"],
            tolerance=rot_tol_deg,
        )

        robot_pitch_step = PITCH_SIGN * pitch_step
        robot_yaw_step = YAW_SIGN * yaw_step
        robot_roll_step = ROLL_SIGN * roll_step

        next_pose = add_axis_step(next_pose, PITCH_TO_ROBOT_ROT_AXIS, robot_pitch_step)
        next_pose = add_axis_step(next_pose, YAW_TO_ROBOT_ROT_AXIS, robot_yaw_step)
        next_pose = add_axis_step(next_pose, ROLL_TO_ROBOT_ROT_AXIS, robot_roll_step)

        info.update(
            {
                "tag_pitch_deg": pitch_deg,
                "tag_yaw_deg": yaw_deg,
                "tag_roll_deg": roll_deg,
                "pitch_error_deg": pitch_error_deg,
                "yaw_error_deg": yaw_error_deg,
                "roll_error_deg": roll_error_deg,
                "robot_pitch_step": robot_pitch_step,
                "robot_yaw_step": robot_yaw_step,
                "robot_roll_step": robot_roll_step,
            }
        )

    check_safe_limits(next_pose)
    return next_pose, info


def is_localize_finished(tag, params, stage):
    targets = get_targets(params)
    enable_rotation = bool(params.get("enable_rotation", ENABLE_ROTATION_3AXIS))
    use_pixel_check = bool(params.get("use_pixel_finish_check", USE_PIXEL_FINISH_CHECK))

    target_u = get_number(params, "target_u", TARGET_U)
    target_v = get_number(params, "target_v", TARGET_V)

    center_tol_mm = float(stage["center_tol_mm"])
    distance_tol_mm = float(stage["distance_tol_mm"])
    rot_tol_deg = float(stage["rot_tol_deg"])

    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    x_error_mm = tag_x_mm - targets["target_x_mm"]
    y_error_mm = tag_y_mm - targets["target_y_mm"]
    distance_error_mm = tag_z_mm - targets["target_distance_mm"]

    pose_centered = (
        abs(x_error_mm) <= center_tol_mm and abs(y_error_mm) <= center_tol_mm
    )
    distance_ok = abs(distance_error_mm) <= distance_tol_mm

    if use_pixel_check:
        cx = float(tag["cx"])
        cy = float(tag["cy"])
        pixel_centered = (
            abs(cx - target_u) <= PIXEL_TOL and abs(cy - target_v) <= PIXEL_TOL
        )
    else:
        pixel_centered = True

    if enable_rotation:
        pitch_deg, yaw_deg, roll_deg = get_corrected_angles(tag, params)

        pitch_error = angle_error_deg(pitch_deg, targets["target_pitch_deg"])
        yaw_error = angle_error_deg(yaw_deg, targets["target_yaw_deg"])
        roll_error = angle_error_deg(roll_deg, targets["target_roll_deg"])

        rotation_ok = (
            abs(pitch_error) <= rot_tol_deg
            and abs(yaw_error) <= rot_tol_deg
            and abs(roll_error) <= rot_tol_deg
        )
    else:
        rotation_ok = True

    return pose_centered and distance_ok and pixel_centered and rotation_ok


def print_localize_status(tag, params, stage, iteration, stage_iters):
    targets = get_targets(params)
    target_u = get_number(params, "target_u", TARGET_U)
    target_v = get_number(params, "target_v", TARGET_V)
    enable_rotation = bool(params.get("enable_rotation", ENABLE_ROTATION_3AXIS))

    cx = float(tag["cx"])
    cy = float(tag["cy"])
    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    x_error_mm = tag_x_mm - targets["target_x_mm"]
    y_error_mm = tag_y_mm - targets["target_y_mm"]
    distance_error_mm = tag_z_mm - targets["target_distance_mm"]

    print()
    print(f"Localization stage '{stage['name']}' iteration {iteration}/{stage_iters}")
    print("------------------------------------------------------------")
    print(f"Vision source:          {describe_vision_source(tag)}")
    print(
        f"Samples:                {tag.get('sample_count', 1)}/{tag.get('requested_sample_count', 1)}"
    )
    print(f"Tag pixel center:       u={cx:.1f}, v={cy:.1f}")
    print(f"Pixel error:            du={cx - target_u:.1f}, dv={cy - target_v:.1f}")
    print(
        f"Camera-space tag pose:  x={tag_x_mm:.2f} mm, y={tag_y_mm:.2f} mm, z={tag_z_mm:.2f} mm"
    )
    print(
        f"Target camera pose:     x={targets['target_x_mm']:.2f} mm, "
        f"y={targets['target_y_mm']:.2f} mm, "
        f"z={targets['target_distance_mm']:.2f} mm"
    )
    print(f"Camera pose error:      x={x_error_mm:.2f} mm, y={y_error_mm:.2f} mm")
    print(f"Target distance error:  {distance_error_mm:.2f} mm")

    for key in ["x_mm", "y_mm", "z_mm", "tag_yaw_deg", "tag_pitch_deg", "tag_roll_deg"]:
        spread_key = key + "_spread"
        if spread_key in tag and tag[spread_key] is not None:
            print(f"Vision spread {key}:    {float(tag[spread_key]):.3f}")

    if enable_rotation:
        pitch_deg, yaw_deg, roll_deg = get_corrected_angles(tag, params)
        print(
            f"Corrected tag rot:      pitch={pitch_deg:.2f} deg, "
            f"yaw={yaw_deg:.2f} deg, roll={roll_deg:.2f} deg"
        )
        print(
            f"Rotation error:         pitch={angle_error_deg(pitch_deg, targets['target_pitch_deg']):.2f}, "
            f"yaw={angle_error_deg(yaw_deg, targets['target_yaw_deg']):.2f}, "
            f"roll={angle_error_deg(roll_deg, targets['target_roll_deg']):.2f} deg"
        )


def execute_localize(robot, command):
    name = command.get("name", "localize")
    required_tag_id = command.get("tag_id", None)
    enable_rotation = bool(command.get("enable_rotation", ENABLE_ROTATION_3AXIS))

    target_u = get_number(command, "target_u", TARGET_U)
    target_v = get_number(command, "target_v", TARGET_V)
    targets = get_targets(command)

    localize_vel = get_number(command, "vel", command.get("localize_vel", LOCALIZE_VEL))
    localize_acc = get_number(command, "acc", command.get("localize_acc", LOCALIZE_ACC))
    localize_ovl = get_number(command, "ovl", command.get("localize_ovl", LOCALIZE_OVL))
    wait_after_move_s = get_number(command, "wait_after_move_s", WAIT_AFTER_MOVE_S)
    sample_delay_s = get_number(command, "sample_delay_s", VISION_SAMPLE_DELAY_S)

    print(f"\nStarting precise stereo localization: {name}")
    print(f"Target pixel/debug center: u={target_u}, v={target_v}")
    print(
        f"Target camera pose: x={targets['target_x_mm']:.2f} mm, "
        f"y={targets['target_y_mm']:.2f} mm, "
        f"z={targets['target_distance_mm']:.1f} mm"
    )
    print(
        f"Target rotation: pitch={targets['target_pitch_deg']:.2f}, "
        f"yaw={targets['target_yaw_deg']:.2f}, roll={targets['target_roll_deg']:.2f} deg"
    )
    print(
        f"Zero offsets: pitch={targets['pitch_zero_offset_deg']:.2f}, "
        f"yaw={targets['yaw_zero_offset_deg']:.2f}, roll={targets['roll_zero_offset_deg']:.2f} deg"
    )
    print(f"Required tag id: {required_tag_id}")
    print(f"Rotation correction: {enable_rotation}")
    print(f"Vision source policy: {vision_source_policy_from_params(command)}")
    print(
        f"Robot yaw axis mapping: pose[{YAW_TO_ROBOT_ROT_AXIS}] with sign {YAW_SIGN:+.1f}"
    )

    last_tag = None

    for base_stage in LOCALIZE_STAGES:
        stage = merged_stage_params(base_stage, command)
        stage_iters = int(stage["max_iters"])
        stage_samples = int(stage["samples"])

        print()
        print(f"===== LOCALIZATION STAGE: {stage['name'].upper()} =====")
        print(
            f"tolerances: center={stage['center_tol_mm']} mm, "
            f"z={stage['distance_tol_mm']} mm, rot={stage['rot_tol_deg']} deg"
        )

        for iteration in range(1, stage_iters + 1):
            tag = collect_stable_tag(
                required_tag_id=required_tag_id,
                enable_rotation=enable_rotation,
                params=command,
                samples=stage_samples,
                delay_s=sample_delay_s,
            )
            last_tag = tag

            print_localize_status(tag, command, stage, iteration, stage_iters)

            if is_localize_finished(tag, command, stage):
                print(f"Localization stage '{stage['name']}' done.")
                break

            current_pose = get_actual_pose(robot)
            next_pose, info = build_corrected_pose(current_pose, tag, command, stage)

            print()
            print("Correction:")
            print(
                f"  X error {info['x_error_mm']:.3f} mm -> robot axis {CAMERA_X_TO_ROBOT_AXIS}: {info['robot_x_step']:.3f} mm"
            )
            print(
                f"  Y error {info['y_error_mm']:.3f} mm -> robot axis {CAMERA_Y_TO_ROBOT_AXIS}: {info['robot_y_step']:.3f} mm"
            )
            print(
                f"  Z error {info['distance_error_mm']:.3f} mm -> robot axis {CAMERA_Z_TO_ROBOT_AXIS}: {info['robot_z_step']:.3f} mm"
            )

            if enable_rotation:
                print(
                    f"  pitch -> robot axis {PITCH_TO_ROBOT_ROT_AXIS}: {info['robot_pitch_step']:.3f} deg"
                )
                print(
                    f"  yaw   -> robot axis {YAW_TO_ROBOT_ROT_AXIS}: {info['robot_yaw_step']:.3f} deg"
                )
                print(
                    f"  roll  -> robot axis {ROLL_TO_ROBOT_ROT_AXIS}: {info['robot_roll_step']:.3f} deg"
                )

            print(f"Current pose: {[round(v, 3) for v in current_pose]}")
            print(f"Next pose:    {[round(v, 3) for v in next_pose]}")

            if next_pose == current_pose:
                print("No correction needed. Localization done.")
                return

            move_cart(
                robot, next_pose, vel=localize_vel, acc=localize_acc, ovl=localize_ovl
            )
            time.sleep(wait_after_move_s)
        else:
            print(
                f"Localization stage '{stage['name']}' reached max iterations without meeting tolerance."
            )
            continue

        # Stage succeeded; continue into next stage, unless it was the final stage.
        if base_stage is LOCALIZE_STAGES[-1]:
            print("Precise localization done.")
            return

    if last_tag is not None:
        print("Last localization measurement:")
        targets = get_targets(command)
        print(
            json.dumps(
                {
                    "x_mm": last_tag.get("x_mm"),
                    "target_x_mm": targets["target_x_mm"],
                    "x_error_mm": None
                    if last_tag.get("x_mm") is None
                    else float(last_tag.get("x_mm")) - targets["target_x_mm"],
                    "y_mm": last_tag.get("y_mm"),
                    "target_y_mm": targets["target_y_mm"],
                    "y_error_mm": None
                    if last_tag.get("y_mm") is None
                    else float(last_tag.get("y_mm")) - targets["target_y_mm"],
                    "z_mm": last_tag.get("z_mm"),
                    "target_distance_mm": targets["target_distance_mm"],
                    "tag_pitch_deg": last_tag.get("tag_pitch_deg"),
                    "tag_yaw_deg": last_tag.get("tag_yaw_deg"),
                    "tag_roll_deg": last_tag.get("tag_roll_deg"),
                    "sample_count": last_tag.get("sample_count"),
                },
                indent=2,
            )
        )

    raise RuntimeError("Precise localization failed to meet final tolerance.")


# ============================================================
# COMMAND EXECUTION FUNCTIONS
# ============================================================


def execute_move_cart(robot, command):
    move_cart(
        robot,
        command["pose"],
        vel=command.get("vel"),
        acc=command.get("acc"),
        ovl=command.get("ovl"),
    )


def execute_line_cart(robot, command):
    move_line(
        robot,
        command["pose"],
        vel=command.get("vel"),
        acc=command.get("acc"),
        ovl=command.get("ovl"),
    )


def execute_arc_cart(robot, command):
    # Accept both the new names and the old working JSON names.
    via_pose = command.get("via_pose", command.get("via"))
    end_pose = command.get("end_pose", command.get("pose"))

    if via_pose is None or end_pose is None:
        raise RuntimeError("arc_cart needs via_pose/end_pose or old-style via/pose.")

    move_arc(
        robot,
        via_pose,
        end_pose,
        vel=command.get("vel"),
        acc=command.get("acc"),
        ovl=command.get("ovl"),
        fallback_segments=command.get("fallback_segments", 12),
    )


def execute_move_joint(robot, command):
    move_joint(
        robot,
        command["joints"],
        pose=command.get("pose"),
        vel=command.get("vel"),
        acc=command.get("acc"),
        ovl=command.get("ovl"),
    )


def execute_local_cart(robot, command):
    """
    TCP-local relative move by default, matching the old working script.

    JSON:
      {"type": "local_cart", "delta": [dx, dy, dz, dRX, dRY, dRZ]}

    Optional:
      "frame": "tcp"    -> default, XYZ delta is interpreted in TCP/tool frame
      "frame": "robot"  -> XYZ delta is added directly to robot pose axes
      "motion": "line"  -> use MoveL instead of MoveCart
    """
    current_pose = get_actual_pose(robot)
    delta = validate_vector("local_cart delta", command["delta"], 6)

    x_sign = get_number(command, "local_cart_x_sign", LOCAL_CART_X_SIGN)
    y_sign = get_number(command, "local_cart_y_sign", LOCAL_CART_Y_SIGN)
    z_sign = get_number(command, "local_cart_z_sign", LOCAL_CART_Z_SIGN)

    applied_delta = list(delta)
    applied_delta[0] *= x_sign
    applied_delta[1] *= y_sign
    applied_delta[2] *= z_sign

    frame = (
        str(command.get("frame", command.get("reference_frame", "tcp"))).strip().lower()
    )
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
        move_line(
            robot,
            next_pose,
            vel=command.get("vel"),
            acc=command.get("acc"),
            ovl=command.get("ovl"),
        )
    else:
        move_cart(
            robot,
            next_pose,
            vel=command.get("vel"),
            acc=command.get("acc"),
            ovl=command.get("ovl"),
        )


def execute_line_tcp(robot, command):
    current_pose = get_actual_pose(robot)
    delta = validate_vector("line_tcp delta", command["delta"][:6], 6)
    next_pose = tcp_delta_to_pose(current_pose, delta)
    move_line(
        robot,
        next_pose,
        vel=command.get("vel"),
        acc=command.get("acc"),
        ovl=command.get("ovl"),
    )


def execute_arc_tcp(robot, command):
    current_pose = get_actual_pose(robot)
    via_delta_raw = command.get("via_delta")
    end_delta_raw = command.get("end_delta", command.get("delta"))

    if via_delta_raw is None or end_delta_raw is None:
        raise RuntimeError(
            "arc_tcp needs via_delta and end_delta, or old-style via_delta and delta."
        )

    via_delta = validate_vector("arc_tcp via_delta", via_delta_raw, 6)
    end_delta = validate_vector("arc_tcp end_delta", end_delta_raw, 6)

    via_pose = tcp_delta_to_pose(current_pose, via_delta)
    end_pose = tcp_delta_to_pose(current_pose, end_delta)

    move_arc(
        robot,
        via_pose,
        end_pose,
        vel=command.get("vel"),
        acc=command.get("acc"),
        ovl=command.get("ovl"),
        fallback_segments=command.get("fallback_segments", 12),
    )


def execute_local_joint(robot, command):
    current_joints = get_actual_joints(robot)
    delta_raw = command.get("delta", command.get("delta_joints"))

    if delta_raw is None:
        raise RuntimeError("local_joint needs delta or old-style delta_joints.")

    delta = validate_vector("local_joint delta", delta_raw, 6)
    target_joints = add_vectors(current_joints, delta)
    move_joint(
        robot,
        target_joints,
        vel=command.get("vel"),
        acc=command.get("acc"),
        ovl=command.get("ovl"),
    )


def execute_wait(command):
    seconds = float(command.get("seconds", command.get("time", 1.0)))
    print(f"Waiting {seconds:.2f} seconds...")
    time.sleep(seconds)


def execute_gripper_activate(robot, command):
    device_id = int(command.get("device_id", GRIPPER_ID))
    activate_gripper(robot, device_id=device_id)


def execute_gripper(robot, command):
    move_gripper(
        robot,
        position=command["position"],
        speed=command.get("speed", GRIPPER_DEFAULT_SPEED),
        torque=command.get("torque", GRIPPER_DEFAULT_TORQUE),
        max_time_ms=command.get("max_time_ms", GRIPPER_MAX_TIME_MS),
        device_id=command.get("device_id", GRIPPER_ID),
        block=command.get("block", GRIPPER_BLOCK),
        gripper_type=command.get("gripper_type", GRIPPER_TYPE),
        activate=command.get("activate", False),
    )


def execute_gripper_status(robot, command):
    print_gripper_status(robot)


def execute_command(robot, command, index, total):
    if not isinstance(command, dict):
        raise RuntimeError(f"Command {index} must be an object/dict, got: {command}")

    command_type = command.get("type")
    name = command.get("name", "unnamed")

    print()
    print("============================================================")
    print(f"COMMAND {index}/{total}: {command_type} - {name}")
    print("============================================================")

    if command_type == "move_cart":
        execute_move_cart(robot, command)
    elif command_type == "line_cart":
        execute_line_cart(robot, command)
    elif command_type == "arc_cart":
        execute_arc_cart(robot, command)
    elif command_type == "move_joint":
        execute_move_joint(robot, command)
    elif command_type == "local_cart":
        execute_local_cart(robot, command)
    elif command_type == "line_tcp":
        execute_line_tcp(robot, command)
    elif command_type == "arc_tcp":
        execute_arc_tcp(robot, command)
    elif command_type == "local_joint":
        execute_local_joint(robot, command)
    elif command_type == "localize":
        execute_localize(robot, command)
    elif command_type == "wait":
        execute_wait(command)
    elif command_type == "gripper_activate":
        execute_gripper_activate(robot, command)
    elif command_type == "gripper":
        execute_gripper(robot, command)
    elif command_type == "gripper_status":
        execute_gripper_status(robot, command)
    else:
        raise RuntimeError(f"Unknown command type: {command_type}")


# ============================================================
# MAIN PROGRAM
# ============================================================


def main():
    print()
    print("Fairino precise stereo JSON job runner")
    print("======================================")
    print(f"Job file: {JOB_FILE}")
    print(f"Vision URL: {VISION_URL}")
    print(f"DRY_RUN: {DRY_RUN}")
    print()

    job = load_job_file(JOB_FILE)
    print(f"Loaded {len(job)} commands from JSON file.")

    robot = connect_robot()

    if ROBOT_PREFLIGHT:
        robot_preflight(robot)

    if MOVE_TO_START_POSE:
        print("Moving to known start pose...")
        move_cart(robot, TEST_START_POSE)
        time.sleep(1.5)
    else:
        current_start_pose = get_actual_pose(robot)
        print("Using current robot position as start pose:")
        print([round(v, 3) for v in current_start_pose])
        time.sleep(1.0)

    total = len(job)
    for i, command in enumerate(job, start=1):
        execute_command(robot, command, i, total)

    print()
    print("Job complete.")


if __name__ == "__main__":
    main()
