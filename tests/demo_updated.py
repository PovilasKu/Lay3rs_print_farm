#!/usr/bin/env python3

"""
Fairino robot JSON job runner with:

- fixed JSON path
- MoveCart absolute movement
- MoveJ joint movement
- TCP-local movement
- straight-line movement using MoveL
- arc movement using MoveC when available
- fallback arc approximation using small MoveL segments
- AprilTag localization

Run on the Control Pi:

    source ~/fairino-venv/bin/activate
    python ~/demo_updated_line_arc_tcp.py

Edit JOB_FILE below to choose the JSON file.
"""

import json
import time
import math
import urllib.request

import Robot


# ============================================================
# JOB FILE SETTINGS
# ============================================================

JOB_FILE = "/home/fairino/demo_sequence.json"
DRY_RUN = False


# ============================================================
# NETWORK SETTINGS
# ============================================================

ROBOT_IP = "192.168.58.2"
VISION_PI_IP = "10.255.2.97"
VISION_URL = f"http://{VISION_PI_IP}:5000/tag"


# ============================================================
# DEFAULT TARGET SETTINGS
# ============================================================

TARGET_U = 640.0
TARGET_V = 360.0
TARGET_DISTANCE_MM = 500.0


# ============================================================
# ROBOT MOVEMENT SETTINGS
# ============================================================

TOOL = 1
USER = 0

VEL = 80.0
ACC = 60.0
OVL = 20.0

MOVE_TO_START_POSE = False
TEST_START_POSE = [400.0, 0.0, 350.0, 0.0, 0.0, 0.0]

# Run basic reset/enable/mode before the job.
ROBOT_PREFLIGHT = True

# Tolerances used only for checking whether a non-zero return code still reached target.
POSE_XYZ_REACHED_TOL_MM = 2.0
POSE_ROT_REACHED_TOL_DEG = 2.0
JOINT_REACHED_TOL_DEG = 0.5


# ============================================================
# SERVO TUNING: POSITION
# ============================================================

MAX_ITERS = 30

CENTER_TOL_MM = 5.0
DISTANCE_TOL_MM = 8.0
PIXEL_TOL = 10.0

GAIN_X = 0.45
GAIN_Y = 0.45
GAIN_Z = 0.35

MAX_XY_STEP_MM = 40.0
MAX_Z_STEP_MM = 60.0

WAIT_AFTER_MOVE_S = 0.7
MAX_VISION_AGE_S = 0.7


# ============================================================
# SERVO TUNING: ROTATION
# ============================================================

ENABLE_ROTATION_3AXIS = True

TARGET_TILT_X_DEG = 0.0
TARGET_TILT_Y_DEG = 0.0
TARGET_ROLL_DEG = 0.0

ROT_TOL_DEG = 2.0

GAIN_TILT_X = 0.25
GAIN_TILT_Y = 0.25
GAIN_ROLL = 0.35

MAX_ROT_STEP_DEG = 1.0

# Robot pose indices:
#   0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
TILT_X_TO_ROBOT_ROT_AXIS = 3
TILT_Y_TO_ROBOT_ROT_AXIS = 5  # actual TCP Z
ROLL_TO_ROBOT_ROT_AXIS = 4

TILT_X_SIGN = -1.0
TILT_Y_SIGN = +1.0
ROLL_SIGN = +1.0


# ============================================================
# CAMERA-TO-ROBOT POSITION AXIS MAPPING
# ============================================================

CAMERA_X_TO_ROBOT_AXIS = 1
CAMERA_Y_TO_ROBOT_AXIS = 2
CAMERA_Z_TO_ROBOT_AXIS = 0

CAMERA_X_SIGN = +1.0
CAMERA_Y_SIGN = -1.0
CAMERA_Z_SIGN = -1.0


# Optional safety limits for robot XYZ/RX/RY/RZ.
# Example:
# SAFE_LIMITS = {
#     0: (-900.0, -500.0),  # X
#     1: (-300.0, 300.0),   # Y
#     2: (500.0, 900.0),    # Z
#     3: (-180.0, 180.0),   # RX
#     4: (-180.0, 180.0),   # RY
#     5: (-180.0, 180.0),   # RZ
# }
SAFE_LIMITS = None

# ============================================================
# GRIPPER SETTINGS
# ============================================================

GRIPPER_ID = 2  # Your configured device ID
GRIPPER_TYPE = 0  # 0 = parallel gripper
GRIPPER_BLOCK = 0  # 0 = blocking, 1 = non-blocking
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
    new_pose[axis] += float(step)
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


# ============================================================
# TCP-LOCAL COORDINATE HELPERS
# ============================================================


def rpy_deg_to_matrix(rx_deg, ry_deg, rz_deg):
    """
    Convert RX/RY/RZ degrees to a rotation matrix.

    Convention used here:
        R = Rz * Ry * Rx

    This is intended for TCP-local XYZ offsets.
    Always test with a small 5 mm move first.
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
    """
    tcp_delta = [tcp_x, tcp_y, tcp_z, dRX, dRY, dRZ]

    XYZ is interpreted in the TCP/tool coordinate system.
    Rotation is added directly to RX/RY/RZ.
    """
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
    rot_errors = [abs(float(actual[i + 3]) - float(target[i + 3])) for i in range(3)]

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
    """
    Normal absolute Cartesian move using MoveCart.
    This is not guaranteed to be a straight-line path.
    """
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
    """
    Absolute straight-line Cartesian move using MoveL.
    Use this when path shape matters.
    """
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
    """
    Joint move using MoveJ.

    If the JSON command contains a matching 'pose', it is passed as desc_pos.
    If that does not work, the script tries the simple joint-only keyword form.
    """
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
    if desc_pos is not None:
        print(f"Matching desc_pos: {[round(v, 3) for v in desc_pos]}")

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
    """
    Smooth fallback curve through approximate via point.
    XYZ and RX/RY/RZ are interpolated numerically.
    This is not a true controller circular arc, but it gives a curved path using MoveL segments.
    """
    a = (1.0 - t) * (1.0 - t)
    b = 2.0 * (1.0 - t) * t
    c = t * t
    return [a * p0[i] + b * p1[i] + c * p2[i] for i in range(6)]


def move_arc_fallback_segments(
    robot, via_pose, end_pose, vel=None, acc=None, ovl=None, segments=12
):
    """
    Fallback arc approximation using multiple MoveL segments.
    Use only when MoveC is unavailable or rejected.
    """
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
    """
    Arc move through via_pose to end_pose.

    First tries Fairino MoveC. If MoveC is missing or rejected, it falls back to
    a multi-segment MoveL curve approximation.
    """
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
    print(f"  vel={vel}, acc={acc}, ovl={ovl}")

    # Different Fairino SDK builds can have slightly different MoveC keyword names.
    # This first form is the common full keyword form.
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


def wait_for_fresh_tag(required_tag_id=None, enable_rotation=True):
    for _ in range(25):
        try:
            tag = read_tag_once()

            if not tag.get("ok"):
                print("Waiting for AprilTag detection...")
                time.sleep(0.2)
                continue

            if required_tag_id is not None:
                seen_tag_id = tag.get("tag_id")
                if int(seen_tag_id) != int(required_tag_id):
                    print(
                        f"Waiting for tag_id={required_tag_id}; currently seeing tag_id={seen_tag_id}..."
                    )
                    time.sleep(0.2)
                    continue

            age_s = float(tag.get("age_s", 999.0))

            if age_s > MAX_VISION_AGE_S:
                print(f"Waiting for fresh tag data, age={age_s:.2f}s...")
                time.sleep(0.2)
                continue

            needed = ["x_mm", "y_mm", "z_mm", "distance_mm", "cx", "cy"]

            if enable_rotation:
                needed += ["tag_tilt_x_deg", "tag_tilt_y_deg", "tag_roll_deg"]

            if not all(k in tag for k in needed):
                print(
                    "Tag detected, but some required values are missing from Vision Pi output."
                )
                print(f"Missing keys: {[k for k in needed if k not in tag]}")
                print(
                    "Make sure apriltag_server.py sends tag_tilt_x_deg, tag_tilt_y_deg, tag_roll_deg."
                )
                time.sleep(0.2)
                continue

            return tag

        except Exception as e:
            print(f"Vision read failed: {e}")
            time.sleep(0.2)

    raise RuntimeError("No fresh AprilTag pose received from Vision Pi.")


# ============================================================
# Gripper functions
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

    # Reset then activate, like the Fairino example.
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

    if max_time_ms < 0 or max_time_ms > 30000:
        raise RuntimeError(f"max_time_ms must be from 0 to 30000, got: {max_time_ms}")

    print("Moving gripper:")
    print(f"  device_id:   {device_id}")
    print(f"  position:    {position}")
    print(f"  speed:       {speed}")
    print(f"  torque:      {torque}")
    print(f"  max_time_ms: {max_time_ms}")
    print(f"  block:       {block}")

    if DRY_RUN:
        print("[DRY RUN] Would move gripper.")
        return

    if activate:
        activate_gripper(robot, device_id=device_id)

    # Newer SDK style: full MoveGripper with gripper type / rotary args.
    try:
        err = robot.MoveGripper(
            device_id,
            position,
            speed,
            torque,
            max_time_ms,
            block,
            gripper_type,
            0.0,
            0,
            0,
        )
    except TypeError:
        # Older Python SDK style: 6 arguments only.
        err = robot.MoveGripper(
            device_id,
            position,
            speed,
            torque,
            max_time_ms,
            block,
        )

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


def build_corrected_pose(current_pose, tag, params):
    """
    Converts camera-relative AprilTag position + orientation error
    into robot XYZ + RX/RY/RZ correction.
    """

    target_distance_mm = float(params.get("target_distance_mm", TARGET_DISTANCE_MM))
    target_tilt_x_deg = float(params.get("target_tilt_x_deg", TARGET_TILT_X_DEG))
    target_tilt_y_deg = float(params.get("target_tilt_y_deg", TARGET_TILT_Y_DEG))
    target_roll_deg = float(params.get("target_roll_deg", TARGET_ROLL_DEG))
    enable_rotation = bool(params.get("enable_rotation", ENABLE_ROTATION_3AXIS))

    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    distance_error_mm = tag_z_mm - target_distance_mm

    cam_x_step = clamp(GAIN_X * tag_x_mm, -MAX_XY_STEP_MM, MAX_XY_STEP_MM)
    cam_y_step = clamp(GAIN_Y * tag_y_mm, -MAX_XY_STEP_MM, MAX_XY_STEP_MM)
    cam_z_step = clamp(GAIN_Z * distance_error_mm, -MAX_Z_STEP_MM, MAX_Z_STEP_MM)

    robot_x_step = CAMERA_X_SIGN * cam_x_step
    robot_y_step = CAMERA_Y_SIGN * cam_y_step
    robot_z_step = CAMERA_Z_SIGN * cam_z_step

    next_pose = list(current_pose)

    next_pose = add_axis_step(next_pose, CAMERA_X_TO_ROBOT_AXIS, robot_x_step)
    next_pose = add_axis_step(next_pose, CAMERA_Y_TO_ROBOT_AXIS, robot_y_step)
    next_pose = add_axis_step(next_pose, CAMERA_Z_TO_ROBOT_AXIS, robot_z_step)

    info = {
        "tag_x_mm": tag_x_mm,
        "tag_y_mm": tag_y_mm,
        "tag_z_mm": tag_z_mm,
        "distance_error_mm": distance_error_mm,
        "cam_x_step": cam_x_step,
        "cam_y_step": cam_y_step,
        "cam_z_step": cam_z_step,
        "robot_x_step": robot_x_step,
        "robot_y_step": robot_y_step,
        "robot_z_step": robot_z_step,
        "tag_tilt_x_deg": 0.0,
        "tag_tilt_y_deg": 0.0,
        "tag_roll_deg": 0.0,
        "tilt_x_error_deg": 0.0,
        "tilt_y_error_deg": 0.0,
        "roll_error_deg": 0.0,
        "robot_rx_step": 0.0,
        "robot_ry_step": 0.0,
        "robot_rz_step": 0.0,
    }

    if enable_rotation:
        tag_tilt_x_deg = float(tag["tag_tilt_x_deg"])
        tag_tilt_y_deg = float(tag["tag_tilt_y_deg"])
        tag_roll_deg = float(tag["tag_roll_deg"])

        tilt_x_error_deg = tag_tilt_x_deg - target_tilt_x_deg
        tilt_y_error_deg = tag_tilt_y_deg - target_tilt_y_deg
        roll_error_deg = tag_roll_deg - target_roll_deg

        tilt_x_step = clamp(
            GAIN_TILT_X * tilt_x_error_deg, -MAX_ROT_STEP_DEG, MAX_ROT_STEP_DEG
        )
        tilt_y_step = clamp(
            GAIN_TILT_Y * tilt_y_error_deg, -MAX_ROT_STEP_DEG, MAX_ROT_STEP_DEG
        )
        roll_step = clamp(
            GAIN_ROLL * roll_error_deg, -MAX_ROT_STEP_DEG, MAX_ROT_STEP_DEG
        )

        robot_rx_step = TILT_X_SIGN * tilt_x_step
        robot_ry_step = TILT_Y_SIGN * tilt_y_step
        robot_rz_step = ROLL_SIGN * roll_step

        next_pose = add_axis_step(next_pose, TILT_X_TO_ROBOT_ROT_AXIS, robot_rx_step)
        next_pose = add_axis_step(next_pose, TILT_Y_TO_ROBOT_ROT_AXIS, robot_ry_step)
        next_pose = add_axis_step(next_pose, ROLL_TO_ROBOT_ROT_AXIS, robot_rz_step)

        info.update(
            {
                "tag_tilt_x_deg": tag_tilt_x_deg,
                "tag_tilt_y_deg": tag_tilt_y_deg,
                "tag_roll_deg": tag_roll_deg,
                "tilt_x_error_deg": tilt_x_error_deg,
                "tilt_y_error_deg": tilt_y_error_deg,
                "roll_error_deg": roll_error_deg,
                "robot_rx_step": robot_rx_step,
                "robot_ry_step": robot_ry_step,
                "robot_rz_step": robot_rz_step,
            }
        )

    return next_pose, info


def is_localize_finished(tag, params):
    target_u = float(params.get("target_u", TARGET_U))
    target_v = float(params.get("target_v", TARGET_V))
    target_distance_mm = float(params.get("target_distance_mm", TARGET_DISTANCE_MM))
    target_tilt_x_deg = float(params.get("target_tilt_x_deg", TARGET_TILT_X_DEG))
    target_tilt_y_deg = float(params.get("target_tilt_y_deg", TARGET_TILT_Y_DEG))
    target_roll_deg = float(params.get("target_roll_deg", TARGET_ROLL_DEG))
    enable_rotation = bool(params.get("enable_rotation", ENABLE_ROTATION_3AXIS))

    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    cx = float(tag["cx"])
    cy = float(tag["cy"])

    distance_error_mm = tag_z_mm - target_distance_mm

    pose_centered = abs(tag_x_mm) <= CENTER_TOL_MM and abs(tag_y_mm) <= CENTER_TOL_MM
    pixel_centered = abs(cx - target_u) <= PIXEL_TOL and abs(cy - target_v) <= PIXEL_TOL
    distance_ok = abs(distance_error_mm) <= DISTANCE_TOL_MM

    if enable_rotation:
        tilt_x_error = float(tag["tag_tilt_x_deg"]) - target_tilt_x_deg
        tilt_y_error = float(tag["tag_tilt_y_deg"]) - target_tilt_y_deg
        roll_error = float(tag["tag_roll_deg"]) - target_roll_deg

        rotation_ok = (
            abs(tilt_x_error) <= ROT_TOL_DEG
            and abs(tilt_y_error) <= ROT_TOL_DEG
            and abs(roll_error) <= ROT_TOL_DEG
        )
    else:
        rotation_ok = True

    return pose_centered and pixel_centered and distance_ok and rotation_ok


def execute_localize(robot, command):
    name = command.get("name", "localize")
    max_iters = int(command.get("max_iters", MAX_ITERS))
    required_tag_id = command.get("tag_id", None)
    enable_rotation = bool(command.get("enable_rotation", ENABLE_ROTATION_3AXIS))

    target_u = float(command.get("target_u", TARGET_U))
    target_v = float(command.get("target_v", TARGET_V))
    target_distance_mm = float(command.get("target_distance_mm", TARGET_DISTANCE_MM))

    print(f"\nStarting localization: {name}")
    print(f"Target pixel: u={target_u}, v={target_v}")
    print(f"Target distance: {target_distance_mm:.1f} mm")
    print(f"Required tag id: {required_tag_id}")
    print(f"Rotation correction: {enable_rotation}")

    for iteration in range(1, max_iters + 1):
        tag = wait_for_fresh_tag(
            required_tag_id=required_tag_id, enable_rotation=enable_rotation
        )

        cx = float(tag["cx"])
        cy = float(tag["cy"])
        tag_x_mm = float(tag["x_mm"])
        tag_y_mm = float(tag["y_mm"])
        tag_z_mm = float(tag["z_mm"])
        distance_error_mm = tag_z_mm - target_distance_mm

        print()
        print(f"Localization iteration {iteration}/{max_iters}")
        print("--------------------------------")
        print(f"Tag pixel center:       u={cx:.1f}, v={cy:.1f}")
        print(f"Pixel error:            du={cx - target_u:.1f}, dv={cy - target_v:.1f}")
        print(
            f"Camera-space tag pose:  x={tag_x_mm:.1f} mm, y={tag_y_mm:.1f} mm, z={tag_z_mm:.1f} mm"
        )
        print(f"Target distance error:  {distance_error_mm:.1f} mm")

        if enable_rotation:
            print(
                f"Camera-space tag rot:   "
                f"tilt_x={float(tag['tag_tilt_x_deg']):.2f} deg, "
                f"tilt_y={float(tag['tag_tilt_y_deg']):.2f} deg, "
                f"roll={float(tag['tag_roll_deg']):.2f} deg"
            )

        if is_localize_finished(tag, command):
            print("Localization done.")
            return

        current_pose = get_actual_pose(robot)
        next_pose, info = build_corrected_pose(current_pose, tag, command)

        print()
        print("Correction:")
        print(
            f"  camera X -> robot axis {CAMERA_X_TO_ROBOT_AXIS}: {info['robot_x_step']:.2f} mm"
        )
        print(
            f"  camera Y -> robot axis {CAMERA_Y_TO_ROBOT_AXIS}: {info['robot_y_step']:.2f} mm"
        )
        print(
            f"  camera Z -> robot axis {CAMERA_Z_TO_ROBOT_AXIS}: {info['robot_z_step']:.2f} mm"
        )

        if enable_rotation:
            print(
                f"  tilt X -> robot rot axis {TILT_X_TO_ROBOT_ROT_AXIS}: {info['robot_rx_step']:.2f} deg"
            )
            print(
                f"  tilt Y -> robot rot axis {TILT_Y_TO_ROBOT_ROT_AXIS}: {info['robot_ry_step']:.2f} deg"
            )
            print(
                f"  roll   -> robot rot axis {ROLL_TO_ROBOT_ROT_AXIS}: {info['robot_rz_step']:.2f} deg"
            )

        print()
        print(f"Current pose: {[round(v, 3) for v in current_pose]}")
        print(f"Next pose:    {[round(v, 3) for v in next_pose]}")

        # Localization uses your known-good MoveCart small-correction path.
        move_cart(
            robot,
            next_pose,
            vel=command.get("vel", VEL),
            acc=command.get("acc", ACC),
            ovl=command.get("ovl", OVL),
        )

        time.sleep(WAIT_AFTER_MOVE_S)

    raise RuntimeError(
        f"Localization failed: maximum iterations reached for command '{name}'."
    )


# ============================================================
# COMMAND EXECUTION
# ============================================================


def execute_move_cart(robot, command):
    pose = validate_vector("pose", command["pose"], 6)
    move_cart(
        robot,
        pose,
        vel=command.get("vel", VEL),
        acc=command.get("acc", ACC),
        ovl=command.get("ovl", OVL),
    )


def execute_line_cart(robot, command):
    pose = validate_vector("pose", command["pose"], 6)
    move_line(
        robot,
        pose,
        vel=command.get("vel", VEL),
        acc=command.get("acc", ACC),
        ovl=command.get("ovl", OVL),
    )


def execute_arc_cart(robot, command):
    via_pose = validate_vector("via", command["via"], 6)
    end_pose = validate_vector("pose", command["pose"], 6)
    move_arc(
        robot,
        via_pose,
        end_pose,
        vel=command.get("vel", VEL),
        acc=command.get("acc", ACC),
        ovl=command.get("ovl", OVL),
        fallback_segments=command.get("fallback_segments", 12),
    )


def execute_move_joint(robot, command):
    joints = validate_vector("joints", command["joints"], 6)
    pose = command.get("pose", None)
    move_joint(
        robot,
        joints,
        pose=pose,
        vel=command.get("vel", VEL),
        acc=command.get("acc", ACC),
        ovl=command.get("ovl", OVL),
    )


def execute_local_cart(robot, command):
    """
    TCP-local relative move.

    Default motion is MoveCart. Add "motion": "line" to force a straight-line MoveL.
    Example:
        {"type": "local_cart", "delta": [0,0,-40,0,0,0], "motion": "line"}
    """
    delta = validate_vector("delta", command["delta"], 6)
    current_pose = get_actual_pose(robot)
    global_delta = tcp_delta_to_global_delta(current_pose, delta)
    next_pose = add_vectors(current_pose, global_delta)

    motion = str(command.get("motion", "cart")).lower()

    print(f"Current pose:           {[round(v, 3) for v in current_pose]}")
    print(f"TCP-local cart delta:   {[round(v, 3) for v in delta]}")
    print(f"Converted global delta: {[round(v, 3) for v in global_delta]}")
    print(f"Next pose:              {[round(v, 3) for v in next_pose]}")
    print(f"Motion type:            {motion}")

    if motion in ["line", "straight", "movel"]:
        move_line(
            robot,
            next_pose,
            vel=command.get("vel", VEL),
            acc=command.get("acc", ACC),
            ovl=command.get("ovl", OVL),
        )
    else:
        move_cart(
            robot,
            next_pose,
            vel=command.get("vel", VEL),
            acc=command.get("acc", ACC),
            ovl=command.get("ovl", OVL),
        )


def execute_line_tcp(robot, command):
    delta = validate_vector("delta", command["delta"], 6)
    current_pose = get_actual_pose(robot)
    end_pose = tcp_delta_to_pose(current_pose, delta)

    print(f"Current pose:       {[round(v, 3) for v in current_pose]}")
    print(f"TCP line delta:     {[round(v, 3) for v in delta]}")
    print(f"Line end pose:      {[round(v, 3) for v in end_pose]}")

    move_line(
        robot,
        end_pose,
        vel=command.get("vel", VEL),
        acc=command.get("acc", ACC),
        ovl=command.get("ovl", OVL),
    )


def execute_arc_tcp(robot, command):
    via_delta = validate_vector("via_delta", command["via_delta"], 6)
    end_delta = validate_vector("delta", command["delta"], 6)

    current_pose = get_actual_pose(robot)
    via_pose = tcp_delta_to_pose(current_pose, via_delta)
    end_pose = tcp_delta_to_pose(current_pose, end_delta)

    print(f"Current pose:       {[round(v, 3) for v in current_pose]}")
    print(f"TCP arc via delta:  {[round(v, 3) for v in via_delta]}")
    print(f"TCP arc end delta:  {[round(v, 3) for v in end_delta]}")
    print(f"Arc via pose:       {[round(v, 3) for v in via_pose]}")
    print(f"Arc end pose:       {[round(v, 3) for v in end_pose]}")

    move_arc(
        robot,
        via_pose,
        end_pose,
        vel=command.get("vel", VEL),
        acc=command.get("acc", ACC),
        ovl=command.get("ovl", OVL),
        fallback_segments=command.get("fallback_segments", 12),
    )


def execute_local_joint(robot, command):
    delta = validate_vector("delta_joints", command["delta_joints"], 6)
    current_joints = get_actual_joints(robot)
    next_joints = add_vectors(current_joints, delta)

    print(f"Current joints:     {[round(v, 3) for v in current_joints]}")
    print(f"Local joint delta:  {[round(v, 3) for v in delta]}")
    print(f"Next joints:        {[round(v, 3) for v in next_joints]}")

    move_joint(
        robot,
        next_joints,
        vel=command.get("vel", VEL),
        acc=command.get("acc", ACC),
        ovl=command.get("ovl", OVL),
    )


def execute_wait(command):
    seconds = float(command.get("seconds", 1.0))
    print(f"Waiting {seconds:.2f} seconds...")
    time.sleep(seconds)


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


# ============================================================
# MAIN PROGRAM
# ============================================================


def main():
    print()
    print("Fairino automatic JSON job runner")
    print("=================================")
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
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped by user.")
    except Exception as e:
        print()
        print(f"ERROR: {e}")
