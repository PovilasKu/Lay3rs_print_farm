#!/usr/bin/env python3

import json
import time
import math
import urllib.request

import sys
import Robot


# ============================================================
# NETWORK SETTINGS
# ============================================================

ROBOT_IP = "192.168.58.2"
ROBOT_RPC_URL = f"http://{ROBOT_IP}:20003/RPC2"

VISION_PI_IP = "10.255.2.97"
VISION_URL = f"http://{VISION_PI_IP}:5000/tag"


# ============================================================
# CAMERA / TARGET SETTINGS
# ============================================================

WIDTH = 1280
HEIGHT = 720

FX = 1960.0
FY = 1960.0
CX = 640.0
CY = 360.0

TARGET_U = 640.0
TARGET_V = 360.0

# Desired distance from camera to AprilTag.
TARGET_DISTANCE_MM = 500.0


# ============================================================
# ROBOT MOVEMENT SETTINGS
# ============================================================

TOOL = 1
USER = 0

VEL = 20.0
ACC = 30.0
OVL = 20.0

# Format: [X, Y, Z, RX, RY, RZ]
TEST_START_POSE = [
    400.0,  # X
    0.0,  # Y
    350.0,  # Z
    0.0,  # RX
    0.0,  # RY
    0.0,  # RZ
]

MOVE_TO_START_POSE = False
DRY_RUN = False


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

MAX_XY_STEP_MM = 20.0
MAX_Z_STEP_MM = 30.0

WAIT_AFTER_MOVE_S = 0.7
MAX_VISION_AGE_S = 0.7


# ============================================================
# SERVO TUNING: ROTATION
# ============================================================
#
# The Vision Pi should send:
#   tag_tilt_x_deg  -> tag plane tilt around camera X
#   tag_tilt_y_deg  -> tag plane tilt around camera Y
#   tag_roll_deg    -> in-image roll / crookedness around camera Z
#
# Target 0,0,0 means:
#   tag is not tilted relative to the camera
#   tag edges are straight in the image
#
# IMPORTANT:
# This is a small-step practical controller. Start with rotation only,
# test one axis at a time, and flip signs if it gets worse.

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
# First guess:
#   camera tilt X -> robot RX
#   camera tilt Y -> robot RY
#   image roll    -> robot RZ
TILT_X_TO_ROBOT_ROT_AXIS = 3
TILT_Y_TO_ROBOT_ROT_AXIS = 5  # actual tcp z
ROLL_TO_ROBOT_ROT_AXIS = 4

# Flip these one by one if that rotation makes the image worse.
TILT_X_SIGN = -1.0
TILT_Y_SIGN = +1.0
ROLL_SIGN = +1.0


# ============================================================
# CAMERA-TO-ROBOT POSITION AXIS MAPPING
# ============================================================
#
# Axis numbers:
#   0 = robot X
#   1 = robot Y
#   2 = robot Z

CAMERA_X_TO_ROBOT_AXIS = 1
CAMERA_Y_TO_ROBOT_AXIS = 2
CAMERA_Z_TO_ROBOT_AXIS = 0

CAMERA_X_SIGN = +1.0
CAMERA_Y_SIGN = -1.0
CAMERA_Z_SIGN = -1.0


# Optional safety limits for robot XYZ.
SAFE_LIMITS = None


# ============================================================
# BASIC HELPERS
# ============================================================


def clamp(value, low, high):
    return max(low, min(high, value))


def is_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


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


# ============================================================
# ROBOT FUNCTIONS
# ============================================================


def connect_robot():
    robot = Robot.RPC(ROBOT_IP)
    print(f"Connected to robot using Fairino SDK: {ROBOT_IP}")
    return robot


def get_actual_pose(robot):
    err, pose = robot.GetActualTCPPose()

    if err != 0:
        raise RuntimeError(f"GetActualTCPPose failed with error code: {err}")

    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"Unexpected pose format: {pose}")

    return [float(v) for v in pose[:6]]


def move_cart(robot, pose):
    pose = [float(v) for v in pose[:6]]
    check_safe_limits(pose)

    if DRY_RUN:
        print(f"[DRY RUN] Would MoveCart to: {[round(v, 3) for v in pose]}")
        return

    print(f"Sending MoveCart: {[round(v, 3) for v in pose]}")

    err = robot.MoveCart(
        desc_pos=pose,
        tool=TOOL,
        user=USER,
        vel=VEL,
        acc=ACC,
        ovl=OVL,
        blendT=-1.0,
        config=-1,
    )

    if err != 0:
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

        raise RuntimeError(f"MoveCart failed with error code: {err}")


# ============================================================
# VISION FUNCTIONS
# ============================================================


def read_tag_once():
    with urllib.request.urlopen(VISION_URL, timeout=1.0) as response:
        return json.loads(response.read().decode("utf-8"))


def wait_for_fresh_tag():
    for _ in range(25):
        try:
            tag = read_tag_once()

            if not tag.get("ok"):
                print("Waiting for AprilTag detection...")
                time.sleep(0.2)
                continue

            age_s = float(tag.get("age_s", 999.0))

            if age_s > MAX_VISION_AGE_S:
                print(f"Waiting for fresh tag data, age={age_s:.2f}s...")
                time.sleep(0.2)
                continue

            needed = ["x_mm", "y_mm", "z_mm", "distance_mm", "cx", "cy"]

            if ENABLE_ROTATION_3AXIS:
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
# CENTERING LOGIC
# ============================================================


def build_corrected_pose(current_pose, tag):
    """
    Converts camera-relative AprilTag position + orientation error
    into robot XYZ + RX/RY/RZ correction.
    """

    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    distance_error_mm = tag_z_mm - TARGET_DISTANCE_MM

    cam_x_step = GAIN_X * tag_x_mm
    cam_y_step = GAIN_Y * tag_y_mm
    cam_z_step = GAIN_Z * distance_error_mm

    cam_x_step = clamp(cam_x_step, -MAX_XY_STEP_MM, MAX_XY_STEP_MM)
    cam_y_step = clamp(cam_y_step, -MAX_XY_STEP_MM, MAX_XY_STEP_MM)
    cam_z_step = clamp(cam_z_step, -MAX_Z_STEP_MM, MAX_Z_STEP_MM)

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

    if ENABLE_ROTATION_3AXIS:
        tag_tilt_x_deg = float(tag["tag_tilt_x_deg"])
        tag_tilt_y_deg = float(tag["tag_tilt_y_deg"])
        tag_roll_deg = float(tag["tag_roll_deg"])

        tilt_x_error_deg = tag_tilt_x_deg - TARGET_TILT_X_DEG
        tilt_y_error_deg = tag_tilt_y_deg - TARGET_TILT_Y_DEG
        roll_error_deg = tag_roll_deg - TARGET_ROLL_DEG

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


def is_finished(tag):
    tag_x_mm = float(tag["x_mm"])
    tag_y_mm = float(tag["y_mm"])
    tag_z_mm = float(tag["z_mm"])

    cx = float(tag["cx"])
    cy = float(tag["cy"])

    distance_error_mm = tag_z_mm - TARGET_DISTANCE_MM

    pose_centered = abs(tag_x_mm) <= CENTER_TOL_MM and abs(tag_y_mm) <= CENTER_TOL_MM

    pixel_centered = abs(cx - TARGET_U) <= PIXEL_TOL and abs(cy - TARGET_V) <= PIXEL_TOL

    distance_ok = abs(distance_error_mm) <= DISTANCE_TOL_MM

    if ENABLE_ROTATION_3AXIS:
        tilt_x_error = float(tag["tag_tilt_x_deg"]) - TARGET_TILT_X_DEG
        tilt_y_error = float(tag["tag_tilt_y_deg"]) - TARGET_TILT_Y_DEG
        roll_error = float(tag["tag_roll_deg"]) - TARGET_ROLL_DEG

        rotation_ok = (
            abs(tilt_x_error) <= ROT_TOL_DEG
            and abs(tilt_y_error) <= ROT_TOL_DEG
            and abs(roll_error) <= ROT_TOL_DEG
        )
    else:
        rotation_ok = True

    return pose_centered and pixel_centered and distance_ok and rotation_ok


# ============================================================
# MAIN PROGRAM
# ============================================================


def main():
    print()
    print("AprilTag robot 6D centering test")
    print("================================")
    print(f"Vision URL: {VISION_URL}")
    print(f"Target pixel: u={TARGET_U}, v={TARGET_V}")
    print(f"Target distance: {TARGET_DISTANCE_MM:.1f} mm")
    print(f"Rotation correction: {ENABLE_ROTATION_3AXIS}")
    print(f"DRY_RUN: {DRY_RUN}")
    print()

    robot = connect_robot()

    if MOVE_TO_START_POSE:
        print("Moving to known start pose...")
        move_cart(robot, TEST_START_POSE)
        time.sleep(1.5)
    else:
        current_start_pose = get_actual_pose(robot)
        print("Using current robot position as start pose:")
        print([round(v, 3) for v in current_start_pose])
        time.sleep(1.5)

    for iteration in range(1, MAX_ITERS + 1):
        tag = wait_for_fresh_tag()

        cx = float(tag["cx"])
        cy = float(tag["cy"])
        tag_x_mm = float(tag["x_mm"])
        tag_y_mm = float(tag["y_mm"])
        tag_z_mm = float(tag["z_mm"])
        distance_error_mm = tag_z_mm - TARGET_DISTANCE_MM

        print()
        print(f"Iteration {iteration}")
        print("--------------------")
        print(f"Tag pixel center:       u={cx:.1f}, v={cy:.1f}")
        print(f"Pixel error:            du={cx - TARGET_U:.1f}, dv={cy - TARGET_V:.1f}")
        print(
            f"Camera-space tag pose:  x={tag_x_mm:.1f} mm, y={tag_y_mm:.1f} mm, z={tag_z_mm:.1f} mm"
        )
        print(f"Target distance error:  {distance_error_mm:.1f} mm")

        if ENABLE_ROTATION_3AXIS:
            print(
                f"Camera-space tag rot:   "
                f"tilt_x={float(tag['tag_tilt_x_deg']):.2f} deg, "
                f"tilt_y={float(tag['tag_tilt_y_deg']):.2f} deg, "
                f"roll={float(tag['tag_roll_deg']):.2f} deg"
            )

        if is_finished(tag):
            print()
            print(
                "Done: AprilTag is centered, at target distance, and rotationally aligned."
            )
            return

        current_pose = get_actual_pose(robot)
        next_pose, info = build_corrected_pose(current_pose, tag)

        print()
        print("Correction:")
        print(
            f"  camera X -> robot axis {CAMERA_X_TO_ROBOT_AXIS}: "
            f"{info['robot_x_step']:.2f} mm"
        )
        print(
            f"  camera Y -> robot axis {CAMERA_Y_TO_ROBOT_AXIS}: "
            f"{info['robot_y_step']:.2f} mm"
        )
        print(
            f"  camera Z -> robot axis {CAMERA_Z_TO_ROBOT_AXIS}: "
            f"{info['robot_z_step']:.2f} mm"
        )

        if ENABLE_ROTATION_3AXIS:
            print(
                f"  tilt X -> robot rot axis {TILT_X_TO_ROBOT_ROT_AXIS}: "
                f"{info['robot_rx_step']:.2f} deg"
            )
            print(
                f"  tilt Y -> robot rot axis {TILT_Y_TO_ROBOT_ROT_AXIS}: "
                f"{info['robot_ry_step']:.2f} deg"
            )
            print(
                f"  roll   -> robot rot axis {ROLL_TO_ROBOT_ROT_AXIS}: "
                f"{info['robot_rz_step']:.2f} deg"
            )

        print()
        print(f"Current pose: {[round(v, 3) for v in current_pose]}")
        print(f"Next pose:    {[round(v, 3) for v in next_pose]}")

        move_cart(robot, next_pose)

        time.sleep(WAIT_AFTER_MOVE_S)

    print()
    print("Stopped: maximum iterations reached before convergence.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("Stopped by user.")
    except Exception as e:
        print()
        print(f"ERROR: {e}")
