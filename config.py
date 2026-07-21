"""
Global settings for the Fairino printer-rack cell.

This file should contain only configuration values, not robot logic.
Edit this file when IP addresses, default speeds, tool numbers, or folder paths change.
"""

from pathlib import Path

# -----------------------------------------------------------------------------
# Folder paths
# -----------------------------------------------------------------------------

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
SEQUENCES_DIR = BASE_DIR / "sequences"
LOGS_DIR = BASE_DIR / "logs"

PRINTERS_FILE = DATA_DIR / "printers" / "printers.json"
PRINTER_GROUPS_FILE = DATA_DIR / "printers" / "printer_groups.json"
RACK_LAYOUT_FILE = DATA_DIR / "printers" / "rack_layout.json"
JOB_CATALOG_FILE = DATA_DIR / "jobs" / "job_catalog.json"
SAFE_LIMITS_FILE = DATA_DIR / "safety" / "safe_limits.json"
BUILDPLATE_SLOTS_FILE = DATA_DIR / "rack" / "buildplate_slots.json"

# -----------------------------------------------------------------------------
# Robot / Vision network settings
# -----------------------------------------------------------------------------

ROBOT_IP = "192.168.58.2"

VISION_PI_IP = "10.255.2.97"
VISION_PORT = 5005
VISION_ENDPOINT = "/tag"
VISION_URL = f"http://{VISION_PI_IP}:{VISION_PORT}{VISION_ENDPOINT}"

# -----------------------------------------------------------------------------
# Runtime mode
# -----------------------------------------------------------------------------

DRY_RUN = False
ROBOT_PREFLIGHT = True
MOVE_TO_START_POSE = False
TEST_START_POSE = [400.0, 0.0, 350.0, 0.0, 0.0, 0.0]

# -----------------------------------------------------------------------------
# Robot movement defaults
# -----------------------------------------------------------------------------

TOOL = 1
USER = 0

VEL = 80.0
ACC = 60.0
OVL = 20.0

LOCALIZE_VEL = 50.0
LOCALIZE_ACC = 30.0
LOCALIZE_OVL = 100.0

POSE_XYZ_REACHED_TOL_MM = 2.0
POSE_ROT_REACHED_TOL_DEG = 2.0
JOINT_REACHED_TOL_DEG = 0.5
WAIT_AFTER_MOVE_S = 0.9

# TCP-local relative move signs. Y is not inverted by default.
LOCAL_CART_X_SIGN = +1.0
LOCAL_CART_Y_SIGN = +1.0
LOCAL_CART_Z_SIGN = +1.0

# -----------------------------------------------------------------------------
# Camera / localization target defaults
# -----------------------------------------------------------------------------

WIDTH = 1280
HEIGHT = 800
TARGET_U = WIDTH / 2.0
TARGET_V = HEIGHT / 2.0

TARGET_X_MM = 0.0
TARGET_Y_MM = 0.0
TARGET_DISTANCE_MM = 500.0

USE_PIXEL_FINISH_CHECK = False
PIXEL_TOL = 10.0

# -----------------------------------------------------------------------------
# Vision validation / filtering
# -----------------------------------------------------------------------------

VISION_SOURCE_POLICY = "require_stereo"  # require_stereo, prefer_stereo, allow_any
PRINT_VISION_SOURCE = True
MAX_VISION_AGE_S = 0.7
VISION_SAMPLE_DELAY_S = 0.05
VISION_MIN_VALID_RATIO = 0.55

# -----------------------------------------------------------------------------
# Camera-to-robot axis mapping for localization correction
# Robot pose indices: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
# -----------------------------------------------------------------------------

CAMERA_X_TO_ROBOT_AXIS = 1
CAMERA_Y_TO_ROBOT_AXIS = 2
CAMERA_Z_TO_ROBOT_AXIS = 0

CAMERA_X_SIGN = +1.0
CAMERA_Y_SIGN = -1.0
CAMERA_Z_SIGN = -1.0

# -----------------------------------------------------------------------------
# Rotation correction mapping
# -----------------------------------------------------------------------------

ENABLE_ROTATION_3AXIS = True

TARGET_PITCH_DEG = 0.0
TARGET_YAW_DEG = 0.0
TARGET_ROLL_DEG = 0.0

PITCH_ZERO_OFFSET_DEG = 0.0
YAW_ZERO_OFFSET_DEG = 0.0
ROLL_ZERO_OFFSET_DEG = 0.0

PITCH_TO_ROBOT_ROT_AXIS = 3
YAW_TO_ROBOT_ROT_AXIS = 5
ROLL_TO_ROBOT_ROT_AXIS = 4

PITCH_SIGN = +1.0
YAW_SIGN = -1.0
ROLL_SIGN = +1.0

# -----------------------------------------------------------------------------
# Localization stages
# -----------------------------------------------------------------------------

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

# -----------------------------------------------------------------------------
# Gripper settings
# -----------------------------------------------------------------------------

GRIPPER_ID = 2
GRIPPER_TYPE = 0
GRIPPER_BLOCK = 0
GRIPPER_MAX_TIME_MS = 2000
GRIPPER_DEFAULT_SPEED = 50
GRIPPER_DEFAULT_TORQUE = 100
