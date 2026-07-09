"""Connection, preflight, status, and readback functions for the Fairino robot."""

import time

import config as cfg
from core.math_utils import wrap_angle_deg

try:
    import Robot  # Fairino SDK, available on the Control Pi
except ImportError:  # Allows dry planning/tests on a non-robot computer.
    Robot = None


def connect_robot():
    if cfg.DRY_RUN and Robot is None:
        print("[DRY RUN] Robot SDK not available. Returning fake robot object.")
        return FakeRobot()

    if Robot is None:
        raise RuntimeError("Could not import Fairino Robot SDK. Activate fairino-venv on the Control Pi.")

    robot = Robot.RPC(cfg.ROBOT_IP)
    print(robot)
    print(f"Connected to robot using Fairino SDK: {cfg.ROBOT_IP}")
    return robot


def try_robot_call(robot, name: str, *args):
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


def robot_preflight(robot) -> None:
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


def get_actual_pose(robot) -> list[float]:
    err, pose = robot.GetActualTCPPose()
    if err != 0:
        raise RuntimeError(f"GetActualTCPPose failed with error code: {err}")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError(f"Unexpected pose format: {pose}")
    return [float(v) for v in pose[:6]]


def get_actual_joints(robot) -> list[float]:
    err, joints = robot.GetActualJointPosDegree(0)
    if err != 0:
        raise RuntimeError(f"GetActualJointPosDegree failed with error code: {err}")
    if not isinstance(joints, list) or len(joints) < 6:
        raise RuntimeError(f"Unexpected joint format: {joints}")
    return [float(v) for v in joints[:6]]


def pose_close(actual: list[float], target: list[float], xyz_tol=None, rot_tol=None) -> bool:
    xyz_tol = cfg.POSE_XYZ_REACHED_TOL_MM if xyz_tol is None else float(xyz_tol)
    rot_tol = cfg.POSE_ROT_REACHED_TOL_DEG if rot_tol is None else float(rot_tol)

    xyz_errors = [abs(float(actual[i]) - float(target[i])) for i in range(3)]
    rot_errors = [abs(wrap_angle_deg(float(actual[i + 3]) - float(target[i + 3]))) for i in range(3)]

    print(f"Pose XYZ errors: {[round(e, 3) for e in xyz_errors]} mm")
    print(f"Pose rot errors: {[round(e, 3) for e in rot_errors]} deg")

    return max(xyz_errors) <= xyz_tol and max(rot_errors) <= rot_tol


def joints_close(actual: list[float], target: list[float], tol_deg=None) -> bool:
    tol_deg = cfg.JOINT_REACHED_TOL_DEG if tol_deg is None else float(tol_deg)
    errors = [abs(float(actual[i]) - float(target[i])) for i in range(6)]
    max_error = max(errors)
    print(f"Joint errors: {[round(e, 4) for e in errors]} deg")
    print(f"Max joint error: {max_error:.4f} deg")
    return max_error <= tol_deg


def print_robot_debug(robot) -> None:
    try:
        err, joints = robot.GetActualJointPosDegree(0)
        print(f"Current joint query return: {err}")
        print(f"Current joints: {joints}")
    except Exception as e:
        print(f"Could not query current joints: {e}")

    try:
        err, robot_codes = robot.GetRobotErrorCode()
        print(f"Robot error query return: {err}")
        print(f"Robot main/sub error code: {robot_codes}")
    except Exception as e:
        print(f"Could not query robot error code: {e}")


def reset_robot_errors_quiet(robot) -> None:
    try:
        robot.ResetAllError()
        time.sleep(0.15)
    except Exception:
        pass


class FakeRobot:
    """Tiny fake object for dry-run testing away from the robot."""

    def __init__(self):
        self.pose = [400.0, 0.0, 350.0, 0.0, 0.0, 0.0]
        self.joints = [0.0, -90.0, 90.0, 0.0, 90.0, 0.0]

    def GetActualTCPPose(self):
        return 0, list(self.pose)

    def GetActualJointPosDegree(self, _flag):
        return 0, list(self.joints)

    def MoveCart(self, desc_pos, **_kwargs):
        self.pose = list(desc_pos)
        return 0

    def MoveL(self, desc_pos, **_kwargs):
        self.pose = list(desc_pos)
        return 0

    def MoveJ(self, joint_pos, **_kwargs):
        self.joints = list(joint_pos)
        return 0

    def ResetAllError(self):
        return 0

    def RobotEnable(self, _enable):
        return 0

    def Mode(self, _mode):
        return 0

    def GetRobotErrorCode(self):
        return 0, [0, 0]
