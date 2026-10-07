import sys
sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux/fairino")

import Robot
import time

ROBOT_IP = "192.168.58.2"

TOOL = 1     # your configured tool
USER = 0     # base/user coordinate frame

def check(ret, name):
    print(f"{name}: {ret}")
    if ret != 0:
        raise RuntimeError(f"{name} failed, error code {ret}")

robot = Robot.RPC(ROBOT_IP)

try:
    check(robot.ResetAllError(), "Reset errors")
    check(robot.RobotEnable(1), "Enable robot")
    print("Mode(0):", robot.Mode(0))
    time.sleep(0.5)

    ret, current_pose = robot.GetActualTCPPose(0)
    check(ret, "Read current TCP pose")

    print("Current TCP pose:")
    print(current_pose)

    # Keep the current TCP position, change only orientation.
    target_pose = [
        current_pose[0],
        current_pose[1],
        current_pose[2],
        0,
       	0,
       	0
    ]

    print("Target TCP pose:")
    print(target_pose)

    # Ask the controller whether this pose has an IK solution near current joints.
    ret, current_joints = robot.GetActualJointPosDegree(0)
    check(ret, "Read current joints")

    ret, has_solution = robot.GetInverseKinHasSolution(
        0,              # 0 = absolute pose in base coordinate system
        target_pose,
        current_joints
    )

    print("IK check return:", ret)
    print("Has solution:", has_solution)

    if ret != 0 or not has_solution:
        raise RuntimeError("The [rx, ry, rz] = [0, 0, 0] orientation is not reachable from here.")

    # Move in Cartesian PTP mode.
    # Use very low speed first.
    ret = robot.MoveCart(
        target_pose,
        TOOL,
        USER,
        vel=50.0,
        acc=0.0,
        ovl=10.0,
        blendT=-1.0,
        config=-1
    )

    check(ret, "MoveCart to rotation zero")

finally:
    robot.CloseRPC()
