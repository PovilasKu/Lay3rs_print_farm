import sys
sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux/fairino")

import Robot
import time

robot = Robot.RPC("192.168.58.2")

try:
    print("Mode:", robot.Mode(0))
    print("Enable:", robot.RobotEnable(1))
    time.sleep(1)

    print("Robot error before clear:", robot.GetRobotErrorCode())

    print("ResetAllError:", robot.ResetAllError())
    time.sleep(1)

    print("Robot error after clear:", robot.GetRobotErrorCode())

    ret, joints = robot.GetActualJointPosDegree(0)
    print("GetActualJointPosDegree ret:", ret)
    print("Current joints:", joints)

finally:
    robot.CloseRPC()
