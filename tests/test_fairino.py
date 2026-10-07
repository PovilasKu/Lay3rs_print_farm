import sys
sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux")

from fairino import Robot

robot = Robot.RPC("192.168.58.2")

for name in [
    "GetSDKVersion",
    "GetSoftwareVersion",
    "GetActualJointPosDegree",
    "GetActualTCPPose",
]:
    try:
        func = getattr(robot, name)
        if name in ["GetActualJointPosDegree", "GetActualTCPPose"]:
            result = func(0)
        else:
            result = func()
        print(name, "=>", repr(result))
    except Exception as e:
        print(name, "FAILED:", repr(e))

try:
    robot.CloseRPC()
except Exception as e:
    print("CloseRPC failed:", repr(e))
