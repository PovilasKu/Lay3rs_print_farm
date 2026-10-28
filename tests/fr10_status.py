import sys

sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux")

from fairino import Robot

ROBOT_IP = "192.168.58.2"

robot = Robot.RPC(ROBOT_IP)

print("SDK:", robot.GetSDKVersion())
print("Software:", robot.GetSoftwareVersion())
print("Joints:", robot.GetActualJointPosDegree(0))
print("TCP:", robot.GetActualTCPPose(0))

robot.CloseRPC()
