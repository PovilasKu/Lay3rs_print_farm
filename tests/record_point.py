#!/usr/bin/env python3

import Robot

robot = Robot.RPC("192.168.58.2")

err_j, joints = robot.GetActualJointPosDegree(0)
err_p, pose = robot.GetActualTCPPose()

print()
print("JOINT JSON:")
print([round(float(v), 3) for v in joints[:6]])

print()
print("CARTESIAN JSON:")
print([round(float(v), 3) for v in pose[:6]])
