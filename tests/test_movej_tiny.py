#!/usr/bin/env python3

import Robot
import time

ROBOT_IP = "192.168.58.2"
TOOL = 1
USER = 0

robot = Robot.RPC(ROBOT_IP)

err, joints = robot.GetActualJointPosDegree(0)
if err != 0:
    raise RuntimeError(f"Could not read joints: {err}")

joints = [float(v) for v in joints[:6]]

print("Current joints:")
print([round(v, 3) for v in joints])

target = list(joints)
target[0] += 1.0  # move only J1 by +1 degree

print("Target joints:")
print([round(v, 3) for v in target])

err = robot.MoveJ(
    target,
    tool=TOOL,
    user=USER,
    vel=10.0,
    acc=30.0,
    ovl=20.0,
)

print("MoveJ return:", err)

time.sleep(0.5)

err2, joints2 = robot.GetActualJointPosDegree(0)
print("After joints:")
print([round(float(v), 3) for v in joints2[:6]])
