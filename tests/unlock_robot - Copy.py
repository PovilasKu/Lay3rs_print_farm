#!/usr/bin/env python3

import time
import Robot

ROBOT_IP = "192.168.58.2"

robot = Robot.RPC(ROBOT_IP)

print("Connected")

def try_call(name, *args):
    print(f"Trying {name}{args} ...")
    try:
        fn = getattr(robot, name)
    except AttributeError:
        print(f"  {name} not available in this SDK")
        return None

    try:
        result = fn(*args)
        print(f"  returned: {result}")
        return result
    except Exception as e:
        print(f"  failed: {e}")
        return None


print()
print("Current state:")
try_call("GetRobotErrorCode")
try_call("GetActualJointPosDegree", 0)
try_call("GetActualTCPPose")
try_call("IsInDragTeach")

print()
print("Stopping possible leftover motion modes...")

# Stop possible jog modes.
for stop_ref in [0, 1, 2, 3, 4]:
    try_call("StopJOG", stop_ref)
    time.sleep(0.1)

# End possible servo modes.
try_call("ServoMoveEnd")
try_call("ServoJEnd")
try_call("ServoCartEnd")
try_call("ServoJTEnd")

# Stop program/motion if those functions exist in your SDK.
try_call("StopMotion")
try_call("ProgramStop")
try_call("LuaStop")

print()
print("Resetting errors...")
try_call("ResetAllError")
time.sleep(0.5)

# Try enabling / setting automatic mode if your SDK has these.
print()
print("Trying enable/mode commands...")
try_call("RobotEnable", 1)
try_call("Mode", 0)
try_call("SetSpeed", 20)

time.sleep(0.5)

print()
print("State after unlock:")
try_call("GetRobotErrorCode")
try_call("GetActualJointPosDegree", 0)
try_call("GetActualTCPPose")
try_call("IsInDragTeach")

print()
print("Unlock script finished.")
