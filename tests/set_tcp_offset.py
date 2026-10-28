import sys

sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux/fairino")

import Robot

ROBOT_IP = "192.168.58.2"

# Tool number to create/update.
# Use this same number later in MoveJ / MoveL / MoveCart.
TOOL_NUM = 1

# TCP offset relative to robot flange:
# [x, y, z, rx, ry, rz]
#
# Example: tool tip is 120 mm straight out from flange Z,
# with no rotation offset.
TCP_OFFSET = [0.0, 0.0, 268.5, -90, 180, -45.0]
# Usually:
# type = 0      normal tool coordinate system
# install = 0   tool mounted on robot end/flange
# loadNum = 0   load setting number
TYPE = 0
INSTALL = 0
LOAD_NUM = 0

robot = Robot.RPC(ROBOT_IP)

try:
    print("Setting TCP offset for Tool", TOOL_NUM)
    print("TCP_OFFSET =", TCP_OFFSET)

    ret = robot.SetToolCoord(TOOL_NUM, TCP_OFFSET, TYPE, INSTALL, TOOL_NUM, LOAD_NUM)

    print("SetToolCoord returned:", ret)

    if ret != 0:
        raise RuntimeError(f"SetToolCoord failed, error code {ret}")

    # Read back current TCP offset
    ret, tcp = robot.GetTCPOffset(0)

    print("GetTCPOffset returned:", ret)
    print("Current TCP offset:", tcp)

finally:
    robot.CloseRPC()
