import sys

sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux/fairino")

import Robot

robot = Robot.RPC("192.168.58.2")

try:
    TOOL_NUM = 1

    # Example TCP:
    # Tool tip is 120 mm along the flange Z axis.
    # No rotation difference from the flange.
    TOOL_TCP = [0.0, 0.0, 127.0, -90, 180, -45.0]
    #        = [                  red, blue, green]

    # type=0 means normal tool coordinate system
    # install=0 means installed on robot end/flange
    # toolID can usually match TOOL_NUM
    # loadNum=0 means use load setting 0
    ret = robot.SetToolCoord(TOOL_NUM, TOOL_TCP, 0, 0, TOOL_NUM, 0)

    print("SetToolCoord returned:", ret)

    ret, current_tcp = robot.GetTCPOffset(0)
    print("GetTCPOffset returned:", ret)
    print("Current TCP:", current_tcp)

finally:
    robot.CloseRPC()
