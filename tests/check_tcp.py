import sys
sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux/fairino")

import Robot

robot = Robot.RPC("192.168.58.2")

try:
    ret, tcp = robot.GetTCPOffset(0)

    print("GetTCPOffset return:", ret)
    print("Raw TCP:", tcp)

    if ret == 0:
        x, y, z, rx, ry, rz = tcp

        print()
        print("Tool position offset:")
        print(f"  X  = {x:.3f} mm")
        print(f"  Y  = {y:.3f} mm")
        print(f"  Z  = {z:.3f} mm")

        print()
        print("Tool rotation offset:")
        print(f"  RX = {rx:.3f} deg")
        print(f"  RY = {ry:.3f} deg")
        print(f"  RZ = {rz:.3f} deg")

finally:
    robot.CloseRPC()
