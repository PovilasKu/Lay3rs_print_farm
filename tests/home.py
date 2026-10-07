import Robot
import time

ROBOT_IP = "192.168.58.2"   # default Fairino controller IP

# IMPORTANT:
# Replace this with a safe home position for YOUR arm.
# Units are degrees: J1, J2, J3, J4, J5, J6
HOME_JOINTS = [48.94659423828125, -104.92791748046875, 154.16494750976562, -47.445491790771484, 91.1454086303711, 0]
RANDOM_JOINTS = [-1.8552753925323486, -124.82992553710938, 123.80416870117188, -101.04789733886719, 91.14888763427734, 0]

TOOL = 0
USER = 0
v = 40.0

def check(ret, action):
    if ret != 0:
        raise RuntimeError(f"{action} failed, error code: {ret}")

robot = Robot.RPC(ROBOT_IP)

try:
    # Auto mode: 0 = automatic, 1 = manual
    check(robot.Mode(0), "Switch to automatic mode")

    # Enable robot: 1 = enable, 0 = disable
    check(robot.RobotEnable(1), "Enable robot")

    time.sleep(1)

    # MoveJ:
    # joint_pos = target joints in degrees
    # vel = speed percentage
    # ovl = overall speed scaling
    # blendT = -1 means blocking: wait until movement finishes
    ret = robot.MoveJ(
        HOME_JOINTS,
        TOOL,
        USER,
        vel=v,
        acc=0.0,
        ovl=30.0,
        blendT=-1.0
    )
    check(ret, "Move to home")

    print("Robot moved to home position.")

finally:
    robot.CloseRPC()
