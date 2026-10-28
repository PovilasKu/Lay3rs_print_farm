import sys

sys.path.insert(0, "/home/fairino/fairino-python-sdk/linux/fairino")

import Robot
import time

ROBOT_IP = "192.168.58.2"

# Change these if needed
TOOL = 1
USER = 0

VEL = 40.0
ACC = 20.0
OVL = 10.0

EPOS = [0.0, 0.0, 0.0, 0.0]
OFFSET_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def check(ret, name):
    print(f"{name}: {ret}")
    if ret != 0:
        raise RuntimeError(f"{name} failed, error code {ret}")


def read_pose_input(prompt, current_pose):
    print()
    print(prompt)
    print("Enter TCP pose as:")
    print("  x y z rx ry rz")
    print("Example:")
    print("  -162.3 -421.8 291.3 0 0 0")
    print()
    print("Special commands:")
    print("  c  = use current pose")
    print("  q  = quit")
    print()

    while True:
        text = input("TCP> ").strip()

        if text.lower() == "q":
            return None

        if text.lower() == "c":
            return current_pose.copy()

        parts = text.replace(",", " ").split()

        if len(parts) != 6:
            print("Please enter exactly 6 numbers: x y z rx ry rz")
            continue

        try:
            pose = [float(p) for p in parts]
            return pose
        except ValueError:
            print("Invalid number. Try again.")


def main():
    robot = Robot.RPC(ROBOT_IP)

    try:
        check(robot.ResetAllError(), "Reset errors")
        check(robot.RobotEnable(1), "Enable robot")
        print("Mode(0):", robot.Mode(0))
        time.sleep(0.5)

        while True:
            print()
            print("=" * 60)

            ret, current_pose = robot.GetActualTCPPose(0)
            check(ret, "Read current TCP pose")

            ret, current_joints = robot.GetActualJointPosDegree(0)
            check(ret, "Read current joints")

            print()
            print("Current TCP pose:")
            print("  x  = %.3f mm" % current_pose[0])
            print("  y  = %.3f mm" % current_pose[1])
            print("  z  = %.3f mm" % current_pose[2])
            print("  rx = %.3f deg" % current_pose[3])
            print("  ry = %.3f deg" % current_pose[4])
            print("  rz = %.3f deg" % current_pose[5])

            print()
            print("Current joints:")
            print(["%.3f" % j for j in current_joints])

            target_pose = read_pose_input("Target TCP pose", current_pose)

            if target_pose is None:
                print("Exiting.")
                break

            print()
            print("Target TCP pose:")
            print(target_pose)

            # Check whether target pose has an IK solution
            ret, has_solution = robot.GetInverseKinHasSolution(
                0, target_pose, current_joints
            )
            check(ret, "IK has-solution call")

            print("Has IK solution:", has_solution)

            if not has_solution:
                print("No IK solution. Not moving.")
                continue

            # Calculate target joints for this TCP pose
            ret, target_joints = robot.GetInverseKinRef(0, target_pose, current_joints)
            check(ret, "Get inverse kinematics")

            print()
            print("Target joints:")
            print(["%.3f" % j for j in target_joints])

            print()
            answer = input("Move robot to this TCP pose? Type YES to move: ").strip()

            if answer != "YES":
                print("Move cancelled.")
                continue

            print()
            print("Moving slowly...")

            # MoveJ is used because it is usually more reliable for testing poses.
            # It reaches the target TCP pose, but the path is not guaranteed to be straight.
            ret = robot.MoveJ(
                joint_pos=target_joints,
                desc_pos=target_pose,
                tool=TOOL,
                user=USER,
                vel=VEL,
                acc=ACC,
                ovl=OVL,
                exaxis_pos=EPOS,
                blendT=-1.0,
                offset_flag=0,
                offset_pos=OFFSET_POS,
            )

            print("MoveJ returned:", ret)

            if ret != 0:
                try:
                    print("Robot error code:", robot.GetRobotErrorCode())
                except Exception as e:
                    print("Could not read robot error code:", e)

            if ret == 0:
                print("Move complete.")

    finally:
        robot.CloseRPC()


if __name__ == "__main__":
    main()
