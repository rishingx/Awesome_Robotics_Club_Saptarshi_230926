# == IMPORTING == #

import vectormath as vmath
from robot_arm import RobotArm

# == FUNCTIONS == #

def input_target():
    prompt = "Enter the target: "
    coords = [ float(x) for x in (input(prompt).split(","))]
    vector = vmath.Vector3(coords)
    return vector

# == MAIN == #

def main():

    n = int(input("Enter the number of joints: "))
    myArm = RobotArm(n)

    print()
    target = input_target()

    if myArm.fabrik(target, 2):
        myArm.output_joint_positions()
    else:
        print("The target point is unreachable!")

if __name__ == "__main__":
    main()