import sys, os
import numpy as np
# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot

# Create robot
robot = Robot()
# Loop through test cases
cases = [[274,0,204,0],[16,4,336,15],[0,-270,106,0],[274,0,204,0]]

np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

# Loop through all test cases
for case in cases:
    print(f"IK solutions to case {case}: ")
    ik = robot.get_ik(np.array(case))   # Get inverse kinematics from given test case
    print(ik)
    print(f"FK solutions to case {case}: ")
    fk = robot.get_ee_pos(ik)           # Get fk from calculated ik to insure the solution matches test case
    print(fk)
    