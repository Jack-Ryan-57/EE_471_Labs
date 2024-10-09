# Important neccessary libraries
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time

# Setup robot
robot = Robot()  # Creates robot object

print("Forward kinematics for [0,0,0,0]:")
print(robot.get_fk(np.radians([0, 0, 0, 0])))
print("Forward kinematics for [15, −45, −60, 90]:")
print(robot.get_fk(np.radians([15, -45, -60, 90])))
print("Forward kinematics for [-90,0,0,0]:")
print(robot.get_fk(np.radians([-90, 0, 0, 0])))