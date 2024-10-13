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

# Zero robot position
traj_time = 10  # Defines the trajectory time
robot.write_time(traj_time)  # Write trajectory time
robot.write_motor_state(True)  # Write position mode
robot.write_joints([0, 0, 0, 0])  # Write joints to zero position

start_time = time.time()
while time.time() - start_time < traj_time: # Continue till the trajectory time has been reached
    print(robot.get_current_fk())  # Fetch current end effector transformation matrix
    print(robot.get_ee_pos(robot.get_joints_readings()[0])) # Get pose from joint readings

# Setup trajectory to be measured
robot.write_joints([15, -45, -60, 90])  # Write joints to zero position

start_time = time.time()
while time.time() - start_time < traj_time: # Continue till the trajectory time has been reached
    print(robot.get_current_fk())  # Fetch current end effector transformation matrix
    print(robot.get_ee_pos(robot.get_joints_readings()[0])) # Get pose from joint readings


# Setup trajectory to be measured
robot.write_joints([-90, 0, 0, 0])  # Write joints to zero position

start_time = time.time()
while time.time() - start_time < traj_time: # Continue till the trajectory time has been reached
    print(robot.get_current_fk())  # Fetch current end effector transformation matrix
    print(robot.get_ee_pos(robot.get_joints_readings()[0])) # Get pose from joint readings