# Important neccessary libraries
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import PickleFiler
import time

mov1 = [0, -45, 60, 50]
mov2 = [0, 10, 50, -45]
mov3 = [0, 10, 0, -80]
movements = [mov1, mov2, mov3, mov1]

#Initialize file
filename = 'robot_movement_data.pkl'
# Setup robot
robot = Robot()  # Creates robot object

# Zero robot position
traj_time = 1  # Defines the trajectory time
robot.write_time(traj_time)  # Write trajectory time
robot.write_motor_state(True)  # Write position mode
robot.write_joints([0, 0, 0, 0])  # Write joints to zero position

traj_time = 5  # Defines the trajectory time
robot.write_time(traj_time)  # Write trajectory time

timestamps = []
joint_angles = []
ee_positions = []

begin_time = time.time()
for movement in movements:
    robot.write_joints(movement)
    start_time = time.time()
    while time.time() - start_time < traj_time: # Continue till the trajectory time has been reached
        timestamps.append(time.time()-begin_time)
        joint_angles.append(robot.get_joints_readings()[0])
        ee_positions.append(robot.get_ee_pos(robot.get_joints_readings()[0]))
    
    
# Sample data to save
data_to_save = {
    'joint_angles': joint_angles,
    'ee_positions': ee_positions,
    'timestamps': timestamps
}

PickleFiler.save_to_pickle(data_to_save, filename)

loaded_data = PickleFiler.load_from_pickle(filename)
print("Data loaded from: ", filename)
joint_angles_saved = np.array(loaded_data['joint_angles'])
ee_positions_saved = np.array(loaded_data['ee_positions'])
timestamps_saved = np.array(loaded_data['timestamps'])
print("Joint angles:", joint_angles_saved)
print("End-effector positions:", ee_positions_saved)
print("Timestamps:", timestamps_saved)

# Plotting the joint positions using Matplotlib
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
axs = axs.flatten() 

# Plot Joint 1      
axs[0].plot(timestamps_saved, joint_angles_saved[:, 0], 'r-', linewidth=2)
axs[0].set_title('Joint 1 (Base Joint)')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Position (deg)')
axs[0].grid(True)

# Plot Joint 2
axs[1].plot(timestamps_saved, joint_angles_saved[:, 1], 'g-', linewidth=2)
axs[1].set_title('Joint 2')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Position (deg)')
axs[1].grid(True)

# Plot Joint 3
axs[2].plot(timestamps_saved, joint_angles_saved[:, 2], 'b-', linewidth=2)
axs[2].set_title('Joint 3')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Position (deg)')
axs[2].grid(True)

# Plot Joint 4
axs[3].plot(timestamps_saved, joint_angles_saved[:, 3], 'm-', linewidth=2)
axs[3].set_title('Joint 4')
axs[3].set_xlabel('Time (s)')
axs[3].set_ylabel('Position (deg)')
axs[3].grid(True)

# Adjust layout
plt.tight_layout()

# Set title
plt.suptitle('Joint Motion Profiles', y=1.02)

# Show the plot
plt.show()