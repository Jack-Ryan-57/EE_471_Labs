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

# Waypoints
mov1 = [0, -45, 60, 50]
mov2 = [0, 10, 50, -45]
mov3 = [0, 10, 0, -80]
movements = [mov1, mov2, mov3, mov1]    # Array for waypoints

np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

# Initialize file
filename = 'robot_movement_data.pkl'

# Setup robot
robot = Robot()  # Creates robot object

# Zero robot position
traj_time = 1  # Defines the trajectory time
robot.write_time(traj_time)  # Write trajectory time
robot.write_motor_state(True)  # Write position mode
robot.write_joints([0, 0, 0, 0])  # Write joints to zero position

traj_time = 5                   # Defines the trajectory time
robot.write_time(traj_time)     # Write trajectory time

timestamps = []                 # Create arrays for metrics to be logged
joint_angles = []
ee_positions = []

begin_time = time.time()                # Create a timer to use for timestamps

for movement in movements:              # Traverse through each waypoint, logging appropriate data
    robot.write_joints(movement)
    start_time = time.time()
    while time.time() - start_time < traj_time: # Continue till the trajectory time has been reached
        timestamps.append(time.time()-begin_time)
        joint_angles.append(robot.get_joints_readings()[0])
        ee_positions.append(robot.get_ee_pos(robot.get_joints_readings()[0]))
    
data_to_save = {                        # Setup data to save
    'joint_angles': joint_angles,
    'ee_positions': ee_positions,
    'timestamps': timestamps
}

PickleFiler.save_to_pickle(data_to_save, filename)          # Save data

################################################################
#PLOT DATA SECTION
################################################################

loaded_data = PickleFiler.load_from_pickle(filename)        # Load data...
print("Data loaded from: ", filename)
joint_angles_saved = np.array(loaded_data['joint_angles'])  # ...into numpy arrays
ee_positions_saved = np.array(loaded_data['ee_positions'])
timestamps_saved = np.array(loaded_data['timestamps'])

# Plots:

#
# (a) Plotting the joint positions
#
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
axs = axs.flatten() 
plt.suptitle('Joint Angles vs. Time')

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

plt.tight_layout()


#
#   (b) Plots for x and z locations of end effector
#
fig2, axs2 = plt.subplots(1, 2, figsize=(10, 8))
axs2 = axs2.flatten()
plt.suptitle('End-Effector Trajectory in X and Z Directions')

axs2[0].plot(timestamps_saved, ee_positions_saved[:, 0]*1000, 'b-', linewidth=2)
axs2[0].set_title('End-Effector X Position')
axs2[0].set_xlabel('Time (s)')
axs2[0].set_ylabel('Position (mm)')
axs2[0].grid(True)

axs2[1].plot(timestamps_saved, ee_positions_saved[:, 2]*1000, 'g-', linewidth=2)
axs2[1].set_title('End-Effector Z Position')
axs2[1].set_xlabel('Time (s)')
axs2[1].set_ylabel('Position (mm)')
axs2[1].grid(True)

plt.tight_layout()

#
# (c) Trajectory in xz plane
#

fig3, axs3 = plt.subplots()
axs3.plot(ee_positions_saved[:, 0]*1000, ee_positions_saved[:,2]*1000, label='Actual trajectory')
axs3.set(xlabel='X location (mm)', ylabel='Z location (mm)')
axs3.grid(True)
plt.title('Trajectory of End-Effector in XZ Plane')

# Extracting 3 xz waypoints for plotting
waypoint1 = robot.get_fk(np.radians([0, -45, 60, 50]))
waypoint2 = robot.get_fk(np.radians([0, 10, 50, -45]))
waypoint3 = robot.get_fk(np.radians([0, 10, 0, -80]))
# Extract x and z locations of waypoints
pt1 = [waypoint1[0, 3]*1000, waypoint1[2, 3]*1000]
pt2 = [waypoint2[0, 3]*1000, waypoint2[2, 3]*1000]
pt3 = [waypoint3[0, 3]*1000, waypoint3[2, 3]*1000]
# Plot desired waypoints
axs3.scatter(x=[pt1[0], pt2[0], pt3[0]], y=[pt1[1], pt2[1], pt3[1]], c='g', label='Desired waypoints')
axs3.legend()


#
# (d) Trajectory in xy plane
#

fig4, axs4 = plt.subplots()
axs4.plot(ee_positions_saved[:, 0]*1000, ee_positions_saved[:,1]*1000, label='Actual trajectory')
axs4.set(xlabel='X location (mm)', ylabel='Y location (mm)')
axs4.grid(True)
plt.title('Trajectory of End-Effector in XY Plane')

# Extract x and y locations of waypoints
pt1 = [waypoint1[0, 3]*1000, waypoint1[1, 3]*1000]
pt2 = [waypoint2[0, 3]*1000, waypoint2[1, 3]*1000]
pt3 = [waypoint3[0, 3]*1000, waypoint3[1, 3]*1000]
# Plot desired waypoints
axs4.scatter(x=[pt1[0], pt2[0], pt3[0]], y=[pt1[1], pt2[1], pt3[1]], c='g', label='Desired waypoints')
axs4.legend()
plt.ylim(-10, 1)


# Show all plots
plt.show()
