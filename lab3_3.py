import sys, os
import numpy as np
import matplotlib.pyplot as plt

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time
import PickleFiler

np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

# Initialize pickle file
filename = 'robot_lab3_data.pkl'

# Setup robot
robot = Robot()

# Set robot to home position
traj_time = 5 # Defines the trajectory time
robot.write_time(traj_time)  # Write trajectory time
robot.write_motor_state(True)  # Write position mode
robot.write_joints([0, 0, 0, 0])  # Write joints to zero position

# Create arrays for metrics to be logged
timestamps = []
joint_angles = []
ee_positions = []

# Define waypoints
waypoints = [[25,-100, 150,-60],[150,80,300,0],[250,-115,75,-45],[25,-100, 150,-60]]

# Create a timer to use for timestamps
begin_time = time.time()

# Move along each path and log data for each wavepoint
for waypoint in waypoints:
    desired_joint_vars = robot.get_ik(np.array(waypoint))   # Get inverse kinematics of waypoint
    robot.write_joints(desired_joint_vars)      # Write joints to calculated angles
    start_time = time.time()
    while time.time() - start_time < traj_time: # Continue till the trajectory time has been reached
        timestamps.append(time.time()-begin_time)   
        joint_angles.append(robot.get_joints_readings()[0]) # Read and store current joint readings
        ee_positions.append(robot.get_ee_pos(robot.get_joints_readings()[0]))   # Read and store forward kinematics values

    pass

data_to_save = {   # Setup data to save
    'joint_angles': joint_angles,
    'ee_positions': ee_positions,
    'timestamps': timestamps
}

PickleFiler.save_to_pickle(data_to_save, filename)          # Save data

loaded_data = PickleFiler.load_from_pickle(filename)        # Load data...
print("Data loaded from: ", filename)
joint_angles_saved = np.array(loaded_data['joint_angles'])  # ...into numpy arrays
ee_positions_saved = np.array(loaded_data['ee_positions'])
timestamps_saved = np.array(loaded_data['timestamps'])

# Plots:

#
# (a) Plotting the x,y,z,alpha positions
#
fig, ax1 = plt.subplots(figsize=(10, 8))  # Create a single plot
plt.suptitle('End Effector Pose vs. Time')

# First y-axis: Position (in mm)
ax1.plot(timestamps_saved, ee_positions_saved[:, 0] * 1000, 'r-', linewidth=2, label='X Position (mm)')  # Convert to mm
ax1.plot(timestamps_saved, ee_positions_saved[:, 1] * 1000, 'g-', linewidth=2, label='Y Position (mm)')  # Convert to mm
ax1.plot(timestamps_saved, ee_positions_saved[:, 2] * 1000, 'b-', linewidth=2, label='Z Position (mm)')  # Convert to mm
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Position (mm)', color='black')
ax1.tick_params(axis='y', labelcolor='black')
ax1.grid(True)

# Add legends for the first y-axis (positions)
lines_1 = [plt.Line2D([0], [0], color=c, lw=2) for c in ['r', 'g', 'b']]
labels_1 = ['X Position (mm)', 'Y Position (mm)', 'Z Position (mm)']

# Second y-axis: Orientation (in degrees)
ax2 = ax1.twinx()  # Create a second y-axis
ax2.plot(timestamps_saved, ee_positions_saved[:, 3], 'm--', linewidth=2, label='Orientation α (deg)')  # Alpha orientation in degrees
ax2.set_ylabel('Orientation (deg)', color='black')
ax2.tick_params(axis='y', labelcolor='black')

# Add legend for the second y-axis (orientation)
lines_2 = [plt.Line2D([0], [0], color='m', lw=2, linestyle='--')]
labels_2 = ['Orientation α (deg)']

# Combine legends from both y-axes
lines = lines_1 + lines_2
labels = labels_1 + labels_2
ax1.legend(lines, labels, loc='upper right')

# Adjust layout
plt.tight_layout()
plt.show()

#
# (b) Plotting the 3D trajectory of the end-effector
#
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.title("3D Trajectory of Robot Arm's End-Effector")

# Extract X, Y, Z coordinates from ee_positions_saved
x_vals = ee_positions_saved[:, 0]  # X coordinates
y_vals = ee_positions_saved[:, 1]  # Y coordinates
z_vals = ee_positions_saved[:, 2]  # Z coordinates

ax.set_xlabel('X Location (m)')
ax.set_ylabel('Y Location (m)')
ax.set_zlabel('Z Location (m)')

# Plot the trajectory in 3D
ax.plot(x_vals, y_vals, z_vals, 'bo-', linewidth=2)

plt.tight_layout()

# Show the plots
plt.show()

#
# (c) Plotting the actual measured joint positions
#
fig, ax = plt.subplots(figsize=(10, 8))  # Create a single plot
plt.suptitle('Measured Joint Angles vs. Time')

# Plot all joints on the same axes
ax.plot(timestamps_saved, joint_angles_saved[:, 0], 'r-', linewidth=2, label='Joint 1 (Base Joint)')
ax.plot(timestamps_saved, joint_angles_saved[:, 1], 'g-', linewidth=2, label='Joint 2')
ax.plot(timestamps_saved, joint_angles_saved[:, 2], 'b-', linewidth=2, label='Joint 3')
ax.plot(timestamps_saved, joint_angles_saved[:, 3], 'm-', linewidth=2, label='Joint 4')

# Set labels and grid
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (deg)')
ax.grid(True)

# Add legend
ax.legend()

# Adjust layout
plt.tight_layout()

#
# (d) Plotting the calculated joint positions
#
fig, ax = plt.subplots(figsize=(10, 8))  # Create a single plot
plt.suptitle('Inverse Kinematics Calculated Joint Angles vs. Time')

# Initialize list to store joint angles from inverse kinematics (IK)
calculated_joint_angles = []

# Apply the get_ik() function to each end-effector pose to calculate the corresponding joint angles
for pose in ee_positions_saved:
    x_mm, y_mm, z_mm, alpha_deg = pose[0] * 1000, pose[1] * 1000, pose[2] * 1000, pose[3]  # Convert to mm where necessary
    pose_mm_deg = np.array([x_mm, y_mm, z_mm, alpha_deg])   # Redefine pose with proper units
    joint_angles = robot.get_ik(pose_mm_deg)  # Calculate joint angles from the end-effector pose
    calculated_joint_angles.append(joint_angles)

# Convert to a NumPy array for easy slicing and plotting
calculated_joint_angles = np.array(calculated_joint_angles)

# Plot all joints on the same axes
ax.plot(timestamps_saved, calculated_joint_angles[:, 0], 'r-', linewidth=2, label='Joint 1 (Base Joint)')
ax.plot(timestamps_saved, calculated_joint_angles[:, 1], 'g-', linewidth=2, label='Joint 2')
ax.plot(timestamps_saved, calculated_joint_angles[:, 2], 'b-', linewidth=2, label='Joint 3')
ax.plot(timestamps_saved, calculated_joint_angles[:, 3], 'm-', linewidth=2, label='Joint 4')

# Set labels and grid
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (deg)')
ax.grid(True)

# Add legend
ax.legend()

# Adjust layout
plt.tight_layout()
plt.show()