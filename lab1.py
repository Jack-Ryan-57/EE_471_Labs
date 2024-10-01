import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot
import time


# Setup robot
traj_time = 2  # Defines the trajectory time
robot = Robot()  # Creates robot object

robot.write_time(traj_time)  # Write trajectory time
robot.write_motor_state(True)  # Write position mode

# Program
robot.write_joints([0, 0, 0, 0])  # Write joints to zero position
time.sleep(traj_time)  # Wait for trajectory completion

traj_time = 10
robot.write_time(traj_time)
robot.write_joints([45, 0, 0, 0])  # Write joints to zero position

# Initialize lists to store timestamps and joint positions
timestamps = []
joint_positions = []

# Loop to collect joint positions and timestamps every half second for the 10-second duration
start_time = time.time()
while time.time() - start_time < traj_time:
    current_time = time.time() - start_time  # Calculate the time since the start of trajectory
    readings = robot.get_joints_readings()  # Get joint readings
    joint_pos = readings[0, :]  # Extract only the joint positions (first row of readings)
    
    # Append current timestamp and joint positions to their respective lists
    timestamps.append(current_time)
    joint_positions.append(joint_pos)
    
    #time.sleep(0.5)  # Wait for half a second

# Convert the collected data into numpy arrays
timestamps_np = np.array(timestamps)
joint_positions_np = np.array(joint_positions)

# Print the collected data
print("Timestamps:", timestamps_np)
print("Joint Positions (in degrees):")
print(np.round(joint_positions_np, 2))  # Print joint positions with 2 decimal places

# Plotting the data using Matplotlib
fig, axs = plt.subplots(4, 1, figsize=(8, 12))

# Plot for Joint 1 (Base Joint)
axs[0].plot(timestamps_np, joint_positions_np[:, 0], 'r-', linewidth=2)
axs[0].set_title('Joint 1 (Base Joint)')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Position (deg)')
axs[0].grid(True)

# Plot for Joint 2
axs[1].plot(timestamps_np, joint_positions_np[:, 1], 'g-', linewidth=2)
axs[1].set_title('Joint 2')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Position (deg)')
axs[1].grid(True)

# Plot for Joint 3
axs[2].plot(timestamps_np, joint_positions_np[:, 2], 'b-', linewidth=2)
axs[2].set_title('Joint 3')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Position (deg)')
axs[2].grid(True)

# Plot for Joint 4
axs[3].plot(timestamps_np, joint_positions_np[:, 3], 'm-', linewidth=2)
axs[3].set_title('Joint 4')
axs[3].set_xlabel('Time (s)')
axs[3].set_ylabel('Position (deg)')
axs[3].grid(True)

# Adjust layout for better appearance
plt.tight_layout()

# Set global title
plt.suptitle('Joint Motion Profiles', y=1.02)

# Show the plot
plt.show()