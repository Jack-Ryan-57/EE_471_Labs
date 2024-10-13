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
traj_time = 2  # Defines the trajectory time
robot.write_time(traj_time)  # Write trajectory time
robot.write_motor_state(True)  # Write position mode
robot.write_joints([0, 0, 0, 0])  # Write joints to zero position
time.sleep(traj_time)  # Wait for trajectory completion

# Setup trajectory to be mesaured
traj_time = 10
robot.write_time(traj_time)
robot.write_joints([45, 0, 0, 0])  # Write joints to zero position

# Lists to store timestamps and joint readings
timestamps = []
joint_positions = []

start_time = time.time()
while time.time() - start_time < traj_time: # Continue till the trajectory time has been reached

    joint_readings = robot.get_joints_readings()  # Fetch current joint readings

    timestamps.append(time.time())  # Save current time stamp
    joint_positions.append(joint_readings[0, :])  # Record joint positions (first row of readings)

# Post production:

# Convert collected data to NumPy arrays
joint_positions_np = np.array(joint_positions)
time_intervals_np = np.diff(np.array(timestamps))   #diff timestamps to get intervals

# Stats
mean_interval = np.mean(time_intervals_np)
median_interval = np.median(time_intervals_np)
max_interval = np.max(time_intervals_np)
min_interval = np.min(time_intervals_np)
std_dev_interval = np.std(time_intervals_np)

# Print stats
print("\nTime Intervals Stats:")
print(f"Mean Interval: {mean_interval:.4f} seconds")
print(f"Median Interval: {median_interval:.4f} seconds")
print(f"Maximum Interval: {max_interval:.4f} seconds")
print(f"Minimum Interval: {min_interval:.4f} seconds")
print(f"Standard Deviation: {std_dev_interval:.4f} seconds")

# Plotting the joint positions using Matplotlib
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
axs = axs.flatten() 

# Plot Joint 1
axs[0].plot(time_intervals_np, joint_positions_np[:, 0], 'r-', linewidth=2)
axs[0].set_title('Joint 1 (Base Joint)')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Position (deg)')
axs[0].grid(True)

# Plot Joint 2
axs[1].plot(time_intervals_np, joint_positions_np[:, 1], 'g-', linewidth=2)
axs[1].set_title('Joint 2')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Position (deg)')
axs[1].grid(True)

# Plot Joint 3
axs[2].plot(time_intervals_np, joint_positions_np[:, 2], 'b-', linewidth=2)
axs[2].set_title('Joint 3')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Position (deg)')
axs[2].grid(True)

# Plot Joint 4
axs[3].plot(time_intervals_np, joint_positions_np[:, 3], 'm-', linewidth=2)
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

# Plotting the histogram
plt.figure(figsize=(8, 6))
plt.hist(time_intervals_np, bins=30, color='c', edgecolor='black')
plt.title('Histogram of Time Intervals Between Readings')
plt.xlabel('Time Interval (seconds)')
plt.ylabel('Frequency')
plt.xlim(0.047,0.049)  # Set x limits
plt.grid(True)
plt.show()
