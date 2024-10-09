import PickleFiler

filename = 'robot_movement_data.pkl'

# Sample data to save
data_to_save = {
    'joint_angles': joint_angles,
    'ee_positions': ee_positions,
    'timestamps': timestamps
}

PickleFiler.save_to_pickle(data_to_save, filename)

loaded_data = PickleFiler.load_from_pickle(filename)
print("Data loaded from: ", filename)
joint_angles_saved = loaded_data['joint_angles']
ee_positions_saved = loaded_data['ee_positions']
timestamps_saved = loaded_data['timestamps']
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