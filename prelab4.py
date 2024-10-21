import numpy as np
from TrajPlanner import TrajPlanner
import matplotlib.pyplot as plt


np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

setpoints = np.array([[15,-45,-60,90], [-90,15,30,-45]])
trajectories = TrajPlanner(setpoints)
cubic_traj = trajectories.get_cubic_traj(traj_t=5, n=6)

for i in range(0, len(cubic_traj)-1):
    fig, ax = plt.subplots(figsize=(10, 8))  # Create a single plot
    ax.plot(cubic_traj[4], cubic_traj[i], 'r-', linewidth=2)
    plt.show()