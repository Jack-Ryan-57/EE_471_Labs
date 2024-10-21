import numpy as np

class TrajPlanner:
    """
    Trajectory Planner class for calculating trajectories for different polynomial orders and relevant coefficients.
    """

    def __init__(self, setpoints):
        """
        Initialize the TrajPlanner class.

        Parameters:
        setpoints (numpy array): List of setpoints to travel to.
        """
        self.setpoints = setpoints

    ## Implement the required methods below. ##

    '''
    Inputs: 
        t_0 - Initial time in seconds
        t_f - Final time in seconds
        q_0 - Initial desired position
        q_f - Final desired position
        v_0 - Initial desired velocity
        v_f - Final desired velocity
    Outputs: 
        (1x4) numpy array containing cubic coefficients necessary to obtain desired cubic shape
    Description:
        Calculates necessary coefficients to generate desired cubic trajectory
    '''
    def calc_cubic_coeff(self, t_0, t_f, q_0, q_f, v_0, v_f):
        A = np.array([
            [1, t_0, np.pow(t_0, 2), np.pow(t_0, 3)],
            [0, 1, 2*t_0, 3*np.pow(t_0, 2)],
            [1, t_f, np.pow(t_f, 2), np.pow(t_f, 3)],
            [0, 1, 2*t_f, 3*np.pow(t_f, 2)]
        ])
        q = np.array([q_0, v_0, q_f, v_f]).transpose()      # Desired states
        return np.matmul(np.linalg.inv(A), q)       # Find coefficients using inv(A)*q
    
    '''
    Inputs: 
        traj_t - Trajectory time in seconds
        coeffs - Coefficients for cubic polynomial
    Ouputs:    
        (n+1)x1 numpy array with all waypoints for single desired trajectory
    Description: 
        Calculates the waypoints for one setpoint dimension for a cubic trajectory
    '''
    def calc_cubic_traj(self, traj_t, n: int, coeffs: np.array):
        interval = traj_t/(n+1)     # Time between two points
        ctr = 0
        # Populate list of time values
        time_list = [ctr]
        for i in range(0, n):
            ctr += interval
            time_list.append(ctr)
        time_list.append(traj_t)
        # Populate array to be returned
        ret_list = []
        for i in range(0, len(time_list)):
            t = time_list[i]
            # a_0 + a_1*t + a_2*t^2 + a_3*t^3
            ret_list.append(coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*np.pow(t, 3))
        return np.array(ret_list)
    
    '''
    Inputs: 
        traj_t - Trajectory time in seconds
        n - number of intermediate points
    Outputs: 
        (n+2)x5 numpy array with trajectories for all 4 joints plus times
    Description: Generates cubic trajectories for motors to match setpoints as defined in class constructor.
        Returns generated trajectories for every set point given as well as times. 
    '''
    def get_cubic_traj(self, traj_t, n):
        q_old = self.setpoints[0]
        q_new = self.setpoints[1]
        ret_arr = []
        # For each joint in the jointspace...
        for i in range(len(self.setpoints[0])):
            # Calculate coefficients for current joint
            coeffs = self.calc_cubic_coeff(0, traj_t, q_old[i], q_new[i], 0, 0)
            print(f'Coeffs for joint {i}: {coeffs}')
            # Calculate cubic trajectory for current joint
            trajectories = self.calc_cubic_traj(traj_t, n, coeffs)
            ret_arr.append(trajectories)
        # Populate list of time values to be returned
        ctr = 0
        interval = traj_t/(n+1)     # Time between two points
        time_list = [ctr]
        for i in range(0, n):
            ctr += interval
            time_list.append(ctr)
        time_list.append(traj_t)
        ret_arr.append(time_list)
        return np.array(ret_arr)




# # Usage Example
# import numpy as np
# from traj_planner import TrajPlanner

# # Define setpoints (example values)
# setpoints = np.array([
#     [15, -45, -60, 90],
#     [-90, 15, 30, -45]
# ])

# # Create a TrajPlanner object
# trajectories = TrajPlanner(setpoints)

# # Generate cubic trajectory
# cubic_traj = trajectories.get_cubic_traj(traj_time=5, points_num=10)
# print(cubic_traj)
