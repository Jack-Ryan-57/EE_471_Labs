# (c) 2024 S. Farzan, Electrical Engineering Department, Cal Poly
# Skeleton Robot class for OpenManipulator-X Robot for EE 471

import numpy as np
from OM_X_arm import OM_X_arm
from DX_XM430_W350 import DX_XM430_W350

"""
Robot class for controlling the OpenManipulator-X Robot.
Inherits from OM_X_arm and provides methods specific to the robot's operation.
"""
class Robot(OM_X_arm):
    """
    Initialize the Robot class.
    Creates constants and connects via serial. Sets default mode and state.
    """
    def __init__(self):
        super().__init__()

        # Robot Dimensions (in mm)
        self.mDim = [77, 130, 124, 126]
        self.mOtherDim = [128, 24]
        
        # Set default mode and state
        # Change robot to position mode with torque enabled by default
        # Feel free to change this as desired
        self.write_mode('position')
        self.write_motor_state(True)

        # Set the robot to move between positions with a 5 second trajectory profile
        # change here or call writeTime in scripts to change
        self.write_time(5)

        # Physical constants
        self.L1 = 0.077
        self.L2 = 0.130
        self.L3 = 0.124
        self.L4 = 0.126
        self.l_21 = 0.128
        self.l_22 = 0.024
        self.angle_offset = np.pi/2-np.arcsin(24/130)

    """
    Sends the joints to the desired angles.
    Parameters:
    goals (list of 1x4 float): Angles (degrees) for each of the joints to go to.
    """
    def write_joints(self, goals):
        goals = [round(goal * DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET) % DX_XM430_W350.TICKS_PER_ROT for goal in goals]
        self.bulk_read_write(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals)

    '''
    Input: a 1 × 4 array corresponding to a row of the DH parameter table for a given joint.
    Return: a 4 × 4 numpy array representing the homogeneous transformation matrix Ai .
    Description: Calculates the intermediate transformation Ai for a DH parameter table row
    '''
    def get_dh_row_mat(self, in_arr: np.array) -> np.array:
        return np.array(
               [[np.cos(in_arr[0]),  -np.sin(in_arr[0])*np.cos(in_arr[3]),   np.sin(in_arr[0])*np.sin(in_arr[3]),    in_arr[2]*np.cos(in_arr[0])],
                [np.sin(in_arr[0]),  np.cos(in_arr[0])*np.cos(in_arr[3]),    -np.cos(in_arr[0])*np.sin(in_arr[3]),   in_arr[2]*np.sin(in_arr[0])],
                [0,                  np.sin(in_arr[3]),                      np.cos(in_arr[3]),                      in_arr[1]],
                [0,                  0,                                      0,                                      1]]
                )
    
    
    '''
    Input: a 1 × 4 numpy array specifying the joint angles.
    Return: a 4 × 4 × 4 numpy array of A matrices for specified joint angles (A1; A2; A3; A4)
    Description: Utilizing get_dh_row_mat(), this method uses the DH table and joint angles to calculate the intermediate transformation Ai for all rows at specific joint angles.
    '''
    def get_int_mat(self, motor_angles: np.array) -> np.array:
        return np.array(
            [
                self.get_dh_row_mat([motor_angles[0],                       self.L1,    0,          -np.pi/2]),
                self.get_dh_row_mat([motor_angles[1] - self.angle_offset,   0,          self.L2,    0       ]),
                self.get_dh_row_mat([motor_angles[2] + self.angle_offset,   0,          self.L3,    0       ]),
                self.get_dh_row_mat([motor_angles[3],                       0,          self.L4,    0       ])
            ]
        )
    
    '''
    Input: a 1 × 4 numpy array specifying the joint angles.
    Return: a 4 × 4 × 4 numpy array of T matrices (transforms from joint i to the base) for specified joint angles
    Description: Using the A matrices from get_int_mat(), this method calculates the accumulative transformations for all joints at specific joint angles
    '''
    def get_acc_mat(self, joint_angles: np.array) -> np.array:
        a = self.get_int_mat(joint_angles)
        a1 = a[0]
        a2 = np.matmul(a1, a[1])
        a3 = np.matmul(a2, a[2])
        a4 = np.matmul(a3, a[3])
        return np.array([a1, a2, a3, a4])
    
    '''
    Input: a 1 × 4 numpy array specifying the joint angles.
    Return: a 4 × 4 numpy array representing the end-effector to base transformation
    Description: Using the A matrices from get_int_mat(), this method calculates the forward kinematics of the robot as a 4 × 4 homogeneous transformation matrix representing the position and orientation of the end-effector frame with respect to the base frame.
    '''
    def get_fk(self, joint_angles: np.array) -> np.array:
        return self.get_acc_mat(joint_angles)[3]
    
    '''
    Input: a 1 x 4 array with desired x, y, z, and alpha for end-effector
    Return: a 4 x 4 array of motor angles for given end-effector pose
    Description: Performs inverse kinematics to calculate motor angles needed to place end-effector in given pose
    '''
    def get_ik(self, pose: np.array) -> np.array:
        pose = [pose[0]/1000, pose[1]/1000, pose[2]/1000, np.deg2rad(pose[3])]
        r = np.sqrt(pose[0]*pose[0] + pose[1]*pose[1])
        r_w = r - self.L4*np.cos(pose[3])
        z_w  = pose[2] - self.L1 - self.L4*np.sin(pose[3])
        d_w = np.sqrt(r_w*r_w + z_w*z_w)
        mu = np.atan2(z_w, r_w)
        cos_beta = (self.L2*self.L2 + self.L3*self.L3 - d_w*d_w)/(2*self.L2*self.L3)
        sin_beta = np.sqrt(1-cos_beta*cos_beta)
        beta1 = np.atan2(sin_beta, cos_beta)
        beta2 = np.atan2(-sin_beta, cos_beta)
        cos_gamma = (d_w*d_w + self.L2*self.L2 - self.L3*self.L3)/(2*d_w*self.L2)
        sin_gamma = np.sqrt(1-cos_gamma*cos_gamma)
        gamma1 = np.atan2(sin_gamma, cos_gamma)
        gamma2 = np.atan2(-sin_gamma, cos_gamma)
        delta = np.atan2(self.l_22, self.l_21)
        theta_2 = np.pi/2 - delta - gamma1 - mu
        theta_3 = np.pi/2 + delta - beta1
        elbow_up = [np.atan2(pose[1], pose[0]), theta_2, theta_3, -pose[3] - theta_2 - theta_3]
        theta_2 = np.pi/2 - delta - gamma2 - mu
        theta_3 = np.pi/2 + delta - beta2
        elbow_down = [np.atan2(pose[1], pose[0]), theta_2, theta_3, -pose[3] - theta_2 - theta_3]
        return np.rad2deg(elbow_up)
    
    '''
    Inputs: None
    Return: 4 × 4 numpy array representing the end-effector to base transformation for current joint angles
–   Description: Retrieves the last read joint angles using get_joints_readings() and calculates the end-effector to base transformation using get_fk()
    '''
    def get_current_fk(self) -> np.array:
        joint_angles = self.get_joints_readings()[0]
        return self.get_fk(np.radians(joint_angles))
    

    '''
    Input: a 1 × 4 numpy array specifying the joint angles
    Output: a 1 × 5 numpy array containing the end-effector position (x, y , z in mm) and orientation (pitch and yaw in degrees)
    Description: Calculates the end-effector position and orientation for given joint angles. It returns the x, y , z coordinates in millimeters and the pitch & yaw in degrees with respect to the base frame.
    '''
    def get_ee_pos(self, joint_angles: np.array) -> np.array:
        yaw = joint_angles[0]
        pitch = -joint_angles[1] - joint_angles[2] - joint_angles[3]
        fk = self.get_fk(np.radians(joint_angles))
        return np.array([fk[0, 3], fk[1, 3], fk[2, 3], pitch, yaw])

    """
    Creates a time-based profile (trapezoidal) based on the desired times.
    This will cause write_position to take the desired number of seconds to reach the setpoint.
    Parameters:
    time (float): Total profile time in seconds. If 0, the profile will be disabled (be extra careful).
    acc_time (float, optional): Total acceleration time for ramp up and ramp down (individually, not combined). Defaults to time/3.
    """
    def write_time(self, time, acc_time=None):
        if acc_time is None:
            acc_time = time / 3

        time_ms = int(time * DX_XM430_W350.MS_PER_S)
        acc_time_ms = int(acc_time * DX_XM430_W350.MS_PER_S)

        self.bulk_read_write(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, [acc_time_ms]*self.motorsNum)
        self.bulk_read_write(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, [time_ms]*self.motorsNum)

    """
    Sets the gripper to be open or closed.
    Parameters:
    open (bool): True to set the gripper to open, False to close.
    """
    def write_gripper(self, open):
        if open:
            self.gripper.write_position(-45)
        else:
            self.gripper.write_position(45)

    def read_gripper(self):
        if open:
            pos = self.gripper.read_position()
        else:
            pos = self.gripper.read_position()
        return pos

    """
    Sets position holding for the joints on or off.
    Parameters:
    enable (bool): True to enable torque to hold the last set position for all joints, False to disable.
    """
    def write_motor_state(self, enable):
        state = 1 if enable else 0
        states = [state] * self.motorsNum  # Repeat the state for each motor
        self.bulk_read_write(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, states)

    """
    Supplies the joints with the desired currents.
    Parameters:
    currents (list of 1x4 float): Currents (mA) for each of the joints to be supplied.
    """
    def write_currents(self, currents):
        current_in_ticks = [round(current * DX_XM430_W350.TICKS_PER_mA) for current in currents]
        self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, current_in_ticks)

    """
    Change the operating mode for all joints.
    Parameters:
    mode (str): New operating mode for all joints. Options include:
        "current": Current Control Mode (writeCurrent)
        "velocity": Velocity Control Mode (writeVelocity)
        "position": Position Control Mode (writePosition)
        "ext position": Extended Position Control Mode
        "curr position": Current-based Position Control Mode
        "pwm voltage": PWM Control Mode
    """
    def write_mode(self, mode):
        if mode in ['current', 'c']:
            write_mode = DX_XM430_W350.CURR_CNTR_MD
        elif mode in ['velocity', 'v']:
            write_mode = DX_XM430_W350.VEL_CNTR_MD
        elif mode in ['position', 'p']:
            write_mode = DX_XM430_W350.POS_CNTR_MD
        elif mode in ['ext position', 'ep']:
            write_mode = DX_XM430_W350.EXT_POS_CNTR_MD
        elif mode in ['curr position', 'cp']:
            write_mode = DX_XM430_W350.CURR_POS_CNTR_MD
        elif mode in ['pwm voltage', 'pwm']:
            write_mode = DX_XM430_W350.PWM_CNTR_MD
        else:
            raise ValueError(f"writeMode input cannot be '{mode}'. See implementation in DX_XM430_W350 class.")

        self.write_motor_state(False)
        write_modes = [write_mode] * self.motorsNum  # Create a list with the mode value for each motor
        self.bulk_read_write(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, write_modes)
        self.write_motor_state(True)

    """
    Gets the current joint positions, velocities, and currents.
    Returns:
    numpy.ndarray: A 3x4 array containing the joints' positions (deg), velocities (deg/s), and currents (mA).
    """
    def get_joints_readings(self):
        readings = np.zeros((3, 4))
        
        positions = np.array(self.bulk_read_write(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION))
        velocities = np.array(self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY))
        currents = np.array(self.bulk_read_write(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT))

        # Take two's complement of velocity and current data
        for i in range(4):
            if velocities[i] > 0x7fffffff:
                velocities[i] = velocities[i] - 4294967296
            if currents[i] > 0x7fff:
                currents[i] = currents[i] - 65536

        readings[0, :] = (positions - DX_XM430_W350.TICK_POS_OFFSET) / DX_XM430_W350.TICKS_PER_DEG
        readings[1, :] = velocities / DX_XM430_W350.TICKS_PER_ANGVEL
        readings[2, :] = currents / DX_XM430_W350.TICKS_PER_mA

        return readings

    """
    Sends the joints to the desired velocities.
    Parameters:
    vels (list of 1x4 float): Angular velocities (deg/s) for each of the joints to go at.
    """
    def write_velocities(self, vels):
        vels = [round(vel * DX_XM430_W350.TICKS_PER_ANGVEL) for vel in vels]
        self.bulk_read_write(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels)
