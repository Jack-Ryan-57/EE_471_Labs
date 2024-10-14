import numpy as np

class IK_Robot_Dummy:
    def __init__(self):
        # Physical constants
        self.L1 = 0.077
        self.L2 = 0.130
        self.L3 = 0.124
        self.L4 = 0.126
        self.l_21 = 0.128
        self.l_22 = 0.024

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
    
if __name__ == '__main__':
    robot = IK_Robot_Dummy()
    cases = [[274, 0, 205, 0] ,[16, 4, 336, 15], [0, -270, 106, 0]]

    for case in cases:
        print(robot.get_ik(case))