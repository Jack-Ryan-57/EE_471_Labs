import numpy as np

class Test_IK:
    def __init__(self):
        # physical constants from previous lab
        self.L1 = 0.077
        self.L2 = 0.130
        self.L3 = 0.124
        self.L4 = 0.126
        self.l_21 = 0.128
        self.l_22 = 0.024

    def get_ik(self, pose: np.array) -> np.array:
        x, y, z, pitch = pose                                       # deconstruct pose into individual variables
        pose = [x/1000, y/1000, z/1000, np.deg2rad(pitch)]          # convert pose variables to meters and radians

        x, y, z, pitch = pose                                       # deconstruct pose into individual variables
        r = np.sqrt(np.square(x) + np.square(y))                    # r = sqrt(x^2 + y^2)
        r_w = r - self.L4*np.cos(pitch)                             # rw = r - L4*cos(alpha)
        z_w  = z - self.L1 - self.L4*np.sin(pitch)                  # zw = z - L1 - L4*sin(alpha)
        d_w = np.sqrt(np.square(r_w) + np.square(z_w))              # dw = sqrt(rw^2 + zw^2)
        mu = np.atan2(z_w, r_w)                                     # mu = arctan(zw/rw)

        # solve for two solutions for beta: cos(beta) = (L2^2 + L3^2 - dw^2)/(2*L2*L3)
        cos_beta = (np.square(self.L2) + np.square(self.L3) - np.square(d_w))/(2*self.L2*self.L3)
        sin_beta = np.sqrt(1-np.square(cos_beta))
        beta1 = np.atan2(sin_beta, cos_beta)
        beta2 = np.atan2(-sin_beta, cos_beta)

        # solve for two solutions for gamma: cos(gamma) = (dw^2 + L2^2 - L3^2)/(2*dw*L2)
        cos_gamma = (d_w*d_w + self.L2*self.L2 - self.L3*self.L3)/(2*d_w*self.L2)
        sin_gamma = np.sqrt(1-cos_gamma*cos_gamma)
        gamma1 = np.atan2(sin_gamma, cos_gamma)
        gamma2 = np.atan2(-sin_gamma, cos_gamma)

        delta = np.atan2(self.l_22, self.l_21)                      # tan(delta) = l22/l21
        theta1 = np.atan2(y, x)                                     # theta1 = arctan(y/x)
        theta2 = np.pi/2 - delta - gamma2 - mu                      # theta2 = pi/2 - delta - gamma - mu
        theta3 = np.pi/2 + delta - beta2                            # theta3 = pi/2 + delta - beta
        theta4 = -pitch - theta2 - theta3                           # theta4 = -alpha - theta2 - theta3

        joint_angles = [theta1, theta2, theta3, theta4]             # only use the case where the robot elbow is "up"

        return np.rad2deg(joint_angles)
    

# main method   
if __name__ == '__main__':
    rb = Test_IK()
    test_cases = [[274, 0, 205, 0] ,[16, 4, 336, 15], [0, -270, 106, 0]]        # run each test case

    for each in test_cases:
        print(rb.get_ik(each))