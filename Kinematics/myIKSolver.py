import numpy as np

class IKSolver:
    """
    Inverse Kinematics Solver
    :use: IKSolver.solve(target)
    """
    def __init__(self):
        self._q_result = np.zeros((6, 8))
        self._T_target = np.eye(4)
        self._T_Tool = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.06], [0, 0, 0, 1]])
        self._a2 = 0.185
        self._a3 = 0.17
        self._d1 = 0.23
        self._d2 = -0.054
        self._d3 = 0.0
        self._d4 = 0.077
        self._d5 = 0.077
        self._d6 = 0.0255

    def solve(self, target):
        """
        Inverse Kinematics Solver
        :param target: [x, y, z, rx, ry, rz] (Cartesian, meter; X-Y'-Z'Euler, rad)
        :return: Joint angle, 6 * n np.array (n = 0 - 8)
        """
        self._get_T_target(target)
        self._get_theta() # get theta
        return self._filter()

    def _get_T_target(self, target):
        self._T_target[:3, :3] = np.dot(np.dot(rotx(target[3]), roty(target[4])), rotz(target[5])) # rotation matrix
        self._T_target[:3, 3] = target[0:3].T # translation matrix
        self._T_target = np.dot(self._T_target, np.linalg.inv(self._T_Tool)) # get T_target

    def _filter(self):
        """
        Filter the result
        :return: Joint angle, 6 * n np.array (n = 0 - 8)
        """
        q_result = self._q_result
        index = np.where(np.abs(q_result[0, :]) < 200/180*np.pi)
        q_result = q_result[:, index[0]]
        index = np.where(np.abs(q_result[1, :]) < 90/180*np.pi)
        q_result = q_result[:, index[0]]
        index = np.where(np.abs(q_result[2, :]) < 120/180*np.pi)
        q_result = q_result[:, index[0]]
        index = np.where(np.abs(q_result[3, :]) < 150/180*np.pi)
        q_result = q_result[:, index[0]]
        index = np.where(np.abs(q_result[4, :]) < 150/180*np.pi)
        q_result = q_result[:, index[0]]
        index = np.where(np.abs(q_result[5, :]) < np.pi)
        q_result = q_result[:, index[0]]
        return q_result

    def _get_theta(self):
        r11, r12, r13, x_p = self._T_target[0, :]
        r21, r22, r23, y_p = self._T_target[1, :]
        rho_x = self._d6 * r23 - y_p
        rho_y = self._d6 * r13 - x_p

        # get theta_1 - situation 1
        theta_1 = np.arctan2(rho_x, rho_y) - np.arctan2(-(self._d2 + self._d4), np.sqrt(round(rho_x ** 2 + rho_y ** 2 - (self._d2 + self._d4) ** 2, 8)))
        if theta_1 > np.pi:
            theta_1 -= 2 * np.pi
        if theta_1 < -np.pi:
            theta_1 += 2 * np.pi
        self._q_result[0, :4] = theta_1

        # get theta_1 - situation 1
        theta_1 = np.arctan2(rho_x, rho_y) - np.arctan2(-(self._d2 + self._d4), -np.sqrt(round(rho_x ** 2 + rho_y ** 2 - (self._d2 + self._d4) ** 2, 8)))
        if theta_1 > np.pi:
            theta_1 -= 2 * np.pi
        if theta_1 < -np.pi:
            theta_1 += 2 * np.pi
        self._q_result[0, 4:8] = theta_1

        # get theta_5 - situation 1
        theta_5 = np.arcsin(-np.sin(self._q_result[(0, 0)]) * r13 + np.cos(self._q_result[(0, 0)]) * r23)
        self._q_result[(4, [0, 1])] = theta_5
        if theta_5 < 0:
            self._q_result[(4, [2, 3])] = -np.pi - self._q_result[(4, 0)]
        else:
            self._q_result[(4, [2, 3])] = np.pi - self._q_result[(4, 0)]

        # get theta_5 - situation 2
        theta_5 = np.arcsin(-np.sin(self._q_result[(0, 4)]) * r13 + np.cos(self._q_result[(0, 4)]) * r23)
        self._q_result[(4, [4, 5])] = theta_5
        if theta_5 < 0:
            self._q_result[(4, [6, 7])] = -np.pi - self._q_result[(4, 4)]
        else:
            self._q_result[(4, [6, 7])] = np.pi - self._q_result[(4, 4)]
    
        self._q_result[5, :] = np.arctan2(-(-np.sin(self._q_result[0, :]) * r12 + np.cos(self._q_result[0, :]) * r22) * np.cos(self._q_result[4, :]), (-np.sin(self._q_result[0, :]) * r11 + np.cos(self._q_result[0, :]) * r21) * np.cos(self._q_result[4, :]))

        for i in range(4): # 4 situations
            T01 = np.array([[np.cos(self._q_result[(0, 2 * i)]), -np.sin(self._q_result[(0, 2 * i)]), 0, 0],
             [np.sin(self._q_result[(0, 2 * i)]), np.cos(self._q_result[(0, 2 * i)]), 0, 0],
             [0, 0, 1, self._d1],
             [0, 0, 0, 1]])
            T45 = np.array([[-np.sin(self._q_result[(4, 2 * i)]), -np.cos(self._q_result[(4, 2 * i)]), 0, 0],
             [0, 0, -1, -self._d5],
             [np.cos(self._q_result[(4, 2 * i)]), -np.sin(self._q_result[(4, 2 * i)]), 0, 0],
             [0, 0, 0, 1]])
            T56 = np.array([[np.cos(self._q_result[(5, 2 * i)]), -np.sin(self._q_result[(5, 2 * i)]), 0, 0],
             [0, 0, -1, -self._d6],
             [np.sin(self._q_result[(5, 2 * i)]), np.cos(self._q_result[(5, 2 * i)]), 0, 0],
             [0, 0, 0, 1]])

            # calculate T14
            T14 = np.dot(np.linalg.inv(T01), self._T_target)
            T14 = np.dot(T14, np.linalg.inv(T56))
            T14 = np.dot(T14, np.linalg.inv(T45))

            x_p = T14[(0, 3)]
            y_p = T14[(2, 3)]
            # get theta_3
            cos_theta_3 = (x_p ** 2 + y_p ** 2 - self._a2 ** 2 - self._a3 ** 2) / (2 * self._a2 * self._a3)
            if np.abs(cos_theta_3) > 1: # the limit by cos()
                self._q_result[:, 2 * i] = 100
                self._q_result[:, 2 * i + 1] = 100
            else:
                theta_3 = np.arccos(cos_theta_3)                   
                self._q_result[(2, 2 * i)] = theta_3
                self._q_result[(2, 2 * i + 1)] = -theta_3

                # get theta_4
                tmp1 = self._a2 + self._a3 * np.cos(self._q_result[(2, [2 * i, 2 * i + 1])])
                tmp2 = self._a3 * np.sin(self._q_result[(2, [2 * i, 2 * i + 1])])
                self._q_result[(1, [2 * i, 2 * i + 1])] = np.arctan2(tmp1 * x_p - tmp2 * y_p, tmp2 * x_p + tmp1 * y_p)

                theta_4 = np.arctan2(-T14[(0, 1)], T14[(0, 0)]) - self._q_result[(1, [2 * i, 2 * i + 1])] - self._q_result[(2, [2 * i, 2 * i + 1])]
                for j in range(2):
                    if theta_4[j] < -np.pi:
                        theta_4[j] += 2 * np.pi
                    if theta_4[j] > np.pi:
                        theta_4[j] -= 2 * np.pi

                self._q_result[(3, [2 * i, 2 * i + 1])] = theta_4

def rotx(x):
    """
    calculate the rotation matrix about x axis
    :param x: angle in radians
    """
    return np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])

def roty(y):
    """
    calculate the rotation matrix about y axis
    :param y: angle in radians
    """
    return np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])

def rotz(z):
    """
    calculate the rotation matrix about z axis
    :param z: angle in radians
    """
    return np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
