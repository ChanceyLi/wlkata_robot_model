import numpy as np
import math
from mirobot import Mirobot

a1, a2, a3, b1, b4, b6 = 32, 108, 20, 80, 176, 20  # 机器人DH参数
upper = [100, 60, 50, 90, 40, 90]  # 机器人角度限制
lower = [-100, -30, -50, -90, -90, -90]


def __the2_inv(x, y, z, theta1, xi):
    s = z - b1
    r = -math.sqrt((x - a1 * math.cos(theta1)) ** 2 + (y - a1 * math.sin(theta1)) ** 2)
    Omega = math.atan2(s, r)
    lambda_ = math.atan2(math.sqrt(b4 ** 2 + a3 ** 2) * math.sin(xi),
                         a2 + math.sqrt(b4 ** 2 + a3 ** 2) * math.cos(xi))
    return -(Omega - lambda_)


def the2_o(x, y, z, theta1, xi):
    s = z - b1
    r = math.sqrt((x - a1 * math.cos(theta1)) ** 2 + (y - a1 * math.sin(theta1)) ** 2)
    Omega = math.atan2(s, r)
    lambda_ = math.atan2(math.sqrt(b4 ** 2 + a3 ** 2) * math.sin(xi),
                         a2 + math.sqrt(b4 ** 2 + a3 ** 2) * math.cos(xi))
    return -(Omega - lambda_)


def my_loss(a):
    if -1e-3 <= a <= 1e-3:
        return 0
    return a


class Motivation:

    size = 6

    def __init__(self):
        self.__theta = [0.0, 0, 0, 0, 0, 0]  # 角度
        self.__Theta = [0.0, 0, 0, 0, 0, 0]  # 弧度
        self.__position = [0.0, 0, 0, 0, 0, 0]
        self.__robot = None
        self.__T = 0
        self.__J = 0

    def set_angle(self, theta):  # 更新角度
        self.__theta = theta
        self.__to_radian()
        self.size = len(self.__Theta)
        is_legal = self.is_legal_angle()
        self.get_axis_with_angle()
        return is_legal

    def set_radian(self, Theta):  # 更新弧度
        self.__Theta = Theta
        self.__to_angle()
        self.size = len(self.__Theta)
        is_legal = self.is_legal_angle()
        self.get_axis_with_angle()
        return is_legal

    def set_position(self, position):
        T = np.matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
        Rx = np.matrix([[1, 0, 0], [0, math.cos(position[3]), -math.sin(position[3])],
                        [0, math.sin(position[3]), math.cos(position[3])]])
        Ry = np.matrix([[math.cos(position[4]), 0, math.sin(position[4])], [0, 1, 0],
                        [-math.sin(position[4]), 0, math.cos(position[4])]])
        Rz = np.matrix([[math.cos(position[5]), -math.sin(position[5]), 0],
                        [math.sin(position[5]), math.cos(position[5]), 0], [0, 0, 1]])
        T[:3, :3] = Rz @ Ry @ Rx
        T[0, 3] = position[0]
        T[1, 3] = position[1]
        T[2, 3] = position[2]
        self.__T = T
        self.inverse_kinematics(T)
        is_legal = self.is_legal_angle()
        return is_legal

    def get_axis_with_angle(self):  # 正运动学得到位置
        T01 = np.matrix([[math.cos(self.__Theta[0]), -math.sin(self.__Theta[0]), 0, 0],
                         [math.sin(self.__Theta[0]), math.cos(self.__Theta[0]), 0, 0],
                         [0, 0, 1, b1],
                         [0, 0, 0, 1]])
        T12 = np.matrix([[math.sin(self.__Theta[1]), math.cos(self.__Theta[1]), 0, a1],
                         [0, 0, 1, 0],
                         [math.cos(self.__Theta[1]), -math.sin(self.__Theta[1]), 0, 0],
                         [0, 0, 0, 1]])
        T23 = np.matrix([[math.cos(self.__Theta[2]), -math.sin(self.__Theta[2]), 0, a2],
                         [math.sin(self.__Theta[2]), math.cos(self.__Theta[2]), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        T34 = np.matrix([[math.cos(self.__Theta[3]), -math.sin(self.__Theta[3]), 0, a3],
                         [0, 0, 1, b4],
                         [-math.sin(self.__Theta[3]), -math.cos(self.__Theta[3]), 0, 0],
                         [0, 0, 0, 1]])
        T45 = np.matrix([[-math.sin(self.__Theta[4]), -math.cos(self.__Theta[4]), 0, 0],
                         [0, 0, -1, 0],
                         [math.cos(self.__Theta[4]), -math.sin(self.__Theta[4]), 0, 0],
                         [0, 0, 0, 1]])
        T56 = np.matrix([[math.cos(self.__Theta[5]), -math.sin(self.__Theta[5]), 0, 0],
                         [0, 0, -1, b6],
                         [math.sin(self.__Theta[5]), math.cos(self.__Theta[5]), 0, 0],
                         [0, 0, 0, 1]])
        T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
        self.__T = T06

        beta = math.atan2(-T06[2, 0], math.sqrt(T06[0, 0] ** 2 + T06[1, 0] ** 2))
        if abs(beta - math.pi / 2) < 0.001:
            alpha = math.atan2(T06[0, 1], T06[1, 1])
            gamma = 0
        elif abs(beta + math.pi / 2) < 0.001:
            alpha = -math.atan2(T06[0, 1], T06[1, 1])
            gamma = 0
        else:
            alpha = math.atan2(T06[2, 1] / math.cos(beta), T06[2, 2] / math.cos(beta))
            gamma = math.atan2(T06[1, 0] / math.cos(beta), T06[0, 0] / math.cos(beta))
        self.__position = [T06[0, 3], T06[1, 3], T06[2, 3], alpha, beta, gamma]

    def get_position(self):
        return self.__position

    def get_matrix(self):
        return self.__T

    def get_angle(self):
        return self.__theta

    def get_radian(self):
        return self.__Theta

    def get_jacobi(self):
        self.jacobi()
        return self.__J

    def get_robot_status(self):
        return self.__robot

    def __to_radian(self):  # 角度变弧度
        for i in range(len(self.__theta)):
            self.__Theta[i] = self.__theta[i] * math.pi / 180

    def __to_angle(self):  # 弧度变角度
        for i in range(len(self.__Theta)):
            self.__theta[i] = self.__Theta[i] * 180 / math.pi

    def is_legal_angle(self):  # 角度合法性验证
        is_legal = [0, 0, 0, 0, 0, 0]
        for i in range(len(self.__Theta)):
            if self.__theta[i] > upper[i]:
                self.__Theta[i] = upper[i] * math.pi / 180
                self.__theta[i] = upper[i]
                print("theta" + str(i + 1) + " outbound!")
                is_legal[i] = 1
            if self.__theta[i] < lower[i]:
                self.__Theta[i] = lower[i] * math.pi / 180
                self.__theta[i] = lower[i]
                print("theta" + str(i + 1) + " outbound!")
                is_legal[i] = 1
        return is_legal

    def robot_initial(self):  # 初始化机器人
        ret = 0
        if not self.__robot:
            try:
                self.__robot = Mirobot(portname='COM4', debug=False)
                self.__robot.home_simultaneous(wait=True)
            except:
                ret = -1
        return ret

    def run_robot(self):  # 更改机器人到当前角度
        target_angles = {1: self.__theta[0], 2: self.__theta[1], 3: self.__theta[2],
                         4: self.__theta[3], 5: self.__theta[4], 6: self.__theta[5]}
        self.__robot.set_joint_angle(target_angles, wait=True)

    def draw_line(self, lines):
        pass

    def get_status(self):
        return self.__robot.get_status()

    def inverse_kinematics(self, T):
        xi1 = 0
        no_solution = 1000
        omega_x, omega_y, omega_z = T[0, 3] + b6 * T[0, 2], T[1, 3] + b6 * T[1, 2], T[2, 3] + b6 * T[2, 2]
        theta1 = math.atan2(omega_y, omega_x)
        # theta11 = math.pi + theta1
        S = omega_z - b1
        R = math.sqrt((omega_x - a1 * math.cos(theta1)) ** 2 + (omega_y - a1 * math.sin(theta1)) ** 2)
        cos_xi = (R ** 2 + S ** 2 - a2 ** 2 - b4 ** 2 - a3 ** 2) / (2 * a2 * math.sqrt(b4 ** 2 + a3 ** 2))
        if abs(cos_xi) <= 1:
            sin_xi = math.sqrt(1 - cos_xi ** 2)
            sin_xi1 = -sin_xi
            # xi = math.atan2(sin_xi, cos_xi)
            xi1 = math.atan2(sin_xi1, cos_xi)

            # theta3 = -(xi + math.atan2(b4, a3))
            theta33 = - (xi1 + math.atan2(b4, a3))
        else:
            # theta3 = no_solution
            theta33 = no_solution

        # S = omega_z - b1
        # R = math.sqrt((omega_x - a1 * math.cos(theta11)) ** 2 + (omega_y - a1 * math.sqrt(theta11)) ** 2)
        # cos_xii = (R ** 2 + S ** 2 - a2 ** 2 - b4 ** 2 - a3 ** 2) / (2 * a2 * math.sqrt(b4 ** 2 + a3 ** 2))
        # if abs(cos_xii) <= 1:
        # sin_xii = math.sqrt(1 - cos_xii ** 2)
        # sin_xii1 = -sin_xii
        # xii = math.atan2(sin_xii, cos_xii)
        # xii1 = math.atan2(sin_xii1, cos_xii)
        #     theta3i = -(xii + math.atan2(b4, a3))
        #     theta33i = -(xii1 + math.atan2(b4, a3))
        # else:
        #     theta3i = NOSOLUTION
        #     theta33i = NOSOLUTION

        # if theta3 == NOSOLUTION:
        #     theta2 = NOSOLUTION
        #     theta22 = NOSOLUTION
        # else:
        #     theta2 = the2_o(omega_x, omega_y, omega_z, theta1, xi)
        #     theta22 = the2_inv(omega_x, omega_y, omega_z, theta1, xi)

        if theta33 == no_solution:
            theta2i = no_solution
            # theta22i = NOSOLUTION
        else:
            theta2i = the2_o(omega_x, omega_y, omega_z, theta1, xi1)
            # theta22i = the2_inv(omega_x, omega_y, omega_z, theta1, xi1)

        # if theta3i == NOSOLUTION:
        #     theta2j = NOSOLUTION
        #     theta22j = NOSOLUTION
        # else:
        #     theta2j = the2_o(omega_x, omega_y, omega_z, theta11, xii)
        #     theta22j = the2_inv(omega_x, omega_y, omega_z, theta11, xii)
        #
        # if theta33i == NOSOLUTION:
        #     theta2k = NOSOLUTION
        #     theta22k = NOSOLUTION
        # else:
        #     theta2k = the2_o(omega_x, omega_y, omega_z, theta11, xii1)
        #     theta22k = the2_inv(omega_x, omega_y, omega_z, theta11, xii1)

        the1, the2, the3 = theta1, theta2i, theta33
        R30 = np.matrix(
            [[math.cos(the1) * math.cos(the2 + the3), -math.cos(the1) * math.sin(the2 + the3), -math.sin(the1)],
             [math.sin(the1) * math.cos(the2 + the3), -math.sin(the1) * math.sin(the2 + the3), math.cos(the1)],
             [-math.sin(the2 + the3), -math.cos(the2 + the3), 0]])
        R63_456 = np.matrix([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        R63 = R63_456.T @ R30.T @ T[:3, :3]

        theta5 = math.atan2(math.sqrt(R63[2, 0] ** 2 + R63[2, 1] ** 2), R63[2, 2])
        if abs(theta5) < 0.001:
            theta4 = 0
            theta6 = math.atan2(-R63[0, 1], R63[0, 0])

        elif abs(theta5 - math.pi) < 0.001:
            theta4 = 0
            theta6 = math.atan2(R63[0, 1], -R63[0, 0])
        else:
            theta4 = -math.atan2(R63[1, 2] / math.sin(theta5), R63[0, 2] / math.sin(theta5))
            theta6 = math.atan2(R63[2, 1] / math.sin(theta5), - R63[2, 0] / math.sin(theta5))
            # theta44 = theta4 + math.pi
            # theta55 = -theta5
            # theta66 = theta6 + math.pi
        the4, the5, the6 = theta4, theta5, theta6

        the1 = float(format(my_loss(the1 * 180 / math.pi), '.4f'))
        the2 = float(format(my_loss(the2 * 180 / math.pi + 90), '.4f'))
        the3 = float(format(my_loss(the3 * 180 / math.pi), '.4f'))
        the4 = float(format(my_loss(the4 * 180 / math.pi), '.4f'))
        the5 = float(format(my_loss(the5 * 180 / math.pi - 90), '.4f'))
        the6 = float(format(my_loss(the6 * 180 / math.pi), '.4f'))

        self.__theta = [the1, the2, the3, the4, the5, the6]
        self.__to_radian()

    def jacobi(self):
        T01 = np.matrix([[math.cos(self.__Theta[0]), -math.sin(self.__Theta[0]), 0, 0],
                         [math.sin(self.__Theta[0]), math.cos(self.__Theta[0]), 0, 0],
                         [0, 0, 1, b1],
                         [0, 0, 0, 1]])
        T12 = np.matrix([[math.sin(self.__Theta[1]), math.cos(self.__Theta[1]), 0, a1],
                         [0, 0, 1, 0],
                         [math.cos(self.__Theta[1]), -math.sin(self.__Theta[1]), 0, 0],
                         [0, 0, 0, 1]])
        T23 = np.matrix([[math.cos(self.__Theta[2]), -math.sin(self.__Theta[2]), 0, a2],
                         [math.sin(self.__Theta[2]), math.cos(self.__Theta[2]), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        T34 = np.matrix([[math.cos(self.__Theta[3]), -math.sin(self.__Theta[3]), 0, a3],
                         [0, 0, 1, b4],
                         [-math.sin(self.__Theta[3]), -math.cos(self.__Theta[3]), 0, 0],
                         [0, 0, 0, 1]])
        T45 = np.matrix([[-math.sin(self.__Theta[4]), -math.cos(self.__Theta[4]), 0, 0],
                         [0, 0, -1, 0],
                         [math.cos(self.__Theta[4]), -math.sin(self.__Theta[4]), 0, 0],
                         [0, 0, 0, 1]])
        T56 = np.matrix([[math.cos(self.__Theta[5]), -math.sin(self.__Theta[5]), 0, 0],
                         [0, 0, -1, b6],
                         [math.sin(self.__Theta[5]), math.cos(self.__Theta[5]), 0, 0],
                         [0, 0, 0, 1]])
        T46 = T45 @ T56
        T36 = T34 @ T46
        T26 = T23 @ T36
        T16 = T12 @ T26
        T06 = T01 @ T16
        J = np.matrix([[0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0]], dtype=float)
        J[3:, 0] = T06[2, :3].T
        J[3:, 1] = T16[2, :3].T
        J[3:, 2] = T26[2, :3].T
        J[3:, 3] = T36[2, :3].T
        J[3:, 4] = T46[2, :3].T
        J[3:, 5] = T56[2, :3].T
        for i in range(3):
            J[i, 0] = T06[0, 3] * T06[1, i] - T06[1, 3] * T06[0, i]
            J[i, 1] = T16[0, 3] * T16[1, i] - T16[1, 3] * T16[0, i]
            J[i, 2] = T26[0, 3] * T26[1, i] - T26[1, 3] * T26[0, i]
            J[i, 3] = T36[0, 3] * T36[1, i] - T36[1, 3] * T36[0, i]
            J[i, 4] = T46[0, 3] * T46[1, i] - T46[1, 3] * T46[0, i]
            J[i, 5] = T56[0, 3] * T56[1, i] - T56[1, 3] * T56[0, i]
        self.__J = J
