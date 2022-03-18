from numpy import arccos, arctan, sin, cos, array, pi, sqrt, arctan2
import numpy as np

"""
Created on Mon Mar 7 2022

Purpose: Solves for a 6 axis robots' joint angles using the 
         inverse kinematic equations I derived geometrically
         on paper. Also checks the inverse kinematics by verifying
         the end effector position using the FK from the DH parameters.

@author: Anthony Stewart
"""


def FK(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]

    # DH Parameters

    DH = array([[q1, np.radians(-90), 0, a1],
                [np.radians(-90) + q2, 0, a2, 0],
                [q3, np.radians(-90), 0, 0],
                [q4, np.radians(90), 0, a3],
                [q5, np.radians(-90), 0, 0],
                ])

    # Homogenous Transformation Matrices
    i = 0
    T_0_1 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1 *
                    cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])

    i = 1
    T_1_2 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1 *
                    cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])
    i = 2
    T_2_3 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1 *
                    cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])
    i = 3
    T_3_4 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1 *
                    cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])

    i = 4
    T_4_5 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1 *
                    cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])

    T_0_5 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5

    vector = array([[0],
                    [0],
                    [0],
                    [1]])

    wrist_center = T_0_5 @ vector

    x = wrist_center[0][0]
    y = wrist_center[1][0]
    z = wrist_center[2][0]

    return [x, y, z]


def IK(target):
    x = target[0]
    y = target[1]
    z = target[2]

    q = [0, 0, 0, 0, 0]

    a = a3
    b = a2
    r = sqrt(x**2 + y**2)
    c = sqrt(r**2 + (z - a1)**2)
    alpha = arccos((a**2 - b**2 - c**2)/(-2*b*c))
    gamma = arccos((c**2 - a**2 - b**2)/(-2*a*b))
    delta = arctan2((z-a1), r)
    q[0] = arctan2(y, x)
    q[1] = np.radians(90) - (alpha + delta)
    q[2] = np.radians(90) - gamma 

    return q


if __name__ == "__main__":
    # Measurements in mm
    a1 = 254.15
    a2 = 250
    a3 = 250
    
    target = [250, 0, 504.15]
    q = IK(target)
    print("IK results for target", target, ":", np.around(np.rad2deg(q), 2))

    actual = FK(q)
    print("FK results from IK values:", np.around(actual,2))
