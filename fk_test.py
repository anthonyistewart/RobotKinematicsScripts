from numpy import sin, cos, array, pi
import numpy as np

"""
Created on Mon Mar 7 2022

Purpose: Generates the homogenous transformation matrix 
         for a robot arm using its DH parameters.

@author: Anthony Stewart
"""

if __name__ == "__main__":
    # DH Parameters

    q1 = np.radians(0)
    q2 = np.radians(0)
    q3 = np.radians(0)
    q4 = np.radians(0)
    q5 = np.radians(0)

    # Measurements in mm
    a1 = 254.15
    a2 = 250
    a3 = 250

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
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1*cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])

    i = 1
    T_1_2 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1*cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])
    i = 2
    T_2_3 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1*cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])
    i = 3
    T_3_4 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1*cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])
    
    i = 4
    T_4_5 = array([[cos(DH[i][0]), -1*sin(DH[i][0])*cos(DH[i][1]), sin(DH[i][0])*sin(DH[i][1]), DH[i][2]*cos(DH[i][0])],
                   [sin(DH[i][0]), cos(DH[i][0])*cos(DH[i][1]), -1*cos(DH[i][0])*sin(DH[i][1]), DH[i][2]*sin(DH[i][0])],
                   [0, sin(DH[i][1]), cos(DH[i][1]), DH[i][3]],
                   [0, 0, 0, 1],
                   ])

    T_0_5 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5

    print(T_0_5)
