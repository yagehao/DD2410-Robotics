#! /usr/bin/env python3

"""
    # {Yage Hao}
    # {yage@kth.se}
"""

import math
import numpy as np 

def scara_IK(point):
    """
    Compute inverse kinematics for scara robot using the following equations:
    x = l1*c1 + l2*c12
    y = l1*s1 + l2*s12
    z = -d3
    phi = theta1 +theta2 + theta4
    """
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0] #q[0] = theta1, q[1]=theta2, q[2]=z
    """
    Fill in your IK solution here and return the three joint values in q
    """

    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    c2 = ((x-l0)**2 + y**2 - (l1**2 + l2**2)) / (2*l1*l2)
    theta2 = math.acos(c2)

    s2 = math.sqrt(1 - c2**2)
    k1 = l1 + l2*c2 
    k2 = l2 * s2 
    theta1 = math.atan2(y, x-l0) - math.atan2(k2, k1)

    q[0] = theta1 
    q[1] = theta2 
    q[2] = z

    return q


def kuka_IK(point, R, joint_positions):
    #print("111111111111111111")

    """
    point = (x,y,z), the desired position of the end-effector.
    R = 3x3 rotation matrix, the desired orientation of the end-effector.
    joint_positions = (q1, q2, q3, q4, q5, q6, q7) the current joint position.
    """

    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    
    ref: lecture 4 slides, page 38
    
    Given target X and initial approximation theta_hat
    repeat
        X_hat = kinematics(theta_hat)
        epsilon_x = X_hat - X
        epsilon_theta = Jacobian(theta_hat)*epsilon_x
        theta_hat = theta_hat - epsilon theta
    until epsilon_x <= tolerance
    """

    # init parameter
    epsilon_x = np.ones(6)
    tolerance = 1e-2
    #L = 0.4
    #M = 0.39

    # Given target X and initial approximation theta_hat
    X = np.array([ [x], [y], [z] ]) 
    theta_hat = q 
    #print(X)

    #counter = 0
    while True:
        #print(theta_hat)
        # X_hat = kinematics(theta_hat)
        T, J = transform_matrix(theta_hat)
        #print(T)
        X_hat = np.array([[T[0][3]], [T[1][3]], [T[2][3]]])

        # epsilon_x = X_hat - X
        epsilon_x = X_hat - X

        # epsilon_theta = Jacobian(theta_hat)*epsilon_x
        p1 = np.array([
                [R[0][0]],
                [R[1][0]],
                [R[2][0]]
            ])
        p2 = np.array([
                [R[0][1]],
                [R[1][1]],
                [R[2][1]]
            ])    
        p3 = np.array([
                [R[0][2]],
                [R[1][2]],
                [R[2][2]]
            ])
        epsilon_r = 0.5 * np.transpose(np.cross(np.transpose(T[:3, [0]]), np.transpose(p1)) + np.cross(np.transpose(T[:3, [1]]), np.transpose(p2)) + np.cross(np.transpose(T[:3, [2]]), np.transpose(p3)))
        epsilon_total = np.append(epsilon_x, epsilon_r, 0)
        J_inv = np.linalg.pinv(J)
        epsilon_theta = np.matmul(J_inv, epsilon_total)
        #print(epsilon_theta.T, "2222222222222222222222222")

        # theta_hat = theta_hat - epsilon theta
        theta_hat -= epsilon_theta.T[0]

    #    counter += 1
        #print(abs(max(epsilon_x)) < tolerance)
        if max(np.abs(epsilon_total)) < tolerance:
            #print(abs(max(epsilon_x)))
            q = theta_hat
            break 
    return q 


def transform_matrix(theta_hat):
    "Compute transform matrix T and Jacobian matrix J."

    # DH table values
    alpha1 = [np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0]
    d1 =  [0, 0, 0.4, 0, 0.39, 0, 0]

    T_base = [[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.311],
                            [0, 0, 0, 1]]
    T_end = [[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.078],
                        [0, 0, 0, 1]]

    V1 = []
    V1.append(np.array([0, 0, 0.311]))
    V2 = []
    V2.append(np.array([0, 0, 1]))
    J = []
    
    # passing the T matrix
    T = T_base
    for i in range(len(theta_hat)):
        T_inter= [[np.cos(theta_hat[i]), -np.sin(theta_hat[i])*np.cos(alpha1[i]), np.sin(theta_hat[i])*np.sin(alpha1[i]), 0],
                    [np.sin(theta_hat[i]), np.cos(theta_hat[i])*np.cos(alpha1[i]), -np.cos(theta_hat[i])*np.sin(alpha1[i]), 0],
                    [0, np.sin(alpha1[i]), np.cos(alpha1[i]), d1[i]],
                    [0, 0, 0, 1]]
        T = np.dot(T, T_inter)
        V1.append(T[0:3, 3])
        V2.append(T[0:3, 2])

    T = np.dot(T, T_end)
    V1.append(T[0:3, 3])
    V2.append(T[0:3, 2])
    
    # Jacobian
    for i in range(len(theta_hat)):
        J1 = np.cross(V2[i], V1[-1] - V1[i])
        J2 = np.hstack((J1, V2[i]))
        J.append(J2)
    J = np.array(J).T 
    return T, J 