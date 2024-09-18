#!/usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,7)) # axis vector of each joint axis
    
    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                    [-0.7040, 0.7102, -0.0053],
                    [0.7102, 0.7040, 0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)

    def twist(omega, q):
        res = [0] * 6
        res[:3] = -np.cross(omega, q)
        res[3:] = omega
        return np.array(res)  # return 6x1 matrix
    
    # xis = [twist(ws[0:3,i], qs[0:3,i]) for i in range(7)]
    xis = np.array([twist(ws[0:3, i], qs[0:3, i]) for i in range(7)])
    xis = xis.T  # Transpose to get the shape (7, 6)

    g_0 = np.zeros((4,4))
    g_0[0:3,0:3] = R
    g_0[3,3] = 1
    g_0[0:3,3] = qs[0:3,7]

    # g = np.eye(4)
    # for i in range(7):
    #     g = g @ kfs.homog_3d(twist(ws[0:3, i], qs[0:3, i]))
    g = kfs.prod_exp(xis, joint_angles)

    return g @ g_0


def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)
    left = joint_state.position[2:9]
    angles = [
        left[2], left[3], left[0], left[1], left[4], left[5], left[6]
    ]
    # return baxter_forward_kinematics_from_angles(angles)

    # END YOUR CODE HERE
    print(baxter_forward_kinematics_from_angles(angles))