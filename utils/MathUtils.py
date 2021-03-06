from casadi import *
import numpy as np
import sys

def quatMultiplication(q1, q2):

    """ 
    
        Performs Hamilton product on two quaternions. 
        Assume that q = [w, x, y, z].
        
        TODO: Test out this function. (May need to write a NumPy version of this.)
    
    """

    c = SX.zeros(4)

    c[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    c[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    c[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    c[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]

    return c 

def unitQuatInversion(q): 

    """ 

        Performs quat inversion for a unit quaternion.

    """
    
    qinv = SX.zeros(4)
    qinv[0] = q[0]
    qinv[1] = -q[1]
    qinv[2] = -q[2]
    qinv[3] = -q[3]

    return qinv

def quat2Rot(eta): 

    R = SX(3,3)
    R[0,0] = 2*(eta[0]**2 + eta[1]**2) - 1
    R[0,1] = 2*(eta[1]*eta[2] - eta[0]*eta[3])
    R[0,2] = 2*(eta[1]*eta[3] + eta[0]*eta[2])
    R[1,0] = 2*(eta[1]*eta[2] + eta[0]*eta[3])
    R[1,1] = 2*(eta[0]**2 + eta[2]**2) - 1
    R[1,2] = 2*(eta[2]*eta[3] - eta[0]*eta[1])
    R[2,0] = 2*(eta[1]*eta[3] - eta[0]*eta[2])
    R[2,1] = 2*(eta[2]*eta[3] + eta[0]*eta[1])
    R[2,2] = 2*(eta[0]**2 + eta[3]**2) - 1

    return R
