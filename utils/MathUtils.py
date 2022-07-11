from casadi import *
import numpy as np
import sys

def quatMultiplication(q1, q2):

    """ 
    
        Performs Hamilton product on two quaternions. 
        Assume that q = [w, x, y, z].
        
        TODO: Test out this function. (May need to write a NumPy version of this.)
    
    """

    c = SX([
        [q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]],
        [q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]],
        [q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]],
        [q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]]
        ])

    return c 

def unitQuatInversion(q): 

    """ 

        Performs quat inversion for a unit quaternion.

    """

    return SX([q[0], -q[1], -q[2], -q[3]])