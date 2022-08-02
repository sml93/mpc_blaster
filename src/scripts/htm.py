#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def compute_T_b_s2(alpha1, alpha2):
    ''' From b to s: @ s1 '''
    hbs1 = np.array([ [ np.cos(alpha1),   0,   np.sin(alpha1),   0.01672   ],
                      [ 0,                1,   0,                0         ],
                      [ -np.sin(alpha1),  0,   np.cos(alpha1),   -0.22937  ],
                      [ 0,                0,   0,                1         ]]) # ANGLES SEEM TO BE HERE INSTEAD. 

    ''' From s1 to s2: @ s2'''
    hs1s2 = np.array([[ 1,  0,              0,               0.0425        ],
                      [ 0,  np.cos(alpha2), np.sin(alpha2),  0,            ],
                      [ 0, -np.sin(alpha2), np.cos(alpha2),  0             ],
                      [ 0,  0,              0,               1             ]])

    ''' From s2 to n: @ n'''
    hs2n = np.array([ [ 1,  0,  0,  -0.05322    ],
                      [ 0,  1,  0,  0           ],
                      [ 0,  0,  1,  -0.15946    ],
                      [ 0,  0,  0,  1           ]])

    return hbs1@hs1s2@hs2n

def compute_T_w_b(phi, theta, psi, position): 

    T = np.eye(4)
    Rot = R.from_euler('zyx', [psi, theta, phi])
    T[0:3, 0:3] = Rot.as_matrix()
    T[0:3, 3] = position

    return T