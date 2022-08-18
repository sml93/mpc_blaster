#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

alpha1 = -np.deg2rad(0)      # pitch of M1
alpha2 = np.deg2rad(0)       # roll of M2
theta = np.deg2rad(0)

ybs = np.deg2rad(0)
gamma_ob = np.deg2rad(0)
psi_ob = np.deg2rad(0)
lob = 1.0
lbs = 0.2
ls1s2 = 0.01    # if this is 0, then the point of pitching is at the S2'
lsn = 0.12
ds1s2 = 0.05
y = 0   # to fill with the position/link in the y axis

''' From b to s: @ s1 '''
hbs1 = np.array([ [ np.cos(alpha1),   0,   np.sin(alpha1),   0.01672              ],
                  [ 0,                1,   0,                0                    ],
                  [ -np.sin(alpha1),  0,   np.cos(alpha1),   -0.22937             ],
                  [ 0,                0,   0,                1                    ]]) # ANGLES SEEM TO BE HERE INSTEAD. 

''' From s1 to s2: @ s2'''

hs1s2 = np.array([[ 1,  0,              0,               0.0425          ],
                  [ 0,  np.cos(alpha2), np.sin(alpha2),  0,              ],
                  [ 0, -np.sin(alpha2), np.cos(alpha2),  0               ],
                  [ 0,  0,              0,               1               ]])

''' From s2 to n: @ n'''
hs2n = np.array([ [ 1,  0,  0,  -0.05322              ],
                  [ 0,  1,  0,  0                     ],
                  [ 0,  0,  1,  -0.15946              ],
                  [ 0,  0,  0,  1                     ]])

p_i = np.array([[0], [0], [0], [1]])

# p_bM = hob @ p_i
# print(p_bM)

# p_s1 = np.dot(hbs1, p_i)
p_s1M = (hbs1) @ p_i
print(p_s1M)
# p_s2 = np.dot(hs1s2, p_i)
p_s2M = (hbs1 @ hs1s2) @ p_i
print(p_s2M)

# p_n = np.dot(hs2n, p_i)
p_nM = (hbs1 @ hs1s2 @ hs2n) @ p_i
print(hbs1 @ hs1s2 @ hs2n)
print(p_nM)

xcoord = [p_s1M[0][0], p_s2M[0][0], p_nM[0][0]]
ycoord = [p_s1M[1][0], p_s2M[1][0], p_nM[1][0]]
zcoord = [p_s1M[2][0], p_s2M[2][0], p_nM[2][0]]
# print(p_nM)
''' Plotting positions '''
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.scatter3D(xcoord, ycoord, zcoord, c='k')
ax.plot3D(xcoord, ycoord, zcoord)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# ax.set_xlim([1.0, 1.1])
# ax.set_ylim([1.0, 1.05])
# ax.set_zlim([0, 0.1])
plt.show()

def compute_T_b_s2():
  pass