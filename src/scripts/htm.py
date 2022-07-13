#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

alpha1 = -np.deg2rad(30)      # pitch of M1
alpha2 = np.deg2rad(13)      # roll of M2
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

''' From o to b: @ b '''
hob = np.array([[np.cos(psi_ob), np.sin(psi_ob), 0, lob*np.cos(gamma_ob)*np.cos(psi_ob)],
                [np.sin(psi_ob), np.cos(psi_ob), 0, lob*np.cos(gamma_ob)*np.cos(psi_ob)],
                [0,              0,              1, lob*np.sin(gamma_ob)*np.cos(psi_ob)],
                [0,              0,              0, 1]])

''' From b to s: @ s1 '''
hbs1 = np.array([[np.cos(alpha1-theta),   0, np.sin(alpha1-theta), lbs*np.cos(ybs)*np.cos(theta)],
                 [ 0,              1, 0,             y],
                 [ -np.sin(alpha1-theta), 0, np.cos(alpha1-theta), lbs*np.sin(ybs)*np.cos(theta)],
                 [ 0,              0, 0,             1]])

''' From s1 to s2: @ s2'''
hs1s2 = np.array([[np.cos(alpha1-theta),   0,  np.sin(alpha1-theta), ls1s2*np.cos(alpha1-theta+np.pi/2)*np.cos(theta)],
                  [ 0,              1,  0,             y ],
                  [ -np.sin(alpha1-theta), 0,  np.cos(alpha1-theta), ls1s2*np.sin(alpha1-theta-np.pi/2)*np.cos(theta)],
                  [ 0,              0,  0,             1]])

''' From s2 to n: @ n'''
hs2n = np.array([[ 1,              0,  np.sin(alpha2-theta),  0],
                 [ 0,              np.cos(alpha2-theta),  0,  y + lsn*np.sin(alpha2)],
                 [ -np.sin(alpha2-theta), 0,  np.cos(alpha2-theta),  -lsn*np.cos(alpha2)],
                 [ 0,              0,  0,              1]])

p_i = np.array([[0], [0], [0], [1]])

p_bM = hob @ p_i
print(p_bM)

# p_s1 = np.dot(hbs1, p_i)
p_s1M = (hob @ hbs1) @ p_i
print(p_s1M)

# p_s2 = np.dot(hs1s2, p_i)
p_s2M = (hob @ hbs1 @ hs1s2) @ p_i
print(p_s2M)

# p_n = np.dot(hs2n, p_i)
p_nM = (hob @ hbs1 @ hs1s2 @ hs2n) @ p_i
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
ax.set_xlim([1.1, 1.3])
ax.set_ylim([1.0, 1.05])
ax.set_zlim([-0.10, 0])
plt.show()
