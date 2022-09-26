#! /usr/bin/env python3
import sys
import csv

import rospy
import numpy as np
import matplotlib.pyplot as plt 

from casadi import *
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from rosPathGen import pathgen

# alph1 = 0
# alph2 = 0
# force = 1.5
arr = []

class sensing:
  def __init__(self, force):
    self.alpha1 = 0.0
    self.alpha2 = 0.0
    self.dne = 0
    self.pe_sub = rospy.Subscriber("p_e", Point, self.getPe)
    self.standoff_sub = rospy.Subscriber("standoff", Float32, self.getRange)
    self.alph1_sub = rospy.Subscriber('alph1', Float32, self.getAlph1)
    self.alph2_sub = rospy.Subscriber('alph2', Float32, self.getAlph2)
    self.p_ex = 0
    self.p_ey = 0
    self.p_ez = 0
    self.dne_hat = 1.0
    self.dne = 0
    self.mu = 1
    self.getmu = []    # for plotting purposes
    self.pe_ranger = []
    self.p_ex_plt = []
    self.p_ey_plt = []
    self.p_ez_plt = []

  def rot_gim(self):
    R_gimbal_1 = SX.eye(3)
    R_gimbal_2 = SX.eye(3)

    """ Rotation about x """ 
    R_gimbal_2[1, 1] = cos(self.alpha2)
    R_gimbal_2[1, 2] = -sin(self.alpha2)
    R_gimbal_2[2, 1] = sin(self.alpha2)
    R_gimbal_2[2, 2] = cos(self.alpha2)

    """ Rotation about y """
    R_gimbal_1[0, 0] = cos(self.alpha1)
    R_gimbal_1[0, 2] = sin(self.alpha1)
    R_gimbal_1[2, 0] = -sin(self.alpha1)
    R_gimbal_1[2, 2] = cos(self.alpha1)

    """ Body to nozzle rotation """
    R_gimbal = R_gimbal_1 @ R_gimbal_2
    return R_gimbal

  def getAlph1(self, msg):
    self.alpha1 = np.deg2rad(msg.data)

  def getAlph2(self, msg):
    self.alpha2 = np.deg2rad(msg.data)

  def getForce(self):
    """ Insert force models here,
    Go figure an interpolation method for distances between each interval"""
    pass

  def getPe(self, msg):
    """ Subscribing to p_e topic """
    self.p_ex = msg.x
    self.p_ey = msg.y
    self.p_ez = msg.z
    # print ([self.p_ex, self.p_ey, self.p_ez])
    self.dne_hat = np.linalg.norm([self.p_ex, self.p_ey, self.p_ez])


  def getRange(self, msg):
    """ Subscribing to standoff topic """
    self.dne = msg.data
    print ("dne: ", self.dne)


  def getDiff(self):
    self.mu = self.dne / self.dne_hat
    print ("mu: " + str(np.round(self.mu, 4)) + "\n")

  def fixedMu(self):
    self.getmu = np.linspace(0.5, 2.5, 5)

  
  def getPeRef(self):
    pe_pub = rospy.Publisher("pe_ref", Point, queue_size=10)
    rospy.init_node('standoff_sensing', anonymous=True)
    # self.pe_ranger = np.multiply([self.p_ex, self.p_ey, self.p_ez], self.mu)

    """ Actual """
    # while not rospy.is_shutdown():
    #   self.getDiff()
    #   pe_ref = Point()
    #   pe_ref.x = np.round(self.p_ex * self.mu, 4)
    #   pe_ref.y = np.round(self.p_ey * self.mu, 4)
    #   pe_ref.z = np.round(self.p_ez * self.mu, 4)
    #   print("pe_ref:\n" + str(pe_ref) + "\n")
    #   pe_pub.publish(pe_ref)

    """ For plotting only """
    alpha = 1
    theta = np.linspace(0, 2*np.pi, 180)
    radius = 2
    length = 2

    run = pathgen(alpha, radius, theta, length)
    a, b, c, d, e, f, z = run.genPath()
    # z = np.linspace(0, 0, 190)

    # arr = np.transpose(np.array(([a,b,z])))
    e = np.array(e)
    f = np.array(f)
    print(a.shape)

    file = open('lemn.csv', 'w')
    writer = csv.writer(file)
    # for i in range(len(a)):
    #   writer.writerow(arr[i])
    # f.close()



    while not rospy.is_shutdown(): 
      self.fixedMu()
      fig = plt.figure()
      ax = fig.gca(projection='3d')
      # ax.set_aspect(2)
      for i in range(len(self.getmu)):
        self.mu = self.getmu[i]
        # print(self.mu)
        pe_ref = Point()
        pe_ref.x = np.round(self.p_ex * self.mu, 4)
        pe_ref.y = np.round(self.p_ey * self.mu, 4)
        pe_ref.z = np.round(self.p_ez * self.mu, 4)
        # print("pe_ref:\n" + str(pe_ref) + "\n")
        pe_pub.publish(pe_ref)
        # self.p_ex_circ = np.round(a * self.mu, 4)
        # self.p_ey_circ = np.round(b * self.mu, 4)
        # self.p_ez_circ = np.round(z * self.mu, 4)
        # arr = np.transpose(np.array(([self.p_ex_circ, self.p_ey_circ, self.p_ez_circ])))
        # ax.scatter3D(self.p_ex_circ, self.p_ey_circ, self.p_ez_circ)
        # ax.plot(self.p_ex_plt, self.p_ey_plt)

        self.p_ex_lemn = np.round(e * self.mu, 4)
        self.p_ey_lemn = np.round(f * self.mu, 4)
        self.p_ez_lemn = np.round(z * self.mu, 4)
        arr = np.transpose(np.array(([self.p_ex_lemn, self.p_ey_lemn, self.p_ez_lemn])))
        ax.scatter3D(self.p_ex_lemn, self.p_ey_lemn, self.p_ez_lemn)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.invert_zaxis()
        print(arr)

        for i in range(len(e)):
          writer.writerow(arr[i])
      file.close()
      plt.show()
      # plt.close()


def main(args):
  try:
    sensing(1.5).getPeRef()
    rospy.init_node('standoff_sensing', anonymous = True)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  # pass


if __name__ == "__main__":
  main(sys.argv)