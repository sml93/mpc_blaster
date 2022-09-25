#!/usr/bin/env python
from array import array
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Point


z = []

class pathgen():
  def __init__(self, alpha, radius, theta, length):
    self.alpha = alpha
    self.radius = radius
    self.theta = theta
    self.length = length
    
  def circle(self):
    a = self.radius * np.cos(self.theta)
    b = self.radius * np.sin(self.theta)
    return a, b

  def square(self):
    x = [self.length,self.length, -self.length, -self.length, self.length]
    y = [self.length, -self.length, -self.length, self.length, self.length]
    return x, y

  def lemniscale(self):
    x = [self.alpha * np.sqrt(2)*np.cos(i) / (np.sin(i)**2+1) for i in self.theta]
    y = [self.alpha * np.sqrt(2)*np.cos(i)*np.sin(i) / (np.sin(i)**2+1) for i in self.theta]
    return x,y

  def genPath(self):
    pub = rospy.Publisher('p_e', Point,  queue_size=10)
    # rospy.init_node('pathgen', anonymous=True)
    rospy.init_node('standoff_sensing', anonymous=True)
    r = rospy.Rate(10)
    msg = Point()

    z = np.linspace(0, 5, 10)

    a, b = self.circle()
    c, d = self.square()
    e,f = self.lemniscale()

    for i in range(len(z)):
      a = np.insert(a, 0, 0)
      b = np.insert(b, 0, 0)
      e = np.insert(e, 0, 0)
      f = np.insert(f, 0, 0)

    z_1 = np.linspace(5, 5, 180)
    z = np.concatenate([z, z_1])
    ht = np.linspace(5, 5, 190)

    # run = 1
    

    """ Publishing paths"""
    while not rospy.is_shutdown():
      for i in range(len(a)):
        msg.x = a[i]
        msg.y = b[i]
        msg.z = 0
      pub.publish(msg)
      # run = 0


      """ Plotting subplots style """
      # figure, (axe1, axe2, axe3) = plt.subplots(1,3)
      # axe1.plot(qa, b)
      # axe1.set_aspect(1)
      # axe1.set_title('Circle')

      # axe2.plot(c,d)
      # axe2.set_aspect(1)
      # axe2.set_title('Square')

      # axe3.plot(e,f)
      # axe3.set_aspect(1)
      # axe3.set_title('Lemniscale')


      """ Plotting 3D """
      fig = plt.figure()
      ax = fig.gca(projection='3d')
      ax.plot3D(a, b, z, c='k')

      ax.set_xlabel('X')
      ax.set_ylabel('Y')
      ax.set_zlabel('Z')

      ax2 = fig.gca(projection='3d')
      ax2.plot3D(e, f, ht, c='r')
      ax2.set_xlabel('X')
      ax2.set_ylabel('Y')
      ax2.set_zlabel('Z')

      # plt.show()
      plt.close()
      return a, b, c, d, e, f, z


def main(args):
  alpha = 1
  theta = np.linspace(0, 2*np.pi, 180)
  radius = 2
  length = 2
  pathgen(alpha, radius, theta, length).genPath()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  

if __name__ == "__main__":
  main(sys.argv)