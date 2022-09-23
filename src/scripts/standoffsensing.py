import sys
import rospy
import numpy as np

from casadi import *
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

# alph1 = 0
# alph2 = 0
# force = 1.5

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
    self.pe_ranger = []

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

  
  def getPeRef(self):
    pe_pub = rospy.Publisher("pe_ref", Point, queue_size=10)
    rospy.init_node('standoff_sensing', anonymous=True)
    # self.pe_ranger = np.multiply([self.p_ex, self.p_ey, self.p_ez], self.mu)
    while not rospy.is_shutdown():
      self.getDiff()
      # print("ok")
      pe_ref = Point()
      pe_ref.x = np.round(self.p_ex * self.mu, 4)
      pe_ref.y = np.round(self.p_ey * self.mu, 4)
      pe_ref.z = np.round(self.p_ez * self.mu, 4)
      print("pe_ref:\n" + str(pe_ref) + "\n")
      pe_pub.publish(pe_ref)



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