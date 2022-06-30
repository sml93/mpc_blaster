#!/usr/bin/env python3

PKG = 'px4'

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from poseParser import poseParser


class uavPose(poseParser):
  def __init__(self):
    rospy.init_node('uavPose')
    self.local_position = PoseStamped()
    self.local_pose_sub = rospy.Subscriber('mavros/local_position/pose',
                                          PoseStamped, self.local_position_cb)    
    self.pos = []

  def local_position_cb(self, msg):
    self.local_position = msg
    self.position_x = self.local_position.pose.position.x
    self.position_y = self.local_position.pose.position.y
    self.position_z = self.local_position.pose.position.z
    self.orient_x = self.local_position.pose.orientation.x
    self.orient_y = self.local_position.pose.orientation.y
    self.orient_z = self.local_position.pose.orientation.z
    # print(self.local_position)
    self.pos = np.array((self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z, self.orient_x, self.orient_y, self.orient_z))
    # print(self.position_x, self.position_y, self.position_z, self.orient_x, self.orient_y, self.orient_z)
    print(self.pos)
    return self.pos

def main():
  run = uavPose()
  try: rospy.spin()
  except KeyboardInterrupt: print("Shutting down")


if __name__ == '__main__':
  main()